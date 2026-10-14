#!/usr/bin/env python3
"""
Nav2 lifecycle compatibility helper and one-shot initial pose publisher.

nav2.launch.py uses standard lifecycle managers and calls this helper only to
publish the initial pose.  Running it without --initial-pose-only retains the
manual lifecycle activation mode for diagnostics.
"""
import shutil
import subprocess
import time
import sys
import os
import argparse
import re

# Localization nodes must come up before navigation nodes.
# map_server must be fully active before amcl is configured (amcl needs /map).
LOCALIZATION_NODES = [
    'map_server',
    'amcl',
]

NAVIGATION_NODES = [
    'controller_server',
    'planner_server',
    'smoother_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]

NODES = LOCALIZATION_NODES + NAVIGATION_NODES

ROS2 = shutil.which('ros2') or '/opt/ros/humble/bin/ros2'


def run(args, timeout=20):
    cmd = [ROS2] + args
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f'[nav2_activate] TIMEOUT: {" ".join(args[:3])}')
        return subprocess.CompletedProcess(cmd, -1, '', 'timeout')
    return r


def node_exists(name):
    r = run(['node', 'list'])
    return f'/{name}' in r.stdout.splitlines()


def get_state(node):
    r = run(['lifecycle', 'get', f'/{node}'], timeout=10)
    # Match the state token itself.  A substring check would classify
    # "inactive" as "active".
    for line in (r.stdout + r.stderr).splitlines():
        match = re.search(
            r'(?:primary\s+state:\s*)?\b(inactive|active|unconfigured|finalized)\b',
            line.lower(),
        )
        if match:
            return match.group(1)
    return None


def wait_lifecycle_ready(node, timeout=30):
    """Wait until lifecycle get returns a known state (service is ready)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        s = get_state(node)
        if s is not None:
            return s
        time.sleep(1)
    return None


def do_transition(node, transition):
    expected = {'configure': 'inactive', 'activate': 'active'}
    r = run(['lifecycle', 'set', f'/{node}', transition], timeout=20)
    combined = (r.stdout + r.stderr).lower()
    if 'successful' in combined:
        return True
    post = get_state(node)
    if post == expected.get(transition):
        return True
    print(f'[nav2_activate] {node} {transition} failed | '
          f'state={post} | {r.stdout.strip()} {r.stderr.strip()}')
    return False


def wait_for_all_nodes(timeout=120):
    print('[nav2_activate] waiting for all nav2 nodes to appear...')
    deadline = time.time() + timeout
    while time.time() < deadline:
        missing = [n for n in NODES if not node_exists(n)]
        if not missing:
            print('[nav2_activate] all nodes found')
            return True
        print(f'[nav2_activate] still missing: {missing}')
        time.sleep(3)
    print('[nav2_activate] TIMEOUT waiting for nodes')
    return False


def drive_to(node, target):
    """Drive node to 'inactive' or 'active', with retries."""
    transitions = {'inactive': 'configure', 'active': 'activate'}
    transition = transitions[target]
    # Wait for lifecycle service to be ready first
    state = wait_lifecycle_ready(node, timeout=30)
    if state is None:
        print(f'[nav2_activate] ERROR: lifecycle service not ready for {node}')
        return False
    if state == target:
        print(f'[nav2_activate] {node} already {target}')
        return True
    # If already active and we only want inactive, that's fine too
    if target == 'inactive' and state == 'active':
        return True
    for attempt in range(1, 7):
        state = get_state(node)
        if state == target:
            return True
        if target == 'active' and state == 'inactive':
            if do_transition(node, 'activate'):
                return True
        elif target == 'inactive' and state == 'unconfigured':
            if do_transition(node, 'configure'):
                return True
        elif state == 'active' and target == 'active':
            return True
        print(f'[nav2_activate] {node} attempt {attempt}: state={state}, want {target}')
        time.sleep(2)
    print(f'[nav2_activate] ERROR: {node} stuck at {get_state(node)}')
    return False


def main():
    args = parse_args()

    if args.initial_pose_only:
        print('[nav2_activate] waiting for AMCL to become active...')
        deadline = time.time() + 120
        while time.time() < deadline:
            if get_state('amcl') == 'active':
                time.sleep(2.0)  # allow AMCL to receive the latched map
                send_initial_pose(as_bool(args.use_sim_time))
                return
            time.sleep(1.0)
        print('[nav2_activate] AMCL did not become active; set pose in RViz manually.')
        return

    print('[nav2_activate] starting — waiting 8s for nodes to initialize...')
    time.sleep(8)

    if not wait_for_all_nodes(timeout=120):
        sys.exit(1)

    # Extra wait for lifecycle services to register
    print('[nav2_activate] nodes visible, waiting 3s for lifecycle services...')
    time.sleep(3)

    # Phase 1: bring up localization first (map_server active → amcl can get /map)
    print('[nav2_activate] --- localization phase ---')
    for node in LOCALIZATION_NODES:
        print(f'[nav2_activate] configure {node}...')
        drive_to(node, 'inactive')
        time.sleep(0.5)
    for node in LOCALIZATION_NODES:
        print(f'[nav2_activate] activate {node}...')
        drive_to(node, 'active')
        time.sleep(1.0)   # give map_server time to publish /map before amcl activates

    # Phase 2: navigation stack
    print('[nav2_activate] --- navigation phase ---')
    for node in NAVIGATION_NODES:
        print(f'[nav2_activate] configure {node}...')
        drive_to(node, 'inactive')
        time.sleep(0.5)
    for node in NAVIGATION_NODES:
        print(f'[nav2_activate] activate {node}...')
        drive_to(node, 'active')
        time.sleep(0.5)

    # Final check
    failed = []
    for node in NODES:
        s = get_state(node)
        if s != 'active':
            failed.append(f'{node}({s})')

    if failed:
        print(f'[nav2_activate] WARNING: not all nodes active: {failed}')
    else:
        print('[nav2_activate] === all nav2 nodes active ===')

    send_initial_pose(as_bool(args.use_sim_time))
    print('[nav2_activate] Done. If pose is wrong, use rviz2 "2D Pose Estimate" to correct it.')


INITIAL_POSE_SCRIPT = '''
import rclpy, time, sys
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init()
n = Node("_init_pose_sender", parameter_overrides=[
    rclpy.parameter.Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, __USE_SIM_TIME__)
])
pub = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
time.sleep(1.5)
for _ in range(3):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = n.get_clock().now().to_msg()
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = 0.5
    msg.pose.covariance[7] = 0.5
    msg.pose.covariance[35] = 0.27
    pub.publish(msg)
    time.sleep(0.5)
print("initialpose sent, stamp.sec=" + str(msg.header.stamp.sec))
rclpy.shutdown()
'''


def send_initial_pose(use_sim_time=True):
    print('[nav2_activate] sending initial pose at map origin (0,0)...')
    # Use system python3 so rclpy C extension is available
    python = shutil.which('python3') or '/usr/bin/python3'
    # Prefer system python3 over miniconda if miniconda is the default
    for candidate in ['/usr/bin/python3', python]:
        if os.path.exists(candidate):
            python = candidate
            break
    pose_script = INITIAL_POSE_SCRIPT.replace(
        '__USE_SIM_TIME__', 'True' if use_sim_time else 'False')
    r = subprocess.run(
        [python, '-c', pose_script],
        capture_output=True, text=True, timeout=20
    )
    if r.returncode == 0:
        print(f'[nav2_activate] {r.stdout.strip()}')
    else:
        print(f'[nav2_activate] initialpose failed: {r.stderr.strip()[-300:]}')
        print('[nav2_activate] Use rviz2 "2D Pose Estimate" to set initial pose manually.')


def as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--initial-pose-only', action='store_true',
                        help='wait for AMCL and publish the initial pose')
    parser.add_argument('--use-sim-time', default='true',
                        help='whether the initial pose node uses /clock')
    return parser.parse_args()


if __name__ == '__main__':
    main()
