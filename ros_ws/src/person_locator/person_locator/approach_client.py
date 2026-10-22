#!/usr/bin/env python3
"""调用 /person/approach 的封装：别的节点或脚本里一行调用「走到人面前」。

    from person_locator.approach_client import approach_person
    ok, msg = approach_person(node, distance=3.0)          # 阻塞到结束

命令行：
    ros2 run person_locator approach_person --distance 3.0 [--timeout 60]
    （face_front 是 person_tracker 的节点参数：ros2 param set /person_tracker face_front true）
"""
import argparse
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from robot_interfaces.action import ApproachPerson


def approach_person(node, distance=3.0, timeout=0.0, search_timeout=0.0, on_feedback=None, wait=True):
    """发送 ApproachPerson goal。

    node 需要已经在某个 executor 里 spin（或 wait=False 自己处理 future）。
    wait=True 时阻塞到动作结束，返回 (success, message)；wait=False 返回 (goal_handle, result_future)。
    """
    client = ActionClient(node, ApproachPerson, '/person/approach')
    if not client.wait_for_server(timeout_sec=5.0):
        return (False, 'person_tracker 不在线（/person/approach）') if wait else (None, None)
    goal = ApproachPerson.Goal(distance=float(distance), timeout=float(timeout),
                               search_timeout=float(search_timeout))
    cb = (lambda m: on_feedback(m.feedback)) if on_feedback else None
    sent = threading.Event()
    fut = client.send_goal_async(goal, feedback_callback=cb)
    fut.add_done_callback(lambda _f: sent.set())
    sent.wait()
    handle = fut.result()
    if not handle.accepted:
        return (False, '被拒绝（已有 approach 在执行？）') if wait else (handle, None)
    result = handle.get_result_async()
    if not wait:
        return handle, result
    done = threading.Event()
    result.add_done_callback(lambda _f: done.set())
    done.wait()
    r = result.result().result
    return r.success, r.message


def main():
    ap = argparse.ArgumentParser(description='走到人面前')
    ap.add_argument('--distance', type=float, default=3.0)
    ap.add_argument('--timeout', type=float, default=0.0)
    ap.add_argument('--search-timeout', type=float, default=0.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('approach_person_cli')
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    def fb(f):
        print(f'  {f.state:<11} #{f.target_id}  距离 {f.distance:.2f}m')

    try:
        ok, msg = approach_person(node, args.distance, args.timeout, args.search_timeout, on_feedback=fb)
        print(('成功: ' if ok else '失败: ') + msg)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
