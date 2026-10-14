"""快捷命令 + 受限 ros2 CLI"""
import shlex
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
import process_mgr

router = APIRouter(prefix="/api/cmd")

# id -> (分组, 显示名, [argv...], 超时秒)；多条 argv 依次执行
QUICK: dict[str, tuple[str, str, list[list[str]], float]] = {
    "node_list":  ("查看", "node list",       [["ros2", "node", "list"]], 8),
    "topic_list": ("查看", "topic list",      [["ros2", "topic", "list", "-t"]], 8),
    "tf_map":     ("查看", "TF map→base",     [["ros2", "run", "tf2_ros", "tf2_echo", "map", "base_footprint"]], 4),
    "tf_odom":    ("查看", "TF lio_odom→base", [["ros2", "run", "tf2_ros", "tf2_echo", "lio_odom", "base_footprint"]], 4),
    "hz_scan":    ("频率", "/scan hz",        [["ros2", "topic", "hz", "/scan"]], 5),
    "hz_lio":     ("频率", "/lio_odom hz",    [["ros2", "topic", "hz", "/lio_odom"]], 5),
    "hz_ekf":     ("频率", "/lio_ekf_odom hz", [["ros2", "topic", "hz", "/lio_ekf_odom"]], 5),
    "hz_yaw":     ("频率", "/robot/yaw hz",   [["ros2", "topic", "hz", "/robot/yaw"]], 5),
    "echo_yaw":   ("查看", "/robot/yaw",      [["ros2", "topic", "echo", "--once", "/robot/yaw"]], 4),
    "loc_score":  ("导航", "定位得分",         [["ros2", "topic", "echo", "--once", "/localizer/score"]], 4),
    "relocalize": ("导航", "原地重定位",       [["ros2", "service", "call", "/relocalize_node/relocalize",
                                               "std_srvs/srv/Trigger"]], 15),
    "clear_cost": ("导航", "清 costmap",      [
        ["ros2", "service", "call", "/global_costmap/clear_entirely_global_costmap",
         "nav2_msgs/srv/ClearEntireCostmap", "{}"],
        ["ros2", "service", "call", "/local_costmap/clear_entirely_local_costmap",
         "nav2_msgs/srv/ClearEntireCostmap", "{}"]], 6),
    "nav_active": ("导航", "nav2 是否激活",    [["ros2", "service", "call",
                                               "/lifecycle_manager_navigation/is_active",
                                               "std_srvs/srv/Trigger"]], 6),
}

# CLI 只放行这些 ros2 子命令；run / launch 等能起任意进程的不开放
CLI_ALLOWED = {"topic", "node", "service", "param", "action", "interface", "lifecycle", "pkg", "doctor"}


class CliReq(BaseModel):
    cmd: str
    timeout: float = 6.0


@router.get("")
def list_quick():
    return [{"id": k, "group": g, "label": l} for k, (g, l, _, _) in QUICK.items()]


@router.post("/quick/{cid}")
def run_quick(cid: str):
    if cid not in QUICK:
        raise HTTPException(404, cid)
    _, label, argvs, timeout = QUICK[cid]
    outputs, rc = [], 0
    for argv in argvs:
        r = process_mgr.run_once(argv, timeout=timeout)
        outputs.append(f"$ {shlex.join(argv)}\n{r['output']}")
        rc = rc or (0 if r["timed_out"] else r["rc"])
    return {"label": label, "rc": rc, "output": "\n".join(outputs)}


@router.post("/cli")
def run_cli(req: CliReq):
    try:
        argv = shlex.split(req.cmd)
    except ValueError as e:
        raise HTTPException(400, f"解析失败: {e}")
    if len(argv) < 2 or argv[0] != "ros2" or argv[1] not in CLI_ALLOWED:
        raise HTTPException(400, f"只支持 ros2 {{{', '.join(sorted(CLI_ALLOWED))}}} ...")
    r = process_mgr.run_once(argv, timeout=max(1.0, min(req.timeout, 30.0)))
    return {"rc": r["rc"], "timed_out": r["timed_out"], "output": r["output"]}
