"""
长驻进程管理 + 一次性 ros2 命令执行。

每个进程独立进程组（start_new_session），停止时对整组发 SIGINT → SIGTERM → SIGKILL，
避免 ros2 launch 的子节点变成孤儿继续占串口。
"""
import collections, os, signal, subprocess, threading, time
from config import ROS2_SETUP, WORKSPACE_SETUP, LOG_BUFFER_SIZE, ROS_DOMAIN_ID

_procs: dict[str, subprocess.Popen] = {}
_meta: dict[str, dict] = {}          # name -> {cmd, started_at, exit_code}
_logs: collections.deque = collections.deque(maxlen=LOG_BUFFER_SIZE)  # (seq, name, line)
_seq = 0
_lock = threading.Lock()


def _env() -> dict:
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)
    env["PYTHONUNBUFFERED"] = "1"
    env["RCUTILS_COLORIZED_OUTPUT"] = "0"
    return env


def _prelude() -> str:
    parts = [f"source {ROS2_SETUP}"]
    if os.path.exists(WORKSPACE_SETUP):
        parts.append(f"source {WORKSPACE_SETUP}")
    return " && ".join(parts)


def _append_log(name: str, line: str) -> None:
    global _seq
    with _lock:
        _seq += 1
        _logs.append((_seq, name, line))


log = _append_log   # 供路由写 "ui" 日志


def _reader(name: str, proc: subprocess.Popen) -> None:
    for raw in iter(proc.stdout.readline, b""):
        _append_log(name, raw.decode(errors="replace").rstrip())
    proc.stdout.close()
    code = proc.wait()
    with _lock:
        if _meta.get(name, {}).get("pid") == proc.pid:
            _meta[name]["exit_code"] = code
    _append_log(name, f"--- 进程退出 code={code} ---")


def _kill_group(proc: subprocess.Popen) -> None:
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    for sig, wait in ((signal.SIGINT, 8.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=wait)
            # launch 已退出，但组里可能还有没收到信号的节点
            time.sleep(0.3)
            os.killpg(pgid, 0)
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return


def start(name: str, cmd: str) -> None:
    stop(name)
    proc = subprocess.Popen(
        ["bash", "-c", f"{_prelude()} && exec {cmd}"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        env=_env(), start_new_session=True,
    )
    with _lock:
        _procs[name] = proc
        _meta[name] = {"cmd": cmd, "pid": proc.pid, "started_at": time.time(), "exit_code": None}
    _append_log(name, f"--- 启动: {cmd} ---")
    threading.Thread(target=_reader, args=(name, proc), daemon=True).start()


def stop(name: str) -> None:
    with _lock:
        proc = _procs.pop(name, None)
    if proc is None:
        return
    _append_log(name, "--- 停止中 ---")
    _kill_group(proc)


def last_cmd(name: str) -> str | None:
    with _lock:
        return _meta.get(name, {}).get("cmd")


def stop_all() -> None:
    with _lock:
        names = list(_procs)
    for n in names:
        stop(n)


def is_running(name: str) -> bool:
    with _lock:
        proc = _procs.get(name)
    return proc is not None and proc.poll() is None


def info(name: str) -> dict:
    with _lock:
        meta = dict(_meta.get(name, {}))
    meta.pop("pid", None)
    meta["running"] = is_running(name)
    return meta


def logs_since(seq: int) -> list[tuple]:
    with _lock:
        return [e for e in _logs if e[0] > seq]


def run_once(args: list[str], timeout: float = 10.0) -> dict:
    """
    执行一次性命令（argv 形式，不经 shell 解析，防注入）。
    超时后对进程组发 SIGINT，返回已有输出——topic hz/echo 这类不会自己退出的命令靠这个收尾。
    """
    proc = subprocess.Popen(
        ["bash", "-c", f'{_prelude()} && exec "$@"', "_", *args],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        env=_env(), start_new_session=True,
    )
    timed_out = False
    try:
        out, _ = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        timed_out = True
        try:
            os.killpg(proc.pid, signal.SIGINT)
            out, _ = proc.communicate(timeout=3)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            out, _ = proc.communicate()
    text = out.decode(errors="replace")
    if len(text) > 20000:
        text = text[:2000] + "\n...(截断)...\n" + text[-16000:]
    return {"rc": proc.returncode, "timed_out": timed_out, "output": text}
