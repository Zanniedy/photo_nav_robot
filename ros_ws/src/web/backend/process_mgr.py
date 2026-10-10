import subprocess, threading, collections, os
from config import ROS2_SETUP, WORKSPACE_SETUP, LOG_BUFFER_SIZE, ROS_DOMAIN_ID

_procs: dict[str, subprocess.Popen] = {}
_logs: dict[str, collections.deque] = {}
_lock = threading.Lock()


def _source_cmd(cmd: str) -> str:
    parts = [f"source {ROS2_SETUP}"]
    if os.path.exists(WORKSPACE_SETUP):
        parts.append(f"source {WORKSPACE_SETUP}")
    parts.append(cmd)
    return " && ".join(parts)


def _reader(name: str, stream):
    for line in iter(stream.readline, b""):
        text = line.decode(errors="replace").rstrip()
        with _lock:
            _logs[name].append(text)
    stream.close()


def start_process(name: str, cmd: str) -> None:
    stop_process(name)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)
    full_cmd = _source_cmd(cmd)
    proc = subprocess.Popen(
        ["bash", "-c", full_cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=env,
    )
    with _lock:
        _procs[name] = proc
        _logs[name] = collections.deque(maxlen=LOG_BUFFER_SIZE)
    t = threading.Thread(target=_reader, args=(name, proc.stdout), daemon=True)
    t.start()


def stop_process(name: str) -> None:
    with _lock:
        proc = _procs.pop(name, None)
    if proc is None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()


def is_running(name: str) -> bool:
    with _lock:
        proc = _procs.get(name)
    return proc is not None and proc.poll() is None


def get_logs(name: str | None = None) -> list[str]:
    with _lock:
        if name:
            return list(_logs.get(name, []))
        merged = []
        for n, dq in _logs.items():
            for line in dq:
                merged.append(f"[{n}] {line}")
        return merged[-LOG_BUFFER_SIZE:]
