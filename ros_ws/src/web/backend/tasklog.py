"""任务日志：任务层（task_runner）和打招呼（greeter）的事件转成一行行中文，单独存，网页「任务」页签看。
不和进程日志混在一起，不会被 ROS 节点的输出刷掉。"""
import collections, threading, time

_lock = threading.Lock()
_buf: collections.deque = collections.deque(maxlen=1500)    # (seq, 时间戳, 类别, 文字)；tick 是每 3s 的状态行
_seq = 0

_STATE = {"running": "执行中", "waiting": "等待", "succeeded": "完成", "failed": "失败",
          "canceled": "已取消", "preempted": "被顶掉"}
_REASON = {"manual": "手动", "person": "遇到人", "subtask": "子任务", "lost": "定位丢失", "nav": "导航没准备好，稍后重试"}
_GREET = {"search": "搜索", "greet": "打招呼", "idle": "空闲"}


def add(kind: str, text: str) -> None:
    global _seq
    with _lock:
        _seq += 1
        _buf.append((_seq, time.time(), kind, text))


def since(seq: int) -> list[dict]:
    with _lock:
        return [{"seq": s, "t": t, "kind": k, "text": x} for s, t, k, x in _buf if s > seq]


def _task(t: dict) -> str:
    name = {"goto": "前往", "guide": "引导", "patrol": "巡逻"}.get(t.get("type"), t.get("type"))
    pts = t.get("points") or []
    pos = f" {t['index'] + 1}/{len(pts)}" if pts and t.get("state") in ("running", "waiting") else ""
    tgt = f" → {t['target']}" if t.get("target") else ""
    return f"{name} {t.get('id', '')}{pos}{tgt}"


def format_event(ev: dict) -> tuple[str, str] | None:
    """事件 → (类别, 文字)；progress 这类太频繁的返回 None"""
    typ, t = ev.get("type", ""), ev.get("task") or {}
    if typ == "task_state":
        msg = f"：{t['message']}" if t.get("message") else ""
        return "task", f"{_task(t)} {_STATE.get(t.get('state'), t.get('state'))}{msg}"
    if typ == "progress":
        # 每走完一个点发一次：message 是「已到达 X」或「去 X 失败，跳过（原因）」
        msg = t.get("message") or ""
        if msg.startswith("已到达") or "跳过" in msg:
            return "task", f"{_task(t)}：{msg}"
        return None
    if typ == "waiting":
        return "task", f"{_task(t)} 暂停（{_REASON.get(ev.get('reason'), ev.get('reason'))}）"
    if typ == "resumed":
        return "task", f"{_task(t)} 继续"
    if typ == "wait_timeout":
        return "task", f"{_task(t)} 等待超时，自动继续"
    if typ == "person_found":
        return "task", f"{_task(t)} 发现人 {ev.get('distance')}m"
    if typ == "estop":
        return "estop", "软件急停" if ev.get("on") else "解除急停"
    if typ.startswith("greet_"):
        st = typ[len("greet_"):]
        return "greet", f"[{_GREET.get(st, st)}] {ev.get('message', '')}"
    return None


def on_event(ev: dict) -> None:
    r = format_event(ev)
    if r is not None:
        add(*r)
