"""
机器人身份：robot.yaml（config.yaml 的 robot_file，默认 ~/max_robot/robot.yaml），每台机器人一份，不进仓库。
  robot_id: "max-001"            全局唯一，[a-z0-9-]，上云后 MQTT 账号、topic 都用它，配置后不能再改
  name: "一号机"                  显示名，随时可改
  api_tokens: [{name, token}]    /api/v1 的 Bearer token（平板、上层调用方）
文件不存在或坏了：robot_id = unset-<hostname>，configured = False，后端照常启动。
坏了（解析不了 / 不是字典 / robot_id 不合法）时 broken = True，所有修改都拒绝，免得把原来的 id 和 token 冲掉。
"""
import contextlib, copy, fcntl, hmac, os, re, secrets, socket
import yaml
from config import ROBOT_FILE

API_VERSION = "1.0"
MAX_TOKENS = 20
_ID_RE = re.compile(r"^[a-z0-9-]{1,32}$")
_TOKEN_NAME_RE = re.compile(r"^[A-Za-z0-9_\-]{1,32}$")
_cache: dict = {"key": None, "data": None}     # 按文件 mtime/size 缓存，status 每秒都要读 robot_id


class ConfigError(ValueError):
    def __init__(self, code: str, msg: str):
        super().__init__(msg)
        self.code = code


@contextlib.contextmanager
def _locked():
    os.makedirs(os.path.dirname(ROBOT_FILE) or ".", exist_ok=True)
    with open(ROBOT_FILE + ".lock", "w") as lf:
        fcntl.flock(lf, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lf, fcntl.LOCK_UN)


def _default() -> dict:
    host = re.sub(r"[^a-z0-9-]", "-", socket.gethostname().lower())[:24] or "robot"
    return {"robot_id": f"unset-{host}", "name": "", "configured": False, "api_tokens": [], "broken": False}


def _read() -> dict:
    try:
        st = os.stat(ROBOT_FILE)
    except FileNotFoundError:
        return _default()
    key = (st.st_mtime_ns, st.st_size)
    if _cache["key"] != key:
        try:
            with open(ROBOT_FILE) as f:
                raw = yaml.safe_load(f) or {}
            if not isinstance(raw, dict):
                raise ValueError
            rid = str(raw.get("robot_id") or "")
            if rid and not _ID_RE.match(rid):
                raise ValueError
        except (OSError, ValueError, yaml.YAMLError):
            # 文件坏了按没配置处理，不让后端起不来；但标上 broken，不许写回去
            _cache.update(key=key, data={**_default(), "broken": True})
            return copy.deepcopy(_cache["data"])
        d = _default()
        if rid:
            d.update(robot_id=rid, configured=True)
        d["name"] = str(raw.get("name") or "")
        d["api_tokens"] = [{"name": str(t["name"]), "token": str(t["token"])}
                           for t in raw.get("api_tokens") or []
                           if isinstance(t, dict) and t.get("name") and t.get("token")]
        _cache.update(key=key, data=d)
    return copy.deepcopy(_cache["data"])


def _check_writable(d: dict) -> None:
    if d["broken"]:
        raise ConfigError("INTERNAL", f"robot.yaml 格式不对，先 ssh 上去修好：{ROBOT_FILE}")


def _write(d: dict) -> None:
    out = {"name": d["name"], "api_tokens": d["api_tokens"]}
    if d["configured"]:
        out = {"robot_id": d["robot_id"], **out}
    tmp = ROBOT_FILE + ".tmp"
    # 一创建就是 0600，不留别人可读的空档
    with os.fdopen(os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600), "w") as f:
        yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)
    os.chmod(tmp, 0o600)                            # 里面有 token
    os.replace(tmp, ROBOT_FILE)
    _cache["key"] = None                            # 有些机器 mtime_ns 分辨率 ~1ms，紧挨着的两次写会同 key；写完一律让下次重读


def _public(d: dict) -> dict:
    return {"robot_id": d["robot_id"], "name": d["name"], "configured": d["configured"],
            "api_version": API_VERSION, "tokens": [t["name"] for t in d["api_tokens"]],
            "broken": d["broken"]}


def load() -> dict:
    with _locked():
        return _read()


def robot_id() -> str:
    return load()["robot_id"]


def public_info() -> dict:
    return _public(load())


def set_identity(robot_id: str | None = None, name: str | None = None) -> dict:
    with _locked():
        d = _read()
        _check_writable(d)
        if robot_id is not None and not (d["configured"] and robot_id == d["robot_id"]):
            if d["configured"]:
                raise ConfigError("ROBOT_NOT_CONFIGURED",
                                  "robot_id 已配置，不能再改（确实要改请 ssh 上去编辑 robot.yaml）")
            if not _ID_RE.match(robot_id):
                raise ConfigError("BAD_REQUEST", "robot_id 只能用小写字母、数字和 -，1~32 个字符")
            d.update(robot_id=robot_id, configured=True)
        if name is not None:
            name = name.strip()
            if len(name) > 64:
                raise ConfigError("BAD_REQUEST", "名字最多 64 个字符")
            d["name"] = name
        _write(d)
        return _public(d)


def create_token(name: str) -> str:
    """生成 token，只在这里返回一次明文"""
    if not _TOKEN_NAME_RE.match(name or ""):
        raise ConfigError("BAD_REQUEST", "token 名只能用字母、数字、_ 和 -，1~32 个字符")
    with _locked():
        d = _read()
        _check_writable(d)
        if any(t["name"] == name for t in d["api_tokens"]):
            raise ConfigError("BAD_REQUEST", f"token 名已存在: {name}")
        if len(d["api_tokens"]) >= MAX_TOKENS:
            raise ConfigError("BAD_REQUEST", f"token 最多 {MAX_TOKENS} 个")
        tok = secrets.token_urlsafe(32)
        d["api_tokens"].append({"name": name, "token": tok})
        _write(d)
        return tok


def delete_token(name: str) -> bool:
    with _locked():
        d = _read()
        _check_writable(d)
        n = len(d["api_tokens"])
        d["api_tokens"] = [t for t in d["api_tokens"] if t["name"] != name]
        if len(d["api_tokens"]) == n:
            return False
        _write(d)
        return True


def check_token(token: str) -> str | None:
    """token 对得上返回它的名字（日志里记是谁调的），否则 None"""
    for t in load()["api_tokens"]:
        if hmac.compare_digest(t["token"].encode(), token.encode()):
            return t["name"]
    return None
