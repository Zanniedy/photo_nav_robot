"""
登录：用户名/密码换一个 HMAC 签名的 cookie（无状态，密钥每次启动随机 → 后端重启需重新登录）。
"""
import hashlib, hmac, secrets, time
from fastapi import APIRouter, HTTPException, Request, Response
from pydantic import BaseModel
from config import AUTH_USER, AUTH_PASSWORD, SESSION_HOURS

COOKIE = "robot_ui_session"
ENABLED = bool(AUTH_PASSWORD)
_secret = secrets.token_bytes(32)
_fails: dict[str, list[float]] = {}   # ip -> 最近失败时间，简单防爆破

router = APIRouter(prefix="/api/auth")


def _sign(payload: str) -> str:
    return hmac.new(_secret, payload.encode(), hashlib.sha256).hexdigest()


def make_token(user: str) -> str:
    exp = int(time.time() + SESSION_HOURS * 3600)
    payload = f"{user}|{exp}"
    return f"{payload}|{_sign(payload)}"


def check_token(token: str | None) -> bool:
    if not ENABLED:
        return True
    if not token:
        return False
    try:
        user, exp, sig = token.rsplit("|", 2)
    except ValueError:
        return False
    return hmac.compare_digest(sig, _sign(f"{user}|{exp}")) and int(exp) > time.time()


class LoginReq(BaseModel):
    username: str
    password: str


@router.get("/me")
def me(request: Request):
    return {"enabled": ENABLED, "ok": check_token(request.cookies.get(COOKIE)), "user": AUTH_USER}


@router.post("/login")
def login(req: LoginReq, request: Request, response: Response):
    ip = request.client.host if request.client else "?"
    now = time.time()
    recent = [t for t in _fails.get(ip, []) if now - t < 60]
    if len(recent) >= 5:
        raise HTTPException(429, "尝试次数过多，1 分钟后再试")
    ok = hmac.compare_digest(req.username.encode(), AUTH_USER.encode()) & \
        hmac.compare_digest(req.password.encode(), AUTH_PASSWORD.encode())
    if not ok:
        _fails[ip] = recent + [now]
        raise HTTPException(401, "用户名或密码错误")
    _fails.pop(ip, None)
    response.set_cookie(COOKIE, make_token(req.username), httponly=True, samesite="strict",
                        max_age=int(SESSION_HOURS * 3600))
    return {"ok": True}


@router.post("/logout")
def logout(response: Response):
    response.delete_cookie(COOKIE)
    return {"ok": True}
