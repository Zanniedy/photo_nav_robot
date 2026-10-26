"""播放固定话语（web/voice/<名字>_<语言>.wav，本机 gen_voice.py 用 piper 预先合成）

只起一个播放子进程，不等它结束；上一句还在播就先停掉。播放器默认 paplay（走 PulseAudio，USB 声卡插上即可，
-d 指定 sink），也可以在 config.yaml 里换成 aplay -D plughw:CARD=...。
"""
import os, shlex, subprocess, threading
from dataclasses import dataclass, fields
from typing import Callable

VOICE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "voice")


@dataclass
class VoiceConfig:
    enabled: bool = True
    lang: str = "zh"                # zh / en，对应 <名字>_<语言>.wav
    player: str = "paplay"          # 播放命令，wav 路径加在最后
    device: str = ""                # 非空：paplay 加 -d，aplay 加 -D
    volume: int = 100               # 0~100：paplay 时播放前把 sink 音量设成这个（10-26 机器人上被调到 20%，几乎听不见）；-1 = 不动
    dir: str = VOICE_DIR

    @classmethod
    def from_dict(cls, d: dict) -> "VoiceConfig":
        conv = {"enabled": bool, "volume": int}
        out = {f.name: conv.get(f.name, str)(d[f.name]) for f in fields(cls) if f.name in (d or {})}
        if "dir" in out:
            out["dir"] = os.path.expanduser(out["dir"])
        return cls(**out)


class Voice:
    def __init__(self, cfg: VoiceConfig | None = None, log: Callable[[str], None] = lambda s: None,
                 popen=subprocess.Popen):
        self.cfg = cfg or VoiceConfig()
        self._log = log
        self._popen = popen
        self._proc = None
        self._lock = threading.Lock()

    def path(self, key: str) -> str:
        return os.path.join(self.cfg.dir, f"{key}_{self.cfg.lang}.wav")

    def command(self, key: str) -> list[str]:
        cmd = shlex.split(self.cfg.player)
        if self.cfg.device:
            cmd += ["-D" if os.path.basename(cmd[0]) == "aplay" else "-d", self.cfg.device]
        return cmd + [self.path(key)]

    def volume_command(self) -> list[str] | None:
        if self.cfg.volume < 0 or os.path.basename(shlex.split(self.cfg.player)[0]) != "paplay":
            return None
        return ["pactl", "set-sink-volume", self.cfg.device or "@DEFAULT_SINK@", f"{min(100, self.cfg.volume)}%"]

    def say(self, key: str) -> bool:
        """开始播放，返回是否真的起了播放进程"""
        if not self.cfg.enabled:
            return False
        p = self.path(key)
        if not os.path.isfile(p):
            self._log(f"语音文件不存在: {p}")
            return False
        env = dict(os.environ)
        # setsid nohup 从 ssh 起后端时可能没有 XDG_RUNTIME_DIR，paplay 找不到 PulseAudio
        env.setdefault("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")
        with self._lock:
            self._stop_locked()
            vc = self.volume_command()
            if vc:
                try:
                    subprocess.run(vc, env=env, capture_output=True, timeout=3)
                except (OSError, subprocess.TimeoutExpired) as e:
                    self._log(f"设置音量失败: {e}")
            try:
                self._proc = self._popen(self.command(key), env=env, stdin=subprocess.DEVNULL,
                                         stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            except OSError as e:
                self._log(f"语音播放失败: {e}")
                return False
        threading.Thread(target=self._watch, args=(self._proc,), daemon=True).start()
        return True

    def stop(self) -> None:
        with self._lock:
            self._stop_locked()

    def _stop_locked(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()
        self._proc = None

    def _watch(self, proc) -> None:
        _, err = proc.communicate()
        if proc.returncode not in (0, -15):             # -15 = 被下一句 terminate
            msg = (err or b"").decode(errors="replace").strip()[-200:]
            self._log(f"语音播放退出码 {proc.returncode}: {msg}")


if __name__ == "__main__":      # 试播：python3 voice.py [名字] [zh|en]，按 config.yaml 的 voice 配置
    import sys
    from config import VOICE
    v = Voice(VoiceConfig.from_dict({**VOICE, **({"lang": sys.argv[2]} if len(sys.argv) > 2 else {})}), log=print)
    key = sys.argv[1] if len(sys.argv) > 1 else "greet"
    print(" ".join(v.command(key)))
    if v.say(key):
        v._proc.wait()
