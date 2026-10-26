import os
from voice import VOICE_DIR, Voice, VoiceConfig


class Proc:
    def __init__(self, cmd, **kw): self.cmd, self.env, self.rc, self.terminated = cmd, kw["env"], None, False
    def poll(self): return self.rc
    def terminate(self): self.terminated, self.rc = True, -15
    def communicate(self): return b"", b""
    @property
    def returncode(self): return self.rc


def make(tmp_path, **kw):
    (tmp_path / "greet_zh.wav").write_bytes(b"x")
    (tmp_path / "greet_en.wav").write_bytes(b"x")
    procs = []
    v = Voice(VoiceConfig(dir=str(tmp_path), **kw), popen=lambda cmd, **k: procs.append(Proc(cmd, **k)) or procs[-1])
    return v, procs


def test_command_by_lang_and_player(tmp_path):
    v, procs = make(tmp_path, lang="en", device="usb_sink")
    assert v.say("greet")
    assert procs[0].cmd == ["paplay", "-d", "usb_sink", str(tmp_path / "greet_en.wav")]
    assert "XDG_RUNTIME_DIR" in procs[0].env
    v.cfg.player, v.cfg.device = "aplay -q", "plughw:CARD=G1200"
    assert v.command("greet") == ["aplay", "-q", "-D", "plughw:CARD=G1200", str(tmp_path / "greet_en.wav")]


def test_new_phrase_stops_previous_and_missing_or_disabled(tmp_path):
    logs = []
    v, procs = make(tmp_path)
    v._log = logs.append
    v.say("greet"); v.say("greet")
    assert procs[0].terminated and len(procs) == 2
    assert not v.say("bye") and "不存在" in logs[-1]
    v.cfg.enabled = False
    assert not v.say("greet") and len(procs) == 2


def test_from_dict_and_repo_wavs():
    c = VoiceConfig.from_dict({"enabled": 0, "lang": "en", "dir": "~/x", "bogus": 1})
    assert c.enabled is False and c.lang == "en" and c.dir == os.path.expanduser("~/x")
    for lang in ("zh", "en"):
        assert os.path.isfile(os.path.join(VOICE_DIR, f"greet_{lang}.wav"))


def test_volume_command():
    v = Voice(VoiceConfig(device="usb_sink", volume=80))
    assert v.volume_command() == ["pactl", "set-sink-volume", "usb_sink", "80%"]
    v.cfg.volume = -1
    assert v.volume_command() is None
    v.cfg.volume, v.cfg.player = 100, "aplay -q"
    assert v.volume_command() is None
    assert VoiceConfig.from_dict({"volume": "60"}).volume == 60
