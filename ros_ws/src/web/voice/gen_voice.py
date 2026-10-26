"""用 piper 把 phrases.yaml 里的固定话语合成 wav（本机跑，机器人上只播放）

第一次用：
  /usr/bin/python3 -m venv <仓库根>/.venv_piper
  <仓库根>/.venv_piper/bin/pip install piper-tts==1.8.0 pyyaml unicode-rbnf sentence-stream transformers
  音色放 ~/max_robot/piper_voices/（hf-mirror.com/rhasspy/piper-voices 下 <音色>.onnx + .onnx.json）
  中文拼音音色还要 g2pW：hf-mirror.com/datasets/rhasspy/piper-checkpoints/resolve/main/zh/zh_CN/_resources/g2pw.tar.gz
    解压到 ~/max_robot/piper_voices/g2pW/；它的分词器 bert-base-chinese 第一次会从 HF 下，脚本默认走 hf-mirror
用法：<仓库根>/.venv_piper/bin/python gen_voice.py [--voices DIR] [--play 声卡]
"""
import argparse, os, subprocess, wave
import yaml

os.environ.setdefault("HF_ENDPOINT", "https://hf-mirror.com")     # g2pW 的 BertTokenizer
for _k in ("ALL_PROXY", "all_proxy"):                             # httpx 不认 socks://，本机代理会让分词器下载报错
    if os.environ.get(_k, "").startswith("socks://"):
        del os.environ[_k]
from piper import PiperVoice, SynthesisConfig  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--voices", default=os.path.expanduser("~/max_robot/piper_voices"))
    ap.add_argument("--play", metavar="SINK", nargs="?", const="", help="合成完用 paplay 播一遍，可指定 sink")
    a = ap.parse_args()
    with open(os.path.join(HERE, "phrases.yaml")) as f:
        cfg = yaml.safe_load(f)
    for lang, v in cfg["voices"].items():
        voice = PiperVoice.load(os.path.join(a.voices, f"{v['name']}.onnx"), download_dir=a.voices)
        syn = SynthesisConfig(length_scale=float(v.get("length_scale", 1.0)))
        for key, texts in cfg["phrases"].items():
            if lang not in texts:
                continue
            out = os.path.join(HERE, f"{key}_{lang}.wav")
            with wave.open(out, "wb") as wf:
                voice.synthesize_wav(texts[lang], wf, syn_config=syn)
            with wave.open(out) as wf:
                print(f"{out}  {wf.getnframes() / wf.getframerate():.1f}s  {texts[lang]}")
            if a.play is not None:
                subprocess.run(["paplay"] + (["-d", a.play] if a.play else []) + [out], check=False)


if __name__ == "__main__":
    main()
