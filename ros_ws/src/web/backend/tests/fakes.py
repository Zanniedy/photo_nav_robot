"""测试替身：临时地图文件（之后加 FakeNav）"""
import os
from conftest import TMP


def make_map(name: str, pixels: bytes = b"\x00" * 16, subdir: str = "") -> str:
    """在临时 maps 目录写一张 4x4 的 pgm + yaml，返回 yaml 路径"""
    d = os.path.join(TMP, "maps", subdir)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, f"{name}.pgm"), "wb") as f:
        f.write(b"P5 4 4 255\n" + pixels)
    y = os.path.join(d, f"{name}.yaml")
    with open(y, "w") as f:
        f.write(f"image: {name}.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return y
