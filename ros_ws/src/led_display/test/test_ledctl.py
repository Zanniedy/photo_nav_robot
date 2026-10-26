"""协议纯函数单测（不需要屏和蓝牙）。期望值是 App 抓到的 / 真机验证过的字节。"""
import io

import pytest
from PIL import Image

from led_display.ledctl import HEIGHT, WIDTH, crc16, crc32c, frame, gif_chunks, prepare_gif, text_gif


def test_crc_reference_values():
    assert crc16(b"123456789") == bytes.fromhex("4b37")  # CRC16/MODBUS 0x4B37，大端发送
    assert crc32c(b"123456789") == 0xE3069283


def test_command_frames():
    assert frame(bytes([0, 1, 1, 30])).hex(" ") == "00 01 01 1e 7c d0"   # 亮度 30，真机验证过
    assert frame(bytes([0, 3, 0, 1, 1]), 16).hex(" ") == "00 03 00 01 01 14 e4" + " 00" * 9
    assert frame(bytes([1, 2, 1, 1])).hex(" ") == "01 02 01 01 48 60"     # 上传结束


def test_gif_chunks_header_and_crc():
    data = bytes(range(256)) * 4  # 1024 字节
    pkts = gif_chunks(data, 491)
    assert len(pkts) == 3
    assert [len(p) for p in pkts] == [491 + 13, 491 + 13, 1024 - 982 + 13]
    for i, p in enumerate(pkts):
        assert p[:3] == bytes([1, 8, 2])
        assert int.from_bytes(p[3:7], "big") == 1024
        assert int.from_bytes(p[7:9], "big") == i
        assert int.from_bytes(p[9:11], "big") == len(p) - 13
        assert p[-2:] == crc16(p[:-2])
    assert b"".join(p[11:-2] for p in pkts) == data


@pytest.mark.parametrize("fit", ["contain", "cover", "stretch"])
def test_prepare_gif_size(tmp_path, fit):
    src = tmp_path / "a.gif"
    frames = [Image.new("RGB", (192, 192), c) for c in ("red", "green", "blue")]
    frames[0].save(src, save_all=True, append_images=frames[1:], duration=80, loop=0)
    im = Image.open(io.BytesIO(prepare_gif(str(src), fit)))
    assert im.size == (WIDTH, HEIGHT)
    assert im.n_frames == 3


def test_text_gif_static_and_scroll():
    try:
        short = Image.open(io.BytesIO(text_gif("Hi")))
    except FileNotFoundError:
        pytest.skip("没有中文字体")
    assert short.size == (WIDTH, HEIGHT) and getattr(short, "n_frames", 1) == 1
    long = Image.open(io.BytesIO(text_gif("hello 你好")))
    assert long.size == (WIDTH, HEIGHT) and long.n_frames > 30
