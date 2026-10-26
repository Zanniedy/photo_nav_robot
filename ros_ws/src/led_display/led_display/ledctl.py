"""LEDCanvas（iLEDCanvas）16x32 蓝牙 LED 软屏控制库。

协议从 LEDCanvas v1.6.3（com.led.ledcanvas）反编译得到：
  - 控制服务 C820：C821 写命令，C822 写大块数据，C823 notify 回复
  - AE00 是杰理 OTA（刷固件）通道，这里不碰
  - 每条命令 = 负载 + CRC16/MODBUS（初值 0xFFFF，多项式 0xA001），CRC 大端
  - 图片/动画直接发 32x16 的 GIF 文件，屏自己解码

不依赖 ROS，可以单独用：

    import asyncio
    from led_display.ledctl import LedPanel

    async def main():
        async with LedPanel() as led:
            await led.show_text("你好 hello")
    asyncio.run(main())
"""

import asyncio
import io
import os
import struct
import time

ADDRESS = "7D:39:31:D9:32:FE"
WIDTH, HEIGHT = 32, 16

CHAR_CMD = "0000c821-0000-1000-8000-00805f9b34fb"
CHAR_DATA = "0000c822-0000-1000-8000-00805f9b34fb"
CHAR_NOTIFY = "0000c823-0000-1000-8000-00805f9b34fb"

# 字体按顺序找；cusong16.ttf 是 LEDCanvas 自带的 16 点阵宋体，有版权，不进仓库，手动拷到第一个位置
FONT_CANDIDATES = [
    os.path.expanduser("~/max_robot/fonts/cusong16.ttf"),
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
]


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack(">H", crc)


def _crc32c_table():
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0x82F63B78 if c & 1 else c >> 1
        table.append(c)
    return table


_CRC32C = _crc32c_table()


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc = _CRC32C[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc ^ 0xFFFFFFFF


def frame(payload: bytes, pad_to: int = 0) -> bytes:
    """加 CRC16，按需补 0（App 里有几条命令固定补到 16 字节）。"""
    out = bytes(payload) + crc16(payload)
    return out.ljust(pad_to, b"\x00")


def gif_chunks(gif: bytes, chunk_size: int) -> list:
    """GIF 切成 C822 数据包：01 08 02 | 总长 u32 | 序号 u16 | 本包长 u16 | 数据 | crc16。"""
    packets = []
    for idx, off in enumerate(range(0, len(gif), chunk_size)):
        part = gif[off:off + chunk_size]
        head = bytes([1, 8, 2]) + struct.pack(">IHH", len(gif), idx, len(part))
        packets.append(frame(head + part))
    return packets


def find_font(font_path=None) -> str:
    for p in ([font_path] if font_path else []) + [os.environ.get("LED_FONT")] + FONT_CANDIDATES:
        if p and os.path.exists(p):
            return p
    raise FileNotFoundError("找不到中文字体，把 cusong16.ttf 放到 ~/max_robot/fonts/ 或设置 LED_FONT")


def prepare_gif(path: str, fit: str = "contain", max_frames: int = 60) -> bytes:
    """任意图片/GIF 转成屏能用的 32x16 GIF。

    fit: contain（等比缩放，留黑边）/ cover（等比裁剪铺满）/ stretch（拉伸）
    """
    from PIL import Image, ImageSequence

    src = Image.open(path)
    frames, durations = [], []
    for i, f in enumerate(ImageSequence.Iterator(src)):
        if i >= max_frames:
            break
        rgba = f.convert("RGBA")
        bg = Image.new("RGBA", rgba.size, (0, 0, 0, 255))
        img = Image.alpha_composite(bg, rgba).convert("RGB")
        if fit == "stretch":
            img = img.resize((WIDTH, HEIGHT), Image.LANCZOS)
        else:
            scale = (min if fit == "contain" else max)(WIDTH / img.width, HEIGHT / img.height)
            w, h = max(1, round(img.width * scale)), max(1, round(img.height * scale))
            img = img.resize((w, h), Image.LANCZOS)
            canvas = Image.new("RGB", (WIDTH, HEIGHT))
            canvas.paste(img, ((WIDTH - w) // 2, (HEIGHT - h) // 2))
            img = canvas
        frames.append(img.quantize(colors=64))
        durations.append(max(20, f.info.get("duration", src.info.get("duration", 100)) or 100))

    buf = io.BytesIO()
    frames[0].save(buf, format="GIF", save_all=True, append_images=frames[1:],
                   duration=durations, loop=0, disposal=1, optimize=False)
    return buf.getvalue()


def text_gif(text: str, color=(255, 0, 0), font_path=None, step: int = 1, frame_ms: int = 60) -> bytes:
    """文字渲染成 32x16 GIF；放不下就从右往左滚动。"""
    from PIL import Image, ImageDraw, ImageFont

    font = ImageFont.truetype(find_font(font_path), 16)
    left, top, right, bottom = font.getbbox(text)
    tw = right - left
    strip = Image.new("RGB", (max(tw, 1), HEIGHT))
    draw = ImageDraw.Draw(strip)
    draw.fontmode = "1"  # 不抗锯齿：16 像素高的屏上像素更清楚
    draw.text((-left, (HEIGHT - (bottom - top)) // 2 - top), text, font=font, fill=color)

    if tw <= WIDTH:
        img = Image.new("RGB", (WIDTH, HEIGHT))
        img.paste(strip, ((WIDTH - tw) // 2, 0))
        frames = [img]
    else:
        frames = []
        for x in range(WIDTH, -tw - 1, -step):
            img = Image.new("RGB", (WIDTH, HEIGHT))
            img.paste(strip, (x, 0))
            frames.append(img)

    frames = [f.quantize(colors=16) for f in frames]
    buf = io.BytesIO()
    frames[0].save(buf, format="GIF", save_all=True, append_images=frames[1:],
                   duration=frame_ms, loop=0, optimize=False)
    return buf.getvalue()



def eyes_gif(color=(0, 160, 255), open_ms: int = 2200) -> bytes:
    """待机用的眼睛动画（32x16）：睁眼 → 眨眼 → 左看 → 右看 → 眨眼，循环。"""
    from PIL import Image, ImageDraw

    def eyes(dx=0, h=10):
        img = Image.new("RGB", (WIDTH, HEIGHT))
        d = ImageDraw.Draw(img)
        cy = HEIGHT // 2
        for cx in (9, 22):                       # 两只眼的中心
            x = cx + dx
            if h <= 2:
                d.line([(x - 4, cy), (x + 4, cy)], fill=color)
            else:
                d.rounded_rectangle([x - 4, cy - h // 2, x + 4, cy + h // 2 - 1], radius=3, fill=color)
                px = x + (2 if dx > 0 else -2 if dx < 0 else 0)      # 瞳孔跟着看的方向偏
                d.rectangle([px - 1, cy - 1, px + 1, cy + 1], fill=(0, 0, 0))
        return img

    blink = [(eyes(h=6), 60), (eyes(h=2), 80), (eyes(h=6), 60)]
    seq = [(eyes(), open_ms), *blink, (eyes(), 700),
           (eyes(dx=-2), 900), (eyes(), 300), (eyes(dx=2), 900), (eyes(), 600), *blink]
    frames = [f.quantize(colors=8) for f, _ in seq]
    buf = io.BytesIO()
    frames[0].save(buf, format="GIF", save_all=True, append_images=frames[1:],
                   duration=[ms for _, ms in seq], loop=0, optimize=False)
    return buf.getvalue()


class LedPanel:
    """一块屏的 BLE 连接。屏同一时间只接受一个连接，手机 App 要先断开。"""

    def __init__(self, address: str = ADDRESS, verbose: bool = False, log=print):
        self.address = address
        self.verbose = verbose
        self.log = log
        self.client = None
        self._rx = asyncio.Queue()

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, *exc):
        await self.disconnect()

    @property
    def connected(self) -> bool:
        return self.client is not None and self.client.is_connected

    async def connect(self):
        from bleak import BleakClient, BleakScanner

        # 已经被 BlueZ 连着的屏不再广播，扫不到就直接按地址连
        dev = await BleakScanner.find_device_by_address(self.address, timeout=5)
        self.client = BleakClient(dev or self.address, timeout=20)
        await self.client.connect()
        # BlueZ 要先 acquire 才知道协商后的 MTU，否则 bleak 一直报 23（每包只能放 7 字节）
        acquire = getattr(self.client._backend, "_acquire_mtu", None)
        if acquire:
            try:
                await acquire()
            except Exception as e:
                self._log(f"读不到 MTU（{e}），用 {self.client.mtu_size}")
        await self.client.start_notify(CHAR_NOTIFY, self._on_notify)
        self._log(f"已连接 {self.address}，MTU={self.client.mtu_size}")

    async def disconnect(self):
        if self.connected:
            await self.client.disconnect()

    def _log(self, msg):
        if self.verbose:
            self.log(msg)

    def _on_notify(self, _sender, data):
        self._log(f"  <- {bytes(data).hex(' ')}")
        self._rx.put_nowait(bytes(data))

    async def _write(self, char: str, data: bytes, response: bool = True):
        self._log(f"  -> {char[4:8]} {data.hex(' ')}")
        await self.client.write_gatt_char(char, data, response=response)

    async def send(self, payload: bytes, pad_to: int = 0):
        await self._write(CHAR_CMD, frame(payload, pad_to))

    async def _wait(self, match, timeout: float = 5.0) -> bytes:
        """等一条满足 match 的回复，其他的跳过。"""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("屏没有回复")
            data = await asyncio.wait_for(self._rx.get(), remaining)
            if match(data):
                return data

    # ---- 简单命令 ----------------------------------------------------------

    async def brightness(self, level: int):
        """1..100"""
        await self.send(bytes([0, 1, 1, max(1, min(100, level))]))

    async def power(self, on: bool):
        await self.send(bytes([0, 3, 0, 1, 1 if on else 0]), pad_to=16)

    async def speed(self, value: int):
        """动画/滚动速度，App 里是 0..100。"""
        await self.send(bytes([0, 2, 0, 1, max(0, min(255, value))]), pad_to=16)

    async def rotate(self, flipped: bool):
        await self.send(bytes([81, 1, 1, 1 if flipped else 0]))

    async def clock(self, style: int = 0, h24: bool = True, show_date: bool = False):
        t = time.localtime()
        await self.send(bytes([14, 1, 10, style, int(h24), int(show_date)])
                        + struct.pack(">H", t.tm_year)
                        + bytes([t.tm_mon - 1, t.tm_mday - 1, t.tm_hour, t.tm_min, t.tm_sec]))

    # ---- GIF 上传 ----------------------------------------------------------

    async def send_gif(self, gif: bytes, slot: int = 0, timeout: float = 5.0):
        """上传 32x16 GIF 并显示。slot 0 = 立即显示（1..n 是播放列表位置）。"""
        while not self._rx.empty():
            self._rx.get_nowait()

        await self.send(bytes([8, 2, 5, slot]) + struct.pack(">I", crc32c(gif)))
        r = await self._wait(lambda d: len(d) >= 4 and d[0] in (8, 24) and d[1] == 2, timeout)
        if r[3] == 3:
            self._log("屏上已经是这个 GIF")
            return
        if r[3] != 1:
            raise RuntimeError(f"屏拒绝上传: {r.hex(' ')}")

        chunk = self.client.mtu_size - 26  # 同 App：getBleMtu() - 13，getBleMtu = MTU - 13
        chunk = chunk if chunk > 20 else 7
        packets = gif_chunks(gif, chunk)
        self._log(f"发送 {len(gif)} 字节，{len(packets)} 包，每包 <= {chunk}")
        for i, pkt in enumerate(packets):
            await self._write(CHAR_DATA, pkt, response=False)
            r = await self._wait(lambda d: len(d) >= 6 and d[0] == 1 and d[1] in (1, 2), timeout)
            if r[1] == 2:
                raise RuntimeError(f"第 {i} 包数据错误: {r.hex(' ')}")
            if r[5] != 1:
                raise RuntimeError(f"第 {i} 包没被接受: {r.hex(' ')}")

        await self.send(bytes([1, 2, 1, 1]))
        r = await self._wait(lambda d: len(d) >= 4 and d[0] == 1 and d[1] == 2, timeout)
        if r[3] != 1:
            raise RuntimeError(f"屏报校验错误: {r.hex(' ')}")

    async def show_image(self, path: str, fit: str = "contain"):
        await self.send_gif(prepare_gif(path, fit))

    async def show_text(self, text: str, color=(255, 0, 0), font_path=None):
        await self.send_gif(text_gif(text, color, font_path))
