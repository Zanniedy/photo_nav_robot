#!/usr/bin/env python3
"""LED 软屏命令行（不需要 ROS）。

    ros2 run led_display led_cli brightness 50
    ros2 run led_display led_cli power off
    ros2 run led_display led_cli gif ~/Downloads/preview.gif --fit cover
    ros2 run led_display led_cli text "你好 Hello" --color 0,255,0
    ros2 run led_display led_cli clock
    ros2 run led_display led_cli eyes --color 0,160,255
"""
import argparse
import asyncio

from led_display.ledctl import ADDRESS, LedPanel, eyes_gif


async def run(a):
    async with LedPanel(a.addr, verbose=a.verbose) as led:
        if a.cmd == "brightness":
            await led.brightness(a.level)
        elif a.cmd == "power":
            await led.power(a.state == "on")
        elif a.cmd == "speed":
            await led.speed(a.value)
        elif a.cmd == "rotate":
            await led.rotate(a.state == "on")
        elif a.cmd == "gif":
            await led.show_image(a.path, a.fit)
        elif a.cmd == "text":
            await led.show_text(a.text, tuple(int(x) for x in a.color.split(",")), a.font)
        elif a.cmd == "clock":
            await led.clock(a.style, not a.h12, a.date)
        elif a.cmd == "eyes":
            await led.send_gif(eyes_gif(tuple(int(x) for x in a.color.split(","))))
    print("ok")


def main():
    p = argparse.ArgumentParser(description="LEDCanvas 16x32 蓝牙 LED 软屏")
    p.add_argument("--addr", default=ADDRESS)
    p.add_argument("-v", "--verbose", action="store_true", help="打印收发的字节")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("brightness").add_argument("level", type=int, help="1..100")
    sub.add_parser("power").add_argument("state", choices=["on", "off"])
    sub.add_parser("speed").add_argument("value", type=int)
    sub.add_parser("rotate").add_argument("state", choices=["on", "off"])
    g = sub.add_parser("gif")
    g.add_argument("path")
    g.add_argument("--fit", choices=["contain", "cover", "stretch"], default="contain")
    t = sub.add_parser("text")
    t.add_argument("text")
    t.add_argument("--color", default="255,0,0", help="R,G,B")
    t.add_argument("--font", default=None)
    c = sub.add_parser("clock")
    c.add_argument("--style", type=int, default=0)
    c.add_argument("--12h", dest="h12", action="store_true")
    c.add_argument("--date", action="store_true")
    sub.add_parser("eyes", help="待机眼睛动画").add_argument("--color", default="0,160,255", help="R,G,B")
    asyncio.run(run(p.parse_args()))


if __name__ == "__main__":
    main()
