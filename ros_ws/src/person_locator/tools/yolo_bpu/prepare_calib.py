#!/usr/bin/env python3
"""生成 hb_mapper 量化校准数据：图片 → letterbox 640 → RGB NCHW float32（0~255，不归一化）。

归一化（/255）写在 config 的 norm_type/scale_value 里，由工具链插进模型，所以这里不要除 255。
校准图要用机器人相机在实际场景拍的（有人、不同光照），50~100 张，比 COCO 图量化效果好。

    python prepare_calib.py --images ~/max_robot/calib_frames --out build/calib_data --max 100
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from export_bpu_onnx import load_rgb  # noqa: E402
from yolo_decode import letterbox  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--images', nargs='+', required=True, help='图片目录（可多个）')
    ap.add_argument('--out', required=True)
    ap.add_argument('--size', type=int, default=640)
    ap.add_argument('--max', type=int, default=100)
    args = ap.parse_args()

    files = []
    for d in args.images:
        for ext in ('*.jpg', '*.jpeg', '*.png', '*.bmp'):
            files += glob.glob(os.path.join(os.path.expanduser(d), '**', ext), recursive=True)
    files = sorted(set(files))
    if not files:
        sys.exit('没找到图片')
    if len(files) > args.max:   # 均匀抽样，避免连续帧扎堆
        files = [files[i] for i in np.linspace(0, len(files) - 1, args.max).astype(int)]
    if len(files) < 20:
        print(f'警告: 只有 {len(files)} 张，建议 50~100 张，量化精度可能偏差')

    os.makedirs(args.out, exist_ok=True)
    for f in glob.glob(os.path.join(args.out, '*.rgb')):
        os.remove(f)
    n = 0
    for fn in files:
        rgb = load_rgb(fn)
        if rgb is None:
            print(f'跳过 {fn}')
            continue
        lb, _, _ = letterbox(rgb, args.size)
        lb.transpose(2, 0, 1).astype(np.float32).tofile(os.path.join(args.out, f'calib_{n:04d}.rgb'))
        n += 1
    print(f'{n} 张校准数据 → {args.out}（float32 NCHW 1x3x{args.size}x{args.size}）')


if __name__ == '__main__':
    main()
