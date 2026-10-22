#!/usr/bin/env python3
"""板端测试转换好的 .bin：读图（或相机一帧）→ nv12 → BPU → 解码，打印结果和耗时。

    python3 bpu_infer.py --model ~/max_robot/models/yolo26s_seg_bpu.bin --image test.jpg --save out.jpg
    python3 bpu_infer.py --model ... --camera /dev/v4l/by-id/usb-AX_USB2.0_AX_USB2.0-video-index0
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
from hobot_dnn import pyeasy_dnn as dnn

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yolo_decode import decode, letterbox, scale_boxes  # noqa: E402


def bgr_to_nv12(bgr):
    h, w = bgr.shape[:2]
    yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).reshape(-1)
    y, u, v = yuv[:h * w], yuv[h * w:h * w * 5 // 4], yuv[h * w * 5 // 4:]
    uv = np.empty(h * w // 2, np.uint8)
    uv[0::2], uv[1::2] = u, v
    return np.concatenate([y, uv])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', required=True)
    ap.add_argument('--layout', help='*_outputs.json，默认和 .bin 同目录同名')
    ap.add_argument('--image')
    ap.add_argument('--camera')
    ap.add_argument('--conf', type=float, default=0.35)
    ap.add_argument('--person-only', action='store_true')
    ap.add_argument('--save')
    ap.add_argument('--runs', type=int, default=20, help='测速次数')
    args = ap.parse_args()

    layout_path = args.layout or os.path.splitext(args.model)[0] + '_outputs.json'
    layout = json.load(open(layout_path))['outputs']
    model = dnn.load(args.model)[0]
    for i, o in enumerate(model.outputs):
        print(f'输出 {i}: {o.name} {o.properties.shape} {o.properties.dtype}')
    if [o.name for o in model.outputs] != [l['name'] for l in layout]:
        print('警告: .bin 的输出名/顺序和 layout json 不一致，解码按 json 顺序')

    if args.camera:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        for _ in range(5):
            ok, bgr = cap.read()
        cap.release()
        if not ok:
            sys.exit('相机读帧失败')
    else:
        bgr = cv2.imread(args.image)
        if bgr is None:
            sys.exit(f'读不了 {args.image}')

    t0 = time.time()
    lb, r, pad = letterbox(bgr, 640)        # letterbox 不分通道顺序，BGR 进 BGR 出
    nv12 = bgr_to_nv12(lb)
    t1 = time.time()
    outs = model.forward(nv12)
    t2 = time.time()
    arrs = [o.buffer for o in outs]
    dets, masks = decode(arrs, layout, conf=args.conf, classes=[0] if args.person_only else None)
    t3 = time.time()
    infer = []
    for _ in range(args.runs):
        ts = time.time()
        model.forward(nv12)
        infer.append(time.time() - ts)
    print(f'预处理 {(t1 - t0) * 1e3:.1f}ms  BPU {np.median(infer) * 1e3:.1f}ms (中位数/{args.runs}次)  '
          f'解码 {(t3 - t2) * 1e3:.1f}ms')

    dets = scale_boxes(dets, r, pad)
    print(f'{len(dets)} 个目标')
    for d in dets:
        print(f'  cls={int(d[5]):2d} score={d[4]:.3f} box=({d[0]:.0f},{d[1]:.0f},{d[2]:.0f},{d[3]:.0f})')
    if args.save:
        vis = bgr.copy()
        for d in dets:
            x1, y1, x2, y2 = d[:4].astype(int)
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(vis, f'{int(d[5])}:{d[4]:.2f}', (x1, max(y1 - 4, 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imwrite(args.save, vis)
        print(f'已保存 {args.save}')


if __name__ == '__main__':
    main()
