"""人体检测：YOLO26-seg 转出来的 BPU .bin（tools/yolo_bpu），只解码 person 框，不算 mask。

解码和 tools/yolo_bpu/yolo_decode.py 一致（box 为 l,t,r,b 到格子中心的距离，单位格；cls 未过 sigmoid），
但只取 person 那一个通道，省掉 80 类的 argmax，板端解码从约 50ms 降到 1ms 量级。
"""
import json
import math

import numpy as np


def letterbox(img, size=640, color=114):
    """居中补边（和 ultralytics 一致）。返回 (size x size 图, 缩放 r, (pad_x, pad_y))"""
    import cv2
    h, w = img.shape[:2]
    r = min(size / w, size / h)
    nw, nh = int(round(w * r)), int(round(h * r))
    if (nw, nh) != (w, h):
        img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    left, top = int(round((size - nw) / 2 - 0.1)), int(round((size - nh) / 2 - 0.1))
    out = np.full((size, size, 3), color, np.uint8)
    out[top:top + nh, left:left + nw] = img
    return out, r, (left, top)


def bgr_to_nv12(bgr):
    import cv2
    h, w = bgr.shape[:2]
    yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).reshape(-1)
    y, u, v = yuv[:h * w], yuv[h * w:h * w * 5 // 4], yuv[h * w * 5 // 4:]
    uv = np.empty(h * w // 2, np.uint8)
    uv[0::2], uv[1::2] = u, v
    return np.concatenate([y, uv])


def _chw(a, ch, nhwc):
    a = np.asarray(a)[0]
    return a.transpose(2, 0, 1) if nhwc else a


def decode_person(outputs, layout, conf=0.4, cls_id=0, nhwc=False, max_det=50):
    """outputs: 按 layout 顺序的输出数组。返回 (N,5) [x1,y1,x2,y2,score]（640 输入坐标），按分数降序"""
    by = {}
    for o, meta in zip(outputs, layout):
        if meta['kind'] in ('box', 'cls'):
            by.setdefault(meta['stride'], {})[meta['kind']] = (o, meta['channels'])
    thr = math.log(conf / (1 - conf))
    boxes, scores = [], []
    for stride in sorted(by):
        (cls, nc), (box, _) = by[stride]['cls'], by[stride]['box']
        logit = _chw(cls, nc, nhwc)[cls_id]
        H, W = logit.shape
        idx = np.flatnonzero(logit > thr)
        if not len(idx):
            continue
        b = _chw(box, 4, nhwc).reshape(4, -1)[:, idx]
        gx, gy = idx % W + 0.5, idx // W + 0.5
        boxes.append(np.stack([gx - b[0], gy - b[1], gx + b[2], gy + b[3]], 1) * stride)
        scores.append(1 / (1 + np.exp(-logit.reshape(-1)[idx])))
    if not boxes:
        return np.zeros((0, 5), np.float32)
    boxes, scores = np.concatenate(boxes), np.concatenate(scores)
    order = np.argsort(-scores)[:max_det]
    dets = np.concatenate([boxes[order], scores[order, None]], 1).astype(np.float32)
    return dedup(dets)


def dedup(dets, iou_thr=0.7):
    """one2one 头本身不出重复框，这里只兜底去掉高度重叠的（量化后偶尔会有）"""
    keep = []
    for d in dets:
        if all(_iou(d, k) < iou_thr for k in keep):
            keep.append(d)
    return np.asarray(keep, np.float32).reshape(-1, 5)


def _iou(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    i = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i + 1e-9)


class BpuPersonDetector:
    """板端 hobot_dnn 推理。输入 BGR 图（任意尺寸），输出原图坐标的 person 框 (N,5)"""

    def __init__(self, model_path, layout_path=None, conf=0.4):
        from hobot_dnn import pyeasy_dnn as dnn
        if layout_path is None:
            layout_path = model_path.rsplit('.', 1)[0] + '_outputs.json'
        with open(layout_path) as f:
            self.layout = json.load(f)['outputs']
        self.model = dnn.load(model_path)[0]
        self.conf = conf
        names = [o.name for o in self.model.outputs]
        if names != [m['name'] for m in self.layout]:
            raise RuntimeError(f'.bin 输出 {names} 和 {layout_path} 不一致')
        box0 = next(i for i, m in enumerate(self.layout) if m['kind'] == 'box')
        shape = tuple(self.model.outputs[box0].properties.shape)
        self.nhwc = shape[-1] == 4 and shape[1] != 4

    def __call__(self, bgr):
        lb, r, (px, py) = letterbox(bgr, 640)
        outs = self.model.forward(bgr_to_nv12(lb))
        dets = decode_person([o.buffer for o in outs], self.layout, self.conf, nhwc=self.nhwc)
        dets[:, [0, 2]] = (dets[:, [0, 2]] - px) / r
        dets[:, [1, 3]] = (dets[:, [1, 3]] - py) / r
        return dets
