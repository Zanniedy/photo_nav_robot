"""YOLO26-seg 头部解码（纯 numpy），PC 上验证切图后的 ONNX 和板端 BPU 推理共用。

切图后模型输出 10 个张量（顺序见 export_bpu_onnx.py 生成的 *_outputs.json）：
    每个尺度 (stride 8/16/32): box (1,4,H,W)  cls (1,nc,H,W)  mc (1,32,H,W)
    proto (1,32,160,160)
box 是相对网格中心的 l,t,r,b 距离（单位=格，YOLO26 无 DFL），cls 未过 sigmoid。
YOLO26 是 one2one 头（NMS-free），按分数阈值 + topk 取结果即可，不需要 NMS。
"""
import numpy as np


def letterbox(img, size=640, color=114):
    """img: HxWx3 uint8。返回 (640x640x3, scale, (pad_x, pad_y))，和 ultralytics 一致（居中补边）"""
    h, w = img.shape[:2]
    r = min(size / w, size / h)
    nw, nh = int(round(w * r)), int(round(h * r))
    px, py = (size - nw) / 2, (size - nh) / 2
    out = np.full((size, size, 3), color, np.uint8)
    resized = _resize(img, nw, nh)
    top, left = int(round(py - 0.1)), int(round(px - 0.1))
    out[top:top + nh, left:left + nw] = resized
    return out, r, (left, top)


def _resize(img, w, h):
    try:
        import cv2
        return cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
    except (ImportError, AttributeError):   # 有的 conda 环境 cv2 和 numpy2 不兼容
        from PIL import Image
        return np.asarray(Image.fromarray(img).resize((w, h), Image.BILINEAR))


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def to_nchw(a, ch):
    """BPU 输出可能是 NHWC，统一成 (C,H,W)"""
    a = np.asarray(a)[0]
    return a if a.shape[0] == ch else a.transpose(2, 0, 1)


def decode(outputs, layout, conf=0.35, max_det=300, classes=None, with_masks=True):
    """
    outputs: 按 layout 顺序的输出数组列表
    layout:  [{"name","kind":box|cls|mc|proto,"stride","channels"}...]（export 生成的 json）
    返回 dets: (N,6) [x1,y1,x2,y2,score,cls]（640 输入坐标），masks: (N,160,160) float 或 None
    """
    boxes, scores, cids, coefs = [], [], [], []
    by, proto = {}, None
    for o, meta in zip(outputs, layout):
        a = to_nchw(o, meta['channels'])
        if meta['kind'] == 'proto':
            proto = a
        else:
            by.setdefault(meta['stride'], {})[meta['kind']] = a
    for stride in sorted(by):
        d = by[stride]
        cls = d['cls']
        nc, H, W = cls.shape
        box = d['box'].reshape(4, -1)
        cls = cls.reshape(nc, -1)
        if classes is not None:
            sub = cls[classes]
            ci = np.asarray(classes)[sub.argmax(0)]
            logit = sub.max(0)
        else:
            ci = cls.argmax(0)
            logit = cls.max(0)
        # 先在 logit 上筛，省掉大部分 sigmoid
        keep = logit > np.log(conf / (1 - conf))
        if not keep.any():
            continue
        idx = np.nonzero(keep)[0]
        gx = (idx % W) + 0.5
        gy = (idx // W) + 0.5
        l, t, r, b = box[:, idx]
        boxes.append(np.stack([gx - l, gy - t, gx + r, gy + b], 1) * stride)
        scores.append(sigmoid(logit[idx]))
        cids.append(ci[idx])
        if with_masks and 'mc' in d:
            coefs.append(d['mc'].reshape(32, -1)[:, idx].T)
    if not boxes:
        return np.zeros((0, 6), np.float32), None
    boxes = np.concatenate(boxes)
    scores = np.concatenate(scores)
    cids = np.concatenate(cids)
    order = np.argsort(-scores)[:max_det]
    dets = np.concatenate([boxes[order], scores[order, None], cids[order, None]], 1).astype(np.float32)
    masks = None
    if with_masks and coefs and proto is not None:
        c = np.concatenate(coefs)[order]
        masks = sigmoid(c @ proto.reshape(32, -1)).reshape(-1, proto.shape[1], proto.shape[2])
        masks = crop_masks(masks, dets[:, :4] * (proto.shape[1] / 640.0))
    return dets, masks


def crop_masks(masks, boxes):
    n, h, w = masks.shape
    xs = np.arange(w)[None, None, :]
    ys = np.arange(h)[None, :, None]
    x1, y1, x2, y2 = [boxes[:, i][:, None, None] for i in range(4)]
    return masks * ((xs >= x1) & (xs < x2) & (ys >= y1) & (ys < y2))


def scale_boxes(dets, r, pad):
    """640 输入坐标 → 原图坐标"""
    out = dets.copy()
    out[:, [0, 2]] = (out[:, [0, 2]] - pad[0]) / r
    out[:, [1, 3]] = (out[:, [1, 3]] - pad[1]) / r
    return out
