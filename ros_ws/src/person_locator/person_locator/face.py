"""人脸检测：OpenCV YuNet（cv2.FaceDetectorYN，CPU），只在人体框的上半部分里找，不跑整图。

模型 face_detection_yunet_2023mar.onnx（opencv_zoo，要 OpenCV >= 4.8）。
整张 640x360 在板子上要 80~90ms，裁成人体框上部一小块后只要几 ms。

YuNet 每个人脸一行 15 个数：x, y, w, h, 右眼 xy, 左眼 xy, 鼻尖 xy, 右嘴角 xy, 左嘴角 xy, score
（"右眼"是图像左边那只，即人的右眼）。
"""
import numpy as np


def head_region(box, img_w, img_h, top=0.45, pad=0.15):
    """人体框 → 头部搜索区域 (x1, y1, x2, y2) 整数像素。取框的上 top 部分，左右各放宽 pad"""
    x1, y1, x2, y2 = box[:4]
    w, h = x2 - x1, y2 - y1
    rx1, rx2 = x1 - w * pad, x2 + w * pad
    ry1, ry2 = y1 - h * 0.05, y1 + h * top
    rx1, ry1 = max(0, int(rx1)), max(0, int(ry1))
    rx2, ry2 = min(img_w, int(np.ceil(rx2))), min(img_h, int(np.ceil(ry2)))
    return rx1, ry1, rx2, ry2


def frontal_ratio(face):
    """鼻尖在两眼之间的水平位置：0.5 = 正脸，靠近 0 或 1 = 侧脸。两眼重合时返回 None"""
    rex, lex, nx = face[4], face[6], face[8]
    span = lex - rex
    if abs(span) < 1e-3:
        return None
    return float((nx - rex) / span)


def is_frontal(face, tol=0.25):
    r = frontal_ratio(face)
    return r is not None and abs(r - 0.5) <= tol


class FaceDetector:
    def __init__(self, model_path, conf=0.6, min_side=128, max_scale=3.0):
        import cv2
        self.cv2 = cv2
        self.net = cv2.FaceDetectorYN.create(model_path, '', (320, 320), conf, 0.3, 50)
        self.min_side = min_side        # 区域太小时放大到短边这么多（远处的人脸只有十几个像素）
        self.max_scale = max_scale
        self._size = None

    def detect_region(self, bgr, region):
        """在 region 里找脸，返回分数最高的一个（整图坐标，15 维）或 None"""
        x1, y1, x2, y2 = region
        crop = bgr[y1:y2, x1:x2]
        h, w = crop.shape[:2]
        if w < 8 or h < 8:
            return None
        s = min(self.max_scale, max(1.0, self.min_side / min(w, h)))
        if s > 1.0:
            crop = self.cv2.resize(crop, (int(w * s), int(h * s)), interpolation=self.cv2.INTER_LINEAR)
        size = (crop.shape[1], crop.shape[0])
        if size != self._size:
            self.net.setInputSize(size)
            self._size = size
        _, faces = self.net.detect(crop)
        if faces is None or not len(faces):
            return None
        f = faces[int(np.argmax(faces[:, -1]))].astype(np.float64).copy()
        f[:14] /= s
        f[[0, 4, 6, 8, 10, 12]] += x1      # 2、3 是 w、h，不加偏移
        f[[1, 5, 7, 9, 11, 13]] += y1
        return f

    def __call__(self, bgr, person_boxes):
        """每个人体框一个结果：15 维人脸数组或 None"""
        h, w = bgr.shape[:2]
        return [self.detect_region(bgr, head_region(b, w, h)) for b in person_boxes]
