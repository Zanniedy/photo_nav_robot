#!/usr/bin/env python3
"""单目相机棋盘格标定，输出 ROS camera_info 格式 YAML。

在线采集（需要显示器，相机可以直接插 PC 上标，内参和主机无关）:
    ros2 run person_locator calibrate --device /dev/video2 --out camera_info.yaml
    按键: SPACE 采一帧  a 切换自动采集  c 立即标定  q 退出

离线（板端无显示器时先存图，再拷回 PC 标）:
    ros2 run person_locator calibrate --images ./calib_imgs --out camera_info.yaml

检查标定结果（左原图 / 右去畸变，看直线是否变直）:
    ros2 run person_locator calibrate --device /dev/video2 --check camera_info.yaml
"""
import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np
import yaml

SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
DETECT_MAX_W = 640  # 先在缩小图上找角点，找到再回原图亚像素细化


def open_camera(device, width, height, fps):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        sys.exit(f'无法打开摄像头: {device}')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'Camera {device}: {w}x{h} @ {cap.get(cv2.CAP_PROP_FPS):.0f}fps')
    if (w, h) != (width, height):
        print(f'警告: 请求 {width}x{height}，实际 {w}x{h}。标定结果只对实际分辨率有效')
    return cap


def find_corners(gray, board):
    """返回原图坐标下的亚像素角点 (N,1,2)，找不到返回 None。"""
    scale = min(1.0, DETECT_MAX_W / gray.shape[1])
    small = cv2.resize(gray, None, fx=scale, fy=scale) if scale < 1.0 else gray
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(small, board, flags=flags)
    if not found:
        return None
    corners = corners / scale
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA)
    return corners


def board_pose_signature(corners, img_size):
    """用于判断两帧姿态是否"足够不同"：归一化中心、尺寸、倾斜。"""
    pts = corners.reshape(-1, 2)
    w, h = img_size
    hull = cv2.convexHull(pts.astype(np.float32))
    area = cv2.contourArea(hull) / (w * h)
    cx, cy = pts.mean(axis=0) / (w, h)
    # 包围盒面积与凸包面积之比反映透视倾斜程度
    box = np.ptp(pts[:, 0]) * np.ptp(pts[:, 1]) / (w * h)
    skew = area / box if box > 0 else 1.0
    return np.array([cx, cy, np.sqrt(area), skew])


def is_new_pose(sig, sigs, thresh=0.08):
    return all(np.linalg.norm(sig - s) > thresh for s in sigs)


def object_points(board, square):
    cols, rows = board
    obj = np.zeros((rows * cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return obj


def calibrate(img_pts, board, square, img_size, flags=0):
    obj = object_points(board, square)
    keep = list(range(len(img_pts)))
    for it in range(3):
        objs = [obj] * len(keep)
        imgs = [img_pts[i] for i in keep]
        rms, K, D, rvecs, tvecs = cv2.calibrateCamera(objs, imgs, img_size, None, None, flags=flags)
        errs = []
        for o, p, r, t in zip(objs, imgs, rvecs, tvecs):
            proj, _ = cv2.projectPoints(o, r, t, K, D)
            errs.append(np.sqrt(np.mean(np.sum((proj - p) ** 2, axis=2))))
        errs = np.array(errs)
        med = np.median(errs)
        bad = [k for k, e in zip(keep, errs) if e > max(3 * med, 1.0)]
        print(f'[第{it + 1}轮] {len(keep)} 帧 RMS={rms:.4f}px  单帧误差 中位数={med:.3f} 最大={errs.max():.3f}')
        if not bad or len(keep) - len(bad) < 10:
            break
        print(f'  剔除误差过大的帧: {bad}')
        keep = [k for k in keep if k not in bad]
    return rms, K, D, keep, rvecs, tvecs


def check_pose_diversity(rvecs, tvecs):
    """正对着拍的帧焦距和距离耦合，不可观；需要足够多倾斜帧和不同距离。"""
    tilts, dists = [], []
    for r, t in zip(rvecs, tvecs):
        R, _ = cv2.Rodrigues(r)
        tilts.append(np.degrees(np.arccos(abs(R[2, 2]))))  # 板法向与光轴夹角
        dists.append(float(np.linalg.norm(t)))
    tilts, dists = np.array(tilts), np.array(dists)
    n_tilt = int(np.sum(tilts > 20))
    msgs = [f'姿态分布: 倾斜角 {tilts.min():.0f}~{tilts.max():.0f}° (>20° 的 {n_tilt} 帧)  '
            f'距离 {dists.min():.2f}~{dists.max():.2f}m']
    ok = True
    if n_tilt < 6:
        msgs.append('警告: 倾斜帧太少（需要 >=6 帧倾斜 20° 以上），焦距 fx/fy 不可信')
        ok = False
    if dists.max() / dists.min() < 1.5:
        msgs.append('警告: 远近变化太小（最远/最近 < 1.5），焦距和畸变不可信')
        ok = False
    print('\n'.join(msgs))
    return ok, msgs


def write_camera_info(path, name, img_size, K, D, rms, n_frames):
    w, h = img_size
    # alpha=0：去畸变后只保留全部有效像素，和 ROS cameracalibrator 默认一致
    P3, _ = cv2.getOptimalNewCameraMatrix(K, D, img_size, 0.0)
    P = np.hstack([P3, np.zeros((3, 1))])
    data = {
        'image_width': w,
        'image_height': h,
        'camera_name': name,
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': K.flatten().tolist()},
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {'rows': 1, 'cols': 5, 'data': D.flatten()[:5].tolist()},
        'rectification_matrix': {'rows': 3, 'cols': 3, 'data': np.eye(3).flatten().tolist()},
        'projection_matrix': {'rows': 3, 'cols': 4, 'data': P.flatten().tolist()},
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        f.write(f'# 标定于 {time.strftime("%Y-%m-%d %H:%M")}，{n_frames} 帧，RMS 重投影误差 {rms:.4f}px\n')
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)
    print(f'已写入 {path}')


def load_camera_info(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    K = np.array(d['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(d['distortion_coefficients']['data'], dtype=np.float64)
    return K, D, (d['image_width'], d['image_height'])


def report(K, D, img_size):
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    w, h = img_size
    print(f'fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}')
    print('D =', np.array2string(D.flatten(), precision=5))
    print(f'水平视场 {np.degrees(2 * np.arctan(w / (2 * fx))):.1f}°  '
          f'垂直视场 {np.degrees(2 * np.arctan(h / (2 * fy))):.1f}°')
    if abs(cx - w / 2) > 0.1 * w or abs(cy - h / 2) > 0.1 * h:
        print('警告: 主点偏离图像中心超过 10%，通常是采集姿态不够多（缺倾斜和边角）')


def run_check(args):
    K, D, size = load_camera_info(args.check)
    report(K, D, size)
    cap = open_camera(args.device, size[0], size[1], args.fps)
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, size, 0.0)
    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, newK, size, cv2.CV_16SC2)
    print('q 退出')
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        und = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
        view = np.hstack([frame, und])
        for y in range(0, view.shape[0], view.shape[0] // 8):
            cv2.line(view, (0, y), (view.shape[1], y), (0, 255, 255), 1)
        cv2.imshow('raw | undistorted', cv2.resize(view, None, fx=0.5, fy=0.5))
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            break
    cap.release()
    cv2.destroyAllWindows()


def run_offline(args, board):
    files = sorted(sum((glob.glob(os.path.join(args.images, e)) for e in ('*.png', '*.jpg', '*.jpeg', '*.bmp')), []))
    if not files:
        sys.exit(f'{args.images} 下没有图片')
    img_pts, img_size = [], None
    for fn in files:
        gray = cv2.imread(fn, cv2.IMREAD_GRAYSCALE)
        size = (gray.shape[1], gray.shape[0])
        if img_size and size != img_size:
            print(f'跳过 {fn}: 分辨率 {size} 与 {img_size} 不一致')
            continue
        img_size = size
        c = find_corners(gray, board)
        print(f'{os.path.basename(fn)}: {"OK" if c is not None else "未检测到棋盘"}')
        if c is not None:
            img_pts.append(c)
    return img_pts, img_size


class _Quit(Exception):
    pass


def run_live(args, board):
    cap = open_camera(args.device, args.width, args.height, args.fps)
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
    print('按键要在图像窗口里按（先点一下窗口），终端里输入无效')
    print('SPACE 采集  a 自动采集开关  c 标定  q 退出不标定  Ctrl+C 用已采帧标定')
    if args.auto:
        print(f'自动模式: 采够 {args.min_frames} 帧后自动开始标定')
    print('要点: 覆盖画面四角和边缘，远近都要有，左右/上下各倾斜 30~45°')
    state = {'img_pts': [], 'size': None}
    try:
        _live_loop(cap, args, board, state)
    except KeyboardInterrupt:
        print('\nCtrl+C，用已采集的帧标定')
    except _Quit:
        sys.exit('已退出，未标定')
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return state['img_pts'], state['size']


def _live_loop(cap, args, board, state):
    img_pts, sigs = state['img_pts'], []
    coverage = None
    auto = args.auto
    last_auto = 0.0
    prev = None
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        img_size = state['size'] = (frame.shape[1], frame.shape[0])
        if coverage is None:
            coverage = np.zeros_like(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = find_corners(gray, board)

        disp = cv2.addWeighted(frame, 1.0, coverage, 0.5, 0)
        take = False
        still = False
        if corners is not None:
            cv2.drawChessboardCorners(disp, board, corners, True)
            # 相邻两帧角点平均位移 < 1.5px 视为静止
            still = prev is not None and float(np.mean(np.linalg.norm(corners - prev, axis=2))) < 1.5
            new = is_new_pose(board_pose_signature(corners, img_size), sigs)
            hint = ('NEW POSE' if new else 'similar to captured') + ('' if still else '  (hold still)')
            cv2.putText(disp, hint, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if new and still else (0, 200, 255), 2)
            if auto and new and still and time.time() - last_auto > 1.0:
                take = True
        prev = corners
        mode = 'AUTO' if auto else 'MANUAL'
        cv2.putText(disp, f'[{mode}] captured {len(img_pts)}/{args.min_frames}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if corners is not None else (0, 0, 255), 2)
        cv2.imshow('calibration', disp)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' ') and corners is not None:
            take = True
            if not still:
                print('提示: 这一帧棋盘在动，可能有运动模糊')
        if take:
            img_pts.append(corners)
            sigs.append(board_pose_signature(corners, img_size))
            last_auto = time.time()
            hull = cv2.convexHull(corners.reshape(-1, 2).astype(np.int32))
            cv2.fillConvexPoly(coverage, hull, (80, 0, 0))
            if args.save_dir:
                cv2.imwrite(os.path.join(args.save_dir, f'calib_{len(img_pts):03d}.png'), frame)
            print(f'采集 {len(img_pts)}')
            if auto and len(img_pts) >= args.min_frames:
                print('采够了，开始标定')
                return
        if key == ord('a'):
            auto = not auto
        if key == ord('c'):
            return
        if key in (ord('q'), 27):
            raise _Quit


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--device', default='/dev/video2')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--board', default='8x11', help='棋盘内角点数 列x行（9x12 个方格 → 8x11）')
    ap.add_argument('--square', type=float, default=0.025, help='方格边长，米')
    ap.add_argument('--min-frames', type=int, default=20)
    ap.add_argument('--camera-name', default='front_camera')
    ap.add_argument('--out', default='camera_info.yaml')
    ap.add_argument('--images', help='离线模式：从该目录读图标定')
    ap.add_argument('--save-dir', help='在线模式：同时把采集的原图存到该目录')
    ap.add_argument('--auto', action='store_true', help='在线模式：检测到新姿态自动采集')
    ap.add_argument('--k3', action='store_true', help='同时估计 k3（广角/鱼眼才需要）')
    ap.add_argument('--force', action='store_true', help='姿态多样性检查不通过也写入结果')
    ap.add_argument('--check', metavar='YAML', help='加载标定结果，实时显示去畸变效果')
    args = ap.parse_args()

    if args.check:
        return run_check(args)

    board = tuple(int(v) for v in args.board.lower().split('x'))
    img_pts, img_size = run_offline(args, board) if args.images else run_live(args, board)
    if len(img_pts) < 10:
        sys.exit(f'有效帧只有 {len(img_pts)}，至少需要 10 帧（建议 20+）')

    # 默认只估 k1 k2 p1 p2（同 ROS cameracalibrator）；窄视场镜头 k3 不可观，放开会过拟合
    flags = 0 if args.k3 else cv2.CALIB_FIX_K3
    rms, K, D, keep, rvecs, tvecs = calibrate(img_pts, board, args.square, img_size, flags)
    report(K, D, img_size)
    diverse, _ = check_pose_diversity(rvecs, tvecs)
    if rms > 1.0:
        print(f'警告: RMS {rms:.3f}px 偏大（一般应 < 0.5px），检查方格尺寸、棋盘是否平整、是否运动模糊')
    if not diverse and not args.force:
        sys.exit('姿态不够多样，结果不写入（确认要写加 --force）。请按提示重新采集')
    write_camera_info(args.out, args.camera_name, img_size, K, D, rms, len(keep))


if __name__ == '__main__':
    main()
