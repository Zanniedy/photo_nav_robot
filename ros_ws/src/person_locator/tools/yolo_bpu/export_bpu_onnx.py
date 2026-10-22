#!/usr/bin/env python3
"""把 ultralytics 导出的 end2end YOLO26-seg ONNX 改成适合地平线 BPU 量化的 ONNX。

原模型尾部有 Transpose/TopK/GatherElements/Mod 等后处理，BPU 不支持或会掉到 CPU，
而且把框坐标、分数、mask 系数拼在一个张量里量化，精度损失大。这里：
  1. 在每个尺度最后一层 Conv 处切断，输出 box/cls/mc 各 3 个 + proto，共 10 个
  2. opset 降到 11（X5 的 hb_mapper 1.24 最高只认 11）。onnx 自带 version_converter 降不下去，
     手动改写 YOLO 里用到的几个算子：Split / Resize / Reshape，其余算子 11~17 语义相同
  3. 用 onnxruntime 跑原模型和切图模型，对比解码结果，确认切图没改变输出
  4. 写出 <out>_outputs.json，记录输出顺序/类型/stride，板端解码要用

用法（需要 onnx onnxruntime numpy，图片读取用 cv2 或 PIL）：
    python export_bpu_onnx.py --onnx yolo26s-seg.onnx --out build/yolo26s_seg_bpu.onnx --image bus.jpg
"""
import argparse
import json
import os
import re
import sys

import numpy as np
import onnx
from onnx import shape_inference, version_converter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yolo_decode import decode, letterbox  # noqa: E402


def load_rgb(path):
    try:
        import cv2
        img = cv2.imread(path)
        return None if img is None else img[:, :, ::-1].copy()
    except (ImportError, AttributeError):   # 有的 conda 环境 cv2 和 numpy2 不兼容
        from PIL import Image
        return np.asarray(Image.open(path).convert('RGB'))


def find_head_outputs(model):
    """找每个尺度 box/cls/mc 最后一层 Conv 的输出。

    检测头里这些 Conv 的输出都直接接 Reshape(1,C,H*W)，靠这个特征定位，不依赖具体节点名。
    one2many 分支（训练用）导出时已被裁掉，只剩 one2one。
    """
    m = shape_inference.infer_shapes(model)
    shapes = {v.name: [d.dim_value for d in v.type.tensor_type.shape.dim]
              for v in list(m.graph.value_info) + list(m.graph.output) + list(m.graph.input)}
    producer = {o: n for n in m.graph.node for o in n.output}
    heads = []
    for n in m.graph.node:
        if n.op_type != 'Reshape':
            continue
        src = producer.get(n.input[0])
        if src is None or src.op_type != 'Conv':
            continue
        name = src.output[0]
        s = shapes.get(name)
        if not s or len(s) != 4:
            continue
        heads.append((name, s))
    if not heads:
        sys.exit('没找到检测头的 Conv→Reshape 结构，模型不是 ultralytics YOLO 检测/分割导出的？')

    # 输出名里有 cv2/cv3/cv4 时按名字分（cv2=box cv3=cls cv4=mask系数），否则按通道数猜
    inp = shapes[m.graph.input[0].name]
    # 骨干里的注意力（C2PSA）也有 Conv→Reshape，只保留检测头的：
    # 名字里有 cv2/cv3/cv4.<尺度> 的按名字分（cv2=box cv3=cls cv4=mask系数），没有就按通道数认
    named = [(n, s) for n, s in heads if re.search(r'cv[234]\.\d', n)]
    nc = len(re.findall(r'\d+:', str({p.key: p.value for p in model.metadata_props}.get('names', ''))))
    out = []
    for name, (_, c, h, w) in (named or heads):
        mm = re.search(r'cv([234])\.(\d)', name)
        if mm:
            kind = {'2': 'box', '3': 'cls', '4': 'mc'}[mm.group(1)]
        elif c in (4, 32, nc):
            kind = 'box' if c == 4 else 'mc' if c == 32 else 'cls'
        else:
            continue
        out.append({'name': name, 'kind': kind, 'stride': inp[2] // h, 'channels': c, 'shape': [1, c, h, w]})
    # proto：原模型里 1x32x160x160 的那个输出
    for o in m.graph.output:
        s = shapes[o.name]
        if len(s) == 4 and s[1] == 32:
            out.append({'name': o.name, 'kind': 'proto', 'stride': None, 'channels': 32, 'shape': s})
    order = {'box': 0, 'cls': 1, 'mc': 2}
    det = sorted([o for o in out if o['kind'] != 'proto'], key=lambda o: (o['stride'], order[o['kind']]))
    return det + [o for o in out if o['kind'] == 'proto'], shapes


def cut(model, outputs, in_name):
    """onnx.utils.Extractor 截子图；输出重命名成简短的名字，方便板端看"""
    ext = onnx.utils.Extractor(model)
    sub = ext.extract_model([in_name], [o['name'] for o in outputs])
    rename = {}
    for o in outputs:
        short = 'proto' if o['kind'] == 'proto' else f"{o['kind']}_s{o['stride']}"
        rename[o['name']] = short
        o['name'] = short
    for n in sub.graph.node:
        n.output[:] = [rename.get(x, x) for x in n.output]
        n.input[:] = [rename.get(x, x) for x in n.input]
    for o in sub.graph.output:
        o.name = rename.get(o.name, o.name)
    return sub


# opset 18 改了这些算子的签名；切出来的子图只要没用到新特性，直接把版本号标成 17 语义不变。
# onnx 自带的 version_converter 没有 18→17 的 Split adapter，所以手动跨这一步
_OPSET18_CHANGED = {'Split', 'Resize', 'Pad', 'ScatterElements', 'ScatterND', 'Col2Im', 'BitwiseAnd',
                    'ReduceMax', 'ReduceMin', 'ReduceMean', 'ReduceSum', 'ReduceProd', 'ReduceL1', 'ReduceL2',
                    'ReduceLogSum', 'ReduceLogSumExp', 'ReduceSumSquare', 'OptionalHasElement', 'GroupNormalization',
                    'LpPool', 'GridSample', 'AffineGrid', 'DFT', 'Gelu', 'StringSplit', 'ImageDecoder'}


def relabel_to_17(model):
    for n in model.graph.node:
        attrs = {a.name for a in n.attribute}
        if n.op_type not in _OPSET18_CHANGED:
            continue
        if n.op_type == 'Split' and 'num_outputs' not in attrs:
            continue    # 13~17 的 Split 同样是 split 作为输入
        if n.op_type == 'Resize' and not attrs & {'antialias', 'axes', 'keep_aspect_ratio_policy'}:
            continue
        sys.exit(f'{n.op_type} ({n.name}) 用到了 opset 18+ 的特性，没法自动降级；用 --opset 19 试试，'
                 f'或者从 .pt 用 opset=11 重新导出')
    for o in model.opset_import:
        if o.domain in ('', 'ai.onnx'):
            o.version = 17


# 11~17 之间签名/语义没变（或只是加了数据类型）的算子；碰到列表外的就报错，不瞎降
_SAME_11_17 = {'Add', 'Sub', 'Mul', 'Div', 'Conv', 'ConvTranspose', 'MaxPool', 'AveragePool', 'Sigmoid', 'Relu',
               'Concat', 'MatMul', 'Transpose', 'Identity', 'Constant', 'Tanh', 'HardSigmoid', 'HardSwish',
               'BatchNormalization', 'Gemm', 'Flatten', 'Pow', 'Sqrt', 'Exp', 'LeakyRelu', 'Clip'}


def downgrade_17_to_11(model):
    from onnx import helper, numpy_helper
    g = model.graph
    init = {i.name: i for i in g.initializer}
    shapes = {v.name: [d.dim_value for d in v.type.tensor_type.shape.dim]
              for v in list(shape_inference.infer_shapes(model).graph.value_info) + list(g.input)}

    def const(name, arr):
        t = numpy_helper.from_array(np.asarray(arr), name)
        g.initializer.append(t)
        init[name] = t
        return name

    empty = None
    for n in g.node:
        attrs = {a.name: a for a in n.attribute}
        if n.op_type == 'Split':
            # 13+ split 是输入，11 是属性
            if len(n.input) > 1 and n.input[1]:
                if n.input[1] not in init:
                    sys.exit(f'{n.name}: split 不是常量，没法降到 opset 11')
                split = numpy_helper.to_array(init[n.input[1]]).astype(int).tolist()
                del n.input[1:]
                n.attribute.append(helper.make_attribute('split', split))
        elif n.op_type == 'Reshape':
            if 'allowzero' in attrs:
                if attrs['allowzero'].i:
                    sys.exit(f'{n.name}: allowzero=1 没法降到 opset 11')
                n.attribute.remove(attrs['allowzero'])
        elif n.op_type == 'Softmax':
            # 11 的 Softmax 把 axis 之后的维度拉平；axis 是最后一维时和 13+ 等价
            axis = attrs['axis'].i if 'axis' in attrs else -1
            rank = len(shapes.get(n.input[0], []))
            if axis not in (-1, rank - 1):
                sys.exit(f'{n.name}: Softmax axis={axis} 不是最后一维，降到 11 语义会变')
        elif n.op_type == 'Resize':
            # 11: X, roi, scales 必填（可空张量）, sizes 可选。统一改成常量 scales，BPU 对 scales 支持最好
            x_shape = shapes.get(n.input[0])
            if len(n.input) >= 3 and n.input[2] and n.input[2] in init:
                scales_name = n.input[2]
            else:
                out = shapes.get(n.output[0])
                if not x_shape or not out or 0 in x_shape + out:
                    sys.exit(f'{n.name}: 推不出 Resize 的输入输出形状')
                scales_name = const(n.name + '_scales', np.array(out, np.float32) / np.array(x_shape, np.float32))
            if empty is None:
                empty = const('resize_empty_roi', np.zeros(0, np.float32))
            n.input[:] = [n.input[0], empty, scales_name]
            for k in ('antialias', 'axes', 'keep_aspect_ratio_policy'):
                if k in attrs:
                    sys.exit(f'{n.name}: Resize 用了 opset 18 的 {k}')
        elif n.op_type not in _SAME_11_17:
            sys.exit(f'{n.op_type} ({n.name}) 没有处理 11 的降级，需要在 downgrade_17_to_11 里补上，'
                     f'或从 .pt 用 opset=11 重新导出')
    prune(model)
    for o in model.opset_import:
        if o.domain in ('', 'ai.onnx'):
            o.version = 11


def prune(model):
    """删掉不再被用到的节点和常量（Resize 的 sizes 原来是一串 Shape/Concat 算出来的）"""
    g = model.graph
    needed = {o.name for o in g.output}
    keep = []
    for n in reversed(list(g.node)):
        if any(o in needed for o in n.output):
            keep.append(n)
            needed.update(i for i in n.input if i)
    dropped = len(g.node) - len(keep)
    del g.node[:]
    g.node.extend(reversed(keep))
    inits = [i for i in g.initializer if i.name in needed]
    del g.initializer[:]
    g.initializer.extend(inits)
    vi = [v for v in g.value_info if v.name in needed]
    del g.value_info[:]
    g.value_info.extend(vi)
    if dropped:
        print(f'  删掉 {dropped} 个无用节点')


def run_ort(path, x):
    import onnxruntime as ort
    sess = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    return sess.run(None, {sess.get_inputs()[0].name: x})


def verify(orig_path, new_path, layout, image, conf):
    rgb = load_rgb(image)
    if rgb is None:
        sys.exit(f'读不了图片 {image}')
    lb, r, pad = letterbox(rgb, 640)
    x = (lb.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]
    ref0, ref_proto = run_ort(orig_path, x)
    ref = ref0[0][ref0[0][:, 4] > conf]
    outs = run_ort(new_path, x)
    dets, masks = decode(outs, layout, conf=conf)
    print(f'原模型 {len(ref)} 个目标，切图+numpy 解码 {len(dets)} 个目标（conf>{conf}）')
    ok = len(ref) == len(dets)
    if ok and len(ref):
        ref = ref[np.argsort(-ref[:, 4])]
        box_err = np.abs(ref[:, :4] - dets[:, :4]).max()
        score_err = np.abs(ref[:, 4] - dets[:, 4]).max()
        cls_eq = bool((ref[:, 5].astype(int) == dets[:, 5].astype(int)).all())
        print(f'  框最大误差 {box_err:.4f}px  分数最大误差 {score_err:.2e}  类别一致 {cls_eq}')
        ok = box_err < 0.5 and score_err < 1e-3 and cls_eq
        # mask：原模型 output0 的 6: 是 32 维系数，自己乘 proto 对比
        coef = ref[:, 6:38]
        p = ref_proto[0].reshape(32, -1)
        ref_m = (1 / (1 + np.exp(-(coef @ p)))).reshape(-1, *ref_proto.shape[2:])
        from yolo_decode import crop_masks
        ref_m = crop_masks(ref_m, ref[:, :4] / 4.0)
        iou = np.logical_and(ref_m > 0.5, masks > 0.5).sum() / max(np.logical_or(ref_m > 0.5, masks > 0.5).sum(), 1)
        print(f'  mask IoU {iou:.4f}')
        ok = ok and iou > 0.99
        for d in dets[:10]:
            print(f'    cls={int(d[5]):2d} score={d[4]:.3f} box=({d[0]:.0f},{d[1]:.0f},{d[2]:.0f},{d[3]:.0f})')
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--onnx', required=True, help='ultralytics 导出的 ONNX')
    ap.add_argument('--out', required=True)
    ap.add_argument('--opset', type=int, default=11, choices=[11, 17],
                    help='X5 hb_mapper 1.24 最高支持 11；17 只用于对比调试')
    ap.add_argument('--image', help='验证用图片（不给就不验证）')
    ap.add_argument('--conf', type=float, default=0.25)
    args = ap.parse_args()

    model = onnx.load(args.onnx)
    meta = {p.key: p.value for p in model.metadata_props}
    in_name = model.graph.input[0].name
    in_shape = [d.dim_value for d in model.graph.input[0].type.tensor_type.shape.dim]
    if 0 in in_shape:
        sys.exit(f'输入形状 {in_shape} 是动态的，BPU 需要固定形状，用 imgsz 固定、dynamic=False 重新导出')
    meta_opset = model.opset_import[0].version
    print(f'输入 {in_name} {in_shape}  opset {meta_opset}  task={meta.get("task")}')

    outputs, _ = find_head_outputs(model)
    sub = cut(model, outputs, in_name)
    for o in outputs:
        print(f'  输出 {o["name"]:10s} {o["kind"]:5s} stride={o["stride"]} {o["shape"]}')

    cur = sub.opset_import[0].version
    if cur != args.opset:
        if cur >= 18 and args.opset < 18:
            relabel_to_17(sub)
            cur = 17
        if args.opset == 11 and cur > 11:
            if cur < 17:
                sub = version_converter.convert_version(sub, 17)
            downgrade_17_to_11(sub)
        print(f'opset {sub.opset_import[0].version} (原 {meta_opset})')
    sub.ir_version = min(sub.ir_version, 7 if args.opset <= 13 else 8)   # 老 onnx 解析器认的 IR 版本
    del sub.metadata_props[:]
    for k in ('names', 'imgsz', 'stride', 'task', 'description'):
        if k in meta:
            sub.metadata_props.add(key=k, value=meta[k])
    onnx.checker.check_model(sub)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    onnx.save(sub, args.out)
    ops = sorted({n.op_type for n in sub.graph.node})
    print(f'已写入 {args.out}，{len(sub.graph.node)} 个节点，算子: {", ".join(ops)}')

    layout_path = os.path.splitext(args.out)[0] + '_outputs.json'
    names = meta.get('names', '')
    with open(layout_path, 'w') as f:
        json.dump({'input': {'name': in_name, 'shape': in_shape}, 'outputs': outputs, 'names': names},
                  f, ensure_ascii=False, indent=1)
    print(f'输出布局 → {layout_path}')

    if args.image:
        if not verify(args.onnx, args.out, outputs, args.image, args.conf):
            sys.exit('验证不通过：切图后的结果和原模型不一致')
        print('验证通过')


if __name__ == '__main__':
    main()
