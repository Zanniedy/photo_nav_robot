# YOLO26-seg → RDK X5 BPU

`model_test/show_2/yolo26s-seg.onnx` 是 ultralytics 的 COCO 80 类实例分割模型，不是人脸模型。class 0 就是 person，人员定位直接用人的框和 mask。

```bash
# 本机：需要 onnx、onnxruntime、numpy，再加 cv2 或 PIL 其中一个（torch_blackwell 环境可以直接用）
PY=~/miniconda3/envs/torch_blackwell/bin/python VERIFY_IMG=~/cs_project/model_test/show_2/bus.jpg \
  bash convert.sh ~/cs_project/model_test/show_2/yolo26s-seg.onnx <校准图片目录> build

# 板端测试
python3 bpu_infer.py --model ~/max_robot/models/yolo26s_seg_bpu.bin --image test.jpg --save out.jpg --person-only
```

| 文件 | 作用 |
|---|---|
| `export_bpu_onnx.py` | 切掉 TopK/GatherElements 后处理，改成 10 个输出：每个尺度 box/cls/mc 各一个，加 proto。opset 20 降到 11（X5 hb_mapper 最高只支持 11），再用 onnxruntime 对比原模型 |
| `prepare_calib.py` | 把校准图片做成 letterbox 640、RGB NCHW、float32，像素值保持 0~255 |
| `config_x5.yaml` | hb_mapper 配置：bayes-e，运行时输入 nv12，scale 为 1/255 |
| `convert.sh` | 串起上面几步，第 3 步在 OpenExplorer docker 里跑 `hb_mapper checker` 和 `makertbin` |
| `yolo_decode.py` | numpy 后处理。YOLO26 是 NMS-free 的 one2one 头，按阈值取结果就行。PC 验证和板端共用这一份 |
| `bpu_infer.py` | 板端用 `hobot_dnn.pyeasy_dnn` 推理，并测速 |

现状：第 1、2 步本机跑通了。在 bus.jpg 上，切图后的结果和原模型完全一致：框误差 0，mask IoU 1.0。第 3 步还没跑过，本机没装 docker。

- OE 镜像默认 `openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8`（Docker Hub），换版本用 `OE_IMAGE=` 覆盖
- 校准图要用机器人相机在实际场景拍，画面里要有人，准备 50~100 张。棋盘标定图只适合用来试流程
- 看 makertbin 日志里各输出的 cosine similarity。精度差的话可以先试 `calibration_type: 'max'`，或者把 C2PSA 的 Softmax 放到 CPU 上算（`run_on_cpu`）
