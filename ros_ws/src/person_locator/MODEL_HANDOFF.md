# 人员定位：模型部分交接（2026-10-21 写，10-22 更新状态表）

目标：机器人相机里检测出人 → 算出人相对机器人的方位角 → 在 2D 激光里取该方位的距离。
分三步：① 相机标定 ② 检测模型上 X5 BPU ③ 和激光融合。

| 步骤 | 状态 |
|---|---|
| ① 标定 | 做了，**结果可疑**（fx=2770，RMS 1.07），见第五节 |
| ② BPU 模型 | **v2 已上板**（10-22，122 张真实照片校准，调试用）：召回 92/97，多检 1；但 117ms/帧，慢在 CPU 上的 Softmax。v3（Softmax 放 BPU）配置写好了还没跑。详见 `HANDOFF.md` 10-22 第二节 |
| ③ 融合 | **第一版已写**：`person_tracker_node.py`（激光取距离 + 跟踪 + `/person/approach` 走到人面前），没在有人的场景实测，见 `HANDOFF.md` |

> 下面第一~四节是 10-21 写的，「第一版 .bin」「接下来按顺序做」里的第 1、3 步已完成，以 `HANDOFF.md` 为准。第二节的转换流程仍然有效，但容器 `rdk_x5_mapper` 已经没了，改用 `sudo docker run --rm -v ~/rdk_x5_mapper:/workspace <镜像> bash -c '...'`

## 一、现在手里有什么

| 东西 | 位置 | 说明 |
|---|---|---|
| 原始模型 | `~/cs_project/model_test/show_2/yolo26s-seg.onnx`（41MB） | ultralytics 8.4.41 导出的 YOLO26s-seg，COCO 80 类，**class 0 = person**（不是人脸模型）。640x640，end2end NMS-free，opset 20 |
| 转换工具 | `person_locator/tools/yolo_bpu/` | 切图 + 降 opset + 校准数据 + hb_mapper 配置 + 板端推理脚本 |
| 工具链容器 | docker 容器 `rdk_x5_mapper`，镜像 `openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8-py310`（hb_mapper 1.24.3 / hbdk 3.49.15） | 宿主机 `~/rdk_x5_mapper` 挂到容器 `/workspace`。用户的 docker 要 `sudo` |
| 转换工作目录 | `~/rdk_x5_mapper/yolo26/`（容器里 `/workspace/yolo26/`） | `yolo26s_seg_bpu.onnx`（opset 11）、`calib_data/`、`config.yaml`、`yolo26s_seg_bpu_outputs.json`、两份 hb_mapper 日志 |
| **第一版 .bin** | `~/rdk_x5_mapper/yolo26/model_output/yolo26s_seg_bpu.bin`（11MB） | 校准数据是 20 张棋盘标定图，只用来验证流程，精度见第三节。文件属主是 root，先 `sudo chown -R $USER:$USER ~/rdk_x5_mapper/yolo26` |
| 相机标定 | `person_locator/config/camera_info.yaml`（机器人上 `~/max_robot/calib/camera_info.yaml`） | 可疑，见第五节 |
| 拍照 | 网页 → 相机 → 预览 → 拍照/连拍，存机器人 `~/max_robot/captures/`，可下载 zip | 已上线，实机测过 |
| SAM2 | `~/cs_project/model_test/show_1/model`（hiera-large 视频版，900MB） | 太大，不上 X5，先不用 |

板端（RDK X5，系统 3.5.0）：`hobot-dnn 3.0.4` 已装，`from hobot_dnn import pyeasy_dnn` 可用，没有 onnxruntime。

## 二、快速开始：重新出一版 .bin

`convert.sh` 里的第 3 步用的是 `docker run`，不加 sudo，镜像 tag 也和用户实际装的（`-py310`）不一样。所以现在的做法是：宿主机上做第 1、2 步，容器里手动跑 hb_mapper。

```bash
# 宿主机
T=~/cs_project/robot/ros_ws/src/person_locator/tools/yolo_bpu
PY=~/miniconda3/envs/torch_blackwell/bin/python        # 不要加 -I，PIL 在 user site 里
O=~/rdk_x5_mapper/yolo26
sudo chown -R $USER:$USER $O

# 1. 切图 + 降到 opset 11 + 用 onnxruntime 对比原模型（模型没换就不用重跑）
$PY $T/export_bpu_onnx.py --onnx ~/cs_project/model_test/show_2/yolo26s-seg.onnx \
    --out $O/yolo26s_seg_bpu.onnx --image ~/cs_project/model_test/show_2/bus.jpg

# 2. 校准数据：用真实场景的照片（网页连拍，50~100 张，有人、远近和光照都要有）
env -u LD_LIBRARY_PATH PATH=/usr/bin:/bin /usr/bin/scp -r sunrise@192.168.10.42:max_robot/captures ~/calib_people
$PY $T/prepare_calib.py --images ~/calib_people --out $O/calib_data

# config.yaml 已经生成好（路径是 /workspace/yolo26/...），改了 config_x5.yaml 才需要重新生成：
sed -e "s#{ONNX}#/workspace/yolo26/yolo26s_seg_bpu.onnx#" -e "s#{PREFIX}#yolo26s_seg_bpu#" \
    -e "s#{CALIB}#/workspace/yolo26/calib_data#" -e "s#{WORKDIR}#/workspace/yolo26/model_output#" \
    $T/config_x5.yaml > $O/config.yaml
```

```bash
# 容器里
sudo docker start rdk_x5_mapper        # 容器已经存在，不要再 docker run --name，会名字冲突
sudo docker exec -it rdk_x5_mapper bash
cd /workspace/yolo26
hb_mapper checker --model-type onnx --march bayes-e --model yolo26s_seg_bpu.onnx
hb_mapper makertbin --model-type onnx --config config.yaml      # 约 5 分钟
grep -A14 "^Output      Cosine" hb_mapper_makertbin.log        # 看输出层精度
```

```bash
# 3. 上板测试（宿主机执行；机器人上的工具目录是 rsync.sh 同步过去的）
S="env -u LD_LIBRARY_PATH PATH=/usr/bin:/bin /usr/bin"
$S/ssh sunrise@192.168.10.42 mkdir -p max_robot/models
$S/scp $O/model_output/yolo26s_seg_bpu.bin $O/yolo26s_seg_bpu_outputs.json sunrise@192.168.10.42:max_robot/models/
# 机器人上：先在网页停掉 camera 进程，否则相机被占用；也可以用 --image 测拍下来的照片
python3 ~/max_robot/ros_ws/src/person_locator/tools/yolo_bpu/bpu_infer.py \
    --model ~/max_robot/models/yolo26s_seg_bpu.bin --image ~/max_robot/captures/<某张>.jpg \
    --person-only --save /tmp/out.jpg
```

流程各步做了什么：

1. `export_bpu_onnx.py`：在检测头每个尺度最后一层 Conv 处切断，去掉 TopK、GatherElements、Mod 这些后处理。输出 10 个张量：
   - `box_s{8,16,32}`：4 通道，l/t/r/b 到格子中心的距离，单位是格
   - `cls_s*`：80 通道，没过 sigmoid
   - `mc_s*`：32 维 mask 系数
   - `proto`：32x160x160

   opset 从 20 降到 11。onnx 自带的 converter 降不下去，脚本里 `downgrade_17_to_11` 手动改写了 Split（split 从输入改成属性）、Resize（改成常量 scales，补一个空的 roi）、Reshape（去掉 allowzero），并检查 Softmax 的 axis 是最后一维。改完用 onnxruntime 对比原模型：**bus.jpg 上 4 个目标，框误差 0，mask IoU 1.0**
2. `prepare_calib.py`：letterbox 到 640，转成 RGB NCHW float32，像素值保持 0~255。除以 255 由 config 里的 `scale_value` 交给工具链插入
3. `config_x5.yaml`：`march: bayes-e`，运行时输入 nv12，编译模式 latency、O3

后处理在 `yolo_decode.py`，纯 numpy，PC 验证和板端共用。YOLO26 是 one2one 头，不用 NMS，阈值筛完按分数排序就行。

## 三、第一版 .bin 的量化结果（棋盘图校准）

输出层 cosine similarity（0.99 以上算好）：

| 输出 | s8 | s16 | s32 |
|---|---|---|---|
| cls | 0.994 | 0.989 | 0.982 |
| box | 0.953 | 0.946 | 0.971 |
| mc | 0.971 | 0.955 | 0.938 |
| proto | 0.924 | | |

- cls 基本能用，box、mc、proto 偏低。第一层 Conv 就只有 0.95，说明**校准数据和实际输入分布差得远**。20 张棋盘图色彩和场景都太单一，这是主要原因
- 两处 C2PSA 注意力（`/model.10/m/m.0/attn/*`、`/model.22/m.0/m.0.1/attn/*`）量化很差：MatMul 是 0.66~0.88，**Softmax 放在 CPU 上算**，整个模型因此被切成 3 段 BPU 子图（日志里 `subgraph_0/1/2`），中间来回搬数据，会多几毫秒

## 四、接下来按顺序做

1. **用真实照片重新校准**（收益最大）：机器人开机后用网页连拍采 50~100 张，按第二节重跑 makertbin，目标是 box 和 cls 都在 0.98 以上
2. **注意力层改 int16**：如果第 1 步之后注意力层还是差，在 `config.yaml` 的 `calibration_parameters` 下加下面这段。字段名是凭经验写的，**没对照 1.24.3 文档核实**，报错就去查 OE 文档里 `node_info` 的写法：
   ```yaml
     node_info: {
       "/model.10/m/m.0/attn/MatMul":       {'InputType': 'int16'},
       "/model.10/m/m.0/attn/MatMul_1":     {'InputType': 'int16'},
       "/model.22/m.0/m.0.1/attn/MatMul":   {'InputType': 'int16'},
       "/model.22/m.0/m.0.1/attn/MatMul_1": {'InputType': 'int16'}
     }
   ```
   还可以试 `calibration_type: 'max'`
3. **上板实测**：用 `bpu_infer.py` 看实际检测结果和 BPU 耗时。cosine 不到 0.99 不一定说明框就不能用，最终看实测。先拿第一版 `.bin` 测速也行。估计 BPU 推理在 20~35ms，加上 JPEG 解码 22ms（实测）和前后处理，整条链路大约 15~20fps。这是估算，没实测过
4. **板端输出格式**：`bpu_infer.py` 假设 `pyeasy_dnn` 返回的是已经反量化的 float，布局 NCHW 或 NHWC 都行（`to_nchw` 两种都兼容）。如果某个输出是 int32 加 scale，要按 `o.properties.scale_data` 乘回去。脚本开头会打印每个输出的 shape 和 dtype，看一眼就知道
5. **太慢的话**：换 yolo26n-seg。或者只要框不要 mask，就用纯检测版。切图和解码都兼容检测版，但 `export_bpu_onnx.py` 的 `verify()` 写死了分割版的两个输出，要改一下
6. 把 `convert.sh` 第 3 步改成 `sudo docker exec rdk_x5_mapper ...`，并把默认镜像 tag 改成 `v1.2.8-py310`，这样一条命令就能跑完。可选

## 五、相机标定的问题（第 3 步融合前要处理）

当前结果：fx=2770 fy=2767 cx=568 cy=289，k1=-0.38 k2=-0.61，RMS 1.07px，20 帧。可疑的地方：

- RMS 偏大。20 张标定图都很糊，Laplacian 方差只有 17~31，清晰的图一般在 100 以上，像是没对好焦或者有运动模糊
- fx 对应的水平视场只有 26°，配上这么大的桶形畸变，不像真实镜头。旧记录 fx≈1052，但不确定是不是同一个相机
- 本机用固定主点、只估 k1 等约束重算，fx 仍在 2700~3500 之间，这批数据本身约束不住

建议先拿尺子量一下真实视场：在已知距离处看画面能装下多宽，hfov = 2·atan(宽/2/距离)。如果视场明显大于 26°，就把镜头对好焦，或者把棋盘放到成像清晰的距离，在网页标定页重标，目标 RMS < 0.5px。原图在机器人 `~/max_robot/calib/images/`，可以用 `ros2 run person_locator calibrate --images <目录>` 离线复算。

方位角对 fx 和 cx 很敏感：像素 u 处的方位角 = atan((u - cx) / fx)。fx 错 2.6 倍，画面边缘的人方位会差十几度。

## 六、第 3 步融合的设计（还没写）

- 新节点订阅 `/camera/image_raw/compressed`、`/camera/camera_info`、`/scan`，用 BPU 检测 person
- 取框底边中点（或 mask 下部的像素），先去畸变，再按 K 反投影成光学系下的射线，经 TF `camera_optical_frame → laser` 转到雷达系，得到水平方位角 θ
- 在 `/scan` 里取 θ ± 半个框宽对应的角度窗口，取窗口内距离的低分位数（比如 20%），去掉背景
- 输出：人在 `base_link` 下的方位角和距离，再发一个 `PoseArray` 或 `MarkerArray` 给 RViz 和网页显示
- 前提是相机外参。现在 `robot.launch.py` 里 `base_link→camera_link` 是 `0.25 0 0.6`，是假值。最要紧的是相机和雷达的相对 yaw，可以让人站在几个不同方位，用"图像检测出的方位"对比"激光里人腿的方位"来拟合

## 七、环境的坑

- 本机 ssh/scp 会加载 miniconda 的 OpenSSL 而报错，统一用 `env -u LD_LIBRARY_PATH PATH=/usr/bin:/bin /usr/bin/ssh ...`
- `torch_blackwell` 环境里 cv2 和 numpy 2 不兼容，import 时报 `_ARRAY_API not found`。工具里已经改成自动退回 PIL。PIL 装在 user site 里，所以**不要用 `python -I` 跑**
- 本机其它 conda 环境和 `/usr/bin/python3` 都没有 onnx
- hb_mapper 1.24.3 **最高只认 opset 11**，17 会报 `maximum supported version is 11`
- 容器里生成的文件属主是 root，宿主机要先 chown 才能改
- 机器人上后台起服务要 `setsid nohup ... < /dev/null &`；`pkill -f` 会把 ssh 命令自己也杀掉，要按 PID kill
- 相机同一时间只能被一个进程打开：`camera_node` 在跑时，`bpu_infer.py --camera` 打不开
- 机器人经常不在线（ssh 超时），这时没法采校准图，也没法上板测
