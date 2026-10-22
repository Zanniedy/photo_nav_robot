#!/bin/bash
# YOLO26-seg ONNX → RDK X5 BPU .bin 一键转换
#
#   bash convert.sh <模型.onnx> <校准图片目录> [输出目录]
#
# 步骤：
#   1. export_bpu_onnx.py  切掉尾部后处理、降 opset，并用 onnxruntime 验证（本机 python）
#   2. prepare_calib.py    生成量化校准数据（本机 python）
#   3. hb_mapper checker + makertbin（地平线 OpenExplorer docker 里跑）
#
# 环境变量：
#   PY        本机 python，需要 onnx onnxruntime numpy，以及 cv2 或 PIL（默认 python3）
#   OE_IMAGE  OpenExplorer docker 镜像（默认 X5 的 CPU 版，见 README）
#   VERIFY_IMG 第 1 步验证用的图片（可选）
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
ONNX="$(realpath "${1:?用法: bash convert.sh <模型.onnx> <校准图片目录> [输出目录]}")"
IMGS="$(realpath "${2:?缺校准图片目录}")"
OUT="$(realpath -m "${3:-$DIR/build}")"
PY="${PY:-python3}"
OE_IMAGE="${OE_IMAGE:-openexplorer/ai_toolchain_ubuntu_20_x5_cpu:v1.2.8}"
NAME="$(basename "${ONNX%.onnx}" | tr '-' '_')_bpu"

mkdir -p "$OUT"
echo "== 1/3 切图 + 验证"
"$PY" "$DIR/export_bpu_onnx.py" --onnx "$ONNX" --out "$OUT/$NAME.onnx" ${VERIFY_IMG:+--image "$VERIFY_IMG"}

echo "== 2/3 校准数据"
"$PY" "$DIR/prepare_calib.py" --images "$IMGS" --out "$OUT/calib_data"

echo "== 3/3 hb_mapper（docker: $OE_IMAGE）"
if ! command -v docker >/dev/null; then
    echo "没装 docker。装好后重跑，或者把 $OUT 拷到装了 OpenExplorer 的机器上手动执行："
    echo "  hb_mapper checker --model-type onnx --march bayes-e --model $NAME.onnx"
    echo "  hb_mapper makertbin --model-type onnx --config config.yaml"
fi
sed -e "s#{ONNX}#/work/$NAME.onnx#" -e "s#{PREFIX}#$NAME#" \
    -e "s#{CALIB}#/work/calib_data#" -e "s#{WORKDIR}#/work/model_output#" \
    "$DIR/config_x5.yaml" > "$OUT/config.yaml"
command -v docker >/dev/null || exit 1

docker run --rm -v "$OUT":/work -w /work "$OE_IMAGE" bash -c "
    set -e
    hb_mapper checker --model-type onnx --march bayes-e --model /work/$NAME.onnx
    hb_mapper makertbin --model-type onnx --config /work/config.yaml
"
# docker 里是 root，产物改回当前用户
docker run --rm -v "$OUT":/work "$OE_IMAGE" chown -R "$(id -u):$(id -g)" /work

BIN="$OUT/model_output/$NAME.bin"
[ -f "$BIN" ] || { echo "没生成 $BIN，看上面 hb_mapper 的日志"; exit 1; }
cp "$OUT/${NAME}_outputs.json" "$OUT/model_output/"
echo
echo "完成: $BIN"
echo "量化精度看 $OUT/model_output 里 hb_mapper_makertbin.log 的 cosine similarity（输出层 >0.99 为好）"
echo "上板: scp $BIN $OUT/model_output/${NAME}_outputs.json sunrise@192.168.10.42:~/max_robot/models/"
