#!/bin/bash
# 用法: source ./make.sh
# 必须用 source 执行，conda deactivate 和 setup.bash 才能生效到当前终端
set -e

# 检测是否以 source 方式执行
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "[error] 请用 'source ./make.sh' 执行，否则环境变量无法生效到当前终端"
    exit 1
fi

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

# 1. 退出 conda，避免污染 colcon 和 ros2 CLI 的 Python 环境
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "[pre] Deactivating conda env: $CONDA_DEFAULT_ENV"
    CONDA_BASE="$(conda info --base 2>/dev/null)"
    if [ -n "$CONDA_BASE" ]; then
        source "$CONDA_BASE/etc/profile.d/conda.sh"
        conda deactivate
    fi
fi

# 2. 安装缺失的系统依赖
if ! dpkg -s libasio-dev &>/dev/null; then
    echo "[dep] Installing missing dependency: libasio-dev..."
    sudo apt-get install -y libasio-dev
fi

# 3. 构建
echo "[1/2] Building..."
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"

# 4. 合并 compile_commands.json（供 clangd 使用）
echo "[2/2] Merging compile_commands.json..."
find "$WORKSPACE_DIR/build" -name "compile_commands.json" | \
    xargs jq -s 'add // []' > "$WORKSPACE_DIR/compile_commands.json"

# 5. source workspace，让当前终端能找到自定义消息类型和可执行文件
source "$WORKSPACE_DIR/install/setup.bash"

echo ""
echo "Done."
echo "  clangd:   Ctrl+Shift+P -> clangd: Restart language server"
echo "  ros2 CLI: 可直接使用，如 ros2 launch robot_nav mapping.launch.py ..."
