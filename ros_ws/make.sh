#!/bin/bash
# 用法: source ./make.sh
# 必须用 source 执行，conda deactivate 和 setup.bash 才能生效到当前终端

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "[error] 请用 'source ./make.sh' 执行，否则环境变量无法生效到当前终端"
    exit 1
fi

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

# ── 1. 退出 conda ──────────────────────────────────────────────────────────────
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "[pre] Deactivating conda env: $CONDA_DEFAULT_ENV"
    CONDA_BASE="$(conda info --base 2>/dev/null)"
    if [ -n "$CONDA_BASE" ]; then
        source "$CONDA_BASE/etc/profile.d/conda.sh"
        conda deactivate
    fi
fi

# ── 2. apt 依赖 ────────────────────────────────────────────────────────────────
_apt_install() {
    if ! dpkg -s "$1" &>/dev/null; then
        echo "[dep] Installing: $1"
        sudo apt-get install -y "$1"
    fi
}

_apt_install libasio-dev
_apt_install ros-humble-libpointmatcher
_apt_install libeigen3-dev
_apt_install libopencv-dev
_apt_install libyaml-cpp-dev
_apt_install libboost-system-dev

# ── 3. 构建 ───────────────────────────────────────────────────────────────────
echo "[1/2] Building..."
if ! colcon build \
        --cmake-force-configure \
        --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -Wno-dev \
        "$@"; then
    echo "[error] Build failed."
    return 1
fi

# ── 4. 合并 compile_commands.json（供 clangd） ─────────────────────────────────
echo "[2/2] Merging compile_commands.json..."
find "$WORKSPACE_DIR/build" -name "compile_commands.json" \
    | xargs jq -s 'add // []' > "$WORKSPACE_DIR/compile_commands.json" 2>/dev/null || true

# ── 5. source workspace ────────────────────────────────────────────────────────
source "$WORKSPACE_DIR/install/setup.bash"

echo ""
echo "Done."
echo "  clangd:   Ctrl+Shift+P -> clangd: Restart language server"
echo "  ros2 CLI: 可直接使用，如 ros2 launch robot_nav mapping.launch.py ..."
