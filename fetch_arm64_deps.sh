#!/bin/bash
# 本机下载 ARM64 deb 包，传到远程离线安装
# 不修改系统 apt 配置，用临时目录隔离
# 用法: bash fetch_arm64_deps.sh

set -e

REMOTE="sunrise@192.168.10.42"
SSH="env -i HOME=/home/lsy PATH=/usr/bin:/usr/sbin:/bin:/sbin LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu ssh"
RSYNC="env -i HOME=/home/lsy PATH=/usr/bin:/usr/sbin:/bin:/sbin LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu rsync -avz --progress"
DEB_DIR="/tmp/arm64_debs"
APT_DIR="/tmp/apt_arm64"

mkdir -p "$DEB_DIR" "$APT_DIR/lists/partial" "$APT_DIR/cache/partial"

# ── 1. 写临时 sources.list ────────────────────────────────────────────────────
cat > "$APT_DIR/sources.list" <<'EOF'
deb [arch=arm64] https://mirrors.ustc.edu.cn/ros2/ubuntu/ jammy main
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ jammy main restricted universe multiverse
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ jammy-updates main restricted universe multiverse
EOF

# ── 2. 更新临时源 ─────────────────────────────────────────────────────────────
echo "[1/3] Updating arm64 package lists..."
sudo apt-get update \
    -o Dir::Etc::SourceList="$APT_DIR/sources.list" \
    -o Dir::Etc::SourceParts="-" \
    -o Dir::State::Lists="$APT_DIR/lists" \
    -o Dir::Cache="$APT_DIR/cache" \
    -o APT::Architecture="arm64" \
    -o APT::Architectures="arm64"

# ── 3. 下载 arm64 包 ──────────────────────────────────────────────────────────
echo "[2/3] Downloading packages..."
cd "$DEB_DIR"

APT_OPTS=(
    -o Dir::Etc::SourceList="$APT_DIR/sources.list"
    -o Dir::Etc::SourceParts="-"
    -o Dir::State::Lists="$APT_DIR/lists"
    -o Dir::Cache="$APT_DIR/cache"
    -o APT::Architecture="arm64"
    -o APT::Architectures="arm64"
)

PACKAGES=(
    ros-humble-libnabo
    ros-humble-libpointmatcher
    ros-humble-pcl-conversions
    ros-humble-slam-toolbox
    ros-humble-rosbridge-server
    ros-humble-rosbridge-suite
    ros-humble-rosbridge-library
    ros-humble-rosbridge-msgs
    ros-humble-rosapi
    ros-humble-rosapi-msgs
    libyaml-cpp-dev
    libboost-system-dev
)

for pkg in "${PACKAGES[@]}"; do
    echo "  -> $pkg"
    sudo apt-get download "${APT_OPTS[@]}" "$pkg" 2>/dev/null || \
        echo "  [warn] $pkg not downloaded, skipping"
done

echo "Downloaded $(ls *.deb 2>/dev/null | wc -l) packages"

# ── 4. 传到远程并安装 ─────────────────────────────────────────────────────────
echo "[3/3] Sending to remote and installing..."
$SSH $REMOTE "mkdir -p ~/arm64_debs"
$RSYNC "$DEB_DIR/"*.deb $REMOTE:~/arm64_debs/
$SSH $REMOTE "cd ~/arm64_debs && sudo dpkg -i --force-depends *.deb 2>&1 | tail -30"

echo ""
echo "Done. On remote run:"
echo "  cd ~/max_robot/ros_ws && bash make_remote.sh rebuild"
