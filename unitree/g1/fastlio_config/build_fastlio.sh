#!/bin/bash
# build_fastlio.sh — 在 ARM64 目标机上原生编译 FAST-LIO2
# 用法: 在 G1 机器人上执行此脚本
#   sudo bash /work/fastlio_config/build_fastlio.sh
#
# 编译产物安装到 /opt/fastlio/install/，供 FastLioPlugin 启动时使用。
set -euo pipefail

INSTALL_DIR="/opt/fastlio"
FAST_LIO_URL="${FAST_LIO_URL:-https://github.com/hku-mars/FAST_LIO.git}"
FAST_LIO_BRANCH="${FAST_LIO_BRANCH:-main}"

echo "[fastlio-build] Refreshing ROS GPG key..."
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

echo "[fastlio-build] Installing dependencies..."
apt-get update
apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libpcl-dev \
    cmake build-essential git

echo "[fastlio-build] Cloning FAST-LIO2 (branch: ${FAST_LIO_BRANCH})..."
mkdir -p /tmp/fastlio_ws/src
cd /tmp/fastlio_ws/src
git clone --depth 1 -b "${FAST_LIO_BRANCH}" "${FAST_LIO_URL}" fast_lio

echo "[fastlio-build] Building..."
cd /tmp/fastlio_ws

# Find ROS2 setup.bash
ROS_SETUP=""
for p in /opt/ros/humble/setup.bash /opt/ros/foxy/setup.bash /ros_ws/install/setup.bash; do
    if [ -f "$p" ]; then
        ROS_SETUP="$p"
        break
    fi
done

if [ -z "${ROS_SETUP}" ]; then
    echo "[fastlio-build] ERROR: No ROS2 setup.bash found. Install ROS2 first or run inside the G1 Docker container."
    exit 1
fi
echo "[fastlio-build] Using ROS2: ${ROS_SETUP}"
source "${ROS_SETUP}"
colcon build --packages-select fast_lio \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --install-base "${INSTALL_DIR}/install"

echo "[fastlio-build] Cleaning up..."
rm -rf /tmp/fastlio_ws

echo "[fastlio-build] Done. FAST-LIO2 installed to ${INSTALL_DIR}/install/"
echo "[fastlio-build] Source it with: source ${INSTALL_DIR}/install/setup.bash"
