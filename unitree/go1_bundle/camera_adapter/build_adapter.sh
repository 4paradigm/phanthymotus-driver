#!/bin/bash
# build_adapter.sh — 在 Go1 Nano 板卡上编译 camera_adapter.cpp → camera_adapter 可执行文件。
#
# 依赖（板卡上应已具备，UnitreecameraSDK 的 example 也依赖它们）：
#   · UnitreecameraSDK：头文件 + 库（官方仓库 clone 到板卡后 make 生成 lib）
#       https://github.com/unitreerobotics/UnitreecameraSDK
#   · OpenCV（板卡自带，UnitreecameraSDK 依赖它）
#   · GStreamer 运行时（gst-launch-1.0 + x264enc/rtph264pay/udpsink 插件）——发流用
#       apt-get install -y gstreamer1.0-tools gstreamer1.0-plugins-{base,good,bad,ugly}
#
# 用法（在板卡上，SDK 路径按实机调整）：
#   export UNITREE_CAMERA_SDK=/home/unitree/UnitreecameraSDK
#   bash build_adapter.sh
#
# 产物：./camera_adapter
#
# ⚠️ 与 robot_interface_v32.cpp 一样，本文件在开发机上无法编译（缺 SDK/相机）。
#    真机编译时若报“UnitreeCamera 无 getFrameSize/getSerialNumber/getRectStereoFrame”等，
#    说明板载 SDK 版本方法名不同——对照 UnitreecameraSDK 的 include/UnitreeCameraSDK.hpp
#    与 examples/ 修正 camera_adapter.cpp 中标了 [SDK-API] 的调用（见 README.md）。
set -e

SDK_DIR="${UNITREE_CAMERA_SDK:-/home/unitree/UnitreecameraSDK}"
OUT="${OUT:-./camera_adapter}"

echo "[adapter] SDK_DIR=${SDK_DIR}"
echo "[adapter] g++ 版本:"; g++ --version | head -1

if [ ! -d "${SDK_DIR}/include" ]; then
  echo "[adapter] ✗ 找不到 ${SDK_DIR}/include —— 先 clone 并 make UnitreecameraSDK，" \
       "或 export UNITREE_CAMERA_SDK=<sdk路径>"
  exit 1
fi

# OpenCV 编译/链接参数（板卡若装了 pkg-config 的 opencv4 用它；否则回退常见路径）。
if pkg-config --exists opencv4; then
  OPENCV_FLAGS="$(pkg-config --cflags --libs opencv4)"
elif pkg-config --exists opencv; then
  OPENCV_FLAGS="$(pkg-config --cflags --libs opencv)"
else
  OPENCV_FLAGS="-I/usr/include/opencv4 -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_videoio"
fi

# UnitreecameraSDK 库名/路径按官方 make 产物（一般是 libunitree_camera.so 或 .a，位于 ${SDK_DIR}/lib）。
SDK_INC="-I${SDK_DIR}/include -I${SDK_DIR}/thirdparty"
SDK_LIB="-L${SDK_DIR}/lib -lunitree_camera"

set -x
g++ -O2 -std=c++14 -pthread \
    camera_adapter.cpp \
    ${SDK_INC} \
    ${SDK_LIB} \
    ${OPENCV_FLAGS} \
    -Wl,-rpath,"${SDK_DIR}/lib" \
    -o "${OUT}"
set +x

echo "[adapter] 产物: $(ls -la "${OUT}")"
echo "[adapter] 冒烟自检（不连相机，仅验证可执行 + 控制口能起）："
echo "  ./camera_adapter --device-id 1 --device-node /dev/video0 --control-port 9301 &"
echo "  printf '{\"cmd\":\"probe\",\"device_id\":1}\\n' | nc 127.0.0.1 9301   # 期望收到一行 JSON"
