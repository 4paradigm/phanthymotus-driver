#!/bin/bash
# build_adapter.sh — 在 Go1 Nano 板卡上编译 camera_adapter.cpp → camera_adapter 可执行文件。
#
# 依赖（板卡上应已具备，UnitreecameraSDK 的 example 也依赖它们）：
#   · UnitreecameraSDK：头文件 + 静态库（官方仓库 clone 到板卡后 make 生成 lib/arm64/*.a）
#       https://github.com/unitreerobotics/UnitreecameraSDK
#   · OpenCV4（板卡自带，UnitreecameraSDK 依赖它；.13/.15 apt 装，.14 装在 /usr/local）
#   注：本 adapter 已不走 gstreamer（改 JPEG-over-UDP），注释里的 GStreamer 依赖已不需要。
#
# 用法（在板卡上，SDK 路径按实机调整）：
#   .13:  export UNITREE_CAMERA_SDK=/home/unitree/UnitreecameraSDK        && bash build_adapter.sh
#   .14:  export UNITREE_CAMERA_SDK=/home/unitree/Unitree/sdk/UnitreeCameraSdk && bash build_adapter.sh
#   .15:  export UNITREE_CAMERA_SDK=/home/unitree/Unitree/sdk/UnitreeCameraSdk && bash build_adapter.sh
#   （默认 SDK_DIR=/home/unitree/UnitreecameraSDK；.14/.15 的 SDK 在 ~/Unitree/sdk/UnitreeCameraSdk）
#   产物路径可改：OUT=./camera_adapter-left bash build_adapter.sh
#
# 产物：${OUT}（默认 ./camera_adapter）
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

# OpenCV 编译/链接参数。三块板 opencv4 布局不一：
#   · .13/.15：apt 装的 /usr/include/opencv4 + /usr/lib/aarch64-linux-gnu，pkg-config opencv4 可用。
#   · .14：opencv4 装在 /usr/local（头 /usr/local/include/opencv4、库 /usr/local/lib），无 .pc 文件 →
#     pkg-config 失败，必须显式 -I/-L + -Wl,-rpath,/usr/local/lib 才能在运行时找到 libopencv_*.so.4.1。
# 顺序：先 pkg-config opencv4 → 再 pkg-config opencv → 再 /usr/local/include/opencv4 + /usr/local/lib
#   → 最后 /usr/include/opencv4 兜底。
OPENCV_CFLAGS=""
OPENCV_LIBS=""
OPENCV_RPATH=""
if pkg-config --exists opencv4; then
  OPENCV_CFLAGS="$(pkg-config --cflags opencv4)"
  OPENCV_LIBS="$(pkg-config --libs opencv4)"
elif pkg-config --exists opencv; then
  OPENCV_CFLAGS="$(pkg-config --cflags opencv)"
  OPENCV_LIBS="$(pkg-config --libs opencv)"
elif [ -f /usr/local/include/opencv4/opencv2/opencv.hpp ]; then
  # .14 路径：头/库都在 /usr/local，无 pkg-config；显式链 + rpath 锁定运行时库路径。
  OPENCV_CFLAGS="-I/usr/local/include/opencv4"
  OPENCV_LIBS="-L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -lopencv_highgui -lopencv_videoio"
  OPENCV_RPATH="-Wl,-rpath,/usr/local/lib"
elif [ -f /usr/include/opencv4/opencv2/opencv.hpp ]; then
  OPENCV_CFLAGS="-I/usr/include/opencv4"
  OPENCV_LIBS="-L/usr/lib/aarch64-linux-gnu -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d -lopencv_highgui -lopencv_videoio"
else
  echo "[adapter] ✗ 找不到 opencv4（pkg-config 与 /usr/local、/usr 均无）；请先装 opencv4。"
  exit 1
fi

# UnitreecameraSDK 静态库在 lib/arm64（三个 .a：libunitree_camera / libtstc_V4L2_xu_camera /
# libsystemlog / libudev）。静态库有循环依赖 → 必须 -Wl,--start-group ... --end-group 让链接器
# 多趟解析。libudev 通常系统已有；若 SDK thirdparty 自带 udev 头/库可一并 -L$SDK/thirdparty。
SDK_INC="-I${SDK_DIR}/include -I${SDK_DIR}/thirdparty"
SDK_LIBDIR="${SDK_DIR}/lib/arm64"
if [ ! -d "${SDK_LIBDIR}" ]; then
  # 兼容个别 SDK 把库直接放 lib/（非 arm64 子目录）的情况。
  SDK_LIBDIR="${SDK_DIR}/lib"
fi
SDK_LIB="-L${SDK_LIBDIR} -Wl,--start-group -lunitree_camera -ltstc_V4L2_xu_camera -lsystemlog -ludev -Wl,--end-group"

# 确保产物目录存在（nano_bootstrap.sh 传 OUT=$SDK/bins/camera_adapter 时 bins 可能还没建）。
mkdir -p "$(dirname "${OUT}")"

set -x
g++ -O2 -std=c++14 -pthread \
    camera_adapter.cpp \
    ${SDK_INC} \
    ${SDK_LIB} \
    ${OPENCV_CFLAGS} ${OPENCV_LIBS} \
    ${OPENCV_RPATH} \
    -o "${OUT}"
set +x

echo "[adapter] 产物: $(ls -la "${OUT}")"
echo "[adapter] 冒烟自检（不连相机，仅验证可执行 + 控制口能起）："
echo "  ./camera_adapter --device-id 1 --device-node /dev/video0 --control-port 9301 &"
echo "  printf '{\"cmd\":\"probe\",\"device_id\":1}\\n' | nc 127.0.0.1 9301   # 期望收到一行 JSON"
