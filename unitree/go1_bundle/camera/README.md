# camera_depth — NX 端准备(深度推流的上游)

`camera_depth.py` 卡在 go1_bundle 容器(树莓派,py3.10+rclpy)里充当 **ROS2 桥**:
连接头部 Jetson NX 上的 `depth_stream`(TCP:9101),把每帧深度 PNG 发布成
`sensor_msgs/CompressedImage` 到 `/<ns>/camera/depth`,Agent Core 订阅即可在画布看深度流。

> 深度计算必须在**头部 NX**(相机 `/dev/video0` 在那),不能在树莓派。本目录的
> `depth_stream.cc` 就是 NX 端持续推流程序;下面是一次性准备步骤。

## 架构
```
[NX: depth_stream —— 相机常开→UnitreeCameraSDK立体深度→TCP:9101 推 PNG 帧]
        ↓  网络 192.168.123.15 → 树莓派
[go1_bundle 容器: camera_depth.py —— rclpy 桥,收帧→发 CompressedImage]
        ↓  DDS(ROS_DOMAIN_ID=42)
[Agent Core 订阅 /<ns>/camera/depth → 画布"查看数据流"]
```

## 在 NX 上编译 depth_stream(一次)
NX = 头部 Jetson NX,`ssh unitree@192.168.123.15`(经树莓派内网),SDK 在
`/home/unitree/Unitree/sdk/UnitreeCameraSdk`(自带预编译 arm64 静态库 + OpenCV)。

```bash
# 把 depth_stream.cc 放进 SDK examples/ 并加编译目标
cp depth_stream.cc ~/Unitree/sdk/UnitreeCameraSdk/examples/
printf '\nadd_executable(depth_stream ./depth_stream.cc)\ntarget_link_libraries(depth_stream ${SDKLIBS})\n' \
  >> ~/Unitree/sdk/UnitreeCameraSdk/examples/CMakeLists.txt
# 若链接缺 -ludev:sudo ln -sf /lib/aarch64-linux-gnu/libudev.so.1 /usr/lib/aarch64-linux-gnu/libudev.so
cd ~/Unitree/sdk/UnitreeCameraSdk/build && cmake .. && make depth_stream
```

## 运行 depth_stream(每次验收前)
相机默认被狗自带 `point_cloud_node` 占用,先释放再起:
```bash
sudo fuser -k /dev/video0; sleep 1
cd ~/Unitree/sdk/UnitreeCameraSdk
./bins/depth_stream stereo_camera_config.yaml 9101
# 打印 "监听 0.0.0.0:9101 ..." 后保持运行;go1_bundle 容器起来后 camera_depth 会自动连上
```
(测完 `sudo reboot` NX 可恢复狗自带点云。)

## 验证
容器内 `camera_depth` 卡 `action=info` 会返回 `connected_to_nx` / `frames_published`;
或在容器里 `ros2 topic hz /<ns>/camera/depth` 看到 ~10Hz。
