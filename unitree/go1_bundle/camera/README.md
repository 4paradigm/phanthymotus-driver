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

---

# test_camera_pointcloud — Nano 端准备(点云推流的上游)【未验收·5 机位可选】

`test_camera_pointcloud.py` 在容器里充当 **ROS2 桥**:按卡的 `position` 配置连**对应板卡**的
`pointcloud_stream`,把每帧点云发布成 `sensor_msgs/PointCloud2` 到固定 topic
`/<ns>/camera/pointcloud`(内容随选定机位切换)。点云由 SDK 的 `getPointCloud`(XYZ 米,相机系)算出。

## 5 路相机 → 板卡 / 设备 / 点云端口

| position | 板卡 IP | device_id | 点云端口 |
|---|---|---|---|
| front | 192.168.123.13 | 1 | 9401 |
| chin  | 192.168.123.13 | 0 | 9402 |
| left  | 192.168.123.14 | 0 | 9403 |
| right | 192.168.123.14 | 1 | 9404 |
| belly | 192.168.123.15 | 0 | 9405 |

> 端口 94xx 与深度 9101、RGB 图传 92xx、RGB 控制 93xx 全部错开,与
> `config.yaml: test_camera_pointcloud.positions` 一一对应。

## 约束(务必知悉)
- **5 选 1、热切、免重启**:每路常驻挂一个 `pointcloud_stream`(**空闲不占相机**);画布改 `position`
  → Pi 卡断旧连新 → 对应 streamer **连上才开相机、断开就释放**,自动切换,无需手动重启。切换时相机
  SDK 初始化约 **3~4s**,期间无帧属正常。
- **一次只读一路**:立体计算吃 Nano CPU;靠"同一时刻只连一路"避免同板两个立体计算并发压垮 CPU。
- **头部与 camera_depth**:若 depth_stream 与某点云路指向 `.13` 同一 device,则同一 device 不能被两个进程同时打开(互斥)。

## 在各板上编译 pointcloud_stream(一次)
每块要用的板(.13/.14/.15)都装一次(SDK 路径同 depth_stream):
```bash
cp pointcloud_stream.cc ~/Unitree/sdk/UnitreeCameraSdk/examples/
printf '\nadd_executable(pointcloud_stream ./pointcloud_stream.cc)\ntarget_link_libraries(pointcloud_stream ${SDKLIBS})\n' \
  >> ~/Unitree/sdk/UnitreeCameraSdk/examples/CMakeLists.txt
cd ~/Unitree/sdk/UnitreeCameraSdk/build && cmake .. && make pointcloud_stream
```

## 常驻运行 pointcloud_stream(每路一个,可开机自启;空闲不占相机)
参数:`<config(选 device)> <port(按上表)> <stride 抽稀,默认4>`。**启动时不开相机**,等 Pi 卡连上才开。
```bash
# .13:front(dev1)→9401  与  chin(dev0)→9402  可同时常驻(空闲各不占相机)
./bins/pointcloud_stream stereo_camera_config_front.yaml 9401 4 &
./bins/pointcloud_stream stereo_camera_config_chin.yaml  9402 4 &
# .14:left(dev0)→9403 / right(dev1)→9404;.15:belly(dev0)→9405 同理
```
> 本程序不自己选相机:由传入的 config(指向该板某 device)决定读哪路。.13/.14 有两路相机,
> 需各自的 config 指向 device 0 / device 1。画布同时只连一路,故 CPU 不会被两个立体计算压垮。

## 验证
容器内 `test_camera_pointcloud` 卡 `action=info` 返回 `position` / `connected_to_nx` /
`frames_published` / `last_frame_points`;或 `ros2 topic hz /<ns>/camera/pointcloud`。
切换机位:改卡的 `position` 配置(config 动作)→ 自动重连到对应板卡端口。点云重,内网吃紧就调大 stride 或降帧率。


