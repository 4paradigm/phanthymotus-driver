# test_camera_depth — Nano 端准备(深度推流的上游)【未验收·5 机位可选】

`test_camera_depth.py` 在容器里充当 **ROS2 桥**:按卡的 `position` 配置连**对应板卡**的
`depth_stream`,把每帧彩色深度图(JPEG)发布成 `sensor_msgs/CompressedImage`(format=jpeg)到
实例专属 topic `/<ns>/camera/<机位>/depth`。深度由 SDK 的 `getDepthFrame`(彩色可视化:红近/青远)算出。
卡 start 才连、stop 就断(NX 侧 depth_stream 连上才开相机、断开即释放,空闲不占相机)。

## 5 路相机 → 板卡 / 设备 / 深度端口

| position | 板卡 IP | device_id | 深度端口 |
|---|---|---|---|
| front | 192.168.123.13 | 1 | 9101 |
| chin  | 192.168.123.13 | 0 | 9102 |
| left  | 192.168.123.14 | 0 | 9103 |
| right | 192.168.123.14 | 1 | 9104 |
| belly | 192.168.123.15 | 0 | 9105 |

> 端口 91xx 与点云 94xx、RGB 图传 92xx、RGB 控制 93xx 全部错开,与
> `config.yaml: test_camera_depth.positions` 一一对应。

## 相机初始化(关键·与点云同款)
depth_stream **按 device_id 现场生成一份最小 config** → `UnitreeCamera(config_file)` → 加载立体标定 →
`getDepthFrame` 才出帧。**不能用 `UnitreeCamera(device_id)` 设备号构造**(无标定 → 连上无帧,
点云 5c8000e 已踩此坑并修复)。一份二进制服务任意一路,免外部 config 文件。

## 自动部署(无需手动编译)
容器首启时 `deploy/nano_bootstrap.sh` 会自动:探测各板 SDK 路径 → `g++` 编译 `depth_stream` →
装成每路一个**空闲 systemd 服务** `go1-depth-<机位>`(空闲不占相机)。故正常情况**无需手动编译**。

## 约束(务必知悉)
- **5 选 1、热切、免重启**:每路常驻挂一个 `depth_stream`(空闲不占相机);画布改 `position`
  → Pi 卡断旧连新 → 对应 streamer **连上才开相机、断开就释放**。切换时相机 SDK 初始化约 **3~4s**,期间无帧属正常。
- **JPEG(不是 PNG)**:画布只渲染 `image/jpeg` 的 CompressedImage(对齐 camera_rgb);彩色深度是给人看的可视化,有损无妨。
- **与点云互斥**:depth 与 pointcloud 指向同一相机时,同一 device 不能被两个进程同时打开(谁连上谁 `fuser` 顶掉对方)。
- **一次一路**:立体计算吃 Nano CPU。

## 手动编译/运行(仅调试;正常由 nano_bootstrap 自动做)
```bash
# 用法:depth_stream <port> <device_id>(按上表);SDK 路径各板不同,先探测
SDK=$(for d in $HOME/UnitreecameraSDK $HOME/Unitree/sdk/UnitreeCameraSdk; do [ -f "$d/include/UnitreeCameraSDK.hpp" ] && echo "$d" && break; done)
cp depth_stream.cc "$SDK/" && cd "$SDK" && mkdir -p bins && g++ -O2 -std=c++14 -pthread depth_stream.cc \
  -I$SDK/include -I$SDK/thirdparty -L$SDK/lib/arm64 -Wl,--start-group -lunitree_camera \
  -ltstc_V4L2_xu_camera -lsystemlog -ludev -Wl,--end-group $(pkg-config --cflags --libs opencv4) \
  -o bins/depth_stream
./bins/depth_stream 9105 0     # 例:belly(.15 dev0);启动时不开相机,等 Pi 卡连上才开
```

## 验证
容器内 `test_camera_depth` 卡 `action=info` 返回 `position` / `connected_to_nx` / `frames_published`;
或 `ros2 topic hz /<ns>/camera/<机位>/depth` 看到 ~10Hz。切换机位:改卡的 `position`(config 动作)→ 自动重连。

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
- **头部与 test_camera_depth**:若 depth_stream 与某点云路指向 `.13` 同一 device,则同一 device 不能被两个进程同时打开(互斥)。

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


