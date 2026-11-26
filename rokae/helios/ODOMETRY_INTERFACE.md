# Helios 里程计原始位姿和速度

Helios `loco_state` 参考天轶同名卡片的双输出。显示话题为 `/{namespace}/loco/state`，
提供 `position`、`rpy`、`velocity`、`yaw_speed`、坐标系、可用状态及两路接收时间。
计算话题为 `/{namespace}/state/odom`，载体是 `std_msgs/String` JSON。

| 数据 | 来源 | 时间与坐标 |
|---|---|---|
| 位置 x/y 和 yaw | VMR SDK `odometry.odom_state` | 原厂里程计坐标，x/y 为米、yaw 为弧度。Driver 接收时间填入 `pose.stamp_ms`，原厂 `timestamp_ns` 独立保存。 |
| vx/vy/wz | 底盘 WebSocket `UiDatagram.current_state.speed` | 当前机体系，m/s 和 rad/s。Driver 接收时间填入顶层 `stamp_ms`，底盘源时间独立保存。 |

状态流保留 `/{namespace}/odom → /{namespace}/chassis_planar` 的坐标名称含义，
实际 `frame_id` 值不带起始斜杠。标准 `pose` 的 z=0 和 yaw 四元数用于表达平面模型，
显示流未测量的 z、roll、pitch 用 null 表达。原厂 SDK 短时断开期间 pose 为空。
恢复后继续报告底盘给出的坐标。画布停卡只更新卡片生命周期状态，Driver 继续观察。

速度侧保持 `motus.odom/1` 的 `frame=body`、`provides=["vx","vy","wz"]` 以及
`twist=[vx,vy,null,null,null,wz]`。静止实测零与缺失轴分别表示。来自 WebSocket 的
地图定位字段不参与 `pose`，因此速度可独立于 SDK 运行。跨传感器时间对齐需要单独验收。

## SDK 运行环境

正式 Driver 镜像包含 Python 适配代码。原厂 SDK 和匹配其 ABI 的 Python 随现场环境
配置。通过 [compose.native-odom.yml](deploy/compose.native-odom.yml) 将本机的
`HELIOS_SDK_PYTHON_DIR` 和 `HELIOS_VMR_SDK_DIR` 只读挂载为 `/opt/sdk-python` 和
`/opt/vmr-sdk`。所需入口为 `/opt/sdk-python/bin/python3.12` 和 `/opt/vmr-sdk/lib`。
Compose 覆盖文件需要与当前 Driver 服务 Compose 一起应用。子进程使用独立的 Python
解释器调用 SDK，ROS 2 Driver 继续使用镜像原有的 Python。默认连接底盘
`192.168.0.100:9003`，可在 `config.yaml` 的 SDK host/port 中配置。

SDK 未安装时，`loco/state` 显示 `pose_state=sdk_unavailable`，标准 `pose=null`。
速度继续通过 WebSocket 发布。SDK 安装但暂时收不到有效位姿时显示 `waiting`。

参考 [天轶 loco_state](https://github.com/4paradigm/phanthymotus-driver/blob/main/x-humanoid/tianyi2.0/loco_state.py)、
[统一里程计格式](../../README_dev.md#robot-odometry-motusodom1) 和
[PR #519](https://github.com/4paradigm/phanthymotus-driver/pull/519) 的两路时间定义。
