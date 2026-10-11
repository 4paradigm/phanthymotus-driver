# PiPER 单臂 + D435 功能卡片（草稿）

把单机 Python 驱动封装为 Phanthy Motus 的 MCP 卡片。硬件为单台 PiPER、
USB-CAN（SocketCAN，1 Mbps）和安装在夹爪上方的 Intel RealSense D435。
目标中控器为 Ubuntu 22.04 **x86_64**、ROS2 Humble、Python 3.10。

## 三张卡片

| 卡片 | 类型 | 功能 |
| --- | --- | --- |
| `piper_status` | resource | 按需读取六关节角度、使能、控制模式和故障码 |
| `piper_arm` | actuator | 初始化预览/执行、受限 J1 转动、保存/返回默认停放位置、解除使能 |
| `piper_photo` | actuator | 当前姿态拍摄 D435 RGB 照片，保存 JPEG + JSON，并发布 `image/jpeg` |

`piper_photo` **不移动机械臂**。相机随腕部运动；拍摄“正前方”前需现场确认
当前姿态和取景，再对这一次请求传 `forward_view_confirmed=true`。
该参数记录人的确认，不代表视觉算法验证或永久标定。默认返回 `false`。
本版本不提供深度图、自动抬头、夹爪动作、IK 或碰撞规划。

## 已验证与待验收

2026-10-10 的单机前身已在现场观察到低速 J1 约 5° 转动及返回，
此前也观察到约 10° 转动；初始化和可靠支撑下解除使能已做现场检查。
**这些记录不等于新卡片的运动验收。** 卡片新增了互斥、取消和参数检查，
需要重新通过平台进行有人值守的运动验收。

本分支已在目标 Ubuntu 上经新卡片读取到正常反馈、六关节未使能，
并拍到 640×480 彩色 JPEG（80 帧预热，对比度约 57.4）。此检查没有发运动指令。
同机 ROS2 `CompressedImage` 发布/订阅检查收到 42,149 字节 JPEG。
完整 Agent Core 画布、容器镜像和新卡片运动流程仍待联调。

## 控制边界

- 启动和 MCP `start` 不连接、不使能、不运动；首次读取才连接 CAN，
  显式使用 `ConnectPort(piper_init=False)`，避免 SDK 隐式初始化。
- `PIPER_MOTION_ENABLED` 默认 `0`。部署人员完成现场准备后才设为 `1`。
  每次运动还要 `workspace_clear=true`，J1 移动还需 `take_control=true`。
- `prepare` 默认仅预览。要求全部电机未使能、非示教状态；最多 3° 的
  SDK 边界修正必须预览并单独允许。先预置反馈角度，再使能并验证稳定。
- 单次 J1 移动 0.1–12°，绝对目标 ±140°，卡片速度 1–10%，默认 5%。
  其它关节保持测量角度；不支持多关节路径规划。成功后保持使能。
- 共享锁拒绝并发的机械臂动作和腕部相机拍照；不排队重放旧动作。
- `stop` 请求取消并尽力保持新鲜反馈角度，**不会失能或断电，也不是急停**。
  停止时返回 `stopping` 表示当前调用尚在退出；完成前拒绝重新 `start`。
  CAN 中断时软件无法保证保持，现场仍需可用的物理断电/急停措施。
- `disable` 必须每次显式 `arm_supported=true`。默认位置不等于已支撑，
  保存文件不会授予后续自动解除使能的权限。
- 反馈超时、模式改变、部分使能、关节越界会拒绝/终止操作。
- MCP 复用仓库 HTTP runtime，未增加独立认证。仅在受信的机器人网络上部署，
  不将 15741 端口暴露到公网；上层平台应保留 actuator 确认。

## 默认位置与返回

`save_default` 保存**当前反馈**到部署数据目录 `default_pose.json`，不会
覆盖 SDK 零点，不会执行厂家校准。仓库没有预置某台设备的私有默认角度。

`return_default` 默认只预览；仅当 J2–J6 已在保存角度 ±1° 内，且 J1
回程不超过 12° 时允许执行。执行只恢复 J1，其余关节保持当前反馈值。
不能把任意姿态一键恢复到六关节零位；超出条件会直接报错。

## Ubuntu 本机运行

先由部署人员确认 `can0` 已按硬件要求配置为 **1 Mbps、正常模式**，
而非 listen-only。卡片不使用 sudo、不改变系统 CAN 配置。
只允许一个控制程序拥有这条总线，停止旧的运动脚本后再运行。

```bash
# 在仓库根目录；机器需要 ROS2 Humble。无需在普通 Mac 安装 ROS。
python3 -m pip install -r agilex/piper/requirements.txt
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD:${PYTHONPATH:-}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml
python3 agilex/piper/main.py
```

先准备平台提供的 DDS profile；ROS namespace、CAN 接口、彩色 V4L2 路径、
数据目录可在主机自己的 `config.yaml` 配置，或用 `CONFIG_PATH` 指定外部副本。
多相机时必须选择彩色设备的 `/dev/v4l/by-id/...` 路径；不要依赖 `/dev/videoN`
枚举顺序。相机 V4L2 读取在子进程中运行，USB 卡死会在预热时间 +8 秒超时，
并释放设备句柄。照片存在主机持久化目录；需要部署方按容量清理旧照片。

## MCP 调用示例

所有例子发送到本机 `http://localhost:15741/mcp`，JSON-RPC 2.0：

```json
{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"piper_status","arguments":{}}}
```

初始化先预览，核对返回的修正量再决定是否执行：

```json
{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"piper_arm","arguments":{"action":"prepare"}}}
```

当前视角拍照（成功返回路径、尺寸、时间、是否人工确认前方、DDS topic）：

```json
{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"piper_photo","arguments":{"action":"capture"}}}
```

在平台卡片中选择动作时，`x-action-params` 给出对应参数。动作同步返回，
最慢 J1 操作约 22 秒；客户端超时不能假定机器人已停止，应显式调用 `stop`
并核对反馈。照片消息为 ROS2 `sensor_msgs/CompressedImage`，保留原拍摄时间，
每秒重发最近一张以支持后加入的画布；这不是实时视频。

## Docker 与架构

本目录 Dockerfile 使用支持 amd64/arm64 的官方 ROS Humble 基础镜像。
**仓库 `build.sh` 目前强制 `linux/arm64`，不能把其镜像直接部署到本次 x86_64 中控器。**
对 x86_64 使用独立构建脚本（打包 common/，不修改仓库统一构建逻辑）：

```bash
bash agilex/piper/build-local.sh agilex-piper:local
# 需要 ARM64 时可指定 PIPER_BUILD_PLATFORM=linux/arm64
```

`deploy/service.yml` 由平台合并，依赖平台注入 host 网络模式并提供 DDS profile。
数据卷持久化照片和默认姿态。重启不会自动使能机械臂。
本次没有运行 Docker daemon，因此未宣称镜像构建/部署成功；review bot
若只提供 ARM64 镜像，需要维护者提供 amd64 构建后再做本机平台验收。

## 测试与后续验收

```bash
python3 -m unittest discover -s tests -p 'test_piper*.py' -v
```

测试包含 MCP HTTP handshake/list/call、严格参数、资源隔离、并发拒绝、
取消与停止竞态、拍照取消、默认位置范围、初始化拒绝路径及 J1 模拟反馈。
22 项 PiPER 测试通过；连同 Docker COPY 检查，共 68 项测试通过。
测试不会连接机械臂。另行运行仓库部署重启检查时，有 2 项在上游已有的
`ubtrobot/u1_pro/deploy/service.yml`（`restart: unless-stopped`）失败；
本目录为 `restart: always`，没有修改其它设备配置。

验收顺序：画布发现三张卡片 → 读取反馈 → 查看 RGB 照片 → 现场准备与
初始化预览 → 显式初始化 → 保存默认位置 → 小幅 J1 转动 → 返回默认位置
→ 可靠支撑后解除使能。取消、断开 USB 和错误恢复也需验证。
由导师及对应机器人负责人完成平台验收后，再考虑将草稿转为正式 PR。
