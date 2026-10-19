# Unitree G1 Driver

## 驱动概述

本驱动将 Unitree G1 的运动、手臂、音频、视觉和状态能力接入 Phanthy Motus Agent Core。驱动通过 MCP HTTP 提供卡片，通过 ROS 传输卡片数据，通过 Unitree SDK 访问本体服务。

默认 MCP 地址为 `http://localhost:15701/mcp`。驱动启动后向 Agent Core 注册；默认 Core 地址为 `https://localhost:15678`，可通过 `AGENT_CORE_URL` 指定。

## 能力与卡片

默认配置见 [config.yaml](config.yaml)，市场卡片清单见 [driver.yaml](driver.yaml)。默认启用表示驱动会加载对应功能，实际可用性取决于设备和本体服务。

| 能力 | 卡片 | 默认状态与用途 |
|---|---|---|
| 运动与模式 | `loco`、`switch_mode`、`switch_mode_expert`、`motion_switcher`、`smart_motion` | 启用；运动指令、模式切换和打断控制 |
| 手臂动作 | `arm` | 启用；厂商预设手势和收臂 |
| 双臂遥操 | `teleop_control` | 启用；接收设备输入，控制原厂 G1_23 固定假手双臂，提供对齐反馈及轨迹录制/回放；见[遥操使用说明](motion/README.md#使用) |
| 网页录制技能 | `g1_teaching` | 复用双臂遥操的录制和回放；在原执行卡完成检查、录制、试运行并保存普通 Skill；见[接入说明](../../common/teaching/README.md#宇树-g1-的使用流程) |
| 连续控制 | `servo`、`servo_eef` | 关闭；分别接收关节目标和末端位姿目标，按需启用 |
| 音频与灯光 | `mic`、`tts`、`speaker`、`led` | 启用；麦克风输入、文本播报、音频播放和 LED 控制 |
| 视觉 | `camera_rgb`、`camera_depth`、`camera_distance`、`vision_capture` | 启用；RealSense 图像、距离、拍照和录像 |
| 雷达与空间服务 | `lidar_cloud`、`controlled_spatial`、`controlled_spatial_map` | 启用；点云、厂商 SLAM 空间操作及地图数据 |
| 本体状态 | `loco_state`、`loco_motion_state`、`motion_events`、`posture`、`imu`、`battery`、`joints`、`mainboard`、`model` | 启用；运动状态、姿态、电池、关节和模型数据 |
| 外接设备 | `ext_mic`、`ext_camera` | 启用；外接麦克风和相机 |

旧 `slam` 插件和独立 `motion_control` 入口默认关闭。通常使用空间服务卡片和 `teleop_control`；不要同时启用 `motion_control` 与 `teleop_control`。

## 构建

G1 使用 `unitree/g1/Dockerfile`，从 ROS 基础镜像安装系统库、数值依赖、Pinocchio wheel 和 CycloneDDS，再复制业务源码。依赖步骤位于源码 COPY 之前，正常构建可复用 Docker 缓存。

在仓库根目录执行本地 ARM64 构建。与现有 build.sh 一样，构建上下文需要包含共享 common 目录：

```sh
G1_BUILD_CONTEXT=$(mktemp -d)
cp -R unitree/g1/. "$G1_BUILD_CONTEXT/"
cp -R common "$G1_BUILD_CONTEXT/common"
docker build --platform linux/arm64 \
  --build-arg PYPI_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple/ \
  -t local/g1:dev "$G1_BUILD_CONTEXT"
```

`ROS_BASE_IMAGE` 可指定 ROS 基础镜像，`PYPI_MIRROR` 可指定 Python 包索引。原有 `./build.sh unitree/g1` 入口保持可用，但该脚本会按 `.env` 配置推送镜像。

### 重新生成 Pinocchio wheel

普通 PyPI `pin` wheel 不包含所需 CasADi 绑定。Dockerfile 通过 `PIN_WHEEL_URL` 直接安装预编译 wheel，适用于 Linux ARM64、Python 3.10 和 glibc 2.35 及以上版本。已有依赖保持不变，pip 使用 `--no-deps`；安装后沿用版本和 CasADi 模型检查。开发环境先导入 `casadi`，再导入 `pinocchio.casadi`；缺失绑定时明确报错。

需要重新生成 wheel 时，在已安装相同数值依赖的 Linux ARM64 环境执行：

```sh
CMEEL_JOBS=1 CMEEL_RUN_TESTS=OFF \
CMEEL_CMAKE_ARGS="-DBUILD_WITH_CASADI_SUPPORT=ON -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -Dcasadi_DIR=/usr/local/lib/python3.10/dist-packages/casadi/cmake" \
python3 -m pip wheel --no-cache-dir --no-build-isolation --no-deps --no-binary=pin \
  --wheel-dir /tmp/pin-wheel pin==4.1.0
```

上传 wheel 后，将 `PIN_WHEEL_URL` 设为实际下载地址，再构建 G1 镜像。`CMEEL_RUN_TESTS=OFF` 仅关闭依赖上游的打包测试，项目回归须单独运行。

## 安装与启动

在 Agent Core Web 界面的部署页选择 Unitree G1 Driver 和已发布版本，完成部署。驱动注册后，在 Canvas 中添加所需卡片。通用操作见[仓库部署说明](../../README.md#deploy-via-web-dashboard-recommended)。

本地开发需要先具备 ROS Humble、项目 ROS 消息包、Unitree SDK 和 G1 运行依赖。准备方法见[运行环境与构建说明](#构建)。在仓库根目录执行：

```sh
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
python3 unitree/g1/main.py eth0
```

将 `eth0` 替换为连接机器人网络的实际网卡。将 ROS 消息包的 setup 路径替换为本机工作空间路径。启动前配置 ROS 环境，要求见“运行配置与限制”。

驱动默认读取同目录的 `config.yaml`；使用 `CONFIG_PATH` 可指定其他配置文件。`mcp_port` 设置 MCP 端口；`ros_namespace` 为空时使用规范化后的主机名。

## 运行配置与限制

- ROS 使用 FastDDS、domain `42` 和 `/opt/phanthy-motus/dds-local.xml`。部署模板设置 `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION` 和 `FASTRTPS_DEFAULT_PROFILES_FILE`；本地运行也需提供这些环境变量及配置文件。
- Unitree SDK 使用 CycloneDDS、domain `0` 和启动参数指定的机器人网卡。容器入口默认使用 `eth0`，可通过 `NETWORK_INTERFACE` 修改。
- [部署模板](deploy/service.yml)使用宿主网络并挂载设备和持久数据目录。相机、麦克风、雷达和空间操作分别依赖对应设备或厂商服务；卡片注册成功不代表已收到传感器数据。
- `servo` 和 `servo_eef` 默认关闭。启用前核对关节布局、夹爪和腰部配置，并阅读对应模块说明。驱动没有统一的多控制器仲裁，不要让多个控制源同时控制同一组关节。
- SDK 请求成功与实测动作完成是不同状态。检查返回结果和本体反馈；软件检查不能替代物理动作验收。

## 开发与验证入口

- [仓库构建流程](../../CONTRIBUTING.md#building-docker-images)：业务镜像构建与发布入口。
- [G1 专项说明](motion/README.md)：遥操使用、运行依赖、数值实现和隔离验证。
- [关节连续控制](servo.py)与[末端连续控制](servo_eef.py)：输入契约、硬件配置和执行限制。
- [G1 测试目录](tests/)：音频、视觉、状态和遥操测试；[共享测试目录](../../tests/)另含 G1 SDK、连续控制和导航测试。

测试环境要求和结果边界以对应测试说明为准。离线测试通过不代表设备、DDS 通信或机器人动作已验收。
