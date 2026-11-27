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
| 右臂方向示意 | `g1_directional_pointing` | 默认关闭；G1_23 固定右手按机身相对角度指向，左臂保持本次实测起始姿态。 |
| 双臂遥操 | `teleop_control` | 启用；接收设备输入，控制原厂 G1_23 固定假手双臂，提供对齐反馈及轨迹录制/回放；见[遥操使用说明](motion/README.md#使用) |
| 网页录制技能 | `g1_teaching` | 复用双臂遥操的录制和回放；在原执行卡完成检查、录制、试运行并保存普通 Skill；见[录制技能说明](skill_workbench/README.md) |
| 连续控制 | `servo`、`servo_eef` | 关闭；分别接收关节目标和末端位姿目标，按需启用 |
| 音频与灯光 | `mic`、`tts`、`speaker`、`led` | 启用；麦克风输入、文本播报、音频播放和 LED 控制 |
| 视觉 | `camera_rgb`、`camera_depth`、`camera_distance`、`vision_capture` | 启用；RealSense 图像、距离、拍照和录像 |
| 雷达与空间服务 | `lidar_cloud`、`controlled_spatial`、`controlled_spatial_map` | 启用；点云、厂商 SLAM 空间操作及地图数据 |
| 本体状态 | `loco_state`、`loco_motion_state`、`motion_events`、`posture`、`imu`、`battery`、`joints`、`mainboard`、`model` | 启用；运动状态、姿态、电池、关节和模型数据 |
| 外接设备 | `ext_mic`、`ext_camera` | 启用；外接麦克风和相机 |

旧 `slam` 插件默认关闭。独立 `motion_control` 卡片入口已移除；双臂遥操使用 `teleop_control`。旧配置显式启用 `motion_control` 时，驱动会报迁移错误。

## G1_23 右臂方向示意

`g1_directional_pointing` 属于 driver 层，将偏航角、俯仰角转换为 G1_23 五关节右臂的指向动作。与 `arm` 的厂商预设手势相比，它允许每次输入不同方向；与末端位姿控制相比，它不要求位置或完整末端姿态。仅适用于原厂五关节手臂和固定假手。方向定义在 `torso_link`：X 向前、Y 向左、Z 向上，随躯干朝向变化。

| Action | 参数 | 结果 |
|---|---|---|
| `point` | `yaw_deg`、`pitch_deg`、可选 `hold_s`（0.1–5 秒，默认 1 秒） | 校验并规划后返回 `accepted` 和唯一 `action_id`，执行终态由 ACP 返回。 |
| `stop` | 本次 `point` 返回的 `action_id` | 请求停止并等待交权；尚未确认时返回 `stopping`，不能当作停止完成。 |

画布例如填写偏航 `20`、俯仰 `20`、保持 `1`；偏航 `+90` 表示机器人左侧，`-90` 表示右侧。输入范围为偏航 `[-180, 180]`、俯仰 `[-90, 90]`，并不代表所有方向都能执行。有限 IK 搜索、原始关节硬限位和整段关节插值路径检查仍可能拒绝请求；搜索失败也不证明机器人在物理上完全不可达。

每次从新鲜实测姿态规划，执行期间占用 `arm_l`、`arm_r`，左臂保持接管时的实测关节角。右臂获得控制权且目标指令已经下发后，反馈计算的指向误差须连续不超过配置阈值（默认 5°）满 `hold_s`；随后沿原指令路径返回本次起点并确认交权，才报告 `completed`。保持完成不等于整项任务已经结束；回程、停止或交权失败仍为 `error`。完成后可以填写新角度再次调用。

默认 `joint_speed_rad_s: 1.0` 是关节指令变化率上限，沿用共享执行器的平滑与限速机制，不承诺实际关节速度等于该数值或等于厂商预设手势速度。模式门控要求新鲜的常规运控状态 `fsm_id=500`、`fsm_mode=0/1`，不再用底座静止、腰速或连续静止时长作准入条件。因此接口可在该模式的行走过程中调用，但本卡不控制行走，也不证明动态全身动作或外部场景的安全性。

**部署后显示卡片。** 新镜像包含模块，默认配置仍关闭该卡。先在实际加载的完整配置中设置 `plugins.g1_directional_pointing.enabled: true`，保留 `solver_mode: bounded_ik`、`left_arm_mode: hold_measured`。同时须已有 `arm`、`loco`、共享 `teleop_control` 启用，遥操模式为 `live`，校准配置指向与本体一致的 `motion/body23_fixed_hand.json`；独立 `servo/servo_eef` 保持关闭。按部署流程重新加载驱动后，确认 `tools/list` 包含 `g1_directional_pointing`，再刷新画布并手动拖入。切勿用 point 局部配置覆盖整份机器人配置。

`ik_reference_q_rad` 和兼容字段 `verified_start_q_rad` 是五关节弧度的 IK 参考/回退种子，不是每次调用必须返回的固定准入姿态。生产调用始终读取当次左右臂反馈。`receipt_directory` 可配置为持久卷上的绝对目录，保存按动作 ID 索引的完整驱动回执。具体启用、验收和失败定位见[现场验收说明](directional_pointing_validation.md)。

**能力边界。** 指示轴是腕关节原点到固定手 TCP 的模型方向，不是某根柔性手指的实际朝向。ACP 的 `actual_direction`、`angular_error_deg` 来自实测关节角的正向运动学，`external_accuracy: pending` 表示外部指向精度尚待测量。左右臂几何检查使用真实左右链和碰撞覆盖；名义路径固定左臂启动姿态，运行中逐反馈样本检查实际两臂。3 mm 是模型包络间距，不是量具认证的物理净空，也不覆盖反馈间运动或停止距离。本卡不识别目标、不导航、不检测环境障碍。几何来源与复现方法见[几何证据](directional_geometry/README.md)。

## 构建

G1 使用 `unitree/g1/Dockerfile`，默认从 `bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest` 安装系统库、数值依赖、Pinocchio wheel 和 CycloneDDS，再复制业务源码。普通 pip 依赖统一维护在 `requirements.txt`，Dockerfile 在编译 CycloneDDS 库后通过 `pip install -r` 安装，包括从源码安装其 Python 绑定。依赖步骤位于业务源码 COPY 之前，正常构建可复用 Docker 缓存。

在仓库根目录执行本地 ARM64 构建。与现有 build.sh 一样，构建上下文需要包含共享 common 目录：

```sh
G1_BUILD_CONTEXT=$(mktemp -d)
cp -R unitree/g1/. "$G1_BUILD_CONTEXT/"
cp -R common "$G1_BUILD_CONTEXT/common"
docker build --platform linux/arm64 \
  --build-arg PYPI_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple/ \
  -t local/g1:dev "$G1_BUILD_CONTEXT"
```

非 Docker 环境安装 requirements 前，也须先安装 CycloneDDS 0.10.5 库并设置 `CYCLONEDDS_HOME`；之后仍须按下一节安装带校验的定制 Pinocchio wheel。

`ROS_BASE_IMAGE` 可指定 ROS 基础镜像，`PYPI_MIRROR` 可指定 Python 包索引。原有 `./build.sh unitree/g1` 入口保持可用，但该脚本会按 `.env` 配置推送镜像。

### 重新生成 Pinocchio wheel

普通 PyPI `pin` wheel 不包含所需 CasADi 绑定。Dockerfile 通过 `PIN_WHEEL_URL` 单独安装预编译 wheel，并使用 pip 原生 URL 哈希校验 `#sha256=${PIN_WHEEL_SHA256}`；内容不匹配时构建失败。该 wheel 适用于 Linux ARM64、Python 3.10 和 glibc 2.35 及以上版本。已有依赖保持不变，pip 使用 `--no-deps`；安装后沿用版本和 CasADi 模型检查。开发环境先导入 `casadi`，再导入 `pinocchio.casadi`；缺失绑定时明确报错。

需要重新生成 wheel 时，在已安装相同数值依赖的 Linux ARM64 环境执行：

```sh
CMEEL_JOBS=1 CMEEL_RUN_TESTS=OFF \
CMEEL_CMAKE_ARGS="-DBUILD_WITH_CASADI_SUPPORT=ON -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -Dcasadi_DIR=/usr/local/lib/python3.10/dist-packages/casadi/cmake" \
python3 -m pip wheel --no-cache-dir --no-build-isolation --no-deps --no-binary=pin \
  --wheel-dir /tmp/pin-wheel pin==4.1.0
```

当前默认 SHA-256 为 `bde8d1726200b93ee2fd291128ccc97d32123db6e1b49dfaa4b311cbfcaa28ef`，来源是2026-10-12已验证的 Linux ARM64 构建产物及当日构建记录；不是以当前下载内容重新确定的期望值。

重新发布时，先在验证后的原始构建产物上计算 `sha256sum` 并保存构建记录，再上传。构建时同时设置 `PIN_WHEEL_URL`（不带 URL fragment）和 `PIN_WHEEL_SHA256`。只换下载地址仍会使用原哈希并拒绝不同内容。`CMEEL_RUN_TESTS=OFF` 仅关闭依赖上游的打包测试，项目回归须单独运行。

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
- G1 的 `LowStateSource` 独占 `rt/lowstate` 订阅，通过独立线程读取和解码，使用 `KeepLast(1)`。状态插件与手臂执行器共享缓存；重复或延迟样本不会刷新源数据时效。停止时先等待读取线程退出，再关闭订阅。
- [Driver SDK 适配层](sdk_adapter.py)组合 SDK 既有 `ChannelFactory.CreateChannel` / `Channel.SetReader` 接口，不修改 SDK、不创建额外 DDS participant。LowState、执行器 FSM 与 `rt/arm_sdk` 观察订阅均保留最新一帧；需要回调的反馈由 Driver 自有线程执行有界读取，停止后 join 并只关闭底层 reader 一次。其他 SDK 订阅和 RPC 回复仍沿用 SDK 回调与可选队列机制；适配层仅过滤自身读取作用域内由 `StopIteration` 产生的空读告警，其他线程及真实异常日志保留。SDK 读异常可能只返回空值，安全判断仍依赖反馈时效，不能以订阅存在证明反馈有效。
- 动作 99 的 no-reply 发送封装位于 Driver 适配层，发送成功不代表姿态完成；不改变现有 Reset/停止/交权状态机，也不让原本不发 99 的操作新增动作。
- [部署模板](deploy/service.yml)使用宿主网络并挂载设备和持久数据目录。相机、麦克风、雷达和空间操作分别依赖对应设备或厂商服务；卡片注册成功不代表已收到传感器数据。
- `servo` 和 `servo_eef` 默认关闭。启用前核对关节布局、夹爪和腰部配置，并阅读对应模块说明。驱动没有统一的多控制器仲裁，不要让多个控制源同时控制同一组关节。
- SDK 请求成功与实测动作完成是不同状态。检查返回结果和本体反馈；软件检查不能替代物理动作验收。

## 开发与验证入口

- [仓库构建流程](../../CONTRIBUTING.md#building-docker-images)：业务镜像构建与发布入口。
- [G1 专项说明](motion/README.md)：遥操使用、运行依赖、数值实现和隔离验证。
- [关节连续控制](servo.py)与[末端连续控制](servo_eef.py)：输入契约、硬件配置和执行限制。
- [G1 测试目录](tests/)：音频、视觉、状态和遥操测试；[共享测试目录](../../tests/)另含 G1 SDK、连续控制和导航测试。

测试环境要求和结果边界以对应测试说明为准。离线测试通过不代表设备、DDS 通信或机器人动作已验收。
