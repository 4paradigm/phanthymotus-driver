# 众擎 EngineAI T800 开发版 Driver

将 T800 开发版底层暴露的 ROS2 接口封装为 phanthymotus-driver 规范的能力卡片
（MCP HTTP server），供 agent-core 画布 / LLM 调用。参照 `unitree/g1` 与
`x-humanoid/tianyi2.0` 的既有模式。

- 目录：`phanthymotus-driver/engineai/t800/`
- MCP 端口：**15708**（15700–15799 区间）
- 硬件：T800 开发版（25 自由度，关节索引 0~24，语义名 `J00_HIP_PITCH_L` … `J24_HEAD_YAW`）

## 能力清单（12 张卡片）

| 卡片 (tool) | 类型 | 机器人话题 (domain 69) | 输出 (domain 42) | 说明 |
|------|------|------------------------|------|------|
| `motion_state` | sensor | `/motion/motion_state` | `/{ns}/motion/state` data/json | 当前运动状态 + 可用转换白名单 |
| `motion_switcher` | actuator | `/motion/set_motion_state` (RELIABLE) | — | 切换目标状态：白名单检查 + 3s 超时 + 状态确认 |
| `loco` | actuator | `/motion/body_vel_cmd` | — | 100Hz 持续发布；限速 x/y ±1 m/s、yaw ±1 rad/s；2s 无命令自停 |
| `arm` | actuator | `/motion/joint_motion_plan/request` + `/state` | `/{ns}/arm/state` data/json | 上肢运动规划（13 关节 [12..24]），前置 `lower_body_balance` |
| `joints` | sensor | `/hardware/joint_state` | `/{ns}/state/joints` data/json | 25 关节按部位聚合，含语义名 |
| `imu` | sensor | `/hardware/imu_info` | `/{ns}/state/imu` data/json | 四元数/RPY/加速度/角速度 |
| `power` | sensor | `/hardware/power_info` | `/{ns}/state/power` data/json | 电量/电压/电流 |
| `gamepad` | sensor | `/hardware/gamepad_keys` | `/{ns}/state/gamepad` data/json | 手柄数字键 + 模拟摇杆 |
| `led` | actuator | `/hardware/led_control` | — | 11 种模式枚举 |
| `mic` | sensor | —（ALSA 采集） | `/{ns}/mic/audio` audio/pcm-16k | 16kHz 单声道，chunk 1024~4096B，满足 ASR 协议 |
| `speaker` | actuator | —（ALSA 播放） | — | 播放 PCM/WAV/本地文件 |
| `joint_bridge` | actuator | `/hardware/joint_command` | — | 500Hz 全身 PD 直通（**config 默认关闭**），前置 `joint_bridge` 模式 |

运动状态机是所有控制卡的前置：`loco` / `arm` / `joint_bridge` 在 dispatch 时
检查当前状态，不在正确模式时返回
`{"state":"error","error":"WRONG_MOTION_STATE","message":"请先切换到 xxx 模式"}`。

T800 支持的运动状态：`idle` / `passive` / `pd_stand` / `rl_basic` /
`lower_body_balance` / `joint_bridge` / `pd_sitground` / `walk_server` /
`rl_mimic_supine_to_stance` / `rl_mimic_prone_to_stance` /
`rl_mimic_stance_to_supine` / `rl_mimic_sitdown_to_stance` /
`rl_mimic_stance_to_sitdown`。

## 架构（双 rclpy context）

```
┌────────────────────────── 机器人运控单元 (domain 69 / CycloneDDS) ──────────────────────────┐
│  /motion/motion_state   /motion/set_motion_state   /motion/body_vel_cmd                    │
│  /motion/joint_motion_plan/{request,state}         /hardware/joint_state                   │
│  /hardware/joint_command                           /hardware/{imu_info,power_info,led_control,gamepad_keys} │
└──────────────────────────────────────────┬─────────────────────────────────────────────────┘
                                           │ ctx_t800（rclpy context，executor_t800）
┌──────────────────────────────────────────┴─────────────────────────────────────────────────┐
│  engineai/t800 driver（main.py + device.py + plugins/*）                                    │
│  ├─ 传感器转发：订阅 domain 69 → 缓存 → 定时 JSON 发布到 domain 42 /{ns}/xxx                │
│  ├─ 控制类：   发布 domain 69（运动状态前置检查）                                           │
│  └─ 音频：     mic 采集/重采样/缓冲 → AudioChunk 发布到 domain 42 /{ns}/mic/audio           │
└──────────────────────────────────────────┬─────────────────────────────────────────────────┘
                                           │ ctx_core（rclpy context，executor_core）
┌──────────────────────────────────────────┴─────────────────────────────────────────────────┐
│  agent-core (domain 42 / FastDDS)  ·  dashboard 画布渲染  ·  perception ASR/TTS            │
└─────────────────────────────────────────────────────────────────────────────────────────────┘
```

进程内按 context 切换 RMW 实现：`ctx_t800` 用 `rmw_cyclonedds_cpp` +
`ROS_DOMAIN_ID=69` + `ROS_LOCALHOST_ONLY=0`；`ctx_core` 用 `rmw_fastrtps_cpp` +
`ROS_DOMAIN_ID=42`（详见 `ros2.py`）。

## 目录结构

```
engineai/t800/
├── main.py            # MCP HTTP server 入口（JSON-RPC 2.0，POST /mcp）
├── device.py          # T800DeviceBundle 插件聚合 + 工具名冲突检查
├── ros2.py            # Ros2Contexts 双 context 管理 + QoS 常量 + T800_TOPICS
├── plugins/           # motion.py / sensors.py / peripherals.py（12 张卡片实现）
├── driver.yaml        # id: t800-driver, port: 15708
├── config.yaml        # 插件开关（joint_bridge 默认关闭）
├── Dockerfile         # ros-base + rmw-cyclonedds-cpp + 官方消息包
├── interface_protocol/  # 官方消息源码（取自 engineai_ros2_workspace，随 driver 入库）
├── requirements.txt
├── deploy/service.yml # host 网络 + privileged + /dev:/dev
└── docs/              # 真机部署实测文档
```

## 本地运行

```bash
cd phanthymotus-driver/engineai/t800
pip install -r requirements.txt
python3 main.py          # 可选：CONFIG_PATH=/path/to/config.yaml
```

- 无 ROS2 环境的机器：`ros2.py` / `device.py` / `main.py` 均可在模块级纯 import
  （rclpy 全部延迟导入），可做静态验证（`python3 -m py_compile *.py`）。
- 真机运行时需机器人侧 ROS2 已启动（domain 69），
  `ros2 topic list` 可见 `/hardware/joint_state` 等话题。

## 构建与部署

```bash
# 方式一：build.sh（从 driver 仓库根目录，上下文为 driver 目录）
bash build.sh engineai/t800

# 方式二：手动构建（构建上下文 = driver 仓库根目录，无需外部 third_party/；
#   interface_protocol 消息包源码已随 driver 入库在 engineai/t800/interface_protocol/，
#   编译失败时才降级 stub 模式运行——消息包不构建、订阅不可用）
docker build -f engineai/t800/Dockerfile -t t800-driver .
```

部署：agent-core 部署时提取镜像内 `deploy/service.yml`（host 网络 + privileged +
`/dev:/dev` 声卡访问），合并进宿主机 `/opt/phanthy-motus/` 的 docker-compose.yml。
服务名 `engineai-t800`，容器名 `embodied-engineai-t800`。

## 与 agent-core 交互

- **注册**：启动后 POST `http://<agent-core>:15678/api/mcp`
  （可用 `AGENT_CORE_URL` 覆盖），payload `{id, name, url, transport, category}`；
  agent-core 随后执行 `initialize` → `tools/list` 注册工具；之后每 30s 心跳。
- **工具调用**：`tools/call`（JSON-RPC 2.0，POST `/mcp`，带 CORS）。
  插件 dispatch 返回纯 dict；`{"state":"error",...}` 会被标记 `isError`。
- **画布渲染**：`topic_out` 的 format 与渲染器映射（README_dev）——
  本 driver 传感器卡全部用 `data/json`（KV 面板），`mic` 用 `audio/pcm-16k`（波形）。
- **perception 消费**：agent-core `topic_subscriber` 订阅 String 话题注入 event_bus；
  ASR 通过 mic 卡的 `audio/pcm-16k` 流（`audio_msgs/AudioChunk`，
  format=`audio/pcm-16k`，16kHz 单声道 PCM_S16_LE，chunk 1024~4096 字节）。

## 安全注意

- `loco`：官方限速 x/y ±1 m/s、yaw ±1 rad/s；100Hz 发布；接收端 2 秒未收到自动停；
  停止 = 显式发零速度。
- `joint_bridge`：500Hz 高频 + 力矩公式 `tau = kp*(q_cmd-q) + kd*(qd_cmd-qd) + ff`，
  数组长度必须 = 25；**config 默认关闭**，需要时先切换到 `joint_bridge` 运动模式。
- 所有控制类卡片均做运动状态前置检查，避免在错误状态下下发指令导致机器人摔倒。
