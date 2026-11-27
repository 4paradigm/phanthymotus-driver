# 启元 Q1 探索版

`battery` 是只读传感器卡。在 Motus 画布添加「启元 Q1 探索版 → battery」后，打开「查看数据流」可查看实时电池读数。驱动启动后，卡片每 0.5 秒向 Core 发布一次 JSON，默认话题为 `/primebot_q1/battery`，格式为 `data/json`。

数据来自 `/aima/hal/bms/state`（`aimdk_msgs/msg/Bms`）。画布 JSON 的 `battery` 对象包含以下字段。

| 指标 | 字段 | 单位 |
| --- | --- | --- |
| 荷电状态（SOC） | `percentage` | %，直接取自 BMS 剩余容量百分比 |
| 电压、电流、输出功率 | `voltage_v`、`current_a`、`power_w` | V、A、W，电流为负表示放电 |
| 电池温度 | `temperature_c` | °C |
| 剩余容量 | `remaining_capacity_mah` | mAh |
| 完整循环次数 | `cycle_count` | 次 |
| 累计充放电容量 | `cycle_total_capacity_mah` | mAh |
| 厂商电池状态位 | `status_bits` | BMS 原始位掩码 |

`info` 返回卡片状态、输出话题与最近读数。driver 默认持续发布电池数据，Motus 的系统动作 `stop` 和 `start` 可以暂停和恢复发布。`battery.state` 为 `waiting` 表示等待首个样本，`live` 表示样本在 5 秒以内，`stale` 表示最后读数已过期。`age_s` 给出样本年龄。 已保存的旧画布需要将 `status` 卡换成 `battery`，并将下游连接指向 `/primebot_q1/battery`。

## ARM64 Docker 部署

`mic` 是内置麦克风传感器。默认输出 `/primebot_q1/mic/audio`，可以在 Motus 画布连到 Perception ASR。驱动通过 PulseAudio 读取 48 kHz 八声道 S16LE，将第 5、6 路混成单声道并重采样。发布的 `audio_msgs/msg/AudioChunk` 为 16 kHz 单声道 S16LE，每帧 1024 字节，端口格式为 `audio/pcm-16k`。

板载 Linux 部署默认将宿主机的 `/run/user/1000/pulse` 挂载到容器的 `/run/q1-pulse`。若实际路径不同，在 Compose 环境中设置 `Q1_PULSE_RUNTIME_DIR`。宿主机需已有对应的录音源和 PulseAudio 访问权限。若音频服务在另一块板且已配置官方 TCP 录音模块，设置 `Q1_PULSE_SERVER=tcp:<音频板地址>:4713`。卡片的 `info` 可查看等待、采集、停止或错误状态。 [Q1 官方录音说明](https://github.com/QyroTech/qyro-q1-sdk/blob/main/docs/zh/接口说明.md#tcp音频录制)

Apple 芯片 Mac 和 Linux ARM64（Jetson／板载机）使用同一套 Ubuntu 22.04 / ROS 2 Humble 驱动镜像。先启动同一台部署主机上的 Motus Core，并接通 Q1 的机器人网段。主机需要 Python 3、Docker 和 Docker Compose v2。在驱动仓库根目录运行：

```bash
python3 primebot/q1/deploy/q1.py build
python3 primebot/q1/deploy/q1.py start
python3 primebot/q1/deploy/q1.py status
python3 primebot/q1/deploy/q1.py stop
```

`build` 在首次部署或更新驱动源码时运行，生成 `primebot-q1:local`。已有 ARM64 驱动镜像时，可用 `start --image <镜像名>`。启动后，`status` 显示 Core 注册情况和最近的电池样本。看到 `registered` 和 `live` 后，在 Motus 画布添加 `battery` 卡片并打开「查看数据流」。`stop` 停止 Q1 服务，Core 继续运行。

- **Apple 芯片 Mac**使用 Docker Desktop。启动命令同时管理宿主机的 DDS 转发、容器内的 UDP 转发和 driver。见 [Mac 有线联调说明](tests/mac/README.md)。
- **Linux ARM64** 上，Core 和 Q1 driver 都使用主机网络，driver 直接通过 Q1 网段读取厂商 DDS。脚本默认读取主机的 `/opt/phanthy-motus/dds-local.xml`，并连接 `https://localhost:15678`。若本机 Core 使用 HTTP 或 XML 在其他位置，可运行 `start --core-url http://127.0.0.1:15678 --core-dds /实际路径/dds-local.xml`。团队的 Linux 部署流程也可继续使用原有 [部署配置](deploy/service.yml)。

镜像编译启元 SDK 提交 `618d995d4362f981ab5945624f8be9343c2cb68b` 的 `aimdk_msgs` 包。构建从[启元开发者社区](https://dev.primebot.com/download)下载官方 v1.1.8.0 SDK，校验 ZIP 的 SHA-256，仅将 `aimdk_msgs` 放进编译工作空间。相同 ZIP 的镜像地址可用构建参数 `Q1_SDK_URL` 指定。colcon 和 rosidl 生成消息接口，`nav_msgs`、`sensor_msgs` 是厂商包声明的消息依赖，Fast DDS 和 PyYAML 分别用于通信与加载驱动配置。机器人读取进程在域 0 使用厂商 DDS 配置，Motus 侧在域 42 使用 Core 的 DDS 配置。Core 通过 `http://localhost:15748/mcp` 访问驱动。驱动沿用 X2 的 Motus 公共运行时。[启元消息源码](https://github.com/PrimebotTech/primebot-q1-sdk/tree/618d995d4362f981ab5945624f8be9343c2cb68b/aimdk_msgs) · [X2 驱动](../../agibot/AimDK_X2/main.py)

在已接入 Q1 的 Linux ROS 环境加载启元工作空间后，可用下列命令核对原始 BMS 消息。

```bash
timeout 5 ros2 topic echo /aima/hal/bms/state --once
```

启元官方快速入门给出开发者模式、Ubuntu 22.04 / Humble 环境、网线直连和 BMS 读取步骤。电池消息字段与状态位可查接口手册。[SDK 快速入门与集成指南](https://github.com/PrimebotTech/primebot-q1-sdk/blob/main/docs/zh/SDK快速入门与集成指南.md) · [接口说明](https://raw.githubusercontent.com/PrimebotTech/primebot-q1-sdk/main/docs/zh/接口说明.md)

本地集成测试及真机验证范围见[验证记录](验证记录.md)。
