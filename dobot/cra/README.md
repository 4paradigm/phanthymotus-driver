# Dobot CRA sensor driver

基于官方 V4 Python SDK 的六轴 CRA 状态驱动，部署在 Jetson 的独立 Docker
容器。卡片功能参考 RM75；本版只接反馈端口，不占用 Dashboard `29999`，不切换
TCP 控制模式，也不使能或移动机械臂。SDK 版本和协议适配说明见 [SDK.md](SDK.md)。

## 卡片

| 卡片 | 内容 |
|---|---|
| `connection` | 配置控制器 IP / 反馈端口；连接、新鲜度、重连错误和有效数据包计数 |
| `joint_states` | 六关节实际/目标角度、实际速度、电流、电机温度、各关节抱闸状态 |
| `joints` | 按 CR3A 官方关节链渲染的六轴实时三维骨架 |
| `robot_info` | 反馈中的机器人类型码、六轴信息；113 自动识别为 CR3A |
| `software_info` | 官方 SDK 版本、固定提交及来源；控制器固件版本标记为不可用 |
| `arm_all_state` | SDK 完整状态：关节、TCP 实际/目标位姿、坐标系、负载、力及 IO |
| `controller_state` | 模式、使能、运行、报警标志、抱闸、碰撞、安全字节、电压电流和数字 IO |

七张 sensor 卡片通过 `std_msgs/String` 发布 JSON，`joints` 使用
`topic_out.format=sensor/skeleton`，其余六张使用 `data/json`；
主题为 `/dobot_cra/state/<card>`，默认 10Hz。SDK 原始关节位置/速度为 deg、deg/s；
`joint_states.position/velocity` 转为 rad、rad/s，并同时保留原始角度。
TCP 位置为 mm，姿态为 deg。力传感器字段存在不等于传感器在线，使用前应检查
`six_force_online`。报警标志不是具体报警码，禁止据此虚构 GetErrorID 结果。

未收到有效数据、断线或数据超过 `stale_timeout_s` 时，`connected=false`、
`stale=true`，状态卡不再输出旧关节位置，返回 `available=false, data=null`。
`transport_connected` 只表示 socket 已连接，不能代替数据有效性。

`joints` 与数值卡 `joint_states` 使用同一个反馈快照，不增加网络连接。
骨架以 `joint1..joint6` 匹配 URDF，`q` 和 `dq` 为 rad 和 rad/s。
另提供平台骨架渲染器调用的 `model` resource（不发布 ROS topic）。
控制器反馈 `CRRobotType=113` 时自动加载官方 V4 `cr3` 模型；来源和 MIT 许可证
见 [resource/README.md](resource/README.md)。人工配置的型号不能覆盖不匹配的反馈型号。
本版骨架只适配已验证识别的 CR3A；其他型号继续提供数值状态，不套用错误模型。
未知型号或尚未识别时 `model.available=false` 并返回空场景，避免平台默认显示 G1 人形骨架。
先确认连接卡收到反馈，再添加/打开 `joints` 卡片，以便渲染器首次加载时取得正确模型。
数据过期时骨架返回 `joints=[], available=false`，静态模型身份可以保留。

## 网络及配置

Jetson 有线口接控制柜 LAN2：Jetson `192.168.200.10/24`，控制柜默认
`192.168.200.1`，无需网关。Wi-Fi 可继续用于 SSH 和互联网。

支持反馈端口 `30004`（默认 8ms）、`30005`（默认 200ms）、`30006`（默认 1000ms）。
接收超时和过期阈值取配置值与三个默认反馈周期中的较大值，从连接卡切换端口时自动调整。
默认配置在 `30004/30005` 上分别为 1 秒接收超时、2 秒过期；在 `30006` 上均为
3 秒，避免 1Hz 数据包稍有延迟就反复断线。若厂家修改了反馈周期，可在配置中进一步增加阈值。
连接状态会报告实际生效的 `receive_timeout_s` 和 `stale_timeout_s`。

SDK 断开连接或关闭 socket 抛出异常时，驱动仍会清除传输状态并继续重连；
`cleanup_error` 单独报告清理错误，不覆盖原始连接错误，收到新的有效数据后清除。

配置来源：

- `config.yaml`：IP 默认空，不配置时不联系任何控制器；`CONFIG_PATH` 可覆盖文件。
- `DOBOT_CONTROLLER_IP`、`DOBOT_FEEDBACK_PORT`、`DOBOT_MODEL_NAME` 覆盖初始值。
- 平台 `connection` 卡片配置 IP 和端口，保存并启动卡片后对当前运行时生效。
  IP 变化时先关闭旧连接、清空缓存，再连接新地址。其余卡片共享同一连接。
  IP 声明为敏感配置，不打包到共享 Solution。
- 卡片 stop 是平台 singleton sensor 的生命周期标记，不关闭其他卡片共享的连接；
  停止整个容器才释放反馈连接。

## 构建与部署

在仓库根目录，沿用平台构建入口：

```bash
bash build.sh --mirror none dobot/cra
```

构建会下载固定版本 SDK 并校验 SHA-256，构建机需要可访问 GitHub codeload。
`build-requirements.txt` 固定 pip、setuptools、wheel、packaging 和 setuptools-scm；
SDK 使用已安装的构建工具构建，不再临时拉取不受约束的构建依赖。
采用 ARM64 ROS base，SDK 和依赖仅安装到镜像。`/deploy/service.yml` 供平台安装器
提取并合并服务，容器名为 `embodied-dobot-cra`，MCP 端口 `15746`。

部署片段默认使用控制柜 LAN2 地址，可在宿主机 compose 的 `.env` 设置：

```dotenv
DOBOT_CONTROLLER_IP=192.168.200.1
DOBOT_FEEDBACK_PORT=30004
# 默认留空，型号码 113 自动识别 CR3A；离线预览可显式设置 CR3A
DOBOT_MODEL_NAME=
```

服务使用 host 网络，只读挂载平台既有的 `/opt/phanthy-motus/dds-local.xml`，
Agent Core 的 ROS domain 为 42；无需 privileged、设备挂载或主机 SDK。
可通过 `DOBOT_AGENT_CORE_URL` 覆盖注册地址，默认 `https://localhost:15678`。

## 本地开发和自动检查

普通开发机可运行协议/MCP 测试，不需要 ROS 或真机：

```bash
python3 -m venv .venv
.venv/bin/pip install -r dobot/cra/build-requirements.txt
.venv/bin/pip install --no-build-isolation -r dobot/cra/requirements.txt pytest
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_dobot_cra_sensors.py -q
.venv/bin/python scripts/check_service_yml.py
```

运行完整驱动需要 ROS 2 Humble。Jetson 上建议通过开发容器挂载源码调试，
避免将 SDK 安装到宿主机的系统 Python。

## PR 真机验收

1. 通过平台安装本分支构建的驱动，确认 `15746/health` 和七张 sensor 卡片可用。
2. 查看 `connection`：`connected=true`、`stale=false`、计数持续增加。
3. 对照 DobotStudio Pro 检查六关节角度、使能/抱闸状态及 TCP 位姿。
   如软件占用 `30004`，先断开软件后复测；本驱动不会连接 `29999`。
   `robot_info` 应识别为 CR3A；添加 `joints` 后应显示六轴机械臂骨架，
   各轴角度与 `joint_states` 一致。
4. 断开控制柜网线：过期后应显示不可用，不能继续显示旧位置为实时数据；
   接回网线后应自动恢复。此测试需在机器人已停止时进行。
5. 停止/启动单张卡片不应干扰其他卡片；重启容器后配置恢复。
6. 不要求打开总线通讯，不发送 RequestControl、使能、运动或 IO 写入命令。

本地模拟测试不等于已完成 ARM64 镜像、DDS、UI 或真机验收；这些由 PR 部署测试确认。
