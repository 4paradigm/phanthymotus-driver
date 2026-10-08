# Unitree Go1 · go1_bundle（状态 + 控制驱动 · 卡片开发蓝本）

> 一张"卡片" = Driver 暴露的一个 MCP 工具 = 平台画布上一个可拖拽、可被大模型单独调用的能力。
>
> 本 bundle 当前发布 **24 张卡**：11 张传感卡（sensor）+ 9 张控制卡（actuator）+ 1 张资源卡（resource）+ 3 张独立视觉卡。
> **4 个聚合文件**：`sensors.py`（11 张）/ `controllers.py`（5 张）/ `ext_devices.py`（4 张）/ `camera.py`（RGB/depth/pointcloud 三张卡），每张卡仍然是自包含的类 + 工厂函数，方便按组评审、多人并行不撞车。
> 目的有二：① 把这些卡干净地上架；② 作为后来者新增其它卡片的开发起点 —— 怎么加卡见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 实现基座

- 官方原始 `unitree_legged_sdk`（Go1 分支 v3.8.6）的 pybind11 模块 `robot_interface`（`HighCmd`/`HighState`）。镜像内 `cmake -DPYTHON_BUILD=ON` 按容器 python 版本构建，**rclpy 可同进程共存** → 状态卡发 ROS2 topic 在画布渲染。
- 所有卡共用**同一个** raw SDK client（`go1_sdk_client.py` → `sdk_proxy.py` 子进程）：唯一的 UDP 收发线程，把 `HighState` 解析成一份线程安全的 `snapshot()`；状态卡只读 `snapshot()` 的不同切片，控制卡（如 `loco`）经该 client 的下发原语（`move`/`stop_move`）发 `HighCmd`。
- 无 `robot_interface` / 无真机时自动 **STUB**（不收发、`fresh=false`），MCP server 仍能起、注册、列 tool，方便无硬件时跑通链路。
- 固定 **HIGHLEVEL**（读 `HighState` / 下发 `HighCmd`）。控制卡须上真机验证量程+安全后才能上架（见 CONTRIBUTING.md §4）。

## 卡片总览

### 传感卡（sensor，读 `HighState` / 系统状态）

| 卡片（= 文件） | 能力 | 输出（ROS2 topic） |
|---|---|---|
| `loco_state` | 运动状态 | `/{ns}/loco/state`：mode / gait / velocity_body_mps / yaw_speed_rad_s / body_height_m / position_m（里程计，漂移） |
| `battery` | 电量（BMS） | `/{ns}/state/battery`：soc_percent / current_ma / cycle_count / temps / cell_voltage_mv |
| `imu` | IMU | `/{ns}/state/imu`：四元数 / 角速度 / 加速度 / 欧拉角 / 温度 |
| `feet` | 足端 | `/{ns}/state/feet`：足底力[4] + 高层时足端相对机身位置/速度 |
| `fall_alarm` | 跌倒/侧翻告警 | `/{ns}/state/fall_alarm`：IMU roll/pitch → ok/tilted/fallen（阈值可配） |
| `odometry` | 里程计 | `/{ns}/state/odometry`：position/yaw + 相对起点位移（只读） |
| `obstacle_range` | 超声波避障 | `/{ns}/state/obstacle_range`：range_raw[4]（仅 HIGHLEVEL；方向/单位官方未定义，原样输出） |
| `udp_diagnostics` | UDP 通信健康 | `/{ns}/state/udp_diagnostics`：收发计数 + CRC/丢包/标志错误计数 |
| `joints` | 12 腿关节 | `/{ns}/state/joints`：q/dq/tau/temp（骨架渲染，需 `model` 卡提供 URDF） |
| `remote_controller` | 无线遥控器 | `/{ns}/state/remote_controller`：16 按键 + 5 摇杆轴（`HighState.wirelessRemote[40]`） |
| `activity_monitor` | 活动度统计 | `/{ns}/state/activity`：后台采样速度/模式，action=report 返回 last_30s + since_start（距离/运动占比/平均速度/峰值/当前模式） |
| `camera_rgb` | RGB 去畸变图像（5 机位·multiInstance） | `start` 才连对应 Nano → CompressedImage；`stop` 断开释放相机 |
| `camera_depth` | 彩色深度图（5 机位·multiInstance） | `start` 才连对应 Nano → CompressedImage；`stop` 断开释放相机 |
| `camera_pointcloud` | XYZ 点云与 JPEG 俯视预览（5 机位·multiInstance） | `start` 才连对应 Nano → PointCloud2；`stop` 断开释放相机 |

### 控制卡（actuator，下发 `HighCmd` / 外设动作；须真机验证量程+安全后上架）

| 卡片（= 文件） | 能力 | 关键动作 |
|---|---|---|
| `loco` | 基础运动 | `move`（三维速度）/ `stop_move` / `balance_stand` / `stand_up` / `stand_down` / `damp` / `recovery_stand` |
| `body_pose` | 机身姿态与高度 | `set_attitude`（roll/pitch/yaw）/ `set_body_height` / `set_foot_raise_height` / `reset` |
| `switch_gait` | 步态切换 | `idle` / `trot` / `trot_run` / `climb_stair` / `trot_obstacle`（高风险步态须 `confirm=true`） |
| `special_motion` | 特殊动作 | `jump_yaw_left` / `straight_hand`（同步阻塞执行，须 `confirm=true`） |
| `gesture` | 表演/表情 | 作揖/点头/摇头/歪头/环视/跳舞/俯卧撑/坐/昂首等（异步） |
| `beep` | 头部扬声器 beep | Nano `beep_adapter.py`（:18082 /v1/beep/actions） |
| `speaker` | 头部扬声器播放 | Nano `speaker_adapter.py`（:18083 /v1/speaker/actions）→ 播放远端音频流 |
| `face_light` | 面部灯带颜色 | `set_color` / `preset` / `off` + 逐灯接口和内部定时灯效（见下方后端能力） |
| `system_health` | 整体健康检查 | `robot_info`：CPU/内存/磁盘/电池/MQTT 体检 |

### 资源卡（resource）

| 卡片（= 文件） | 能力 | 输出 |
|---|---|---|
| `model` | Go1 四足 URDF | 返回 URDF，供 `joints` 骨架渲染 |

`{ns}` 为 `config.yaml` 的 `ros_namespace`（默认 `bundle`；留空则取 hostname）。

## 相机架构（camera.py 三合一）

五路相机（front/chin/left/right/belly）分布在三块 Nano 板（.13/.14/.15）：

```
Nano 板 (.13/.14/.15)              Pi 驱动容器 (.161)
┌─ rgb_stream (TCP :9201~9205) ──▶ camera.py (camera_rgb)        → /{ns}/vision/{pos}/mono
├─ depth_stream (TCP :9101~9105) ─▶ camera.py (camera_depth)     → /{ns}/camera/{pos}/depth
└─ pointcloud_stream (TCP :9401~9405) → camera.py (camera_pointcloud) → /{ns}/camera/{pos}/pointcloud
```

- 三路均为**按需开相机**：卡 `start` 才建 TCP 连接，Nano 侧才打开相机；`stop` 断开，Nano `_exit(0)` 释放相机（systemd `Restart=always` 重启待命）。
- **同一物理相机三路互斥**：同一机位的 rgb / depth / pointcloud 不能同时 `start`，谁先连上谁占设备。
- Nano 端服务由容器首启时 `nano_bootstrap.sh` 自动编译部署（`rgb_stream` 默认启用；`depth_stream` / `pointcloud_stream` 须传 `DEPTH_ENABLE=1` / `PCL_ENABLE=1`）。

## 卡片装配约定（关键）

**卡名 == 模块名 == config.yaml 里的 key。** `main.py` 遍历 `config.yaml` 中
`enabled: true` 的卡名，`import_module(卡名)` 并调用其 `make_plugin(...)` 装配。
聚合文件（`sensors.py`/`controllers.py`/`ext_devices.py`）各自 `import_module` 后，
通过同名 `make_*` 函数（如 `make_battery`/`make_loco`/`make_beep`）找到卡片类。所以：

> **新增一张卡**：若属于传感类，在 `sensors.py` 末尾追加 `Plugin` + `make_<卡名>`；
> 控制类加到 `controllers.py`；外部设备加到 `ext_devices.py`。
> 然后在 `config.yaml` 打开它。不用改 `main.py`。

## 接口约定（与平台其它驱动一致）

- **状态卡读取**：无业务输入。`action=info`（或 `read`/`get`）返回最新数据 + `topic_out`；每条数据带 `timestamp_ms` / `control_level` / `fresh`，**无新包不伪造**。
- **生命周期**：每张卡都处理 `start`/`stop`：`start → {"state":"running"}`、`stop → {"state":"idle"}`。
- **`dispatch()` 返回**：一律 plain dict（或 `None`），由 MCP 处理器自动包 `{"content":[...]}`，**不要**自己预包。
- **ROS2 可选**：装了 rclpy → 按各卡频率发 topic；没装 → 只支持 MCP `action=info` 轮询（`topic_out` 为空）。
- **`topic_out` 格式**：每个条目必须是 `{"topic": "/path/to/topic", "format": "data/json"}` 对象，不能是裸字符串。

## 文件结构

```
go1_bundle/
├── main.py                 # MCP server 入口 + 按 config 卡名自动装配（HIGHLEVEL）
├── go1_sdk_client.py       # 共享 raw SDK client（已由 sdk_proxy.py 子进程承接）
├── sdk_proxy.py            # SDK 子进程代理：隔离 robot_interface 避免 GIL 冲突
│   ── 聚合卡文件（sensors.py = 12 张）──
├── sensors.py              # 状态卡合集：battery/imu/feet/fall_alarm/obstacle_range/
│                           #   remote_controller/udp_diagnostics/loco_state/odometry/joints/
│                           #   activity_monitor/model
│   ── 聚合卡文件（controllers.py = 5 张）──
├── controllers.py          # 运动控制合集：loco/body_pose/switch_gait/gesture/special_motion
│   ── 聚合卡文件（ext_devices.py = 4 张）──
├── ext_devices.py          # 外部设备合集：beep/speaker/face_light/system_health
│   ── 视觉卡（三合一）──
├── camera.py               # RGB / 深度 / 点云 三合一（multiInstance）
│   ── 外设适配器（非卡片，Nano 侧服务）──
├── beep_adapter.py         # beep 卡的 Nano 侧适配器（:18082）
├── speaker_adapter.py      # speaker 卡的 Nano 侧适配器（:18083）
├── config.yaml             # 卡片开关 / 端口 / 命名空间
├── driver.yaml             # 驱动元数据（id / port / 描述）
├── requirements.txt        # 运行期 pip 依赖（pyyaml / paho-mqtt；rclpy/robot_interface 由镜像构建）
├── Dockerfile              # ARM64；镜像内构建 robot_interface.so
├── deploy/                 # Nano 侧服务部署（nano_bootstrap.sh 等）
├── deploy/camera/          # Nano 侧 C++ 流媒体源码（含 rgb_stream.cc / nvjpeg_worker.cc / depth_stream.cc / pointcloud_stream.cc）
├── README.md               # 本文件
└── CONTRIBUTING.md         # ★ 如何在此基础上新增卡片
```

## 本地运行（无硬件 STUB 亦可）

```bash
cd unitree/go1_bundle
pip install -r requirements.txt          # pyyaml / paho-mqtt；rclpy/robot_interface 缺失会自动降级
python3 main.py                          # 默认读 config.yaml；CONFIG_PATH 可覆盖

# 另开一个终端，列卡片：
curl -s localhost:15717/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list"}' | python3 -m json.tool

# 读一次 battery：
curl -s localhost:15717/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"battery","arguments":{"action":"info"}}}'
```

无 `robot_interface`（如开发机 Mac）时数据为空、`fresh=false`，属正常 STUB 行为。

## 构建镜像

```bash
# 在仓库根目录：
./build.sh go1_bundle
```

启动示例（含 Nano 侧部署）：

```bash
sudo docker run --rm --name go1_bundle \
  --network host --privileged --ipc host --pid host \
  -e NETWORK_INTERFACE=eth0 -e ROS_DOMAIN_ID=42 \
  go1_bundle:test
```

如需同时部署 depth / pointcloud 的 Nano 端常驻服务：

```bash
sudo docker run --rm --name go1_bundle \
  --network host --privileged --ipc host --pid host \
  -e NETWORK_INTERFACE=eth0 -e ROS_DOMAIN_ID=42 \
  -e DEPTH_ENABLE=1 \
  -e PCL_ENABLE=1 \
  go1_bundle:test
```

> ⚠ `DEPTH_ENABLE=1` / `PCL_ENABLE=1` 会在 Nano 板上装常驻 systemd 服务。三张视觉卡指向同一物理相机时仍互斥，按需启动即可。

## 端口

`15715`（MCP）。平台驱动端口区间 `15700–15799`。

## agent core 发现不了驱动的常见原因

1. **启动时序**：agent core 启动时会对所有已注册 MCP 做一次 auto-ping。若 go1_bundle 尚未就绪，工具列表会被持久化为空，之后不会自动重试。解决：容器稳定后手动触发一次 ping：
   ```bash
   curl -sk -X POST https://localhost:15678/api/mcp/<mcp_id>/ping
   ```
2. **ping 内部异常**：工具的 `info` action 返回格式不合规（如 `topic_out` 使用裸字符串而非 `{"topic":..., "format":...}` 对象）会导致 agent core 解析崩溃，整个 ping 失败。
3. **重复注册**：同一 `server_name` 被注册两次时 agent core 会去重合并，若 URL 未更新则找不到设备。

---

新增卡片请从 **[CONTRIBUTING.md](CONTRIBUTING.md)** 开始。

## face_light：在原卡内控制静态色、12 灯和定时灯效

仍然只注册一张 `face_light`（`ext_devices.py::FaceLightPlugin`），没有新增灯光卡，
没有修改 Agent Core、运动 SDK 或其他机器人。默认 `backend: sdk`，实机所有灯光
统一调用官方 SDK。保留 `set_color`/`preset`/`off` 的调用接口，静态颜色保持到下一次有效指令。
不再提供灯光 MQTT 后端；旧画布的 `backend: mqtt` 配置会返回明确迁移提示，需更新为 `sdk`。
不将越界值截断、字符串转整数或发送错误当作成功。

所有静态颜色、逐灯控制和灯效统一使用官方 SDK（`setLedColor` + `sendCmd`）。
画布不再提供后端选择，也不接受 `backend: simulated`。离线测试仅在测试代码中注入记录器，
不会连接机器人；没有自动回退到模拟成功。真机显示仍需现场验收。
所有指令共用同一帧写入路径，每个实例只运行一个灯效，实机只创建 SDK 写入器。
默认配置为 `backend: sdk` 和 `sdk_exclusive: true`（画布显示 Yes），无需每次设置。
此默认值要求部署前已停止旧 `faceLightMqtt` 等其他灯光写入源，不会自动停止现场程序。
保持 `faceLightServer` 运行，并准备下文规定目录中的官方 SDK。
已有画布若保存过 No，需要改为 Yes 并保存一次；已保存的配置优先于新的默认值。

调用现有 `face_light` 工具，将下列对象作为 `arguments`：

```json
{"action":"set_color","r":255,"g":40,"b":0}
{"action":"preset","name":"blue"}
{"action":"set_led","index":0,"r":255,"g":0,"b":0}
{"action":"set_leds","colors":[[255,0,0],[0,255,0],[0,0,255],[255,0,0],[0,255,0],[0,0,255],[255,0,0],[0,255,0],[0,0,255],[255,0,0],[0,255,0],[0,0,255]]}
{"action":"blink","r":255,"g":0,"b":0,"period_s":1,"duration_s":5}
{"action":"breathe","r":0,"g":80,"b":255,"period_s":2,"duration_s":10}
{"action":"fade","r":255,"g":0,"b":0,"to_r":0,"to_g":0,"to_b":255,"period_s":2,"duration_s":10}
{"action":"chase","r":0,"g":255,"b":0,"period_s":2,"duration_s":10}
{"action":"off"}
{"action":"info"}
```

`set_led` 保持其余灯为上一帧的软件记录颜色；从未发送过颜色时，其余灯以黑色初始化，
不能理解为读取了灯带现状。`colors` 恰好为 12 个 RGB 三元组，数组位置就是 SDK 编号。
RGB 必须是 0–255 的整数，编号必须是 0–11 整数，不接受 bool、浮点数或字符串。
为兼容旧 `set_color` 调用，省略的 RGB 通道仍默认 0，省略 preset 名仍默认 off。
未知预设、错误数组和不合法时长返回 `INVALID_ARGUMENT`，不会改变当前灯效。
不支持的指令也不会中断当前有效灯效。

`period_s` 是完整周期（0.2–3600 秒，默认 2）：闪烁前半亮、后半灭；
呼吸按余弦曲线缩放 RGB；渐变从 RGB 到目标 RGB 再返回；流水灯一周期依次通过 0–11。
`duration_s` 为 0.05–3600 秒（默认 5），到期关闭全部灯，线程结束。
四种灯效声明 `x-completion`（兜底等待 3610 秒），接受后立即返回唯一 `action_id`。
`x-resource: face_light` 将灯效的资源等待限定到灯光通道。
Go1 其他执行类工具也声明资源：`beep`/`speaker` 共用 `mouth`，
`loco`/`body_pose`/`switch_gait`/`gesture`/`special_motion` 共用 `base`。
`system_health`/`activity_monitor` 保留既有执行类类型和访问权限，保守归入其读取的
本体 `base` 通道。这些工具无需等待灯效的资源释放；同一调用序列仍遵循 Agent Core 的动作次序。
线程向 `AGENT_CORE_URL` 的 `/api/acp/complete` 回报 `completed`（到期且关闭帧发送成功）、
`cancelled`（指令抢占、停止或配置变化）或 `error`（后台发送失败）。
完成结果仍是软件记录，不表示实际灯光已显示；回调连接失败会记录日志，
HTTP 回报使用独立线程，请求超时为 3 秒，灯效抢占不等待 HTTP。
最多允许 8 个待回报动作；名额用尽时新灯效返回 `RESOURCE_BUSY`，静态色及 off 仍可执行。
stop 先发关闭帧、关闭 SDK，再回收回报线程。静态指令保持同步返回，不创建异步动作。
定时执行不需要模型反复调用；刷新最多约 120 Hz（短周期）/通常 20 Hz，
实际速度受发送耗时和调度影响。亮度仅通过 RGB 缩放，无硬件亮度接口假设。

有效静态指令、单灯指令和新灯效先取消并等待旧灯效退出，然后发送新帧。
`off` 取消灯效后发送黑色，但保持后端连接；`stop`（生命周期或 dispatch）
取消灯效、发送黑色、关闭连接并回收线程。停止后指令返回 `NOT_AVAILABLE`，
需再次 `start`。重复 start/stop 可安全调用。不能连接或发送时返回失败；
后台发送失败也结束灯效并记录 `last_error`。连接不可用时不能保证硬件已经熄灭。
SDK 适配器启动握手成功后才返回 ready；`info.connected` 仅表示软件进程存活。
SDK 发送失败时结束适配器进程，防止继续发送旧帧；恢复须 stop/start。

`info` 返回 `mode`、`running`（灯效线程）、`connected`、能力表、12 灯位置映射、
最近成功发送的 `colors` 和时间及 `last_error`。首次发送前 `colors: null`。
这些均为软件记录，统一标注 `state_source: software_record` 和
`hardware_verified: false`，无硬件状态反馈。SDK `ok: true` 仅表示 UDP 发送调用成功，
不表示实际灯带显示成功。

### 官方编号与真实 SDK 接入缺口

已核对 [官方文档第 5 节](https://github.com/UnitreeSupport/Unitree_Docs/blob/master/docs/get_started/Go1_Edu.md)
及其 [LED 编号图](https://raw.githubusercontent.com/UnitreeSupport/Unitree_Docs/master/docs/get_started/images/LED.bmp)
和 [官方示例截图](https://raw.githubusercontent.com/UnitreeSupport/Unitree_Docs/master/docs/get_started/images/example.png)。
**面向狗头的观察者视角**：画面左侧自上到下为 0–5，画面右侧为 6–11；
这不是机器人自身左右。`info.led_map.row_from_top` 从 0 开始。
该对应关系来自官方图，还没有在本机 Go1 上逐灯点亮核对。

2026-10-08 经实机主控 SSH 只读核实：主控为树莓派
（内网地址 `192.168.123.161`）；从主控访问头部 Nano `192.168.123.13`。
SDK 实际路径为 `/home/unitree/Unitree/sdk/faceLightSDK_Nano`，
版本为 **`v1.0.1: first UDP version`**。取得 `FaceLightClient.h`、`LEDPixel.h`、
`main.cpp`、`CMakeLists.txt`、`version.txt` 和 ARM64/AMD64 两个库，
本地副本的 SHA-256 与实机逐项一致，证据见 [SDK_AUDIT.md](deploy/face_light/SDK_AUDIT.md)。

真实头文件确认：`setLedColor(uint32_t id, const uint8_t *rgb)`、
`setAllLed(const uint8_t *rgb)` 和 `void sendCmd()`。
RGB 数组按 R/G/B 传入，SDK 自己完成内部 GRB 转换和 UDP 打包。
适配器不构造硬件报文，不猜测目的端口，不增加 MQTT 逐灯字节。

`deploy/face_light/adapter.cpp` 实现独立 SDK 进程：一帧设置 12 个编号，再调用一次
`sendCmd()`。因为 SDK 无返回值，适配器通过动态符号拦截它实际调用的 `sendto`，
检查实际发送错误/长度；没有观测到调用也返回失败，不伪装成功。
SDK 进程通过本地管道回报结果，卡片统一控制抢占与停止；管道超时/退出/错误时
终止并回收该进程。正常关闭通过 EOF 调用 SDK 析构，进程退出也回收 SDK 分配。
**SENT 只代表 UDP 套接字接受数据，不代表灯珠已显示或 Nano 服务已接收。**
`info.connected` 在 SDK 模式中表示本地 SDK 进程就绪，不表示硬件在线。
画布共享配置通过 `action=config` 应用；配置发生变化会取消灯效并关闭旧后端，需
重新启动卡片。重复下发相同配置不改变状态，避免平台每次调用前重放配置中断灯效。

已用现场官方 ARM64 库在无网络的本地 Linux 容器编译、链接和加载适配器，
验证网络不可用时返回失败。此版适配针对已核实的 v1.0.1 API；其他 SDK 版本须重新核对。
Mac 原生不能加载这些 Linux ELF 库。现场 Nano 为 Ubuntu 18.04，当前验证容器
为 Ubuntu 22.04；部署时需在目标系统或匹配 sysroot 上重新编译，不能将本地
容器二进制直接当作 Nano 兼容性证据。库副本保存在被忽略的
`.local/face-light-sdk/faceLightSDK_Nano/`，未加入 Git，不对外再分发。

先前现场检查时，`faceLightServer`（接收灯光的服务）和 `faceLightMqtt`（MQTT 写入桥）均在运行。
检查期间没有停止或修改它们；部署时需重新核实当前状态。切换至 SDK 前需由操作者停止 **faceLightMqtt 写入源**，
保留 **faceLightServer**；也要停止其他正在发送灯光的测试程序，避免抢写。
卡片不会自动停止现场服务。`sdk_exclusive` 默认 true；显式设为 false（No）时启动失败。
默认 Yes 表示按单一 SDK 写入源部署，并不证明其他写入源已经停止；卡片不能远程核实这一点。

### 构建与配置 SDK 后端（尚未部署到实机）

在有对应架构官方 SDK 的 Linux 主机，使用现有 cmake/g++，不增加大型依赖：

```bash
cmake -S unitree/go1/deploy/face_light -B /tmp/go1-face-build \
  -DFACE_LIGHT_SDK_DIR=/absolute/path/to/faceLightSDK_Nano
cmake --build /tmp/go1-face-build
```

构建产物为 `face_light_sdk_adapter` 和其同目录的 `vendor_sdk/`。
**两者一起保留/搬移**，适配器通过相对库搜索路径加载匹配的官方库。
既有 Dockerfile 已复制整个 `deploy/`，其中包括适配器源码；SDK 库由操作者挂载。
镜像通过 `/deploy/face_light/run_sdk.sh` 在首次启动 SDK 后端时编译，并缓存
编译产物。只需将完整官方 SDK 放到机器人
`/opt/phanthy-motus/data/go1/faceLightSDK_Nano`（现有数据卷已映射）；SDK 库不打包进公共镜像。
启动脚本与 SDK 根目录固定，不在画布暴露路径配置；`config`/`start` 传入其他
`sdk_executable` 或 `sdk_dir` 会返回 `INVALID_ARGUMENT`，且保留已有运行状态。
SDK 头文件、库及示例应由操作者从可信官方来源放入该目录，不能接受未经审查的上传文件。
启动脚本在编译前验证两个头文件、版本文件和当前架构库的 SHA-256，
必须匹配 [SDK_AUDIT.md](deploy/face_light/SDK_AUDIT.md) 中已审查的 v1.0.1；
缺失、内容变化或校验工具不可用时拒绝启动。升级 SDK 需重新审查并更新校验值。
缺少文件、编译失败或架构不匹配时启动失败，不自动回退 MQTT/模拟。

修改原卡配置，不再注册其他卡：

```yaml
face_light:
  enabled: true
  backend: sdk
  sdk_exclusive: true  # 默认值；部署前停止 faceLightMqtt 等其他写入源
```

### 本地验证与真机验收

```bash
python -m pytest -q tests/test_go1_face_light.py
```

测试使用显式模拟帧和 SDK 管道测试进程，覆盖旧接口在 SDK 下的兼容性、严格参数校验、12 灯映射、
四种效果的时间变化、到期关闭、静态/效果抢占、停止竞争、停止后无旧帧、
线程回收、发送失败、旧 MQTT 配置迁移提示、SDK 独占前置条件/进程超时/异常/回收和唯一卡片装配。
它们不证明真机灯带表现。

`tests/face_light_native_check.py` 必须在仅 lo 网卡启用且无 IPv4 路由的断网 Linux 容器运行，
设置 `SDK_DIR` 和 `REAL_ADAPTER`：原生成功路径用实际 SDK 头文件加假的动态库
（Unix 域套接字，无机器人网络），真实官方库仅测试断网失败路径；另验证 12 个
RGB 调用、输入校验、错误回报、进程关闭及库包可搬移。

SDK 文件已取得和审查；下一步需要获准部署并切换灯光写入源，再进行真机灯光验收：
使用原 `set_color`/`preset`/`off` 接口检查 SDK 整条色和关闭，逐灯 0–11 单独点亮拍照验证位置/RGB 通道；
发送包含 12 种可区分颜色的帧，观察四种效果的周期/持续时间；效果中切换静态色、
off 和停止，持续观察旧颜色不再返回；断连时确认失败记录，恢复后检查无旧队列重放。
记录视频/照片与软件发送记录，分别说明实测和软件结果。本轮没有机器人运动、
物理灯光验收；目标系统临时编译/导入检查通过，临时目录已清理。
正式容器/画布验收需使用提交分支构建的镜像；单独 SDK 调用不能代替画布验收。
