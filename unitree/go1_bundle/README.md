# Unitree Go1 · go1_bundle（状态 + 控制驱动 · 卡片开发蓝本）

> 一张"卡片" = Driver 暴露的一个 MCP 工具 = 平台画布上一个可拖拽、可被大模型单独调用的能力。
>
> 本 bundle 当前发布 **26 张卡**：14 张传感卡（sensor）+ 10 张控制卡（actuator）+ 1 张资源卡（resource）+ 1 张视觉卡（camera）。
> **一张卡 = 一个自包含的 `.py` 文件**（如 `loco_state.py` / `battery.py` / `loco.py` / `gesture.py` …），方便按卡评审、按卡提交、多人并行不撞车。
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
| `test_camera_depth` | 双目深度推流（5 机位·multiInstance；test=未验收） | 卡 `start` 才连对应 Nano 板 `depth_stream` → ROS2 CompressedImage（jpeg）；`stop` 断开释放相机 |
| `test_camera_pointcloud` | 双目点云推流（5 机位·multiInstance；test=未验收） | 卡 `start` 才连对应 Nano 板 `pointcloud_stream` → ROS2 PointCloud2；`stop` 断开释放相机 |
| `camera_rgb` | RGB 相机推流（5 机位·multiInstance） | 卡 `start` 才连对应 Nano 板 `rgb_stream`（TCP :9201~9205）→ ROS2 CompressedImage（jpeg）；`stop` 断开释放相机 |
| `test_activity` | 活跃度检测（test=未验收） | 检测机器人是否静止/活跃 |

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
| `face_light` | 面部灯带颜色 | `set_color` / `preset` / `off`（经 MQTT） |
| `test_light_effect` | 面部灯带动效（test=未验收） | `solid` / `blink` / `breathe` / `fade` / `brightness_up` / `brightness_down` / `preset` / `off`（经 MQTT） |
| `test_battery_off` | 电池关断（不可逆，test=保守留前缀） | `battery_off`：低层 `LowCmd.bms.off=0xA5` → 主控板（.10:8007）**真断电池**，区别于只关 Pi 的 `poweroff`；**不可逆、不能远程恢复**；须 `confirm=true` + `reason`，前置：状态 fresh 且机器人静止 |

### 资源卡（resource）

| 卡片（= 文件） | 能力 | 输出 |
|---|---|---|
| `model` | Go1 四足 URDF | 返回 URDF，供 `joints` 骨架渲染 |

`{ns}` 为 `config.yaml` 的 `ros_namespace`（默认 `bundle`；留空则取 hostname）。

## 相机架构（camera_rgb / depth / pointcloud 三路共用）

五路相机（front/chin/left/right/belly）分布在三块 Nano 板（.13/.14/.15）：

```
Nano 板 (.13/.14/.15)              Pi 驱动容器 (.161)
┌─ rgb_stream (TCP :9201~9205) ──▶ camera_rgb.py       → /{ns}/vision/{pos}/mono
├─ depth_stream (TCP :9101~9105) ─▶ test_camera_depth.py → /{ns}/camera/{pos}/depth
└─ pointcloud_stream (TCP :9401~9405) → test_camera_pointcloud.py → /{ns}/camera/{pos}/pointcloud
```

- 三路均为**按需开相机**：卡 `start` 才建 TCP 连接，Nano 侧才打开相机；`stop` 断开，Nano `_exit(0)` 释放相机（systemd `Restart=always` 重启待命）。
- **同一物理相机三路互斥**：同一机位的 rgb / depth / pointcloud 不能同时 `start`，谁先连上谁占设备。
- Nano 端服务由容器首启时 `nano_bootstrap.sh` 自动编译部署（`rgb_stream` 默认启用；`depth_stream` / `pointcloud_stream` 须传 `DEPTH_ENABLE=1` / `PCL_ENABLE=1`）。

## 卡片装配约定（关键）

**卡名 == 模块名 == 文件名 == config.yaml 里的 key。** `main.py` 遍历 `config.yaml` 中
`enabled: true` 的卡名，`import_module(卡名)` 并调用其 `make_plugin(...)` 装配。所以：

> **新增一张卡 = 新建 `<卡名>.py` + 在 `config.yaml` 打开它。不用改 `main.py`。**

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
│   ── 传感卡（sensor）──
├── loco_state.py           # 运动状态
├── battery.py              # 电池 BMS
├── imu.py                  # IMU
├── feet.py                 # 足端力/位置
├── fall_alarm.py           # 跌倒/侧翻告警
├── odometry.py             # 里程计
├── obstacle_range.py       # 超声波避障
├── udp_diagnostics.py      # UDP 通信健康
├── joints.py               # 12 腿关节（骨架渲染）
├── remote_controller.py    # 无线遥控器
├── test_camera_depth.py    # 双目深度推流（5 机位·multiInstance；test=未验收）
├── test_camera_pointcloud.py # 双目点云推流（5 机位·multiInstance；test=未验收）
├── camera_rgb.py           # RGB 相机推流（5 机位·multiInstance）
├── test_activity.py        # 活跃度检测（test=未验收）
│   ── 控制卡（actuator）──
├── loco.py                 # 基础运动（move/stop_move/stand 等）
├── body_pose.py            # 机身姿态与高度
├── switch_gait.py          # 步态切换
├── special_motion.py       # 特殊动作（jump/straight_hand）
├── gesture.py              # 表演/表情（异步）
├── beep.py                 # 头部扬声器 beep
├── speaker.py              # 头部扬声器播放
├── face_light.py           # 面部灯带颜色（经 MQTT）
├── test_light_effect.py    # 面部灯带动效（test=未验收，经 MQTT）
├── test_battery_off.py     # 电池关断（低层真断电，不可逆）
│   ── 资源卡（resource）──
├── model.py                # Go1 URDF（供 joints 骨架渲染）
│   ── 外设适配器（非卡片，Nano 侧服务）──
├── beep_adapter.py         # beep 卡的 Nano 侧适配器（:18082）
├── speaker_adapter.py      # speaker 卡的 Nano 侧适配器（:18083）
├── config.yaml             # 卡片开关 / 端口 / 命名空间
├── driver.yaml             # 驱动元数据（id / port / 描述）
├── requirements.txt        # 运行期 pip 依赖（pyyaml / paho-mqtt；rclpy/robot_interface 由镜像构建）
├── Dockerfile              # ARM64；镜像内构建 robot_interface.so
├── deploy/                 # Nano 侧服务部署（nano_bootstrap.sh 等）
├── camera/                 # Nano 侧 C++ 流媒体源码（rgb_stream.cc / depth_stream.cc / pointcloud_stream.cc）
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

> ⚠ `DEPTH_ENABLE=1` / `PCL_ENABLE=1` 会在 Nano 板上装常驻 systemd 服务。depth / pointcloud / camera_rgb 指向同一物理相机，三者互斥，按需启用即可。

## 端口

`15717`（MCP）。平台驱动端口区间 `15700–15799`。

## agent core 发现不了驱动的常见原因

1. **启动时序**：agent core 启动时会对所有已注册 MCP 做一次 auto-ping。若 go1_bundle 尚未就绪，工具列表会被持久化为空，之后不会自动重试。解决：容器稳定后手动触发一次 ping：
   ```bash
   curl -sk -X POST https://localhost:15678/api/mcp/<mcp_id>/ping
   ```
2. **ping 内部异常**：工具的 `info` action 返回格式不合规（如 `topic_out` 使用裸字符串而非 `{"topic":..., "format":...}` 对象）会导致 agent core 解析崩溃，整个 ping 失败。
3. **重复注册**：同一 `server_name` 被注册两次时 agent core 会去重合并，若 URL 未更新则找不到设备。

---

新增卡片请从 **[CONTRIBUTING.md](CONTRIBUTING.md)** 开始。
