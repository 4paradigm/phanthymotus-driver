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
| `face_light` | 面部灯带颜色 | `set_color` / `preset` / `off` + 逐灯接口和内部定时灯效 |
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

## face_light 的 SDK 部署与升级

`face_light` 是控制卡，默认启用并统一使用官方 SDK；旧 MQTT 后端不再提供。
镜像不包含私有 SDK，需先准备 SDK 并停止其他灯光写入源，否则卡片不可用或灯光会被覆盖。

1. 从现场头部 Nano 核实并取得 `faceLightSDK_Nano`。已审计来源为
   `/home/unitree/Unitree/sdk/faceLightSDK_Nano`，版本 `v1.0.1: first UDP version`；
   必需文件为 `include/FaceLightClient.h`、`include/LEDPixel.h`、`version.txt`
   和与运行主控架构匹配的 `lib/libfaceLight_SDK_arm64.so` 或 `lib/libfaceLight_SDK_amd64.so`。
   校验值及接入依据见 [SDK_AUDIT.md](deploy/face_light/SDK_AUDIT.md)。其他版本需重新审计，不能跳过校验。
2. 将 SDK 放在主控宿主机 `/opt/phanthy-motus/data/go1/faceLightSDK_Nano`，
   确保驱动容器内同一路径可读。若部署已经挂载整个 data 目录，沿用该挂载；
   否则在驱动服务的 Compose `volumes` 中添加以下只读挂载，再重建该驱动容器：

   ```yaml
   volumes:
     - /opt/phanthy-motus/data/go1/faceLightSDK_Nano:/opt/phanthy-motus/data/go1/faceLightSDK_Nano:ro
   ```

   不要将厂商 SDK 提交到 Git 或打进镜像。以下命令在主控执行，容器名按实际部署替换：

   ```bash
   docker exec embodied-unitree-go1 /deploy/face_light/run_sdk.sh --check
   ```

   必须返回 `VERIFIED official faceLight SDK v1.0.1; no hardware command sent`。
   此检查只验证文件和校验值，不加载 SDK，也不发送灯光指令。
3. 获得现场负责人允许后，在头部 Nano 核实 `faceLightMqtt` 和 `faceLightServer` 进程。
   若仍使用已审计的自启布局，先备份
   `/home/unitree/Unitree/autostart/faceLightMqtt/faceLightMqtt.sh`，
   仅注释其中启动 `./bin/faceLightMqtt &` 的行；若由其他服务管理，应停止并禁用对应 MQTT 灯光服务。
   然后在 Nano 上结束已有 MQTT 灯光写入进程，并复查：

   ```bash
   pgrep -a -x faceLightMqtt
   pkill -TERM -x faceLightMqtt
   pgrep -a -x faceLightMqtt     # 应无输出；若重新出现，先处理其自启来源
   pgrep -a -x faceLightServer  # 必须仍在运行
   ```

   使用进程所属用户或经授权的管理员身份操作；保持 `faceLightServer` 运行，
   同时停止其他 SDK 灯光客户端。停止桥接后，依赖旧 `face_light/color` MQTT 主题的程序将无法控制灯带。
   回退前先停止 SDK 卡片，再恢复备份的自启配置和 MQTT 灯光服务，避免两个写入源同时运行。
4. 保持 `plugins.face_light` 的 `enabled: true`、`backend: sdk`、`sdk_exclusive: true`。
   已保存的画布配置若为 MQTT 或 No，需更新为 SDK/Yes。默认 Yes 是部署前提声明，卡片不会代为停止现场进程。
   启动卡片后先调用 `info`，检查 `config_valid`、`available` 和 `connected`；失败时查看 `unavailable_reason`。
   首次启动会编译 SDK 适配器；若首次编译超时，确认编译完成后重试。
   `--check` 成功和 `info` 可用都不代表灯带已实际显示，颜色和灯效仍需现场观察验收。

`set_color`、`preset`、`off` 保留原调用方式。`set_led` 编号范围 0–11；
`set_leds` 可选 `color_format: hex`（12 个以空格或逗号分隔的六位 RGB 颜色）或
`rgb_array`（原 12 组 RGB 数组，画布可粘贴 JSON）。RGB 与编号留空按 0 执行，
`colors` 留空将全部灯设为黑色。周期、时长留空分别使用 2 秒、5 秒的有效默认值。
`fade` 在 `duration_s` 内单向切换起始 RGB 到目标 `to_r/to_g/to_b`，随后保持目标颜色；
闪烁、呼吸、流水灯到期关闭。新灯光指令会抢占旧灯效。

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
