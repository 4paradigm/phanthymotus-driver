# Unitree Go1 · go1_bundle（状态 + 基础控制驱动 · 卡片开发蓝本）

> 一张“卡片” = Driver 暴露的一个 MCP 工具 = 平台画布上一个可拖拽、可被大模型单独调用的能力。
>
> 本 bundle 是从完整 Go1 驱动中**切出的最小可运行蓝本**，当前发布 3 张状态卡（`loco_state`、`battery`、`obstacle_range`）+ 1 张控制卡（`spin`）。
> **一张卡 = 一个自包含的 `.py` 文件**（`loco_state.py` / `battery.py` / `obstacle_range.py` / `spin.py`），方便按卡评审、按卡提交、多人并行不撞车。
> 目的有二：① 把这些卡干净地上架；② 作为后来者新增其它卡片的开发起点 —— 怎么加卡见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 实现基座

- 官方原始 `unitree_legged_sdk`（Go1 分支 v3.8.6）的 pybind11 模块 `robot_interface`（`HighCmd`/`HighState`）。镜像内 `cmake -DPYTHON_BUILD=ON` 按容器 python 版本构建，**rclpy 可同进程共存** → 状态卡发 ROS2 topic 在画布渲染。
- 所有卡共用**同一个** raw SDK client（`go1_sdk_client.py`）：唯一的 UDP 收发线程，把 `HighState` 解析成一份线程安全的 `snapshot()`；状态卡只读 `snapshot()` 的不同切片，控制卡（如 `spin`）经该 client 的下发原语（`move`/`stop_move`）发 `HighCmd`。
- 无 `robot_interface` / 无真机时自动 **STUB**（不收发、`fresh=false`），MCP server 仍能起、注册、列 tool，方便无硬件时跑通链路。
- 固定 **HIGHLEVEL**（读 `HighState` / 下发 `HighCmd`）。控制卡须上真机验证量程+安全后才能上架（见 CONTRIBUTING.md §4）。

## 卡片总览（本 bundle 共 4 张：3 状态卡 + 1 控制卡）

| 卡片（= 文件） | 类型 | 能力 | 输出 / 关键动作 |
|---|---|---|---|
| `loco_state`（`loco_state.py`） | sensor | 运动状态 | `/{ns}/loco/state`：mode / gait / velocity_body_mps / yaw_speed_rad_s / body_height_m / position_m(里程计,漂移) |
| `battery`（`battery.py`） | sensor | 电量(BMS) | `/{ns}/state/battery`：soc_percent / current_ma / cycle_count / temps / cell_voltage_mv |
| `obstacle_range`（`obstacle_range.py`） | sensor | 超声波避障 | `/{ns}/state/obstacle_range`：range_raw[4]（仅 HIGHLEVEL；方向/单位官方未定义，原样输出） |
| `spin`（`spin.py`） | actuator | 原地转圈（闭环） | `action=spin`：转 degrees 度（direction left/right，闭环读 IMU yaw 自停，异步）；`action=stop`：立即停稳打断。前置：狗须已站立 |

`{ns}` 为 `config.yaml` 的 `ros_namespace`（默认 `bundle`；留空则取 hostname）。

## 卡片装配约定（关键）

**卡名 == 模块名 == 文件名 == config.yaml 里的 key。** `main.py` 遍历 `config.yaml` 中
`enabled: true` 的卡名，`import_module(卡名)` 并调用其 `make_plugin(...)` 装配。所以：

> **新增一张卡 = 新建 `<卡名>.py` + 在 `config.yaml` 打开它。不用改 `main.py`。**

这也是拆成“一卡一文件”的意义：每个人提交自己的 `<卡名>.py`，互不冲突。

## 接口约定（与平台其它驱动一致）

- **状态卡读取**：无业务输入。`action=info`（或 `read`/`get`）返回最新数据 + `topic_out`；每条数据带 `timestamp_ms` / `control_level` / `fresh`，**无新包不伪造**。
- **生命周期**：每张卡都处理 `start`/`stop`：`start → {"state":"running"}`、`stop → {"state":"idle"}`。
- **`dispatch()` 返回**：一律 plain dict（或 `None`），由 MCP 处理器自动包 `{"content":[...]}`，**不要**自己预包。
- **ROS2 可选**：装了 rclpy → 按各卡频率发 topic；没装 → 只支持 MCP `action=info` 轮询（`topic_out` 为空）。

## 文件结构

```
go1_bundle/
├── main.py            # MCP server 入口 + 按 config 卡名自动装配（HIGHLEVEL）
├── go1_sdk_client.py  # 共享 raw SDK client：UDP 收 HighState → snapshot()；控制卡经其下发原语发 HighCmd
├── loco_state.py      # 状态卡：运动状态（自包含：builder + MCP 插件 + 可选 ROS2 发布）
├── battery.py         # 状态卡：电池 BMS（自包含）
├── obstacle_range.py  # 状态卡：超声波避障（自包含）
├── spin.py            # 控制卡：原地转圈（闭环，自包含）
├── config.yaml        # 卡片开关 / 端口 / 命名空间
├── driver.yaml        # 驱动元数据（id / port / 描述）
├── requirements.txt   # 运行期 pip 依赖（pyyaml；rclpy/robot_interface 由镜像提供/构建）
├── Dockerfile         # ARM64；镜像内构建 robot_interface.so
├── deploy/service.yml # compose 服务片段
├── README.md          # 本文件
└── CONTRIBUTING.md    # ★ 如何在此基础上新增卡片
```

## 本地运行（无硬件 STUB 亦可）

```bash
cd go1_bundle
pip install -r requirements.txt          # 仅 pyyaml；rclpy/robot_interface 缺失会自动降级
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
# 在仓库根目录，配置好 .env 后：
./build.sh go1_bundle
```

## 端口

`15717`（MCP）。平台驱动端口区间 `15700–15799`；与同机完整 Go1 驱动（`15715`）错开。

---

新增卡片请从 **[CONTRIBUTING.md](CONTRIBUTING.md)** 开始。
