# 在 go1_bundle 上新增卡片 · 开发指南

本文件教你在这个最小蓝本上**新增一张能力卡片**。核心约定：

> **每张卡是一个自包含的 `class Plugin` + `make_plugin()`。** 聚合文件（`sensors.py`/`controllers.py`/`ext_devices.py`）内按卡分隔，**卡名 == config.yaml 里的 key == make_plugin 的 make_ 前缀**。
> 新增一张卡 = 在对应聚合文件中追加一个 `Plugin` 类 + `make_<卡名>` + 在 `config.yaml` 打开它，**同时需要在 `main.py` 的 `Go1Bundle.__init__` 方法中添加对应导入和创建逻辑**。

先读完“心智模型”和“插件契约”，再照“新增状态卡 / 新增控制卡”步骤照做。

---

## 1. 心智模型

```
                    ┌───────────────── main.py ─────────────────┐
robot_interface     │  遍历 config.plugins 中 enabled 的卡名：    │
(HighState) ──UDP──▶│    import_module(卡名).make_plugin(...)     │
                    │                                            │
                    │   ┌─ go1_sdk_client.Go1HighSdkClient ─┐    │
                    │   │  唯一 UDP 收发线程                  │    │
                    │   │  _parse_state() → snapshot()(dict) │◀── 所有状态卡都读它
                    │   └────────────────────────────────────┘    │
                    │                                            │
                    │   plugins: [loco_state, battery, ...]      │──▶ tools/list, tools/call
                    └────────────────────────────────────────────┘
```

关键点：

- **按组聚合、装配**：`main.py` 按以下规则装配卡片：
  1. 读取 `config.yaml` 中 `enabled` 的卡名
  2. 根据卡类型从对应模块导入：传感器卡从 `sensors`、控制卡从 `controllers`、外设卡从 `ext_devices`
  3. 使用 `getattr(module, f"make_{card_name}")` 调用对应工厂函数
  4. 要求模块中必须存在 `make_<卡名>` 函数
  传感卡都在 `sensors.py`（导出 `make_battery`/`make_imu`/`make_joints`…），
  控制卡都在 `controllers.py`（导出 `make_loco`/`make_gesture`…），
  外部设备都在 `ext_devices.py`（导出 `make_beep`/`make_speaker`…）。
  因此卡与卡之间零耦合，每人改不同文件，提交不撞车。
- **共享快照**：`Go1HighSdkClient.snapshot()` 已把整帧 `HighState` 解析成 dict（不止两张卡用到的字段，
  `imu`/`joints`/`foot_*`/`range_obstacle`/`wireless_remote` 都在里头）。**新增状态卡通常不用碰
  `go1_sdk_client.py`**，直接从 `snapshot()` 取字段即可。
- **STUB 优先**：没有真机 / 没有 `robot_interface` 时一切照跑，只是数据为空、`fresh=false`。开发全程可在开发机上做。

> 注：`sensors.py` 内的 `battery` 和 `imu` 各带了一份几乎相同的插件样板（ROS2 发布 + `dispatch`）。
> 这是"每张卡行为独立"的刻意取舍；等卡稳定后可把公共部分抽成一个 `card_base.py` 再合并，不影响对外契约。
>
> **STUB 模式行为**：没有真机 / 没有 `robot_interface` 时：
> - 所有数值字段返回 0 或合理的默认值
> - `fresh: false`，`timestamp_ms` 为当前时间
> - 控制卡返回 `{ok: false, code: "STUB_MODE", message: "Running in STUB mode"}`
> - 可以通过环境变量 `STUB_DATA_PATTERN` 指定模拟数据文件

---

## 2. 插件契约

每个卡模块必须导出一个工厂函数：

```python
def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
```

**文件结构约定**：
- `sensors.py`：包含所有传感器卡片（battery, imu, joints, model 等）
- `controllers.py`：包含所有控制卡片（loco, body_pose, gesture 等）  
- `ext_devices.py`：包含所有外部设备卡片（beep, speaker, face_light 等）
- `camera.py`：包含相机相关卡片

`main.py` 只对返回的 plugin 对象调用这几个方法：

| 方法 | 作用 | 返回 |
|---|---|---|
| `get_tool(self)` | 声明这张卡的 MCP 工具（`name`/`type`/`description`/`inputSchema`/可选 `topic_out`） | 一个 dict（多工具卡可改实现 `get_tools()` 返回 list） |
| `start(self)` / `stop(self)` | 生命周期钩子 | — |
| `dispatch(self, action, args)` | 处理一次调用 | plain dict 或 `None`（`None` = 未知 action） |

`make_plugin` 的入参：`plugin_config` 是 `config.yaml` 里该卡的子 dict；`namespace` 用于填 topic；
`executor` 是 rclpy executor（可能为 `None`）；`client` 是共享的只读 SDK client。

约定（务必遵守，保持与平台其它驱动一致）：

- `dispatch` **返回 plain dict**，不要自己包 `{"content":[...]}`——MCP 处理器会包。未知 action 返回 `None`。
- 生命周期返回：状态/外设卡 `start → {"state":"running"}`、`stop → {"state":"idle"}`；控制卡 `start → {"state":"ready"}`。
- 状态卡数据一律带 `timestamp_ms` / `control_level` / `fresh`（见各卡 `build()` 里的公共头），**无新包不伪造**。
- 工具命名：状态卡用名词（`battery`、`imu`）；控制卡用 `{device}_{action}` 或单工具 + `action` enum（见 §4）。

---

## 3. 新增一张**状态卡**（最常见，2 步）

状态卡都在 `sensors.py` 内。以加一张 `imu` 卡为例——`snapshot()["imu"]` 已经有数据。

**① 打开 `sensors.py`，在 `model.py` 块后面追加 `imu` 的 Plugin + `make_imu`（参照同文件内 `battery.py` 块的样板）：**

```python
# 追加到 sensors.py 末尾（其余样板照抄同文件内的 battery 插件：ROS2 import、Plugin 类、make_plugin）
_CARD_IMU = "imu"
_TOPIC_IMU = "/{ns}/state/imu"
_HZ_IMU = 20.0
_NODE_IMU = "go1_imu"
_DESC_IMU = "Go1 IMU — quaternion(wxyz)/gyro/accel/rpy/temp; attitude_may_drift on acceleration"

def _build_imu(snap):
    d = {"timestamp_ms": int(time.time() * 1000),
         "control_level": snap.get("control_level", "HIGHLEVEL"),
         "fresh": bool(snap.get("fresh", False))}
    imu = snap.get("imu")
    if imu is None:
        d["available"] = False
        return d
    d.update(imu)                 # quaternion_wxyz / gyroscope / accelerometer / rpy / temperature_c
    d["attitude_may_drift"] = True
    return d
```

**② 在 `config.yaml` 打开它：**

```yaml
  imu:
    enabled: true
```

完成——**需要在 `main.py` 的 `Go1Bundle.__init__` 中添加：**
```python
if pc.get("imu", {}).get("enabled", False):
    import sensors
    self._plugins.append(sensors.make_imu(pc["imu"], namespace, executor, client))
    print("[bundle] imu loaded")
```
构建镜像时会自动包含整个 `sensors.py` 文件，无需为每个卡单独复制。

`fmt` 常用 `data/json`（画布当 JSON 渲染）；需要特殊渲染器时用对应格式（如骨骼 `sensor/skeleton`，那种卡还需附带 URDF `model` 资源工具，超出本蓝本范围）。

> 如果你要的字段 **snapshot 里还没有**：在 `go1_sdk_client.Go1HighSdkClient._parse_state()` 里从 `HighState`
> 多解析一个字段塞进 `out`（字段名对照官方 `comm.h`），然后回到上面两步。

---

## 4. 新增一张**控制卡**（会下发命令，须格外小心）

本蓝本是**只读**的：`go1_sdk_client.py` 刻意不含任何下发原语，client 的 `_loop()` 只发
`InitCmdData` 初始化的空闲 `HighCmd`（`mode=0`）作心跳。要加控制卡，需要：

**① 让 client 能下发命令。** 在 `Go1HighSdkClient` 上加加锁的控制原语，并让 `_loop` 下发被合成的 `cmd`：

```python
def move(self, vx, vy, vyaw, gait=1):
    # 量程校验后加锁写 self._cmd（mode=2/velocity/yawSpeed），后台 _loop 会 SetSend 下发
    ...
```
> ⚠️ 高层运控走 `HighCmd`（目标 `.161:8082`）；低层关节控制走 `LowCmd`+`Safety`（目标 `.10:8007`），二者**互斥**、不能同实例同时用。若做低层卡需另起一个 LOWLEVEL client 并引入 `control_level` 开关。

**② 在 `controllers.py` 或 `ext_devices.py` 末尾追加控制卡的 `Plugin` + `make_<卡名>`（参照同文件内已有卡块的样板）**。控制卡的 `inputSchema` 含 `action`(enum)；多参数动作建议用 `x-action-params` 拆分，平台会把每个动作拆成可单独调用的函数。返回约定：

- 成功：`{ok: true, card, action, control_level, applied, timestamp_ms}`
- 失败：`{ok: false, code, message}`，`code` ∈ `INVALID_ARGUMENT` / `PRECONDITION_FAILED` / `SAFETY_LIMIT` / `RESOURCE_BUSY` 等。
- **越界 / 缺 confirm 一律拒绝，不静默截断**。
- **安全控制**：危险动作（关电、特殊动作、低层）必须在 `inputSchema` 中标记 `x-is-dangerous: true`，并要求用户通过 `confirm` 参数确认（传 `true` 才执行）。

**③ 在 `config.yaml` 打开它**，**同时在 `main.py` 的 `Go1Bundle.__init__` 中添加对应导入和创建逻辑**。控制卡必须**上真机验证量程与安全**后才能上架。

---

## 5. 元数据与部署要一并改

新增卡后按需更新：

- `config.yaml`：加该卡的 `enabled` 开关（及卡自己的参数）。
- `driver.yaml`：`description` 补上新卡能力（评审/上架看这里）。
- `Dockerfile`：**新增聚合文件时添加 `COPY <文件名> /work/<文件名>`**（目前 4 个聚合文件 `sensors.py`/`controllers.py`/`ext_devices.py`/`camera.py` 各一行）；用到新系统包（如音频 `alsa-utils`）在 `apt-get install` 里补上；新增 pip 依赖写进 `requirements.txt`（注意版本约束，如 `numpy>=1.20.0`）。
- `deploy/service.yml`：一般不用改，除非新卡需要额外挂载 / 端口 / 环境变量。
- `main.py`：**需在 `Go1Bundle.__init__` 中添加对应导入和创建逻辑**。

---

## 6. 本地自测（无硬件即可）

```bash
python3 -c "import ast,glob; [ast.parse(open(f).read()) for f in glob.glob('*.py')]"   # 语法
python3 main.py &                        # STUB 起服务（数据空、fresh=false 属正常）
# 列出卡片，确认你的新卡在里面：
curl -s localhost:15717/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list"}' | python3 -m json.tool
# 调你的新卡：
curl -s localhost:15717/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"<你的卡>","arguments":{"action":"info"}}}'
```

有真机时用 `NETWORK_INTERFACE` 指向 Go1 板载网口，验证 `fresh=true` 且数值合理；控制卡务必悬空 / 低速实测。

## 7. 提交前检查清单

- [ ] 新卡在对应聚合文件（`sensors.py`/`controllers.py`/`ext_devices.py`）末尾，导出 `make_plugin(...)`，遵守 §2 契约。
- [ ] 卡名/key/文件名/`NODE` 一致且唯一；`config.yaml` 已 `enabled: true`。
- [ ] `Dockerfile` 已加 `COPY <卡名>.py`；`driver.yaml` 描述已更新；新 pip 依赖进 `requirements.txt`。
- [ ] `dispatch` 返回 plain dict；未知 action 返回 `None`；数据带 `timestamp_ms`/`control_level`/`fresh`。
- [ ] 控制卡：量程 / confirm / 错误码齐全，且已上真机验证。
- [ ] `tools/list` 能看到新卡，`tools/call` 行为符合预期（STUB + 真机各测一遍）。
- [ ] Docker 镜像能构建（`./build.sh go1_bundle`）。

## 8. 相关

- 平台完整的驱动开发规范见仓库根 `README_dev.md`（MCP JSON-RPC 方法、`inputSchema`/`configSchema`/`x-action-params`、注册与心跳、端口分配 15700–15799）。
- 更完整的 Go1 卡片实现（运控 / 相机 / 音频 / 面灯等）可参考同仓库其它 go1 bundle。
