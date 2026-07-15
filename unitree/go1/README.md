# Unitree Go1 Sensor Driver (PhanthyMotus)

Unitree Go1 四足机器狗的 PhanthyMotus 传感器驱动。以 `unitree/go2/` 为模板、**MCP 对外接口对齐 Go2**;
只暴露 **3 张已实机验证的只读状态卡**,用「离线 fake 自动降级」让整套代码在没有 ROS2/SDK/硬件的
开发机上也能 import / 运行 / 测试。

**卡片(全部 sensor,只读):`loco_state` · `imu` · `feet`。** MCP over HTTP,端口 `15704`。

---

## 1. 卡片一览

| 卡片 | 类型 | 频率 | topic | 内容 |
|---|---|---|---|---|
| `loco_state` | sensor | 10Hz | `/{ns}/loco/state` | 模式/步态/里程 position/机身高度/速度 |
| `imu` | sensor | 20Hz | `/{ns}/state/imu` | 四元数(wxyz)/角速度/加速度/欧拉角/温度 |
| `feet` | sensor | 10Hz | `/{ns}/state/feet` | 四足足底力原始值(高层另带足端相对机身位姿) |

读取方式:`tools/call` → `{"name":"<卡>","arguments":{"action":"read"}}`(纯只读,core 只出数据流、不给执行按钮)。
另有一个 `model`(resource)工具返回 `resource/go1_model.urdf` 供网页骨架渲染。

---

## 2. 架构

```
   LLM / Dashboard ──HTTP JSON-RPC──►  Agent Core (:15678) ──ROS2──► 前端
                                            ▲ 注册/心跳        ▲ topic 中继(数据面)
                              ┌─────────────┴───────────────────┴──────────┐
                              │            go1 驱动 (:15704)                 │
                              │  main.py ── Bundle + MCP server + 注册        │
                              │     ├── plugins/mt_state.py  3 张只读卡        │
                              │     ├── go1_ctrl.py  数据后端(读真实 HighState)│
                              │     └── ros_bridge.py  rclpy 隔离(有/无都能跑) │
                              └──────────────────────│──────────────────────┘
                                        后台线程读取  │ UDP HighState
                                                      ▼   Unitree Go1
```

**两个关键机制**
- **离线 fake 自动降级**:数据后端 `mt.backend=factory`(狗上工厂高层 `.so` 读真实 `HighState`);
  开发机 import 不到 `.so` → 自动降 fake,整套照跑、离线可测。
- **rclpy 隔离(`ros_bridge.py`)**:除它外无任何模块顶层 import rclpy;无 ROS2 时发布变 no-op +
  线程定时器。→ 无 ROS2 的开发机也能起。

> 本驱动只读、不下发运动。数据面经 Agent Core 的 ROS2 topic 中继呈现到前端 DATA STREAMS。

---

## 3. 文件清单

| 文件/目录 | 作用 |
|---|---|
| `main.py` | 入口:读 config → 起数据后端 + ros_bridge → 注册 3 张卡 → MCP server + 注册 Agent Core |
| `go1_ctrl.py` | Go1 经典 SDK 控制/状态后端(本驱动只用其状态读取;factory 读真实 HighState,无 SDK 降 fake) |
| `go1_hl.py` | Go1 高层客户端(框架依赖,保留;本传感器驱动不下发运动) |
| `ros_bridge.py` | rclpy 隔离层(real 发 topic / 无 rclpy 离线 no-op) |
| `plugins/mt_state.py` | 状态卡实现(本驱动注册 `LocoStateCard`/`ImuCard`/`FeetCard`) |
| `plugins/mt_base.py` `plugins/base.py` | 卡片基类 / 返回包络 |
| `config.yaml` | 端口、后端、卡片参数 |
| `driver.yaml` `Dockerfile` `requirements.txt` `resource/` | 元数据 / 构建 / 依赖 / URDF |

---

## 4. 运行

```bash
# 开发机离线跑(无 ROS2/SDK/硬件,自动 fake)
python3 main.py

# 查工具
curl -s http://localhost:15704/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list"}'

# 读一张卡
curl -s http://localhost:15704/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"feet","arguments":{"action":"read"}}}'
```

狗上运行:`mt.backend=factory` 需要工厂高层 `.so`(`robot_interface_high_level`,cpython-37);
容器构建见 `Dockerfile`(`../../build.sh unitree/go1`)。

---

## 5. 与 Go2 的关系

以 `unitree/go2/` 为模板,MCP 接口对齐;通信底层换成 Go1 的高层 UDP(读 `HighState`)。
本驱动聚焦 3 张实机验证过的只读传感器卡;运动/避障/语音/视频/导航等能力不在本次范围。
