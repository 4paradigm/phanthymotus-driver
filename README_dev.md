# Driver 开发指南

硬件驱动层（Layer 1），以 MCP HTTP Server 形式暴露设备能力给 Agent Core。

---

## 目录结构

每个 driver 是一个独立的 Python 包：

```
drivers/
├── <provider>/
│   └── <model>/
│       ├── main.py            # MCP HTTP Server 入口
│       ├── device.py          # 设备插件实现
│       ├── config.yaml        # 插件启用配置
│       ├── driver.yaml        # 元数据（ID、端口、描述）
│       ├── Dockerfile         # ARM64 容器构建
│       └── requirements.txt   # Python 依赖
```

示例：`drivers/unitree/g1/`、`drivers/phanthy/remote_control/`

---

## MCP 协议

每个 driver 实现 [MCP](https://modelcontextprotocol.io) JSON-RPC 2.0 over HTTP，暴露三个方法：

| Method | 说明 |
|--------|------|
| `initialize` | 握手，返回 `serverInfo.name` |
| `tools/list` | 列出所有工具（含 schema） |
| `tools/call` | 调用工具 `{name, arguments}` |

HTTP endpoint 统一为 `/mcp`（POST）。

---

## 工具定义规范

每个工具返回一个 dict，包含以下字段：

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `name` | string | 是 | 工具名（如 `loco`、`mic`），在同一 driver 内唯一 |
| `type` | string | 是 | `sensor`（数据流）\| `actuator`（可执行）\| `processor`（数据处理）\| `resource`（静态资源） |
| `description` | string | 是 | 工具描述，LLM 和前端都会使用 |
| `inputSchema` | object | 是 | JSON Schema，定义调用参数 |
| `configSchema` | object | 否 | 持久化配置 schema（如 API Key），前端渲染为配置表单 |
| `topic_out` | array | 否 | 输出的 ROS2 DDS topic 列表 `[{topic, format}]` |
| `topic_in` | array | 否 | 输入的 ROS2 DDS topic 列表 `[{format}]` |

### 工具类型

- **sensor**: 数据流工具，不可直接调用。通过 `start`/`stop` 系统 action 控制，数据通过 ROS2 topic 推送
- **actuator**: 可执行动作的工具。通过 `action` 字段分发不同操作
- **processor**: 数据处理工具。接收输入 topic 数据，处理后输出到 topic

### inputSchema

标准 JSON Schema 格式。对于 actuator 工具，通常包含 `action` 字段（enum）来区分不同操作：

```python
"inputSchema": {
    "type": "object",
    "properties": {
        "action": {
            "type": "string",
            "enum": ["move", "stop"],
            "description": "Action to perform",
        },
        "vx": {"type": "number", "description": "Forward velocity"},
    },
    "required": ["action"],
}
```

### configSchema

可选。定义需要用户在前端配置的持久化参数（如 API Key、模型名）。前端自动渲染配置表单。

```python
"configSchema": {
    "type": "object",
    "properties": {
        "api_key": {"type": "string", "description": "API Key", "format": "password"},
        "model":   {"type": "string", "description": "Model name"},
    },
    "required": ["api_key"],
}
```

---

## x-action-params 规范

### 问题

当一个工具有多个 action 且不同 action 需要不同参数时（如 `loco` 的 `move` 需要速度参数，`stop` 不需要），所有参数被 union 到一个 flat schema 中，导致：

1. LLM 看到所有参数混在一起，无法区分哪些属于哪个 action
2. 前端同时显示所有字段，用户体验差

### 解决方案

在 `inputSchema` 中声明 `x-action-params` 字段，为每个 action 指定对应的参数列表和独立描述。

### 格式

```python
"inputSchema": {
    "type": "object",
    "properties": {
        "action": {"type": "string", "enum": ["move", "stop", "set_stand_height"], ...},
        "vx":     {"type": "number", "description": "Forward velocity"},
        "height": {"type": "number", "description": "Standing height 0.0-1.0"},
    },
    "required": ["action"],
    "x-action-params": {
        "move":             {"params": ["vx", "vy", "vyaw"], "description": "Move the robot with velocities"},
        "stop":             {"params": [],                    "description": "Stop all movement"},
        "set_stand_height": {"params": ["height"],            "description": "Set standing height"},
    },
}
```

每个 action 条目：

| 字段 | 类型 | 说明 |
|------|------|------|
| `params` | string[] | 该 action 使用的参数 key 列表（`action` 字段本身无需列入） |
| `description` | string | 该 action 的独立描述，用于 LLM function description |

### 效果

Agent Core 会自动处理 `x-action-params`：

- **LLM 侧**：自动拆分为多个独立 function（如 `mcp__unitree__loco__move`、`mcp__unitree__loco__stop`），每个只包含对应参数
- **前端侧**：canvas 卡片中切换 action 下拉框时，只显示对应参数字段
- **Driver 侧**：无需任何调度逻辑变化，Agent Core 调用时自动注入 `action` 到 args

### 何时使用

- 工具有多个 action，且**不同 action 需要不同参数**时必须使用
- 所有 action 共用相同参数时不需要（如 `switch_mode` 的所有 mode 都只需 `mode` 字段）
- 单 action 工具不需要

### 完整示例

```python
def get_tool(self) -> dict:
    return {
        "name": "loco",
        "type": "actuator",
        "description": "G1 locomotion control — move, stop, set height, wave/shake hand",
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": ["move", "stop", "set_stand_height", "wave_hand", "shake_hand"],
                    "description": "Action to perform",
                },
                "vx":         {"type": "number",  "description": "Forward velocity m/s [-1, 1]"},
                "vy":         {"type": "number",  "description": "Lateral velocity m/s [-1, 1]"},
                "vyaw":       {"type": "number",  "description": "Yaw rotation rad/s [-2, 2]"},
                "continuous": {"type": "boolean", "description": "Keep moving until stop (default false)"},
                "height":     {"type": "number",  "description": "Normalized height 0.0-1.0"},
                "turn":       {"type": "boolean", "description": "Turn while waving (default false)"},
            },
            "required": ["action"],
            "x-action-params": {
                "move":             {"params": ["vx", "vy", "vyaw", "continuous"], "description": "Move the robot with specified velocities"},
                "stop":             {"params": [],                                 "description": "Stop all movement immediately"},
                "set_stand_height": {"params": ["height"],                         "description": "Set the robot's standing height (0.0-1.0)"},
                "wave_hand":        {"params": ["turn"],                           "description": "Perform a waving hand gesture"},
                "shake_hand":       {"params": [],                                 "description": "Perform a handshake gesture"},
            },
        },
    }
```

---

## Plugin 生命周期

每个设备功能封装为一个 Plugin 类，需实现：

```python
class MyPlugin:
    PREFIX = "my_tool"  # 工具名前缀（用于多工具插件）

    def __init__(self, plugin_config: dict, namespace: str, executor, ...):
        """初始化。plugin_config 来自 config.yaml，namespace 是 ROS2 命名空间。"""
        pass

    def get_tool(self) -> dict:
        """返回单个工具定义。"""
        # 或 get_tools(self) -> list 返回多个

    def start(self) -> None:
        """启动插件（如开始采集数据）。"""
        pass

    def stop(self) -> None:
        """停止插件。"""
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        """分发工具调用。action 从 args 中 pop 出来，args 包含剩余参数。"""
        if action == "do_something":
            return {"result": "ok"}
        return None
```

- 提供 `get_tool()` 返回单工具，或 `get_tools()` 返回多个
- `dispatch()` 中 `action` 已从 args 中提取，若无 action 字段则等于 tool name
- sensor 类型工具的 dispatch 通常返回 None（数据通过 topic 推送）

---

## driver.yaml 元数据

```yaml
id: g1-driver                   # 唯一 ID
name: Unitree G1 Bundle          # 显示名
category: driver                 # 固定 "driver"
hardware_provider: unitree       # 硬件厂商
hardware_model: "g1"             # 硬件型号
image_name: g1                   # Docker 镜像名（不含 registry 前缀）
port: 15701                      # MCP HTTP 端口
mcp_url: "http://localhost:15701/mcp"  # MCP endpoint
description: "..."               # 设备描述
```

---

## config.yaml

控制插件启用：

```yaml
mcp_port: 15701
ros_namespace: ""   # 留空自动使用 hostname

plugins:
  mic:
    enabled: true
  tts:
    enabled: true
  speaker:
    enabled: true
  led:
    enabled: true
  loco:
    enabled: true
  arm:
    enabled: true
  state:
    enabled: true
```

路径通过 `CONFIG_PATH` 环境变量指定（默认同目录下）。

---

## 注册与心跳

Driver 启动后自动向 Agent Core（port 15678）注册：

```
POST http://<agent-core>:15678/api/mcp
{
  "id": "g1-driver",
  "name": "Unitree G1 Bundle",
  "url": "http://<driver-ip>:15701/mcp",
  "transport": "http"
}
```

Agent Core 收到后执行 `initialize` → `tools/list`，注册工具到 registry。

---

## 端口规范

Driver 端口分配在 **15700–15799** 范围：

| Driver | Port |
|--------|------|
| Unitree G1 | 15701 |
| Phanthy Remote Control | 15710 |

新 driver 应选择未占用的端口，WebSocket 端口通常为 MCP 端口 +1。

---

## 构建与部署

```bash
# 从 drivers/ 根目录构建
./build.sh <provider>/<model>   # e.g. ./build.sh unitree/g1

# 或手动 Docker 构建
cd drivers/unitree/g1
docker build -t g1-driver .
```

- 所有 Dockerfile 基于 ARM64 架构
- 使用 Tencent Cloud 镜像源加速
- 镜像命名格式：`${REGISTRY}/${IMAGE_NAMESPACE}/${image_name}:${TAG}`
- 环境变量配置见 `.env.example`
