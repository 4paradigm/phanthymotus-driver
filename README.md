# Phanthy Motus Drivers

[中文文档](README_zh.md) | [Official Website](https://motus.phanthy.com)

Hardware drivers for the **[Phanthy Motus](https://github.com/4paradigm/phanthymotus)** embodied AI platform.

Each driver is a standalone [MCP](https://modelcontextprotocol.io) HTTP server that exposes hardware capabilities as tools. Drivers automatically register with the [Phanthy Motus Agent Core](https://github.com/4paradigm/phanthymotus) on startup.

## Available Drivers

| Driver | Hardware | Port | Description |
|--------|----------|------|-------------|
| `unitree/g1` | Unitree G1 Humanoid | 15701 | Locomotion, arm control, mic, speaker, LED, state monitoring |
| `phanthy/remote_control` | Remote Control Bridge | 15710 | Remote control relay |

## Quick Start

### Prerequisites

- Docker (ARM64)
- A running [Phanthy Motus Agent Core](https://github.com/4paradigm/phanthymotus) instance

### Deploy via Web Dashboard (Recommended)

The easiest way to deploy a driver is through the Agent Core Web Dashboard. Navigate to **Deploy** in the top menu — you can browse reviewed and published driver versions, select a version, and deploy with one click. No manual build required.

### Build & Deploy Your Own Driver

If you need to build from source or develop a custom driver:

```bash
cp .env.example .env  # Fill in registry credentials

# Build a specific driver
./build.sh unitree/g1
```

When run without arguments, `build.sh` shows an interactive multi-select menu to choose which drivers to build. You can also pass driver paths directly for CI usage:

```bash
# Build multiple drivers
./build.sh unitree/g1 phanthy/remote_control
```

Once the driver container starts, it registers itself with Agent Core at `http://<agent-core>:15678/api/mcp`. You can then see the device and its tools in the Web Dashboard.

### Run Locally (without Docker)

```bash
cd unitree/g1
pip install -r requirements.txt
python main.py
```

## How It Works

1. Driver starts as an MCP HTTP server on its designated port
2. Driver sends a registration request to Agent Core
3. Agent Core discovers the driver's tools via MCP `initialize` and `tools/list`
4. Tools become available to the LLM agent and appear in the Web Dashboard
5. The LLM agent can invoke tools via MCP `tools/call`

## Writing a New Driver

Want to add support for new hardware? See the **[Driver Development Guide](README_dev.md)** for the full specification, including:

- MCP protocol implementation (JSON-RPC 2.0 methods)
- Tool definition spec (`inputSchema`, `configSchema`, `multiInstance`, `x-action-params`)
- Instance management (`multiInstance` flag, `scope` for config fields)
- Plugin lifecycle (`__init__`, `get_tool`, `start`, `stop`, `dispatch`)
- `driver.yaml` and `config.yaml` metadata format
- Registration and heartbeat with Agent Core
- Port allocation (15700–15799 range)

Quick overview:
- Each driver implements MCP JSON-RPC 2.0 over HTTP (`initialize`, `tools/list`, `tools/call`)
- Tool naming convention: `{device}_{action}` (e.g., `loco_move`, `mic_start`)
- Driver port range: **15700–15799**

### Topic Inference via `info` Action

**All** tools that produce or consume ROS2 topics must implement an `info` action. The Agent Core canvas calls `info(instance_id, input_topic)` immediately after a card is placed or wired, and uses the returned `topic_out`/`topic_in` as the **authoritative** topic path. Static definitions in `tool.topic_out` are used only as a fallback when `info()` is unavailable.

**Rule: driver owns topic path logic; canvas only reads the result.**

| Tool type | `info()` input | `topic_out` computation |
|-----------|---------------|------------------------|
| Static sensor (mic, imu, camera…) | — | Return fixed `self._topic` |
| multiInstance sensor (ext_mic, ext_camera) | `instance_id` | `/{namespace}/{tool}/{instance_id}/…` (replace `-` → `_`) |
| Processor (asr, tts) | `input_topic` | `{input_topic}/{tool_name}` |

Example for a static sensor:
```python
def dispatch(self, action: str, args: dict) -> dict | None:
    if action == "info":
        return {"state": self.state, "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}]}
    return None
```

> **Note:** ROS2 topic names only allow alphanumerics, `_`, `~`, `{`, `}`. Canvas card IDs
> contain hyphens (e.g. `card-abc123`), so drivers must sanitize `instance_id` before
> embedding it in a topic path: `instance_id.replace('-', '_')`.

The Agent Core canvas calls this endpoint immediately after a card is placed or wired,
so output port labels are populated without waiting for `start`.

See [CONTRIBUTING.md](CONTRIBUTING.md) for development setup and PR guidelines.

## License

[Apache License 2.0](LICENSE)
