# Phanthy Motus Drivers

[中文文档](README_zh.md)

MCP hardware drivers for the [Phanthy Motus](https://github.com/4paradigm/phanthymotus) embodied AI platform.

Each driver is an MCP HTTP server that exposes hardware capabilities as tools via JSON-RPC 2.0.

## Available Drivers

| Driver | Hardware | Port |
|--------|----------|------|
| `unitree/g1` | Unitree G1 Humanoid Robot | 15701 |
| `phanthy/remote_control` | Remote Control Bridge | 15710 |

## Quick Start

### Docker Build

```bash
cp .env.example .env  # Fill in registry credentials

# Build all drivers
./build.sh unitree/g1
./build.sh phanthy/remote_control
```

### Local Development

Each driver can be run standalone:

```bash
cd unitree/g1
pip install -r requirements.txt
python main.py
```

## Writing a New Driver

See the [Driver Development Guide](README_dev.md) or refer to existing drivers.

### MCP Protocol

Implement these JSON-RPC 2.0 methods:

| Method | Description |
|--------|-------------|
| `initialize` | Handshake, return `serverInfo.name` |
| `tools/list` | Declare tools with `inputSchema` + `configSchema` |
| `tools/call` | Handle tool invocations |

### Tool Naming Convention

`{device}_{action}` — e.g., `loco_move`, `arm_grasp`, `mic_start`

### Directory Structure

```
your_driver/
├── main.py          # MCP server entry point
├── device.py        # Hardware communication
├── config.yaml      # Default configuration
├── driver.yaml      # Driver metadata (name, description, bus types)
├── Dockerfile
└── requirements.txt
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

[Apache License 2.0](LICENSE)
