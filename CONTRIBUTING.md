# Contributing

We welcome contributions! Here's how to get started.

## Development Setup

### Prerequisites

- Python 3.12+
- [uv](https://docs.astral.sh/uv/) package manager
- ROS2 Humble (for DDS features)
- Docker (for building ARM64 images)

### Local Development

```bash
# Clone the repo
git clone https://github.com/4paradigm/phanthymotus-driver.git
cd phanthymotus-driver

# Run a driver locally
cd unitree/g1
pip install -r requirements.txt
python main.py
```

### Building Docker Images

```bash
cp .env.example .env  # Configure registry settings

# Build a specific driver
./build.sh unitree/g1
./build.sh phanthy/remote_control
```

- All Dockerfiles target ARM64 architecture
- Tencent Cloud mirrors are used for PyPI/Docker images
- Image naming: `${REGISTRY}/${IMAGE_NAMESPACE}/${image_name}:${TAG}`

## Project Structure

```
phanthymotus-driver/
├── unitree/
│   └── g1/                  # Unitree G1 Humanoid Robot (port 15701)
│       ├── main.py          # MCP server entry point
│       ├── device.py        # Hardware communication & plugin implementations
│       ├── config.yaml      # Plugin enable/disable configuration
│       ├── driver.yaml      # Driver metadata (ID, port, description)
│       ├── Dockerfile
│       └── requirements.txt
├── phanthy/
│   └── remote_control/      # Remote Control Bridge (port 15710)
│       └── ...
├── build.sh                 # Unified build script
└── README_dev.md            # Driver Development Guide (detailed spec)
```

## Writing a New Driver

For the complete driver development specification, see the **[Driver Development Guide](README_dev.md)**, which covers:

- MCP protocol implementation (JSON-RPC 2.0 methods)
- Tool definition spec (`inputSchema`, `configSchema`, `x-action-params`)
- Plugin lifecycle (`__init__`, `get_tool`, `start`, `stop`, `dispatch`)
- `driver.yaml` and `config.yaml` metadata format
- Registration and heartbeat with Agent Core
- Port allocation (15700–15799 range)

### Quick Reference

Drivers are MCP HTTP servers. Implement these JSON-RPC 2.0 methods:

| Method | Description |
|--------|-------------|
| `initialize` | Handshake, return `serverInfo.name` |
| `tools/list` | Declare tools with `inputSchema` + `configSchema` |
| `tools/call` | Handle tool invocations |

Tool naming convention: `{device}_{action}` (e.g., `loco_move`, `mic_start`, `arm_grasp`)

## Related Repositories

- **[phanthymotus](https://github.com/4paradigm/phanthymotus)** — The main platform (Agent Core + Perception Stack)

## Pull Request Process

1. Fork the repo and create a feature branch
2. Make your changes
3. Test the driver locally with a running Agent Core instance
4. Submit a PR with a clear description

### Review Checklist

Reviewers should confirm each of these before approving a driver change:

- [ ] **Sensitive config fields are declared.** Every `configSchema` property holding
      a credential, token, license key, private endpoint or personal identifier
      declares `"format": "password"` or `"x-sensitive": true`. Canvas config is
      packaged into shareable Solutions and uploaded to the Resource Center;
      packaging can only blank what the tool declares, so an unmarked secret is
      published in clear text. See [README_dev.md](README_dev.md#marking-sensitive-fields-x-sensitive).
- [ ] `dispatch()` returns a plain dict, never a pre-wrapped `[{"type": "text", ...}]`
- [ ] Per-instance state is guarded per the concurrency rules (`tools/call` runs on
      its own thread); `stop` can cancel a concurrent `start`
- [ ] `topic_out[].format` matches the intended dashboard renderer
- [ ] Actuator tools are actuator-typed, so Agent Core's safety confirmation applies

## Code Style

- Python: Follow PEP 8, use type hints where practical
- Keep dependencies minimal
- Each driver should be self-contained in its own directory

## License

By contributing, you agree that your contributions will be licensed under the Apache License 2.0.
