# Phanthy Motus 硬件驱动

[English](README.md)

[Phanthy Motus](https://github.com/4paradigm/phanthymotus) 具身智能平台的 MCP 硬件驱动集合。

每个驱动是一个 MCP HTTP 服务器，通过 JSON-RPC 2.0 将硬件能力暴露为工具。

## 可用驱动

| 驱动 | 硬件 | 端口 |
|------|------|------|
| `unitree/g1` | Unitree G1 人形机器人 | 15701 |
| `phanthy/remote_control` | 远程控制桥接 | 15710 |

## 快速开始

### Docker 构建

```bash
cp .env.example .env  # 填写镜像仓库凭据

# 构建驱动
./build.sh unitree/g1
./build.sh phanthy/remote_control
```

### 本地开发

每个驱动可独立运行：

```bash
cd unitree/g1
pip install -r requirements.txt
python main.py
```

## 开发新驱动

参考现有驱动或阅读 [驱动开发指南](README_dev.md)。

### MCP 协议

实现以下 JSON-RPC 2.0 方法：

| 方法 | 说明 |
|------|------|
| `initialize` | 握手，返回 `serverInfo.name` |
| `tools/list` | 声明工具（含 `inputSchema` + `configSchema`）|
| `tools/call` | 处理工具调用 |

### 工具命名规范

`{设备}_{动作}` — 如 `loco_move`、`arm_grasp`、`mic_start`

### 目录结构

```
your_driver/
├── main.py          # MCP 服务器入口
├── device.py        # 硬件通信
├── config.yaml      # 默认配置
├── driver.yaml      # 驱动元数据（名称、描述、总线类型）
├── Dockerfile
└── requirements.txt
```

## 贡献

参见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 许可证

[Apache License 2.0](LICENSE)
