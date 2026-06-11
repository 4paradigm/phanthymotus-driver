# Phanthy Motus 硬件驱动

[English](README.md) | [官网](https://motus.phanthy.com)

**[Phanthy Motus](https://github.com/4paradigm/phanthymotus)** 具身智能平台的硬件驱动集合。

每个驱动是一个独立的 [MCP](https://modelcontextprotocol.io) HTTP 服务器，将硬件能力暴露为工具。驱动启动后自动注册到 [Phanthy Motus Agent Core](https://github.com/4paradigm/phanthymotus)。

## 可用驱动

| 驱动 | 硬件 | 端口 | 说明 |
|------|------|------|------|
| `unitree/g1` | Unitree G1 人形机器人 | 15701 | 运动控制、机械臂、麦克风、扬声器、LED、状态监控 |
| `phanthy/remote_control` | 远程控制桥接 | 15710 | 远程控制中继 |

## 快速开始

### 环境要求

- Docker（ARM64）
- 一个运行中的 [Phanthy Motus Agent Core](https://github.com/4paradigm/phanthymotus) 实例

### 通过 Web Dashboard 部署（推荐）

最简单的方式是通过 Agent Core 的 Web Dashboard 部署驱动。在顶部菜单进入 **部署** —— 可以浏览已审核发布的驱动版本，选择版本后一键部署，无需手动构建。

### 构建和部署自定义驱动

如果需要从源码构建或开发自定义驱动：

```bash
cp .env.example .env  # 填写镜像仓库凭据

# 构建指定驱动
./build.sh unitree/g1
```

不传参数时，`build.sh` 会显示交互式多选菜单，选择要构建的驱动。也可以直接传路径用于 CI：

```bash
# 构建多个驱动
./build.sh unitree/g1 phanthy/remote_control
```

驱动容器启动后会自动向 Agent Core（`http://<agent-core>:15678/api/mcp`）发送注册请求。注册成功后即可在 Web Dashboard 中看到设备及其工具。

### 本地运行（无需 Docker）

```bash
cd unitree/g1
pip install -r requirements.txt
python main.py
```

## 工作原理

1. 驱动作为 MCP HTTP 服务器在指定端口启动
2. 驱动向 Agent Core 发送注册请求
3. Agent Core 通过 MCP `initialize` 和 `tools/list` 发现驱动的工具
4. 工具对 LLM Agent 可用，并显示在 Web Dashboard 中
5. LLM Agent 通过 MCP `tools/call` 调用工具

## 开发新驱动

想要为新硬件添加驱动？请参阅 **[驱动开发指南](README_dev.md)** 获取完整规范，包括：

- MCP 协议实现（JSON-RPC 2.0 方法）
- 工具定义规范（`inputSchema`、`configSchema`、`x-action-params`）
- Plugin 生命周期（`__init__`、`get_tool`、`start`、`stop`、`dispatch`）
- `driver.yaml` 和 `config.yaml` 元数据格式
- 注册与心跳机制
- 端口分配（15700–15799 范围）

简要概述：
- 每个驱动实现 MCP JSON-RPC 2.0 over HTTP（`initialize`、`tools/list`、`tools/call`）
- 工具命名规范：`{设备}_{动作}`（如 `loco_move`、`mic_start`）
- 驱动端口范围：**15700–15799**

参见 [CONTRIBUTING.md](CONTRIBUTING.md) 了解开发环境搭建和 PR 指南。

## 许可证

[Apache License 2.0](LICENSE)
