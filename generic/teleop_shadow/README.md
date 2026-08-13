# 通用遥操作 Shadow Driver

这是一个不连接机器人的 Driver-only 遥操作实现，用来先验证 Meta Quest / PICO
原生 OpenXR 客户端、配对、WebRTC 双 DataChannel、帧校验、租约和最终安全停止。
它永久保持 `actuation_enabled=false`，最终输出只记录 `would_apply` 和
`would_stop`。

## 与未修改 Core 的边界

- Core 只按现有 MCP 协议注册和调用卡片，不签发 session、epoch、fence、心跳或
  RTC ticket。
- MCP HTTP 固定监听 loopback，兼容 stock Core 不发送 `Authorization` 的行为；
  非 loopback 请求会被拒绝。
- Driver 本地创建独占 session、单调 epoch、私有 fence 和一次性短效 RTC ticket。
- 头显只连接独立的 TLS/WSS Capture 端口；`/offer` 不对外提供兼容降级路径。
- 只有已配对 Capture WSS 上的合法聚焦状态能够续租。Pose 和 RTC DataChannel
  `heartbeat` 都不能续租。

## 卡片操作

`teleop_session` 是标准 actuator 卡，支持：

- `start`：创建或返回当前 Driver-owned Shadow session；接受 Core 自动注入的
  `instance_id`。
- `info` / `status`：返回卡片、Capture、RTC、Pose、lease 和 final-dispatch 状态。
- `pair_headset`：返回一次性 `pairing_id`、`pairing_code`、明确的 `wss_url` 和
  公共 CA 证书 base64；不会返回 TLS 私钥或 session fence。
- `revoke_headset`：撤销持久化头显凭据并安全释放当前 session。
- `pause`、`release`、`emergency_stop`、生命周期 `stop`：均经过不可丢弃的最终
  safe-stop 路径。

`teleop_state` 是只读状态卡。实时 60Hz/90Hz Pose 永远不走 MCP。

## 两个监听面

| 监听面 | 默认地址 | 用途 |
| --- | --- | --- |
| MCP/health | `http://127.0.0.1:15711` | stock Core 的 `initialize`、`tools/list`、`tools/call`、`/health` |
| Capture | `wss://<机器人地址>:15712/ws/teleop-capture` | 头显配对、凭据重连、presence、assignment 和 SDP |

生产启动会在任何 Capture TLS 配置错误时 fail closed。Capture 证书必须：

- 位于独立目录 `/opt/phanthy-motus/data/teleop-capture-tls`，不要复用或挂载
  Core 私钥；
- 证书 SAN 与 `MOTUS_CAPTURE_WSS_URL` 的 hostname/IP 完全匹配；
- 证书和私钥能组成有效服务端证书对；
- 返回头显的公开 PEM 链解码后不超过 32768 bytes（base64 最多 43692 字符）；
- URL 必须是 `wss://.../ws/teleop-capture`。

示例：为文档保留地址 `192.0.2.110` 创建独立实验室自签证书。请替换为实际
Driver 地址；命令不会向终端输出私钥内容：

```bash
sudo install -d -o 10001 -g 10001 -m 700 \
  /opt/phanthy-motus/data/teleop-capture-tls
sudo openssl req -x509 -newkey rsa:3072 -sha256 -nodes -days 365 \
  -keyout /opt/phanthy-motus/data/teleop-capture-tls/key.pem \
  -out /opt/phanthy-motus/data/teleop-capture-tls/cert.pem \
  -subj '/CN=motus-teleop-capture' \
  -addext 'subjectAltName=IP:192.0.2.110'
sudo chown 10001:10001 \
  /opt/phanthy-motus/data/teleop-capture-tls/key.pem \
  /opt/phanthy-motus/data/teleop-capture-tls/cert.pem
sudo chmod 600 /opt/phanthy-motus/data/teleop-capture-tls/key.pem
sudo chmod 644 /opt/phanthy-motus/data/teleop-capture-tls/cert.pem
export MOTUS_CAPTURE_WSS_URL='wss://192.0.2.110:15712/ws/teleop-capture'
```

首次通过卡片 `pair_headset` 配对后，Driver 只将单设备凭据的 SHA-256 摘要、客户端
类型和版本原子写入配置的 `MOTUS_CAPTURE_STATE_FILE`（0600），并 fsync 文件及父目录，
不落盘原始凭据。
原始凭据由头显保存。Driver 重启后头显可凭该凭据自动重连；它可先保持
`xr_standby`，PC 之后点击卡片 `start` 时 Driver 会自动下发 assignment，无需用户
在头显里再次点击或重新配对。

宿主机状态目录需要预先创建并交给容器内 UID 10001，避免 Docker 自动创建一个
不可写的 root 目录：

```bash
sudo install -d -o 10001 -g 10001 -m 700 \
  /opt/phanthy-motus/data/teleop-shadow
```

多实例部署使用 `deploy/instances.example.yml` 的 schema v4 和
`render_instances.py`。每个实例必须有全局唯一的 MCP/Capture 端口、WSS URL，
并配置一个所有实例共享、只用于注册协调的 `registration_coordination_dir`。先创建
这个独立持久目录并交给容器 UID 10001：

```bash
sudo install -d -o 10001 -g 10001 -m 700 \
  /opt/phanthy-motus/data/teleop-registration
```

生成器要求 Core CA、每个 Capture TLS/状态目录和注册协调目录都已存在，宿主机输入
必须是 canonical 真实绝对路径；叶节点或任一祖先为 symlink 都会被拒绝。它还按真实
路径及文件系统 identity 拒绝同实例/跨实例 alias、相等或祖先/子孙重叠。最终 bind
source 只使用校验后的 canonical 路径，注册协调目录不能包含 Capture 状态、TLS 或
Core CA。

所有多实例容器在该目录共享一个 0600 `flock` 和原子/fsync 的 barrier marker。锁覆盖
完整 Core `POST /api/mcp` 和响应读取；上一请求收到响应，或在 timeout、transport
error、取消边界结束后，下一实例必须等到新的 Unix 秒才发送。这样确定性规避 stock
Core 以整秒生成 MCP id 的碰撞。单实例 manifest 不设置协调文件，因此注册不取锁也
不增加启动等待。生成的 Compose 不包含 Driver Bearer 或共享 RTC ticket secret。

## 本地验证

```bash
cd generic/teleop_shadow
uv run --offline --python 3.14 --with-requirements requirements.txt \
  python -m unittest discover -s tests -v
```

真实本地 Shadow E2E 覆盖：MCP start → 一次性配对 → 原生 wire-compatible
assignment → Driver 内部 RTC ticket → WebRTC 双 DataChannel → Pose
`would_apply` → pause/release，以及断开、focus loss、presence timeout 进入 HOLD。
