# Unitree G1_23 Driver-owned 遥操作

G1 根 Driver 在不修改 Agent Core 的前提下提供三张 MCP 卡：
`teleop_session`、`teleop_state` 和零输出的 `teleop_ik`。V1 接收
Quest/PICO 原生 OpenXR 头部与双控制器姿态，只控制 G1_23 的十个手臂关节；
底盘和塑胶假手均不输出。

## 运行边界

- `http://127.0.0.1:15701/mcp`：只供机器人本机的 stock Core 调用，不要求
  Driver Bearer，不依赖 Core 颁发 session、epoch、fence、heartbeat 或 ticket。
- `http://127.0.0.1:15701/health`：同样仅接受真实 TCP loopback；不会输出配对码、
  头显凭据、RTC ticket 或私钥。
- `wss://<G1-IP>:15702/ws/teleop-capture`：供已配对原生 OpenXR Capture 使用的
  独立 TLS 控制面。公开 `/offer` 已禁用，RTC offer 只能通过已认证 WSS 提交。
- Driver 本地生成独占 session、单调 fence 和一次性短效 RTC ticket。只有已认证且
  处于 `xr_standby`、`rtc_connecting` 或 `streaming` 的 Capture presence 能续租；
  姿态帧、RTC ping 和 DataChannel heartbeat 都不能续租。

MCP 与 Capture 必须使用不同监听端口。开启遥操作时 MCP 强制绑定 loopback，
Capture 才监听局域网。缺少 TLS 文件、WSS URL 不是 `wss://`、URL 端口与监听端口
不同，或证书 SAN 不匹配 WSS 主机时，Driver 会拒绝启动。

## 首次部署

默认 `config.yaml` 保持 `teleop.enabled: false`，现有 G1 部署不变。要先做 Shadow，
将 `config.teleop-shadow.example.yaml` 作为 `CONFIG_PATH`，并保持：

```yaml
plugins:
  arm:
    enabled: false
teleop:
  enabled: true
  mode: shadow
  capture:
    port: 15702
    public_wss_url: wss://10.110.12.110:15702/ws/teleop-capture
```

`plugins.arm` 与遥操作是互斥的手臂控制权，配置冲突会在创建任何遥操作 publisher
前失败。Shadow 仍运行真实控制器坐标变换、Pinocchio/CasADi IK 和 LowState 对比，
但不会创建或写入 `rt/arm_sdk` publisher。

Capture 证书必须独立于 Agent Core 证书。下面示例适用于 WSS 地址中的
`10.110.12.110`；若使用 DNS 名称，应把两处 IP 改为该名称，并将
`subjectAltName=IP:...` 改为 `subjectAltName=DNS:...`。
Driver 下发给原生 Capture 的公共 CA PEM（可含证书链）按 base64 解码后最多
32768 字节；对应标准 base64 文本上限为 43692 字符。超过任一边界会在启动时
fail-closed，私钥内容永远不会进入配对结果。

```sh
sudo install -d -o root -g root -m 0700 \
  /opt/phanthy-motus/data/g1-teleop-capture-tls
sudo openssl req -x509 -newkey rsa:3072 -sha256 -nodes -days 365 \
  -subj '/CN=10.110.12.110' \
  -addext 'subjectAltName=IP:10.110.12.110' \
  -addext 'keyUsage=critical,digitalSignature,keyEncipherment' \
  -addext 'extendedKeyUsage=serverAuth' \
  -keyout /opt/phanthy-motus/data/g1-teleop-capture-tls/key.pem \
  -out /opt/phanthy-motus/data/g1-teleop-capture-tls/cert.pem
sudo chown root:root \
  /opt/phanthy-motus/data/g1-teleop-capture-tls/key.pem \
  /opt/phanthy-motus/data/g1-teleop-capture-tls/cert.pem
sudo chmod 0600 /opt/phanthy-motus/data/g1-teleop-capture-tls/key.pem
sudo chmod 0644 /opt/phanthy-motus/data/g1-teleop-capture-tls/cert.pem
openssl x509 \
  -in /opt/phanthy-motus/data/g1-teleop-capture-tls/cert.pem \
  -noout -ext subjectAltName
sudo install -d -o root -g root -m 0700 \
  /opt/phanthy-motus/data/g1-teleop-state
```

当前 G1 容器以 root 运行，因此上面的目录权限与实际运行 UID 一致。部署文件将
Capture TLS 目录只读挂载到 `/etc/motus-g1-capture-tls`，并将独立状态目录挂载到
`/var/lib/motus-g1-teleop`。绝不能把 Core 的 `/opt/phanthy-motus/data/certs`
当作 Capture 私钥目录；证书 SAN 通常也不匹配头显访问的 G1 IP。

## 卡片操作与自动重连

首次使用时，在 PC 卡片中按以下顺序操作：

1. `teleop_session.start` 创建 Driver 本地 session。此时租约尚未启动，机器人无输出。
2. `teleop_session.pair_headset` 返回一次性配对码、精确 `wss_url` 和公共 CA PEM 的
   base64。由可信 ADB 启动脚本把这些信息交给 Quest/PICO 原生 Capture。
3. 头显完成配对并进入 `xr_standby` 后，Driver 自动下发 assignment，再在 WSS 内
   完成 WebRTC 双 DataChannel 协商。
4. 先发送松开握把的中立帧，再以更高 `clutch_sequence` 按下握把，才会进入动作状态。

配对成功后，Driver 只把单设备凭据的 SHA-256 摘要、客户端类型和版本原子写入
`capture.json`，文件固定为 `0600`；写入顺序为临时文件 flush/fsync、原子 replace、
目标权限修正、父目录 fsync，避免已返回凭据只停留在页缓存或未落盘目录项。明文
凭据保存在头显应用中。replace 前失败会回滚内存状态；replace 后的权限或目录
fsync 失败会返回 `capture_state_unavailable`，但保留已经提交的新内存状态，避免
Driver 重启前后接受不同凭据。之后头显可先自动
连接并持续 `xr_standby`，即使 PC 还没按 `start` 也不会断开或创建 RTC。PC 开始新
会话后，Driver 会自动下发新 assignment，用户不需要再次在 VR 内点击或重连。
Driver 重启后同一头显也可凭持久凭据自动连接。

`pause` 会进入可恢复但需要重新中立/握把的安全状态；`release`、`stop` 和
`emergency_stop` 都会撤销本地控制权、关闭 RTC 并等待最终安全停止确认。
`revoke_headset` 还会删除持久配对。失焦、`error`、`xr_ended`、WSS 断开和
presence 超时都会在任何 socket await 之前同步进入一次 HOLD；重复失焦消息不会
重复下发停止。

三张卡都容忍 stock Core 自动注入的可选 `instance_id`。`teleop_state` 和
`teleop_ik` 的 `start/info/stop` 只是卡片生命周期兼容动作，不改变实时控制所有权。

## 输出状态与 IK 诊断

卡片和 health 使用三个互不混淆的字段：

- `actuation_enabled`：配置是否允许 Live 硬件输出。
- `publisher_present`：本进程是否已创建唯一 `rt/arm_sdk` publisher。
- `output_active`：当前是否正在写有效权重/目标。Live 的 `start/info` 在尚未动作时
  会返回 `true / true / false`；Shadow 始终为 `false / false / false`。

`teleop_ik` 提供 `solve`、`self_test`、`reset` 和 `status`。它与实时控制共享同一个
G1_23 solver 和锁，但诊断自身的 `diagnostic_hardware_output`、
`diagnostic_publisher_present`、`diagnostic_output_active` 永远为 false。诊断完成后
会恢复实测关节 seed，且任何 session 活动期间拒绝求解。`frame_json` 使用与 RTC
完全相同的姿态和四元数归一化容差，畸形 JSON、重复字段、NaN、未知字段、超限
输入及非单位四元数都会被拒绝。

## Live 门槛与已知边界

只有目标架构镜像冷启动 IK 检查通过、G1 当前 `mode_machine=4` 后，才可同时设置
`teleop.mode: live` 与 `teleop.live.enabled: true`。Live 在 IK warm-up 成功后才创建
唯一 `G1ArmSdkPort`，并保留中立帧、重新握把、姿态超时、LowState 新鲜度、固定
`0.5 rad/s` 关节速度和五帧零权重安全停止合同。

V1 只接受 `client_kind=native_openxr`，适用于 Quest 和 PICO 原生 OpenXR 客户端；
不把 WebXR 浏览器当作回退。当前 RTC 未配置 STUN/TURN，验收形态是头显与 G1 可
直接互通的局域网，不是跨 NAT 或不稳定 SSH 隧道。

## 离线验证

在本目录运行：

```sh
PYTHONDONTWRITEBYTECODE=1 uv run --python 3.10 \
  --with 'aiohttp>=3.13,<3.15' --with 'aiortc>=1.14,<1.16' \
  --with 'cryptography>=44,<47' --with 'PyYAML>=6.0' \
  --with 'numpy==1.26.4' \
  python -m unittest discover -s tests -p 'test_teleop*.py' -v
```

测试包含真实 TLS WSS、真实 aiortc offer/answer、双 DataChannel、Shadow
`would_apply`、Live fake publisher、自动 standby assignment、凭据重启、ticket
重放/过期、失焦/断连/超时 HOLD、TLS/SAN/loopback、32 KiB CA 边界、凭据文件
原子替换与父目录 fsync、IK 零输出与 Shadow 零 publisher。
