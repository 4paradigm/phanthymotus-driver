# Driver-owned 遥操作协议

## 控制权

MCP `start` 只在 Driver 内部生成 `session_id`、递增 `epoch` 和私有 `fence`。
这些字段不从 Core 接收。`session_id` 和 `epoch` 会作为非秘密关联标识出现在卡片
状态和 Capture assignment；`fence` 始终只存在于 Driver 内部，不会出现在状态、
health、assignment 或公开 RTC Frame。最终 adapter 在每次输出前仍校验 authority、
session generation、deadline、deadman 和 tracking。

会话只有一个已配对 Capture owner。Capture credential 只验证
`/ws/teleop-capture` 的首条 in-band 消息，禁止放在 URL/query 中。

## Stock Core 注册协调

单实例沿用直接注册快速路径。schema v4 多实例部署为所有 Driver 挂载一个独立、
持久且不与 TLS/凭据状态重叠的协调目录。Driver 用非阻塞 `flock` 串行化实际
`POST /api/mcp`；锁保持到响应正文读取完成，并把响应、transport error、timeout 或
已开始 POST 的取消所在 Unix 秒原子写入 0600 barrier marker、fsync 父目录后才释放。
下一 Driver 只有进入更新后的 Unix 秒才能 POST，因此不依赖调度时隙或启动 sleep 的
概率。等待锁和跨秒都通过 async sleep，不阻塞 MCP/Capture event loop。

## Native Capture v1

服务器保持现有客户端 wire 协议：

1. `pair` 或 `credential`；
2. `paired` 或 `connected`；
3. `presence` / `presence_ack`；
4. PC 卡片已启动且头显聚焦后，服务器发送 `assignment`；
5. 头显对该 assignment 发送且只能发送一次 `signaling_offer`；
6. Driver 基于当前私有 authority 为精确 SDP 创建并立即消费一次性 ticket，返回
   `signaling_answer`；ticket 和 fence 从不发送给头显；
7. `teleop-control` 必须 ordered/reliable；`teleop-pose` 必须 unordered 且
   `maxRetransmits=0`。

首次绑定以及合法的 `xr_standby`、`rtc_connecting`、`streaming` presence 才会建立或
续 lease；Pose、SDP offer、`peer_ping`、DataChannel `heartbeat` 都不续 lease。ICE/
DTLS 的 15 秒宽限固定锚定于最近一次合法 presence，重复 offer 不能滚动延长。
`browser_ready`、`xr_ended` 或 `error` 会同步撤销 assignment、关闭 RTC、进入 HOLD
并等待最终 stop ack。

## 恢复与终止

- Pose timeout 或 RTC channel 断开进入 HOLD；再次动作要求同 generation 双通道恢复
  且 `clutch_sequence` 严格增加。
- Capture WSS 断开立即进入 HOLD；持久凭据重连后可获得新 assignment。
- `pause` 是锁存安全状态；必须 release 后重新 start。
- `release`、`emergency_stop`、生命周期 stop 和 `revoke_headset` 均撤销 authority，
  旧 Frame、旧 ticket 和旧 RTC callback 不能复活 session。
- WSS presence timeout 会立即撤销 assignment 并进入 HOLD；若持久凭据在 Driver motion
  lease 到期前重连，可获得新 assignment。motion lease 一旦到期则当前 authority
  不可逆失效，需要 PC 再次 start。
