# Bumi 受控直播 Card（小红书 / 虎牙）

## 小红书最小版本：真人授权，Bumi 独立推流

**用途：** 让 Bumi 复用已有摄像头和麦克风，经 FFmpeg + RTMP 直接向小红书或虎牙直播，无需安装 OBS，也无需 Windows 中转。仅用于有人监督的测试。

### 新用户操作

1. 用小红书账号进入「三方推流」页面，按平台要求完成每场真人授权，获取「服务器」和「推流码」。截图中曾暴露过部分密钥，务必刷新后使用新密钥，**不要把密钥发送给 AI 或写入仓库**。
2. 在本地将服务器地址末尾加上 `/`，再拼接完整推流码。例如服务器 `rtmp://live-push.xhscdn.com/live` 与带 `txSecret`、`txTime` 的推流码拼成一条完整 RTMP URL；**只在 Agent Core 画布的 live_stream 齿轮中粘贴**并保存。不要把完整地址直接输入终端命令。
3. 在画布齿轮里保存**本场新推流地址**，就视为操作员授权本场直播；5 分钟内直接调用 **go_live**，无须 SSH 授权码。仅允许一次，结束或失败后需更新本场推流地址。
4. 确认 Camera、Mic 工作正常，保存本场地址后调用 **go_live**，再检查小红书观众端是否看到实时画面、听到声音；`start` 不会公开直播。如小红书页面提示「继续直播」等确认步骤，仍须按平台要求操作。
5. 调用 **end_live** 停播，并在观众端确认直播结束。再次开播请获取新的推流码并在齿轮更新。旧地址被恢复/重复保存不会重新授权开播，避免自动恢复触发公开直播。

### 支持范围与运行说明

- **平台白名单**：仅接受官方 `live-push.xhscdn.com/live/<stream-id>?txSecret=...&txTime=...` 的小红书地址，或 `direct.huya.com` 下 `huyalive` 的虎牙地址；其他主机一律拒绝。
- 视频：复用现有 CameraPlugin 的 JPEG 帧缓存，640×480 / 15 FPS / H.264。不会再次打开 RealSense。
- 音频：订阅现有 ROS2 MicPlugin AudioChunk，16 kHz PCM → AAC 44.1 kHz。
- 推流：FFmpeg 把画面和声音编码为 FLV，`librtmp1` 在单独的可终止 Python 子进程里连接平台。凭据通过进程间私有管道传递，不出现在子进程命令行。数据面不经过 Agent Core 或 LLM。
- **保存本场新地址相当于一次短期授权**：接下来 5 分钟内可以执行一次 `go_live`；容器重启或重放旧配置不会重新授权。网络阻塞的 RTMP 子进程可以被强制终止，不会自动重连。
- info 的 streaming 只说明编码进程正在处理音视频，**实际直播间播放仍要从观众端确认**。

### 故障排查

| 状态/错误码 | 说明 | 建议 |
| --- | --- | --- |
| idle | 当前没有直播进程 | 保存本场新推流地址后执行 go_live |
| starting | 正在连接直播服务器 | 稍后调用 info |
| streaming | 正在发送音视频 | 在观众端检查 |
| CAMERA_UNAVAILABLE | 没有可用的 RGB 图像 | 检查 Camera Card |
| MIC_UNAVAILABLE | 未收到实时音频 | 检查 Mic Card |
| ENCODER_START_FAILED | FFmpeg 创建失败 | 检查 Driver 镜像 |
| RTMP_PROCESS_EXITED | 推流进程异常退出 | 刷新当前平台地址，检查 Bumi 网络 |
| VIDEO_PIPE_FAILED | 视频帧超时或写入阻塞 | 检查摄像头、系统负载 |
| AUDIO_PIPE_FAILED | 麦克风断流或写入阻塞 | 检查麦克风、系统负载 |
| APPROVAL_REQUIRED | 当前没有有效的一次性开播授权 | 获取本场新推流地址，在画布齿轮保存 |
| RTMP_CONNECT_TIMEOUT | 建连超过 12 秒 | 检查网络或密钥，获取本场新推流地址后重试 |
| RTMP_WRITE_TIMEOUT | 单次发送超过 4 秒 | 检查网络，确认观众端已停播 |

### 凭证与权限

- Card 配置中完整 URL 标为 password + x-sensitive，防止输入时明文展示及打包分享 Solution 时直接导出。**密钥仅作为 configSchema 配置字段**：齿轮保存后由 Agent Core 的 `/api/canvas/tool-config` 以内部 `action=config` 调用应用，并在设备重新连接/启动时重放；`config` 不列入 `x-action-params`，因为 Agent Core 会把该列表中的每个动作都作为 LLM 可调用工具公开。LLM 只能调用不含密钥的 `go_live` / `end_live` / `info` 等动作。
- 为兼容旧部署，Driver 的凭据文件路径暂保留为 `/opt/phanthy-motus/data/live_stream/huya-rtmp-url`，但可保存两平台白名单地址。Linux 文件模式 0600、父目录 0700，原子更新并拒绝符号链接。源代码、config.yaml、返回状态和 Docker 日志都不应包含完整 URL。
- **安全边界：** 保存本场与此前不同的推流 URL 后，Driver 在内存中开启 5 分钟的一次性开播许可。旧配置重放不会授予许可。但这**不是完整的调用者身份鉴权**：保存后的许可窗口内，能访问 MCP 的其他调用者可能抢先调用 `go_live`。因此必须将 Bumi MCP 限制在可信网络，实际开播需有人监督；要公开生产部署仍须补上调用者身份和授权绑定。Agent Core 画布配置数据库可能保存明文 RTMP URL，高权限进程也可访问内存。
- Bumi 直推平台不会关闭 Camera、Depth、Mic Card。视频帧缺失时重复上一帧以保持音频同步；连接完成后清除建连期间积压的旧音频。真机必须继续监控 CPU、延迟、掉帧和音画偏差。

### 本地回归

从 Driver 仓库根目录运行：

    python -X utf8 -m pytest noetix/bumi/live_stream/tests -q

Windows 上可指定一个自己有权限的 --basetemp 路径。音视频双 pipe 推流需要在 Linux ARM64 容器中真机验证。
