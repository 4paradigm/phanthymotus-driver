# Bumi 受控直播 Card（小红书 / 虎牙）

## 小红书最小版本：真人授权，Bumi 独立推流

**用途：** 让 Bumi 复用已有摄像头和麦克风，经 FFmpeg + RTMP 直接向小红书或虎牙直播，无需安装 OBS，也无需 Windows 中转。仅用于有人监督的测试。

### 新用户操作

1. 用小红书账号进入「三方推流」页面，按平台要求完成每场真人授权，获取「服务器」和「推流码」。截图中曾暴露过部分密钥，务必刷新后使用新密钥，**不要把密钥发送给 AI 或写入仓库**。
2. 在 **Agent Core Canvas 的直播助手界面**，小红书分别填写「服务器地址」与「直播推流码」，虎牙填写完整地址。Canvas 将两项拼接成完整 RTMP URL，再通过 Core 发送给 Driver。
3. **Driver 内部接口只有一个 `rtmp_url` 字段**，接收 Canvas 合成的完整地址。如果绕过 Canvas 直接使用原生 Card 配置，必须输入一条完整 RTMP URL；原始 MCP `config` 还需要 Core 签发的有效授权。保存后五分钟内可尝试一次开播，过期可在 Canvas 重新授权。
4. 确认 Camera、Mic 工作正常，在 Canvas 点击「确认开始直播」后由 Core 签名调用 **go_live**，再在平台观众端检查音视频；`start` 不会公开直播。如平台另需确认，按官方提示操作。
5. 使用 **end_live** 停播，并在观众端确认直播结束。再次开播必须重新进行 Canvas 用户确认；Driver 重启或后台重放旧配置不会自动授权。

### 支持范围与运行说明

- **平台白名单**：接受小红书官方直播助手实测的 `live-push-reg-tx.xhscdn.com/live/<stream-id>?txSecret=...&txTime=...`，同时兼容原有 `live-push.xhscdn.com`；虎牙仅允许 `direct.huya.com` 下的 `huyalive` 地址。只开放已确认的完整主机名，不允许任意 xhscdn 子域名。
- 视频：复用现有 CameraPlugin 的 JPEG 帧缓存，640×480 / 15 FPS / H.264。不会再次打开 RealSense。
- 音频：订阅现有 ROS2 MicPlugin AudioChunk，16 kHz PCM → AAC 44.1 kHz。
- 推流：FFmpeg 把画面和声音编码为 FLV，`librtmp1` 在单独的可终止 Python 子进程里连接平台。凭据通过进程间私有管道传递，不出现在子进程命令行。数据面不经过 Agent Core 或 LLM。
- **短期授权**：只有认证后的 Canvas 操作才可授权五分钟内的一次开播；原始 MCP `config`/`go_live` 必须携带由同机 Core 签发的单次、20 秒有效的证明。容器重启或后台重放旧配置不会重新授权。网络阻塞的 RTMP 子进程可强制终止，不会自动重连。
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
| APPROVAL_REQUIRED | 没有有效开播授权或调用未经 Core 签名 | 在 Canvas 重新确认本场直播；勿直接调用原始 MCP 开播 |
| RTMP_CONNECT_TIMEOUT | 建连超过 12 秒 | 检查网络或密钥，获取本场新推流地址后重试 |
| RTMP_WRITE_TIMEOUT | 单次发送超过 4 秒 | 检查网络，确认观众端已停播 |

### 凭证与权限

- Card 配置中完整 URL 标为 password + x-sensitive，防止输入时明文展示及打包分享 Solution 时直接导出。**密钥仅作为 configSchema 配置字段**：齿轮保存后由 Agent Core 的 `/api/canvas/tool-config` 以内部 `action=config` 调用应用，并在设备重新连接/启动时重放；`config` 不列入 `x-action-params`，因为 Agent Core 会把该列表中的每个动作都作为 LLM 可调用工具公开。LLM 只能调用不含密钥的 `go_live` / `end_live` / `info` 等动作。
- 为兼容旧部署，Driver 的凭据文件路径暂保留为 `/opt/phanthy-motus/data/live_stream/huya-rtmp-url`，但可保存两平台白名单地址。Linux 文件模式 0600、父目录 0700，原子更新并拒绝符号链接。源代码、config.yaml、返回状态和 Docker 日志都不应包含完整 URL。
- **安全边界：** Driver 端 `config` 与 `go_live` 都要求同机 Agent Core 签发的 HMAC 证明，绑定动作和完整 RTMP URL，具有 20 秒有效期、随机 nonce 和一次性防重放检查。共享密钥位于 `/opt/phanthy-motus/data/live_stream/.operator-approval-key`，文件 0600、目录 0700，只由 Core 创建。原始 MCP 请求不能自行配置新的直播目标或开播。此方案仅支持 Core 和 Driver 共享可信数据卷的 Bumi 机器；其他跨机部署须另行实现安全密钥分发。Core 数据库仍可能保存明文 RTMP URL，高权限进程仍可能访问内存，生产级静态秘密管理需要单独评估。
- Bumi 直推平台不会关闭 Camera、Depth、Mic Card。视频帧缺失时重复上一帧以保持音频同步；连接完成后清除建连期间积压的旧音频。真机必须继续监控 CPU、延迟、掉帧和音画偏差。

### 本地回归

从 Driver 仓库根目录运行：

    python -X utf8 -m pytest noetix/bumi/live_stream/tests -q

Windows 上可指定一个自己有权限的 --basetemp 路径。音视频双 pipe 推流需要在 Linux ARM64 容器中真机验证。
