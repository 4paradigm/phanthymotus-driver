# Bumi 虎牙直播 Card

## 虎牙直播 Card — 三分钟上手

**用途：** 让 Bumi 的摄像头和麦克风直接向虎牙直播，不需要 Windows 电脑、OBS 或其他平台。

### 新用户操作

1. 进入 [虎牙主播中心](https://i.huya.com/)，完成主播认证；打开「普通/VR 远程推流」，选择「普通直播」，复制**完整 RTMP 推流地址**。过期后先刷新地址，切勿将地址发到公开聊天。
2. 在 Bumi Agent Core 画布中拖入「虎牙直播 live_stream」Card。点击 Card 的**齿轮**，在「虎牙 RTMP 完整推流地址」中粘贴刚才复制的内容，然后保存。
3. 确认 Camera 和 Mic 工作正常，执行 **go_live**。这才会正式开始公开直播。系统生命周期动作 start 不会开播。
4. 用手机/浏览器打开你的虎牙房间，确认视频和声音。通过 info 查看实时状态；执行 **end_live** 结束直播。停止画布也会结束直播。
5. 地址失效或刷新后，先 end_live，再去卡片齿轮填写新地址并保存。通常无需每次重新配置。

### 支持范围与运行说明

- **仅支持虎牙**：官方 direct.huya.com 下的 RTMP(S) huyalive 地址，其他 URL 会被拒绝。
- 视频：复用现有 CameraPlugin 的 JPEG 帧缓存，640×480 / 15 FPS / H.264。不会再次打开 RealSense。
- 音频：订阅现有 ROS2 MicPlugin AudioChunk，16 kHz PCM → AAC 44.1 kHz。
- 推流：由 Bumi Driver 容器内现有 FFmpeg 发送。**数据面不经过 Agent Core 或 LLM**。
- 开播只由 go_live 触发。断网或 FFmpeg 退出则报告 error，不擅自恢复公开直播。
- info 的 streaming 只说明编码进程正在处理音视频，**实际直播间播放仍要从观众端确认**。

### 故障排查

| 状态/错误码 | 说明 | 建议 |
| --- | --- | --- |
| idle | 当前没有直播进程 | 执行 go_live |
| starting | 正在连接虎牙 | 稍后调用 info |
| streaming | 正在发送音视频 | 在观众端检查 |
| CAMERA_UNAVAILABLE | 没有可用的 RGB 图像 | 检查 Camera Card |
| MIC_UNAVAILABLE | 未收到实时音频 | 检查 Mic Card |
| ENCODER_START_FAILED | FFmpeg 创建失败 | 检查 Driver 镜像 |
| RTMP_PROCESS_EXITED | 推流进程异常退出 | 刷新虎牙 URL，检查 Bumi 网络 |
| VIDEO_PIPE_FAILED | 视频帧超时或写入阻塞 | 检查摄像头、系统负载 |
| AUDIO_PIPE_FAILED | 麦克风断流或写入阻塞 | 检查麦克风、系统负载 |

### 凭证与权限

- Card 配置中完整 URL 标为 password + x-sensitive，防止输入时明文展示及打包分享 Solution 时直接导出。
- Driver 的持久化副本放在 /opt/phanthy-motus/data/live_stream/huya-rtmp-url，Linux 文件模式 0600；源代码、config.yaml、返回状态和 Docker 日志都不包含完整 URL。
- **已知安全边界：** Agent Core 画布配置数据库仍可能含明文，FFmpeg 进程参数可能被本机管理员查看；不是端到端加密的凭证库。必须限制管理员访问，禁止向非管理员公开 Card 的开播工具。
- 设备端音视频直推虎牙不会关闭 Camera、Depth、Mic Card。运行时要记录 CPU、掉帧、音画偏差并完成真机持续推流验收。

### 本地回归

从 Driver 仓库根目录运行：

    python -m pytest noetix/bumi/live_stream/tests -q

Windows 上可指定一个自己有权限的 --basetemp 路径。音视频双 pipe 推流需要在 Linux ARM64 容器中真机验证。
