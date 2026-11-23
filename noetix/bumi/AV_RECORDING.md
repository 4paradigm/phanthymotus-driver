# Bumi 有声录像：实现与秒表验收

## 改动

现有 `vision_capture.record_video` 新增 `include_audio`，默认 `false`。
开启后输出 H.264 + AAC MP4；拍照、无声录像和停止动作沿用原 Card。

旧实现把实际收到的帧全部按固定 FPS 编码，帧数不足时视频会变短、快进。
新实现按摄像头消息时间戳选择每个输出时刻的画面，缺帧时重复前一帧，保持真实时间。
录制结束后编码，因此操作等待时间会包括设备就绪、实际录制与编码，不等于成片时长。

复用来源：`unitree/g1/av_recording.py` 的有界输入队列、PCM 时间轴、突发包处理、
音频缺失检测和任务清理。共享实现放在 `common/av_recording.py`；G1 驱动未修改。
视频采用显式 CFR 时钟，兼容机器人镜像中的 FFmpeg 4.4，也便于在 Windows 验证。

现有 Bumi 麦克风进程额外发布内部话题 `/<namespace>/mic/audio_recording`：
单声道 PCM16、16 kHz、主机采集完成时间戳。原话题及 SDK 起始时间戳约定保留。
SDK 若返回其他有效采样率，则先重采样到 16 kHz，避免数数声音变慢或时长错误。
没有额外打开麦克风或 RealSense。这里是主机时钟软件同步，非硬件同步。

## 安装前提

1. 构建并部署包含本次源码的 Bumi 驱动镜像。仅刷新页面不会安装代码。
2. `plugins.camera.enabled`、`plugins.mic.enabled`、`plugins.vision_capture.enabled` 均为 true。
3. 现有 Dockerfile 已安装 FFmpeg，并通过 `COPY common/ /work/common/` 带入共享模块。
   部署服务已将 `/opt/phanthy-motus/data` 持久化到宿主机同一路径。
4. 更新驱动后重新获取工具列表；`record_video` 应显示“录制现场声音”选项。

驱动容器中可检查代码是否已更新：

```bash
docker exec embodied-noetix-bumi python3 -c 'from pathlib import Path; print(Path("/work/common/av_recording.py").is_file())'
docker exec embodied-noetix-bumi ffmpeg -version
```

## 画布操作

相机、麦克风保持运行，先确认 camera 预览正在刷新。
手动在 `vision_capture` 选择 `record_video`，填写：

```json
{"duration_s": 10, "include_audio": true}
```

通过 MCP 调用时参数包含动作：

```json
{"action": "record_video", "duration_s": 10, "include_audio": true}
```

录制 Card 内部订阅本机已有数据源，不需要额外拉一条 mic 到录像卡的线。
由 Agent/Skill 调用时，再把 `vision_capture` 绑定到决策核心。
不要同时执行 TTS 播报，避免验收时把机器人自己的声音录入。

首次返回 `queued` 和 `action_id` 仅表示任务已接受。等待 ACP 的 completed，
或在 `info.last_recording` 查看同一 action_id 的终态、文件路径和时长。
`recording` 是采集，`encoding` 是封装文件；只有 `completed` 且 `ok=true` 才成功。

## 真机验收：秒表同时数数

1. 将手机或电脑秒表放在 camera 预览中央，确保数字清楚。关闭本次演示不需要的播报。
2. **提前启动秒表，让秒表一直运行**，不要把网页点击时间当成视频的起点。
3. 执行 10 秒有声录像。你在镜头前看着秒表，每到整数秒清楚读出那个整数；
   开始、中段、结束附近各做一次可见的拍手，便于同时对照动作和声音。
4. 继续数到任务采集结束，再等待编码完成。不要在 queued 后立即下载半成品。
5. 下载文件，回放核对：
   - 成片时长约 10 秒（建议文件容器时长容差 ±0.1 秒）。
   - 画面末尾秒表值减开头秒表值约 10 秒；15 fps 下首末帧跨度正常约为 9.93 秒。
   - 第一声、中间、最后一声读数均对应画面秒表；拍手声对应手掌接触画面。
   - 建议第一版人工验收目标：声画差不超过约 0.3 秒，且到结尾不持续扩大。
6. 再录 30 秒，重点检查开始与末尾的误差，避免短片看着正常、长片逐渐漂移。

验收时记录文件名、录制参数、开始/中段/结束对应读数与偏差。软件同步目标需以
Bumi 实机样片确认，离线测试不代替真机验收。

## 下载

完成结果的 `file_path` 对应机器人宿主机路径。例如在 Windows 新终端运行：

```powershell
scp -P 2222 noetix@192.168.3.22:/opt/phanthy-motus/data/vision_capture/videos/实际文件名.mp4 .
```

将文件名替换为完成结果中的实际名称。此目录不是需要手动穿透的容器私有路径。

机器人上可以用 FFprobe核查轨道和时长：

```bash
docker exec embodied-noetix-bumi ffprobe -v error -show_entries format=duration:stream=codec_type,codec_name,sample_rate,duration -of json /opt/phanthy-motus/data/vision_capture/videos/实际文件名.mp4
```

应有视频轨道与 AAC 音频轨道，音频采样率为 16000。轨道时长可能因 AAC 编码边界
与容器时长存在小差异，还需实际回放判断声音与画面的对应关系。

## 失败与回归

- 未启用 mic、没有音频、只收到零 PCM、长时间断音：明确失败，不伪装有声成功。
- 摄像头连续超过一秒没有新帧：失败，不把静止旧画面扩展成成功录像。
- 音视频时钟倒退、明显跳变、缓存或临时存储超限：失败。
- `stop`：取消当前任务，清理临时文件和半成品，释放本次订阅，不停止 camera/mic。
- 同时再录：返回 `RECORD_IN_PROGRESS`；线程启动失败则释放任务位置。
- `include_audio=false` 仍可录制无声视频，并同样修复缺帧造成的播放速度问题。
- 原 `capture_photo` 仍使用相机缓存，不依赖麦克风。

## 本地验证

```text
python -X utf8 -m pytest -q noetix/bumi/tests/test_bumi_av_recording.py noetix/bumi/tests/test_mic_card.py
```

真实编码用例需要 PATH 中可用的 FFmpeg。该用例运行实际 Bumi 录制 worker、共享
录制模块及编码器，模拟相机丢帧，将合成秒表图像和已知时刻的音频脉冲合成 10 秒视频，
再解码验证时长与声画时间。设置 `BUMI_AV_EVIDENCE_DIR` 可保留合成样片及时间数据。
**合成样片不是机器人实拍，也不包含真人数数；真机秒表和数数验收仍需执行。**

本次本地验证（2026-11-23）：专项测试 30 项通过，实际工具 Schema 转换检查通过。
编码验证使用 Windows FFmpeg 7.1；合成输入故意缺少 30 帧，最终输出 150 帧、10 秒、
160000 个音频采样。3 秒处画面闪光与音轨脉冲对应。Bumi ARM64 镜像构建、ROS2
实时数据接入和真人秒表数数尚未执行，不能由上述本地结果推定真机偏差。
