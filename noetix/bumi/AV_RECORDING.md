# Bumi 有声录像验收

`vision_capture.record_video` 使用现有相机和麦克风。缺帧时按采集时间戳补帧，
音频按同一主机时钟对齐。启用声音后，缺少有效 PCM 或采集中断会返回失败。

## 画布测试

1. 部署待测 Bumi 驱动，启用 `camera`、`mic` 和 `vision_capture`，确认相机预览刷新。
2. 在 `vision_capture` 选择 `record_video`，设置 `duration_s=10` 和 `include_audio=true`。
   卡片内部订阅设备数据。在 Skill 中使用时，将 `vision_capture` 绑定到 `decision_core`。
3. 提前启动秒表，将数字完整放在相机画面内。从开始到结束按秒表读出整数，
   并在起始、中段和结束附近各拍一次手。
4. 等待 ACP 完成，或通过 `info.last_recording` 按 `action_id` 查找终态。
   成功时从机器人宿主机下载 `file_path` 对应的 MP4。
5. 核对文件为 H.264 视频和 16 kHz AAC 音频，容器时长为 10 秒（容差 ±0.1 秒）。
   以画面读数和可见拍手为基准，检查开始、中段和结束的声音，人工验收目标为
   声画误差约 0.3 秒以内且无持续漂移。用 30 秒再测一次。
6. 回归无声录像、拍照及录制中取消。取消后由 ACP 或 `info.last_recording`
   确认取消终态。确认没有留下该次录制的文件。

在本机终端下载，替换 SSH 端口、账号、机器人地址及完成结果的 `file_path`。

```powershell
scp -P 2222 ssh_user@robot_host:/path/to/video.mp4 .
```

## 本地回归

```text
python -X utf8 -m pytest -q noetix/bumi/tests/test_bumi_av_recording.py noetix/bumi/tests/test_mic_card.py
```

将 FFmpeg 加入 PATH，使编码测试执行实际音视频封装与解码。
专项测试使用合成秒表和声音脉冲，真机结果以现场样片为准。
