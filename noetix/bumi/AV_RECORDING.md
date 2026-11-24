# Bumi 有声录像验收

`vision_capture.record_video` 使用现有相机和麦克风。缺帧时按采集时间戳补帧，
音频按同一主机时钟对齐。启用声音后，缺少有效 PCM 或采集中断会返回失败。

## 画布测试

1. 部署待测 Bumi 驱动，启用 `camera`、`mic` 和 `vision_capture`，确认相机预览刷新。
2. 在 `vision_capture` 选择 `record_video`，设置 `duration_s=10`。`include_audio` 默认
   为 `true`（有声），选择 `false` 录制无声视频。
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

## 验证记录

- Bumi 真机镜像 `release.261124.e02d43d` 已验证 10 秒和 30 秒有声录像。成片分别为
  10.000 秒和 30.000 秒，包含 H.264 和 16 kHz AAC，两段均通过 FFmpeg 完整解码。
  3 秒无声录像和取消清理也通过。冷启动时须等麦克风出流后再录制。
- 提交 `4d00bacd` 的本地专项验证通过 34 项，包含省略 `include_audio` 时生成
  音轨、显式传 `false` 时仅生成视频轨，以及合成秒表和声音脉冲的时间核对。
- 新版 ARM64 镜像的画布默认选项、真人秒表数数和拍手的声画偏差待验收。
