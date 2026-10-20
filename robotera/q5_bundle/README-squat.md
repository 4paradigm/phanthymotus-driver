# Q5 深蹲计数卡片

在固定机位、手动选择深蹲的场景下，连接画布：

1. Q5 `camera_rgb`（`image/jpeg`）→ perception `pose`。
2. `pose` 的 **完整骨架**输出 `.../poses/skeleton`（`sensor/pose2d`，通常为输出口 1）→ Q5 `squat_counter`；不要接缺少关节的精简 `/poses` 输出。
3. `squat_counter` 输出口 0 `.../fitness/squat/events` → `decision_core`。输出口 1 `status` 与输出口 2 `debug` 只接 Dashboard，不送大模型。
4. TTS 的 PCM 输出 → Q5 `speaker`。语音清零另接 Q5 `mic` → ASR 正常识别输出 → `decision_core`。

Q5 主驱动使用 ROS Domain 211，`camera_rgb` 已由媒体桥送进 Agent Core 的 Domain 42。`pose` 与深蹲处理进程都运行在 Domain 42；`squat_counter` 的 MCP 操作通过 Q5 主驱动代理到该进程。若子进程启动失败，卡片的 `start`/`status` 返回错误，不会假装正在计数。

检测状态机和默认阈值位于 `common/squat_logic.py`，与 Bumi 共用：先站稳 2 秒建立基线，再检测下蹲、蹲底、起身和站稳。基线只在完整动作结束时小幅跟随新的站姿；可见状态下挪步或镜头位置变化使旧基线失效时，卡片会取消未完成动作，等待约 0.8 秒清晰、稳定的站姿并自动重建基线，保留已计次数。目标丢失后在远处重新出现或跟踪侧切换仍需重新完成约 2 秒站姿校准。换人会开启新会话；同一人短暂离开再回来会保留次数。阈值可通过 `config.yaml` 的 `plugins.squat_counter` 调整，务必结合 Q5 镜头实际画面校准。建议让被检测者全身入画，采用略偏侧面的固定视角。Q5 镜头若无法完整看到被检测者，应使用能接入 `pose` 的外部固定相机。

`status` 返回实时累计数、`input_stale`、接收/解析帧数；`debug` 返回中文角度、脚踝坐标、阈值判定、自动更新基线进度和未计数原因。初次排查先确认 `frames_received` 持续增长、`input_stale=false`，再看 `判定原因`。`reset` 只清零计数，不能修复中断的 pose 或 ASR 流。

技能指令模板见 `fitness_squat_skill.md`。将其安装到 Resource Center，绑定 `squat_counter` 和 TTS 工具；修改本文件不会自动更新已安装的技能。使用 `pytest tests/test_bumi_squat_counter.py robotera/q5_bundle/test_q5_squat_counter.py -q` 做本地回归。真机视角、ROS 跨进程通信和语音延迟仍需在 Q5 上验证。
