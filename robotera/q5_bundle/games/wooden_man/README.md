# Q5 一二三木头人

新增的 `wooden_man` 是规则处理卡。摄像头、Pose、ASR、TTS、扬声器均为独立画布卡片，不在规则卡内创建模型或调用隐藏的 MCP 工具。它不发送底盘、腿、头或手臂命令；请让 Q5 保持已经建立的站姿，游戏项目中不要放运动控制卡。

## 画布连线

在已部署的 Motus 中添加 Q5 驱动以及提供 ASR / Pose / TTS 的感知服务，然后新建项目，添加以下卡片：

```text
Q5 mic ──audio/pcm-16k──> ASR ──text/plain───────────┐
                                                  v
                                          一二三木头人 wooden_man
                                                  ^
Q5 camera_rgb ──image/jpeg──> Pose ──sensor/pose2d────┘

wooden_man 的 speech ──text/plain──> TTS ──audio/pcm-16k──> Q5 speaker
wooden_man 的 status ──data/json──> 画布状态显示（无需接 TTS）
```

Pose 必须输出 **人体 COCO-17 二维关键点**，不能连接机器人自己的 `joints` / `sensor/skeleton`。协议为 `std_msgs/String` 承载 JSON：

```json
{"persons":[{"id":1,"keypoints":[[100,100,0.9]]}],"image_size":[848,480]}
```

上例只展示结构；真实帧需有 17 个关键点，坐标为像素，建议输出至少 5 Hz。只支持单玩家，肩和髋必须清晰可见，身体至少八个关键点有效。多人、遮挡或丢帧持续超过 1.5 秒，会播报没有看清并结束，不会判为过关。

ASR 输入支持纯文本或 `{"text":"游戏开始","is_final":true}` / `{"transcript":"游戏开始","final":true}`。中间结果标记为 false 时忽略。去除空格和标点后，整句必须是“游戏开始”。游戏中重复口令不重启。结束后再喊一次可重新玩。

启动整个画布项目，规则卡 `start` 仅进入等待口令；不会自行开始游戏。画布通过 `input_topics` 传入两条上游 topic。若当前 Motus 版本只传入单个 `input_topic`，把 ASR 和 Pose 的真实输出 topic 分别填入规则卡配置的 `asr_topic`、`pose_topic`。从上游卡 `info.topic_out` 复制，不能猜名字。规则卡输出固定为 `/<机器人命名空间>/wooden_man/speech` 和 `/status`。

## 游戏时序和判定

1. 玩家说“游戏开始”，播报“游戏开始，五秒后开始”，从发出播报起计时 5 秒。
2. 播报“一，二，三，木头人！”，默认预留 3 秒语音及 0.5 秒站稳时间。
3. 随后的 3 秒观察窗口，以第一帧有效姿态为基准。任一可见关键点相对固定躯干长度移动超过 7%，连续三帧达到阈值，播报“你动了，游戏结束”。头部或单个手腕的移动也计入；不会通过每帧更新基准放过缓慢移动。COCO-17 无法检测手指或眨眼等没有对应关键点的细小动作。
4. 保持不动进入下一轮；三轮全部通过，播报“三轮完成，你赢了，游戏结束”。

默认 `cue_seconds=3.0` 是给外部 TTS 的固定播报预算，并非 TTS 播放完成回执。实际网络/TTS较慢时，应调大此值，确保“木头人”说完才开始判断。画布暂未接入真实机器验证，首次使用请按实际 TTS 延迟校准。不要把扬声器回声作为 ASR 输入；尽量启用现有 ASR 的回声消除。

可配置：`watch_seconds`（观察时长）、`motion_threshold`（躯干比例阈值）、`moving_frames`（连续帧数）、`confidence`、`pose_timeout`、`cue_seconds`、`settle_seconds`。`reset` 回到等待口令；`stop` 销毁游戏 DDS 子进程。三轮次数及开始等待 5 秒固定。

## 构建

仓库根目录，使用开发规范中的脚本构建 ARM64 镜像（自动包含 `common/`）：

```bash
bash build.sh --mirror none robotera/q5_bundle
```

本地没有配置镜像仓库时不会推送。生成的标签见脚本输出，形式为 `local/phanthy-motus/drivers/robotera/q5:release.YYMMDD.COMMIT`。导出镜像、传到机器人后使用同目录的 `deploy.sh`，它只更新现有 compose 的 Q5 服务。

## 验证

仓库根目录运行离线行为测试：

```bash
python -X utf8 -m pytest tests/test_q5_wooden_man.py tests/test_dockerfile_copies.py -q
```

构建后将 `IMAGE` 替换成真实标签，在开发机运行镜像内检查。第一条验证真实 MCP HTTP 包装及卡片装配，第二条使用真实 ROS 2 DDS 通信、模拟 ASR/Pose 数据验证三轮游戏与移动失败；不会调用运动控制，也不需要接机器人：

```bash
docker run --rm --platform linux/arm64 --entrypoint bash IMAGE -lc 'source /opt/ros/humble/setup.bash && source /q5_ws/install/setup.bash && python3 /work/games/wooden_man/smoke_mcp.py'
docker run --rm --platform linux/arm64 --entrypoint bash IMAGE -lc 'source /opt/ros/humble/setup.bash && source /q5_ws/install/setup.bash && python3 /work/games/wooden_man/smoke_ros.py'
```

真实摄像头 Pose 推理精度、ASR 识别与 TTS 播放延迟仍需在机器人接好画布后验收。部署脚本生成 `docker-compose.wooden-man.yml`；之后手工运行 compose 时也要带上这个 override，避免基础 compose 中的旧镜像覆盖新版本。
