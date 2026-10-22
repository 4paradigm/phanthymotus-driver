# Q5 一二三木头人：语音与识别

单人、固定三轮。`wooden_man` 使用画布提供的 ASR 与 Pose，输出语音给 TTS，并显示判动部位的骨架高亮。保留灵敏度预设、可选手指识别、部位播报、准备引导及语音重开。

游戏不发布机器人动作、不订阅动作反馈，不依赖位置直控准备或手臂归零。机器人保持现有站姿；原来的通用机器人控制工具不属于本游戏。

## 从手臂动作版更新

1. 先停止整个旧游戏项目，等待已有动作停止。
2. 删除 `wooden_man_motion` 卡及 `wooden_man/actions` 连线；游戏项目也不需要 `arm_control`、`arm_gesture`。
3. 部署原 PR 生成的新驱动镜像，刷新工具列表；必要时重新添加 `wooden_man` 卡，以去掉缓存的 actions 端口及动作配置。
4. 新版本已删除专用动作模块、动作输出和反馈订阅。旧配置中的 `robot_actions=true`、`motion_feedback_topic`、动作超时字段会被忽略，不能重新启用动作。
5. 保留下述七张卡和六条连线，重新启动整个项目。

若暂时尚未部署新镜像，可先在旧版配置里关闭 `robot_actions`、移除动作卡，再运行语音与识别流程。

## 完整画布

共 **7 张卡**：`mic`、`asr`、`camera_rgb`、`pose`、`wooden_man`、`tts`、`speaker`。

| 起点输出 | 终点输入 | 格式 |
| --- | --- | --- |
| `mic` 音频 | `asr` 音频 | `audio/pcm-16k` |
| `asr` 实际产生识别文字的输出 | `wooden_man` ASR 输入 | `data/json` |
| `camera_rgb` RGB 图像 | `pose` 图像 | `image/jpeg` |
| `pose` **skeleton** 骨架输出 | `wooden_man` Pose 输入 | `sensor/pose2d` |
| `wooden_man` **speech** | `tts` 文字 | `data/json` |
| `tts` 合成音频 | `speaker` 音频 | `audio/pcm-16k` |

木头人的 `status` 是监视用 JSON，`highlight` 是监视用 JPEG，两者不需要接线，也不接 TTS。原始摄像头画面在 `camera_rgb → 查看数据流` 看；骨架高亮在 `wooden_man → 查看数据流 → highlight` 看。

ASR 有两路输出：上方 `asr`、下方 `asr_background`。以实际监视数据为准：哪一路有 `{"text":"游戏开始"}` 就接哪一路。`trigger_mode=vad` 使用 `asr`，此时 background 无数据是正常的；唤醒词模式下，未触发唤醒词的普通话语通常走 background，需开启 `publish_background`。本机器人此前确认 `/nvidia_desktop/mic/audio/asr` 有数据，可在木头人高级配置 `asr_topic` 明确填写该地址；其他机器人应复制实际 ASR 输出 topic，不能照抄命名空间。`pose_topic` 同理。

连好后通过顶部“开启智能控制”启动整个项目。木头人 ACTION 显示 `reset`，Pose 显示照片识别动作，是因为画布隐藏了 start/stop/info 生命周期操作；无需执行照片识别启动实时流。不要连接 decision_core。

## 游戏流程

1. 启动后安静等待“游戏开始”，不检查玩家、不播报没有看清。玩家可以先靠近机器人喊口令，再走到镜头前。
2. 默认开启 `host_interaction`：收到口令才检查单人身体是否持续清晰，至少约 0.8 秒和 5 帧，不要求站在中央。看不清时语音引导，提醒间隔至少 6 秒；最多准备 20 秒，可用 `preparation_timeout` 调整。
3. 看清后播报“游戏开始，五秒后开始”，从这句播报起等待 5 秒。关闭 `host_interaction` 时收到口令直接进入该倒计时。
4. 播报“一，二，三，木头人！”，默认预留 `cue_seconds=3.0` 秒播报预算及 `settle_seconds=0.5` 秒玩家站稳时间，再观察 `watch_seconds=3.0` 秒。
5. 本轮没动，播报“不错，第二／三轮来了”，预留语音预算后进入下一轮并重新取基准。共三轮，无机器人动作或动作反馈等待。
6. 判动后立即说“你的左手臂动了，游戏结束”等具体部位，冻结并标红对应骨架区域。多处移动只播报一个，左右按玩家自身解剖侧。三轮通过则播报“三轮完成，你赢了，游戏结束”。
7. 开启主持交互时，结束后询问“再挑战一次吗？”回答“再来／再来一次／好的／可以”重新准备，回答“不玩了”回到安静待机。完整口令“游戏开始”也能重新玩。游戏中重复口令不会重启。

开始口令去除空格和标点后须完整等于“游戏开始”；支持 JSON 顶层 `text` 或 `transcript`，标记 `final=false` 或 `is_final=false` 的中间结果忽略。播报使用 `std_msgs/String` 包装 JSON `{"text":"..."}`，与 TTS 的 `data/json` 协议一致。

播报预算不是 TTS 播放完成回执，实际播放较慢时调大 `cue_seconds`。尽量使用现有 ASR 的回声消除。

## 灵敏度与判动

| `sensitivity` | 身体位移／基准躯干长度 | 手指位移／基准手掌长度 |
| --- | --- | --- |
| `relaxed` 宽松 | 0.10 | 0.22 |
| `standard` 标准，默认 | 0.07 | 0.15 |
| `sensitive` 灵敏 | 0.04 | 0.10 |
| `custom` 自定义 | `motion_threshold` | `finger_threshold` |

以每轮第一帧有效姿态为固定基准，按头、躯干、左右手臂、左右腿分别统计连续超阈帧数；默认同一部位连续三帧判动，单帧尖峰不算，缓慢漂移也不会通过移动基准被放过。默认 `confidence=0.5`，肩髋和至少八个身体关节须可见。多人、换人或关键点持续无效超过 `pose_timeout=1.5` 秒会结束，不会当成没动过关。准备阶段另要求脚踝清楚可见。

旧配置里单独改过数字阈值的会保留为 custom，显式选择预设时以预设值为准。阈值越小越灵敏，也越容易受关键点抖动影响。

## 手指识别

同时开启 **Pose 的 `hands`** 和 **木头人的 `detect_fingers`**，继续连接原来 Pose 的 `sensor/pose2d` skeleton 输出，不接手势文字输出。默认关闭手指检测，兼容原有身体玩法。若感知服务没有 hands 选项，需要使用支持手部模型的感知服务；规则卡不自行加载模型。

协议为 COCO-17 后追加左右各 21 个手部关键点，共 59 点，并提供 `keypoint_names`，名字带 `left_hand_`／`right_hand_` 前缀。按名字查找，避免猜测数组顺序。以手腕为原点、基准手掌长度归一化，观察手指相对手腕的变化，纯平移手掌不误报成手指动作。

手掌参考长度至少 12 像素，21 点须满足置信度要求。默认 `finger_frames=2` 个不同手部观测连续超阈才判动。Pose 会复用缓存手部结果，重复坐标不重复计数；两只手每轮均须至少三个不同有效观测，末次新观测不能超过 pose_timeout。缺手、缺元数据、缓存长期未更新时提示没有看清双手，不会直接过关。上游没有独立手部推理时间戳，坐标变化只能保守去重，精度需真机验收。

## 高亮监视

`highlight` 为独立骨架 JPEG，使用 `sensor_msgs/CompressedImage`，端口 `image/jpeg`。绿色是有效骨架，红色和红框为判动区域，失败保留判定帧直到 reset 或重新开始。图上 Left fingers=左手手指，Left arm=左手臂，Torso=躯干；底部显示双手观测状态。它不叠加到 RGB，以免异步画面错位；RGB 在摄像头卡单独监视。

`status` 提供 phase、round、reason、moved_part、moved_part_label、highlight_indices 和 hands 等信息。`reset` 回到安静等待口令，`stop` 释放游戏 DDS 进程。修改参数后停止并重启项目。

## 更新原 PR

不构建本地镜像。Windows PowerShell：

```powershell
cd D:\phanthymotus-driver
git add -A robotera/q5_bundle tests/test_q5_wooden_man.py tests/test_q5_wooden_man_host.py
git commit -m "refactor(q5): remove wooden man robot actions and keep voice vision gameplay"
git push origin feat/q5-wooden-man
```

在原 PR #427 评论 `/request_bot_review`，用机器人最新回复的镜像部署命令。部署后删除旧动作卡并刷新游戏卡配置，不复用旧镜像标签。

## 验证

```bash
python -X utf8 -m pytest tests/test_q5_wooden_man.py tests/test_q5_wooden_man_host.py tests/test_dockerfile_copies.py -q
```

`smoke_mcp.py` 检查真实 HTTP MCP；`smoke_ros.py` 使用真实 DDS、模拟 ASR/Pose 验证基础三轮、部位播报、手指及 JPEG；`smoke_interaction.py` 开启主持交互，验证口令后准备、轮间语音和再来。均不连接运动硬件，不需要 SDK 控制准备，且断言游戏不发布 actions。可使用已有 ROS 环境只读挂载当前源码测试，不需要构建新镜像。

```bash
PYTHONPATH=robotera/q5_bundle python robotera/q5_bundle/games/wooden_man/check_perception_contract.py /path/to/perception/plugins
```

协议检查读取感知服务真实 ASR/Pose/TTS 定义，执行真实 ASR 发布和 TTS 文字回调；源码包含 hand_runtime 时验证其合并手部关键点。ASR 实际识别精度、Pose 手指精度和 TTS 播放延迟仍需真机验证。
