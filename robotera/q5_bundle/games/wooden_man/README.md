# Q5 一二三木头人：语音与识别

单人／多人、固定三轮。`wooden_man` 使用画布提供的 ASR 与 Pose，完成邀请、规则问答、人数确认和举手登记，输出语音给 TTS，并显示玩家编号及判动部位的骨架高亮。保留灵敏度预设、可选手指识别和语音重开。

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

ASR 设置唤醒词“小范小范”，保持 **`publish_background=true`**。画布继续接已调通的上方 `asr`，木头人会自动补充订阅其同路径 `asr_background`，接收“不用介绍”“两个人”等无需再次唤醒的回答。待机时 background 不能发起游戏，只在游戏会话中处理回答；唤醒词由 ASR 处理，木头人不重复实现唤醒检测。`trigger_mode=vad` 时主输出没有 background 标记，也能使用本游戏。

本机器人此前确认 `/nvidia_desktop/mic/audio/asr` 有数据，可在木头人高级配置 `asr_topic` 保持该地址；其他机器人复制实际 topic，不照抄命名空间。自动补订阅仅适用于以 `/asr` 或 `/asr_background` 结尾的标准端口；自定义名字时通过 `input_topics` 同时传入两路。用 `info.input_topics` 检查实际订阅。

Pose 的 **`max_persons` 至少设置为玩家数加背景人数**，避免上游就把玩家截掉。游戏 `max_players` 默认 5，按机器人算力和可用画面调整；代码最多允许配置 10，但不代表已验证十人实时性能。

连好后通过顶部“开启智能控制”启动整个项目。木头人 ACTION 显示 `reset`，Pose 显示照片识别动作，是因为画布隐藏了 start/stop/info 生命周期操作；无需执行照片识别启动实时流。不要连接 decision_core。

## 游戏流程

1. 启动后安静待机，不检查玩家。说“小范小范，我们来玩木头人游戏吧”或“小范小范，我们来玩一二三木头人游戏”，进入 `rules_question`，回复“好啊好啊，需要我介绍规则吗？”旧口令“游戏开始”也进入问答。否定、不想玩、询问游戏是什么或怎么玩不触发开局。
2. 回答“需要／介绍一下”进入规则介绍；“不用／不需要／直接开始”跳过。介绍说明任何一人移动本局结束，全员坚持三轮成功。之后问“这次几个人一起玩？”接受“一个人／我一个人／两个人／我们有两个人一起玩”等。人数不清或超上限时重新问，不自行猜测。回答等待从播报预算结束后计时，默认 `dialogue_timeout=20` 秒。
3. 进入 `registering`：请参加的人**站立举手**。肩髋与腿部几何须满足站立条件；不要求双腿笔直，一条腿清晰伸向身体下方即可。允许手举过同侧肩膀，或屈肘使手腕举到肩旁且高于手肘；不要求举过头顶。躯干至少 10 像素，候选人数须恰好等于回答人数，同一批 ID 有效举手累计至少 `registration_seconds=1` 秒且 5 帧。明显坐着、趴着、没举手的路人不登记；人数多或少时提醒，不擅自挑选。默认等待 `registration_timeout=30` 秒。无需站在画面中央，也没有固定距离条件。
4. 登记后按**摄像头画面从左到右**编号 P1、P2……，提示放下手。等待已登记玩家放手站稳且连续清晰约 0.8 秒、5 帧；手指模式还要求每个人双手清楚。准备默认 `preparation_timeout=20` 秒。清楚后播报“游戏开始，五秒后开始”，从这句播报起等待 5 秒。
5. 播报“一，二，三，木头人！”，默认预留 `cue_seconds=3.0` 秒播报预算及 `settle_seconds=0.5` 秒站稳，再观察 `watch_seconds=3.0` 秒。每个玩家独立保存本轮基准、身体与手指计数，全部有效且没动才过关；轮间语音后重新取基准。共三轮，无机器人动作。
6. 任一玩家判动即结束整局，例如“2号玩家，你的左手臂动了，游戏结束”，只冻结并标红该玩家的对应部位。多处移动只播报一个，左右按玩家自身解剖侧。三轮通过播报“三轮完成，大家赢了，游戏结束”。不逐人淘汰。
7. 始终按 Pose 的持续跟踪 ID 认登记玩家，不按每帧检测列表重新编号，也不会把附近路人顶替为玩家。登记后蹲下属于玩家移动，不会被排除。观察时某玩家持续不可见超过 `pose_timeout=1.5` 秒，或观察结束仍缺少完整证据，进入 `paused`；原 ID 恢复并站好后重来**当前轮**，不算输赢。恢复默认等待 `recovery_timeout=20` 秒；若 Pose 已换 ID，不能证明还是原玩家，超时结束后重新登记。
8. 结束后询问“再挑战一次吗？”回答“再来／再来一次／好的／可以”重新进入规则问答和登记。说“取消游戏／不玩了／结束游戏”回到待机；游戏中重复邀请不会重启。

支持 JSON 顶层 `text` 或 `transcript`，标记 `final=false` 或 `is_final=false` 的中间结果忽略；带 ASR 起止时间戳的重复话语去重，当前会话自身完整播报回声忽略。播报期间不消费问答，避免抢答串到下一个问题。播报使用 `std_msgs/String` 包装 JSON `{"text":"..."}`，与 TTS 的 `data/json` 协议一致。关闭 `host_interaction` 仅用于旧版单人“游戏开始”直入倒计时，不支持新问答和多人登记，正常使用请保持开启。

播报预算不是 TTS 播放完成回执。对话与轮间语音按每秒约 4 个字、至少 `cue_seconds` 估计时间；实际播放较慢时调大 `cue_seconds`。等待机器人说完再回答，尽量使用现有 ASR 的回声消除。游戏是有限状态对话，不是任意闲聊理解。

## 举手登记排查

登记使用独立的 `registration_confidence=0.3`，接近 Pose 默认的关键点显示阈值；比赛判动仍使用 `confidence=0.5`，灵敏度设置不变。举手只用身体模型的肩、肘、腕，不需要开启手指模型。短暂关键点抖动允许 `registration_grace=0.35` 秒，长时间丢失或换 ID 清除确认，不会用路人顶替。

举手约一秒确认后，在登记提示语尚未播完时也会保留结果；同一玩家仍站立可见即可，不要求一直举到播报结束。进入正式准备后仍需身体达到比赛置信度要求；手指模式额外要求双手清楚。

`wooden_man → highlight` 在登记时显示每个跟踪编号的状态：`HOLD HAND`（举手中）、`CONFIRMED`（已确认）、`BODY UNCLEAR`（身体不清）、`CHECK LEGS`（站姿或腿部不清）、`RAISE HAND / SHOW WRIST`（举手未确认）。明确坐姿／卧姿的背景人物显示 `SITTING / IGNORE`、`LYING / IGNORE`。底部显示候选／确认人数和有效保持时间，不再只输出“0位候选”。

`status.registration_debug` 给出每人的 `track_id`、`posture`、`reason`、中文 `reason_label`、有效身体关节数、左右手腕／手肘是否可见，以及相对同侧肩和肘的高度（按躯干长度归一化）。`registration_confirmed_ids` 是已保持足够时间、等待播报完成的 ID。若仍失败，查看该 JSON 即可区分手腕置信度、举手高度和站姿问题；截图不能代替原始关键点诊断。

## 灵敏度与判动

| `sensitivity` | 身体位移／基准躯干长度 | 手指位移／基准手掌长度 |
| --- | --- | --- |
| `relaxed` 宽松 | 0.10 | 0.22 |
| `standard` 标准，默认 | 0.07 | 0.15 |
| `sensitive` 灵敏 | 0.04 | 0.10 |
| `custom` 自定义 | `motion_threshold` | `finger_threshold` |

以每人每轮第一帧有效姿态为固定基准，按头、躯干、左右手臂、左右腿分别统计连续超阈帧数；默认同一部位连续三帧判动，单帧尖峰不算，缓慢漂移也不会通过移动基准被放过。默认 `confidence=0.5`，肩髋和至少八个身体关节须可见。某玩家缺失会清除连续判动证据，不能拿另一个人的数据补齐。准备阶段另检查站姿和放下手。

旧配置里单独改过数字阈值的会保留为 custom，显式选择预设时以预设值为准。阈值越小越灵敏，也越容易受关键点抖动影响。

## 手指识别

同时开启 **Pose 的 `hands`** 和 **木头人的 `detect_fingers`**，继续连接原来 Pose 的 `sensor/pose2d` skeleton 输出，不接手势文字输出。默认关闭手指检测，兼容原有身体玩法。若感知服务没有 hands 选项，需要使用支持手部模型的感知服务；规则卡不自行加载模型。

多人启用手指检测时，Pose **`hand_max_rois` 至少设置为玩家数 × 2**（两人至少 4），默认 2 只够一个人的双手。背景近处的手也会竞争推理预算，画面中有其他手时需适当增加。每个玩家的手部点必须在其自己的 `persons` 记录内；缺少任意玩家的手部观测都不能判集体成功。提高人数和手部预算会增加 Jetson 推理负担，需真机测帧率。

协议为 COCO-17 后追加左右各 21 个手部关键点，共 59 点，并提供 `keypoint_names`，名字带 `left_hand_`／`right_hand_` 前缀。按名字查找，避免猜测数组顺序。以手腕为原点、基准手掌长度归一化，观察手指相对手腕的变化，纯平移手掌不误报成手指动作。

手掌参考长度至少 12 像素，21 点须满足置信度要求。默认 `finger_frames=2` 个不同手部观测连续超阈才判动。Pose 会复用缓存手部结果，重复坐标不重复计数；两只手每轮均须至少三个不同有效观测，末次新观测不能超过 pose_timeout。缺手、缺元数据、缓存长期未更新时提示没有看清双手，不会直接过关。上游没有独立手部推理时间戳，坐标变化只能保守去重，精度需真机验收。

## 高亮监视

`highlight` 为独立骨架 JPEG，使用 `sensor_msgs/CompressedImage`，端口 `image/jpeg`。绿色是有效骨架，红色和红框为判动区域，失败保留判定帧直到 reset 或重新开始。图上 Left fingers=左手手指，Left arm=左手臂，Torso=躯干；底部显示双手观测状态。它不叠加到 RGB，以免异步画面错位；RGB 在摄像头卡单独监视。

`status` 提供 phase、round、reason、expected_players、registered_players、registration_candidates、players（各玩家编号、track_id、可见性、有效帧、手部状态）、moved_player、moved_track_id、moved_part、highlight_indices 等。图中 P1/P2 是登记编号，Background 是未登记人物；仅判动者对应部位变红。`reset` 回到安静等待口令，`stop` 释放游戏 DDS 进程。修改参数后停止并重启项目。

## 更新原 PR

不构建本地镜像。Windows PowerShell：

```powershell
cd D:\phanthymotus-driver
git add -A robotera/q5_bundle tests/test_q5_wooden_man.py tests/test_q5_wooden_man_host.py
git commit -m "fix(q5): accept bent raised hands and explain registration failures"
git push origin feat/q5-wooden-man
```

在原 PR #427 评论 `/request_bot_review`，用机器人最新回复的镜像部署命令。部署后删除旧动作卡并刷新游戏卡配置，不复用旧镜像标签。

## 验证

```bash
python -X utf8 -m pytest tests/test_q5_wooden_man.py tests/test_q5_wooden_man_host.py tests/test_dockerfile_copies.py -q
```

`smoke_mcp.py` 检查真实 HTTP MCP；`smoke_ros.py` 使用真实 DDS、模拟 ASR/Pose 验证兼容模式的三轮、部位播报、手指及 JPEG；`smoke_interaction.py` 开启主持交互，验证两路 ASR 问答、两人举手登记、忽略背景、三轮通关及二号玩家判动高亮。均不连接运动硬件，不需要 SDK 控制准备，且断言游戏不发布 actions。可使用已有 ROS 环境只读挂载当前源码测试，不需要构建新镜像。

```bash
PYTHONPATH=robotera/q5_bundle python robotera/q5_bundle/games/wooden_man/check_perception_contract.py /path/to/perception/plugins
```

协议检查读取感知服务真实 ASR/Pose/TTS 定义，执行真实 ASR 发布和 TTS 文字回调；源码包含 hand_runtime 时验证其合并手部关键点。ASR 实际识别精度、Pose 手指精度和 TTS 播放延迟仍需真机验证。
