# Q5 一二三木头人

`wooden_man` 是规则与主持处理卡；`wooden_man_motion` 是独立的手臂动作执行卡。摄像头、Pose、ASR、TTS、扬声器均为独立画布卡片，不在规则卡内创建模型或调用隐藏的 MCP 工具。游戏卡只发可观察的动作事件，执行卡通过共享 Q5 命令路由控制手臂。Q5 保持已经建立的站姿，不控制底盘、腿或头，不自动切换控制模式。

## 画布连线

在已部署的 Motus 中添加 Q5 驱动以及提供 ASR / Pose / TTS 的感知服务，新建项目。正式游戏共 **8 张卡、7 条线**：`mic`、`camera_rgb`、`asr`、`pose`、`wooden_man`、`wooden_man_motion`、`tts`、`speaker`。

```text
Q5 mic ──audio/pcm-16k──> ASR ──data/json────────────┐
                                                  v
                                          一二三木头人 wooden_man
                                                  ^
Q5 camera_rgb ──image/jpeg──> Pose ──sensor/pose2d────┘

wooden_man 的 speech ──data/json──> TTS ──audio/pcm-16k──> Q5 speaker
wooden_man 的 actions ──data/json──> wooden_man_motion
wooden_man 的 status ──data/json──> 画布状态显示（无需接 TTS）
wooden_man 的 highlight ──image/jpeg──> 查看数据流／监控（骨架高亮，无需连线）

wooden_man_motion 的 feedback ──配置topic返回──> wooden_man
```

| 起点输出 | 终点输入 | 格式 |
| --- | --- | --- |
| `mic` 音频 | `asr` 音频 | `audio/pcm-16k` |
| `asr` 的 `asr_background`（默认唤醒模式） | `wooden_man` 的 ASR 输入 | `data/json` |
| `camera_rgb` RGB 图像 | `pose` 图像 | `image/jpeg` |
| `pose` 的 **skeleton 骨架输出** | `wooden_man` 的 Pose 输入 | `sensor/pose2d` |
| `wooden_man` 的 **speech** | `tts` 文字输入 | `data/json` |
| `tts` 合成音频 | `speaker` 音频输入 | `audio/pcm-16k` |
| `wooden_man` 的 **actions** | `wooden_man_motion` 唯一输入 | `data/json` |

反馈不要再画一条回到木头人的线，避免画布拓扑形成环。木头人配置里的 `motion_feedback_topic` 已自动填好同一 Q5 驱动动作卡的 feedback 路径；其值必须等于动作卡 `info.topic_out` 中 `feedback` 的真实 topic，例如 `/<命名空间>/wooden_man_motion/feedback`。这是显式配置的 DDS 反馈订阅，不是额外 MCP 调用。不要把 `actions`、`status` 或 `feedback` 接到 TTS。

### 第一次启用手臂动作

1. 先按 Q5 现有流程建立稳定站姿。使用现有 **`arm_control` → `prepare`** 完成位置直控准备；该准备流程会实际执行厂商的垂手、抬臂及模式切换，不由游戏自动执行。
2. 准备完成后，用 **`arm_gesture` → `reset`** 将手臂回到现有中性姿态，再开启游戏项目。这两张调试卡不接游戏数据线，可放在单独调试项目中；正式游戏只保留上述 8 张卡。
3. 木头人配置保持 `host_interaction=true`、`robot_actions=true`；动作卡的 `image_mirrored` 默认 false。只有摄像头原始输出确实左右镜像时才开启，不要根据网页预览镜像猜测。
4. 如暂时只验证语音与视觉，可关闭 `robot_actions`，不用启动动作卡；准备引导和语音重开仍可使用。

执行卡只写肩俯仰、肩侧摆、肘俯仰三个关节（单侧或双侧），其余手臂关节须有新鲜反馈且靠近中性位。动作限制到 URDF 范围，单次目标相对起点最多 0.8 rad，插补速度最多约 0.20 rad/s、每步最多 0.01 rad。关节反馈接收年龄须不超过 500 ms，机器人 FSM 反馈不超过 1000 ms；动作中持续复查，缺失关节立即停止后续插补。正常完成会回到动作前实测姿态，并确认关节到达（误差不大于 0.08 rad），才发 completed。缺少位置直控准备、起始姿态偏离、过期反馈、其他控制器占用或发布冲突时拒绝执行。不要在正式游戏项目并行运行其他手臂／身体控制卡。

指向是 **按图像九宫格做手臂方位示意**，不是三维精确指尖定位。判断采用高亮区域中心：图像左右决定出哪只手臂，上中下决定抬臂高度，播报的左右仍按玩家解剖侧。实际幅度与相机方向必须在真机上校准，离线模拟不证明机械碰撞净空或真实指向精度。

Q5 在完成位置直控准备后仍可能报告厂商 `/mpc_policy_node` 端点；现有 `arm_control` 也不把它的存在直接当作冲突。主持动作卡兼容根命名空间下的单个同名 MPC 端点，不需要停止该厂商节点。未知发布者、其他命名空间的同名节点、多个 MPC 端点、重复的 `q5_body_command` 发布者仍拒绝执行，错误消息会指出节点。这个兼容不代表节点名称能够证明关节指令互斥；位置直控准备、实时 FSM、共享路由控制权、关节反馈及实测到位检查依然必须通过。

Pose 必须输出 **人体 COCO-17 二维关键点**，不能连接机器人自己的 `joints` / `sensor/skeleton`。协议为 `std_msgs/String` 承载 JSON：

```json
{"persons":[{"id":1,"keypoints":[[100,100,0.9]]}],"image_size":[848,480]}
```

上例只展示结构；真实帧需有 17 个关键点，坐标为像素，建议输出至少 5 Hz。只支持单玩家，肩和髋必须清晰可见，身体至少八个关键点有效。多人、遮挡或丢帧持续超过 1.5 秒，会播报没有看清并结束，不会判为过关。

ASR 两路输出 `asr`、`asr_background` 都是 `data/json`，选择其中一路连接到木头人的 ASR 输入。**默认唤醒词模式下，直接喊“游戏开始”通常走 `asr_background`**，需开启 ASR 的 `publish_background`；若 ASR 设置为 `trigger_mode=vad`，则接 `asr`。不要为本游戏连接 decision_core。ASR 输入支持 `{"text":"游戏开始","background":true,"priority":0}`，也兼容带 `is_final` / `final` 的文字消息。中间结果标记为 false 时忽略。去除空格和标点后，整句必须是“游戏开始”。游戏中重复口令不重启。结束后再喊一次可重新玩。

播报端口 `speech` 的 ROS 消息为 `std_msgs/String`，内容是 `{"text":"一，二，三，木头人！"}` 这样的 JSON 对象，与真实 TTS 的 `data/json` 输入及顶层 `text` 字段一致。`status` 同样是 JSON，但只用于监视状态，不接入 TTS。

启动整个画布项目，规则卡 `start` 仅进入等待口令；不会自行开始游戏。画布通过 `input_topics` 传入两条上游 topic。若当前 Motus 版本只传入单个 `input_topic`，把 ASR 和 Pose 的真实输出 topic 分别填入规则卡配置的 `asr_topic`、`pose_topic`。从上游卡 `info.topic_out` 复制，不能猜名字。规则卡输出固定为 `/<机器人命名空间>/wooden_man/speech` 和 `/status`。

当前画布把 `start` / `stop` / `info` 作为生命周期动作隐藏，不显示在 ACTION 下拉框：木头人只显示 `reset`，Pose 显示照片/URL识别，Speaker 显示音量动作，这不代表它们不支持实时流。连好线后，通过顶部“开启智能控制”启动整个项目，由画布自动调用各卡的 `start`。无需在 Pose 下拉框寻找“实时模式”，也不要执行照片识别来启动流处理。

Core 会先通过隐藏的 `config` 动作下发保存的参数，再调用 `start`。本卡显式处理 `config`，校验并保存参数，但不启动游戏；相同参数重复下发不会重启正在运行的游戏。无效参数通过 MCP 错误返回，且不会覆盖上一次有效配置。修改运行中的参数会先停止本卡，由下次显式 `start` 使用新配置。

## 游戏时序和判定

1. 开启整个画布后安静等待“游戏开始”。待机期间不检查玩家、不播报没有看清或准备好了；摄像头和 Pose 数据流仍可正常监视。玩家可以先靠近机器人喊口令，再走到镜头前。
2. 收到“游戏开始”后，才检查单人身体关键点是否持续清晰（至少约 0.8 秒、5 帧）。不要求站在画面中央，靠左靠右均可；身体没露全／手部模式看不清双手时才给语音引导，提示至少间隔 6 秒。准备检查默认最多等待 20 秒。看清后做欢迎手势，收到动作卡 completed 后额外等待 `robot_settle_seconds=1.0` 秒，且不早于播报预算，再说“游戏开始，五秒后开始”，从这句播报起固定等待 5 秒。
3. 播报“一，二，三，木头人！”，默认预留 3 秒语音及 0.5 秒站稳时间。
4. 随后的 3 秒观察窗口，以第一帧有效姿态为基准。标准预设中，关键点相对固定躯干长度移动超过 7%，同一部位连续三帧达到阈值即失败。播报如“你的左手臂动了，游戏结束”，冻结并标红判定骨架，同时请求手臂指向该高亮区域的方位；同时移动多处只播报一个。左右按玩家自身解剖侧，非观看屏幕的左右。头部、躯干、左右手臂、左右腿分别判断，部位轮流抖动不会合并成连续三帧。手指检测见下文；眨眼不检测。Pose 缺少图像尺寸时不猜指向，改做鼓励示意。
5. 没动则说“不错，第二／三轮来了”，做轮间手势，回位并站稳后才播报下一轮口令、重新取基准。手臂动作和站稳期间不判断玩家移动，也不沿用上轮基准。
6. 三轮全部通过，说“三轮完成，你赢了，游戏结束”，做双臂庆祝并回位。失败则完成指向／鼓励并回位。之后询问“再挑战一次吗”；回答“再来／再来一次／好的／可以”等会重新检查站位，回答“不玩了”回到待机。也可以再次说完整口令“游戏开始”。游戏和动作进行中忽略重复开始口令。

动作请求带唯一 request_id、重试和心跳，执行卡去重，不因重复消息重复抬臂；游戏只接受当前请求的反馈。动作未反馈在 30 秒超时后暂停，不会直接进入观察。游戏停止、reset、请求心跳中断约 2 秒或动作卡停止都会取消后续插补，释放路由控制权；异常取消不强制执行回位轨迹。检查动作卡 `feedback.state/message` 后重新准备／reset。反馈错误不会伪装成动作完成。

默认 `cue_seconds=3.0` 是给外部 TTS 的固定播报预算，并非 TTS 播放完成回执。实际网络/TTS较慢时，应调大此值，确保“木头人”说完才开始判断。新增手指判定仍需在机器人上验收，请按实际 TTS 延迟校准。不要把扬声器回声作为 ASR 输入；尽量启用现有 ASR 的回声消除。

### 灵敏度、手指和高亮

部署新镜像后刷新驱动工具列表，必要时删掉旧 `wooden_man` 卡并重新添加，保留原来的语音和视觉连线，新增 `wooden_man_motion` 卡及 actions 连线。打开木头人卡片的配置：

| `sensitivity` 灵敏度 | 身体阈值／躯干长度 | 手指阈值／手掌长度 |
| --- | --- | --- |
| `relaxed` 宽松 | 0.10 | 0.22 |
| `standard` 标准（默认） | 0.07 | 0.15 |
| `sensitive` 灵敏 | 0.04 | 0.10 |
| `custom` 自定义 | 使用高级配置 `motion_threshold` | 使用高级配置 `finger_threshold` |

阈值越小越灵敏，也越容易受到关键点抖动影响。默认标准档保留原有身体阈值。旧配置中单独修改过的数字阈值会保留为自定义；显式选择预设后以预设值为准。`moving_frames` 默认 3（身体连续帧）；`finger_frames` 默认 2（不同手部观测采样），不是两个身体帧。

要检测手指，同时开启 **Pose 卡配置的 `hands`**（Hand keypoints and gestures）和 **木头人卡的 `detect_fingers`**。默认关闭以兼容旧 Pose 和已有身体玩法。需要的仍是原来 Pose 的 **`sensor/pose2d` skeleton 输出**，不用换到手势文字输出，不用加第二张 Pose 卡。照片识别 ACTION 与实时游戏无关。Pose 的 `hands` 会额外加载手部模型；若现有感知服务没有这个选项，应先更新支持手部关键点的感知服务，仅更新 Q5 驱动不能补出手部推理能力。

手部协议为 COCO-17 后追加左右各 21 点，共 59 点，并提供 `keypoint_names`，手部名字带 `left_hand_` / `right_hand_` 前缀。按名字查找，不猜手部数组顺序。每轮独立建立双手基准，以手腕为原点、固定基准手掌长度归一化，检测手指相对手腕的形状变化；纯粹平移手掌由身体手腕规则判断，不误报为手指。

请让双手无遮挡、手指在画面中足够大；手掌参考长度低于 12 像素或关键点置信度不足不会参与判定。上游还可能有更严格的手部距离门限。连续重复的手部坐标作为缓存跳过，不累积判动采样数；每轮两只手均需至少 3 个不同的有效观测，且末次新观测不能超过 `pose_timeout`。缺失手部元数据、只见一只手、坐标缓存长期不更新等情况会播报“没有看清双手”，结束本局，**不会当成手指没动而过关**。遮挡后重新看清手会重新取手部基准，因此遮挡本身不是作弊检测。上游未提供独立手部推理时间戳，坐标变化仅用于保守去重，不能证明模型每次都产生了新推理。

打开 `wooden_man` 的“查看数据流”，选 **`highlight` 图像**：绿色为有效骨架，红色和红框为判动部位，失败时保留判定骨架，直到 reset 或再次喊“游戏开始”。底部显示双手是否可见和采样数。图中文字为英文：Left fingers=左手手指，Left arm=左手臂，Torso=躯干。这是独立骨架监视图，**没有把骨架叠加到 RGB 画面上**；原始摄像头仍在 `camera_rgb` 的“查看数据流”看。这样避免异步 RGB/Pose 画面错位导致高亮误指。图像使用 `sensor_msgs/CompressedImage` JPEG，端口 `image/jpeg`；状态 JSON 附带 `moved_part`、`moved_part_label`、`highlight_indices` 和双手观测状态，speech 仍为 TTS 所需的 `data/json`。

高级配置还保留 `watch_seconds`（观察时长）、`confidence`、`pose_timeout`、`cue_seconds`、`settle_seconds`，新增 `preparation_timeout`（站位准备最多 20 秒）、`motion_timeout`（动作反馈最多 30 秒）、`robot_settle_seconds`（动作完成后站稳 1 秒）。修改配置后停止并重新启动画布生效。`reset` 取消当前动作并回到等待口令；`stop` 销毁游戏 DDS 子进程。单人限制、固定三轮、口令后的五秒倒计时保持；新增准备互动在五秒倒计时之前完成。

## 通过原 PR 构建与部署

本项目后续不要求本地构建镜像。Windows PowerShell 中更新现有分支和 PR #427：

```powershell
cd D:\phanthymotus-driver
git add robotera/q5_bundle/wooden_man.py robotera/q5_bundle/wooden_man_motion.py robotera/q5_bundle/config.yaml robotera/q5_bundle/driver.yaml robotera/q5_bundle/Dockerfile robotera/q5_bundle/games/wooden_man tests/test_q5_wooden_man.py tests/test_q5_wooden_man_host.py
git commit -m "feat(q5): add interactive wooden man host and arm pointing"
git push origin feat/q5-wooden-man
```

在原 PR 评论 `/request_bot_review`，等待机器人生成本次提交的新镜像。使用它新回复的部署命令，不复用旧标签。本地代码修改不会自动更新机器人。更换镜像前先拉取新镜像、停止 Q5 驱动旧容器并关闭其自动重启，保留 core、perception、actucore；确认新 `embodied-robotera-q5` 的 `Config.Image` 与新标签一致，再刷新画布。

## 验证

仓库根目录运行离线行为测试：

```bash
python -X utf8 -m pytest tests/test_q5_wooden_man.py tests/test_q5_wooden_man_host.py tests/test_dockerfile_copies.py -q
```

PR 镜像构建后将 `IMAGE` 替换成真实标签，在隔离的开发运行环境中检查。也可复用已有 ROS 镜像、只读挂载当前 Q5 源码并设置 `PYTHONPATH` 测试，不需要重新构建。第一条检查两张卡的真实 MCP HTTP 包装及装配，第二条是禁用主持交互的基础游戏回归，第三条验证完整主持流程：

```bash
docker run --rm --platform linux/arm64 --entrypoint bash IMAGE -lc 'source /opt/ros/humble/setup.bash && source /q5_ws/install/setup.bash && python3 /work/games/wooden_man/smoke_mcp.py'
docker run --rm --platform linux/arm64 --entrypoint bash IMAGE -lc 'source /opt/ros/humble/setup.bash && source /q5_ws/install/setup.bash && python3 /work/games/wooden_man/smoke_ros.py'

docker run --rm --platform linux/arm64 --entrypoint bash IMAGE -lc 'source /opt/ros/humble/setup.bash && source /q5_ws/install/setup.bash && python3 /work/games/wooden_man/smoke_interaction.py'
```

`smoke_interaction.py` 使用真实 HTTP 与 ROS 2 DDS，包括请求重试、动作完成反馈和语音重开；Q5 客户端、共享命令路由及关节反馈均为内存模拟，不连接机器人、不发布厂商关节命令。它覆盖准备→欢迎→三轮→庆祝→再来→判动指向→动作中 reset。单元测试另外覆盖限位、每步幅度、位姿反馈不达标、准备缺失、关节过期、控制权冲突、重复请求去重、旧反馈拒绝和失去心跳。

真实摄像头 Pose 推理精度、ASR 识别与 TTS 播放延迟仍需在机器人接好画布后验收。部署脚本生成 `docker-compose.wooden-man.yml`；之后手工运行 compose 时也要带上这个 override，避免基础 compose 中的旧镜像覆盖新版本。

可进一步针对实际感知服务源码运行协议检查（不启动模型）。源码目录应包含 `asr.py`、`tts.py`、`pose.py`：

```bash
PYTHONPATH=robotera/q5_bundle python robotera/q5_bundle/games/wooden_man/check_perception_contract.py /path/to/perception/plugins
```

该检查读取真实卡片端口定义，并运行真实 ASR 后台消息发布函数和真实 TTS 文字回调，验证 `ASR JSON -> 游戏 -> TTS JSON`。目录同时含 `agent-core_src_tool_config.py` 时，还会运行 Core 实际的配置计划函数。镜像内 `smoke_ros.py` 通过真实 HTTP MCP 执行 `config -> start -> info -> reset -> stop`，同时验证 DDS 游戏链路，避免只绕过控制面直接调用规则类。
