# LinkerHand L20 USB-CAN 驱动与四张卡片

本目录适配第四范式 Agent Core / PhanthyMotus，当前开发和实测设备为 **L20-10**：16 个主动关节、USB-CAN、五指指尖触觉反馈。L20 Lite（又称 L10）的 RS-485/Modbus 驱动位于相邻的 `l20_lite` 目录。

| 卡片 | 类型 | 功能 |
| --- | --- | --- |
| `hand` | `actuator` | 16 主动关节位置控制、张开、握拳、自然姿态与状态读取 |
| `hand_gesture` | `actuator` | 预置手势、0–9 数字、计数及已调试的拇指避碰路径 |
| `hand_grasp` | `actuator` | 纯位置侧向抓握、向下抓握和释放，不依赖触觉反馈 |
| `hand_tactile` | `sensor` | 按卡片选择采集左右手触觉，发布实时 JSON 和 JPEG 热力图 |

抓握与触觉感知独立运行。`hand_grasp` 不按指尖压力调节、不设置目标压力，也不依据压力判断是否抓住物体。当前触觉读数没有 `touch_scalar→N` 标定关系，原始值不能直接换算为牛顿。

## 1. 硬件连接与关节映射

使用 Linux SocketCAN，默认接口 `can0`、速率 `1,000,000 bit/s`。本部署已经实测的左右手 CAN ID 如下，以 `config.yaml` 为准，不自动交换或镜像：

| 设备 | 配置项 | CAN ID |
| --- | --- | --- |
| 左手 | `left_can_id` | `0x28`，十进制 40 |
| 右手 | `right_can_id` | `0x27`，十进制 39 |

关节位置采用 `0..255` 原始协议值，不是角度。20 项向量的映射如下；四个保留槽位不参与到位判断：

| Python 下标 | 对应关节，顺序为拇指／食指／中指／无名指／小指 |
| --- | --- |
| `0..4` | 五指指根弯曲，`*_pitch` |
| `5..9` | 五指侧摆，`*_yaw` |
| `10` | 拇指横摆／旋转，`thumb_roll` |
| `11..14` | 保留槽位 |
| `15..19` | 五指指尖弯曲，`*_tip` |

当前 L20-10 默认采用串联指令帧 `0x41..0x45`，每指打包根弯、侧摆及尖弯，拇指另含旋转；扭矩限值通过 `0x51..0x55` 等指令设置。底层也保留旧 L20 并联协议兼容入口，但不能将两种帧布局混用。

## 2. 卡片功能

### 2.1 `hand`：基础控制

在动作区选择本次操作的 `BOUND_SIDE`（`left` 左手或 `right` 右手），再执行动作。左右手选项初始留空，未选择或传入非法值会报错，不发送运动指令；每次请求都必须明确提供目标，不借用其他卡片或上一次执行的选择。

| 动作 | 行为 |
| --- | --- |
| `info` | 探测设备并读取所选设备的运行状态，不驱动关节 |
| `get_state` | 读取主动关节位置、速度及电机状态 |
| `open` | 移动到 `open_pose` 张开姿态 |
| `close` | 移动到 `fist_pose` 握拳姿态 |
| `reset` | 移动到 `natural_pose` 自然姿态后卸力 |
| `relax` | 移动到自然下垂姿态，可指定到位后的保持力矩 |
| `set_positions` | 按独立关节字段设置位置，留空的关节保留当前值 |
| `stop` | 停止所选设备在本卡片类型中的动作；抓握卡片持有该手时不抢占卸力，应执行 `hand_grasp.release` |

运动必须传 `confirm_motion: true`。`speed_ratio` 为 `1.0x/1.5x/2.0x/2.5x/3.0x`，当前配置基础速度为 85；`set_torque` 为本次运动的原始输出上限，`hold_torque` 为到位保持上限，默认 0。卸力表示发送电机输出限值 0，不表示切断设备电源。基础控制卡片不再提供 `get_tactile`，触觉数据通过 `hand_tactile` 获取。

### 2.2 卡片二：`hand_gesture` (手势与计数卡片)

每次执行需要选择 `BOUND_SIDE`；同一动作也可以从另一张手势卡片对同侧设备执行，不存在长期绑定关系。`stop` 按明确选择的左右手停止手势任务。

* **工具名称**：`hand_gesture`，类型：`actuator`
* **主要动作**：
  * `ok_gesture`：OK 手势（拇指与食指捏合，其余伸直）。
  * `thumb_up_gesture`：点赞手势（大拇指竖起，四指收紧）。
  * `v_sign_gesture`：剪刀手/胜利手势（食指、中指伸直呈 V 字）。
  * `natural_gesture`：移动到配置的自然下垂姿态。
  * `number`：仅做出所选的 0–9 数字手势，不先握拳回到 0，也不依次数过中间数字；切换保留已有避碰步骤。
  * `count`：数字计数动作，支持从 `start_num` 连续过渡到 `end_num`（0–9 支持正数与倒数）。
* **避障保护 (Clearance)**：
  * 拇指横跨手掌时自动执行外展避让（Clearance lift），防止与四指发生物理干涉。
  * 当前数字为 `1`、`2`、`9`，或目标数字为 `1`、`2`、`3`、`4`、`9` 时（切换到同一数字及以下三阶段路径除外），使用两阶段路径：先保持其余四指的实测位置，仅将拇指四关节移动到该手数字 `5` 的已配置姿态；确认到位后，四指和拇指共同移动到目标手势。非数字手势进入上述目标数字时也应用目标侧规则。
  * 三阶段路径优先于两阶段路径：**拇指先让位到数字 `5` → 拇指保持让位、四指移动到目标 → 四指确认到位后，拇指最后进入目标**。双向应用于 `0↔3`、`0↔4`、`0↔7`、`1↔3`、`1↔4`；单向应用于 `7→1`、`7→2`、`6→3`、`8→3`、`9→3`、`6→4`、`8→4`、`9→4`。用户确认无刮擦的反方向保留原有路径。计数初始化从 `3`、`4`、`7` 回到 `0` 也使用三阶段路径。任意阶段未到位都会中止后续动作。
  * `7→0`、`7→1`、`7→2` 的拇指让位细分为两小步：先保持侧摆、旋转及四指的实测位置，在同一个目标命令中将拇指指尖和指根同时伸展至 `255`；两者确认到位后，拇指四关节才移动到该手数字 `5` 的姿态。随后四指到位、拇指收尾，合计四个目标步骤；每一步均检查到位，可中断，继续沿用本次所选速度。
  * 数字 `0` 的实测反馈中，食指侧摆多次为 `159`，与配置目标 `193` 有偏差。数字手势识别、以及三阶段路径的到位检查中，不用已收拢手指的侧摆偏差否定到位；仍严格检查拇指四关节、四指指根和指尖，以及伸展手指的侧摆。三阶段中等待四指到位时，指根和指尖容差收紧至 `min(position_tolerance, 10)`，防止尚未充分伸展或收拢就收回拇指。原始目标姿态不变。
  * `0→7` 曾在第二阶段因 `pinky_pitch=15/0` 超过 10 点容差而中止，未执行拇指收尾。因此仅该方向第二阶段的小拇指指根沿用配置的 `position_tolerance`（当前为 `20`）；其余关节和已验证路径仍使用原有到位要求，目标姿态数值不变。
  * 让位参考读取对应侧的 `left_number_poses[5]` 或 `right_number_poses[5]`，索引为 `[0,5,10,15]`。当前配置的拇指指根、侧摆、旋转、指尖值分别为 `[255,70,255,255]`。避碰阶段按上述关节范围和容差检查到位；否则中止、卸力，通过 ACP 和驱动日志报告失败阶段、未到位关节的实测值及目标值。
  * `number` 与 `count` 共用上述数字切换规则；计数开始时也检查当前姿态，避免初始化绕过让位。`number` 不增加计数过程，`count` 的数字范围和展示顺序不变。仅拇指让位有时会恰好形成完整数字 `5`，这不是额外展示或计数步骤。
  * 速度按配置的基础速度乘以所选倍速计算，四舍五入并限制在 `255`；避碰阶段不另设速度。当前基础速度 `85` 对应 `1.0x/1.5x/2.0x/2.5x/3.0x` 的原始值为 `85/128/170/212/255`。离线检查验证双向路由、实测反馈回归、命令顺序、到位检查与中断；不同倍速下的实际机械间隙仍需分别通过实机验证。

运动参数中的 `hold_torque` 默认为 0，动作到位后卸力；需要继续保持姿态时可显式设置非零值。`count` 的 `hold_time_s` 是数字展示间隔。

### 2.3 `hand_grasp`：纯位置抓握

使用流程：**选择本次操作的左右手 → 选择抓法与速度 → 执行 → 需要放物时选择同侧设备并执行 `release`**。

| 动作 | 准备姿态 | 收拢目标 |
| --- | --- | --- |
| `side_grasp` | U 型包围姿态 | 五指指根 `0..4`、五指指尖 `15..19` 全部设为 0；侧摆与拇指旋转保留准备值 |
| `top_grasp` | 数字 5 的展开姿态 | 五指指根 `0..4`、四指指尖 `16..19` 设为 0；拇指尖 `15` 保持 255 伸直以避碰，侧摆与旋转保持准备值 |
| `release` | 继承所选设备当前抓握模式 | 五指同时恢复该模式准备姿态以松开物体，然后恢复最终张开姿态并卸力 |

两种抓法设置 `bound_side`、`speed_ratio` 和 `confirm_motion: true`；释放设置 `bound_side` 和 `confirm_motion: true`。速度原始值固定为 **85／128／170／212／255**，分别对应 1.0x～3.0x；准备与收拢使用相同的所选速度。释放沿用设备预设 `release_speed=80`。

准备阶段保留原来的拇指松开、让位及张开路径，并通过新位置反馈检查到位。准备到位后，一次下发完整收拢目标，不再逐点调压。**收拢阶段不等待实际位置到达 0**：物体会正常阻挡关节，不能因此判定抓取失败。返回“指令已下发，保持至 release”，其中 `command_sent=true`、`holding=true`；`success=true` 仅代表本次操作成功下发，不代表确认抓住了物体，也不代表关节已经到位。

下发后电机保留该位置目标及固定输出上限，直到主动对同侧设备执行 `release` 或明确停止。没有触觉保持线程，也没有到期自动释放。抓握状态按 `(左右手, CAN 接口)` 保存，因此另一张抓握卡片选择相同设备后也能正确释放。抓握保持期间，其他执行器卡片不能驱动同一只手；需要换动作时先释放。重复抓握会被拒绝，避免意外重新开手。`release` 可中断所选设备尚在准备中的抓握；重复释放复用正在执行的释放任务。

释放先读取新位置。如果已处于最终张开姿态的容差内，直接卸力，返回 `already_released=true`，不重复经过 U 型。否则依次确认准备姿态、最终张开姿态到位，再返回 `hand_opened=true`、`torque_released=true`。通信错误、准备或释放不到位仍会报错并尝试卸力，不会假报张开。

当前准备向量如下，保持用户提供的关节标定：

```yaml
grasp_prep_pose: [255,255,255,255,255,119,193,148,105,42,61,255,255,255,255,255,255,255,255,255]
top_grasp_prep_pose: [255,255,255,255,255,70,200,132,55,0,255,255,255,255,255,255,255,255,255,255]
```

可通过 `left_grasp_prep_pose`／`right_grasp_prep_pose`，以及 `left_top_grasp_prep_pose`／`right_top_grasp_prep_pose` 分别覆盖左右手标定。不会自动镜像；拇指尖在向下抓握的最终收拢目标中始终保持 255。

`config.yaml` 中的抓握配置仅保留启用开关、准备姿态、释放速度、固定输出上限 `motion_current=230`、位置轮询间隔 `poll_interval_s=0.05`、准备／释放到位超时 `pose_timeout_s=5.0` 和容差 `position_tolerance=20`。固定输出上限不是实测力或电流。旧压力目标、压力上限、步长、保持时间等配置已移除。

仅公开 `side_grasp`、`top_grasp`、`release` 三个动作；旧 `grasp`、`hold`、`get_tactile`、`get_matrix_touch` 不再支持。传入旧 `target_force`、`max_force`、`hold_duration_s` 等动作参数会报已移除参数，需按新卡片填写。内部保留 `config/start/stop/info` 供平台生命周期和状态查询使用，不再保存左右手绑定。默认同步返回本次下发／释放结果；API 可传 `wait_for_result=false` 获取 `action_id`，随后通过 ACP 接收完成通知。

### 2.4 `hand_tactile`：独立触觉传感器

类型为 `sensor`，不发送位置、速度或扭矩控制指令，不要求运动确认。

| 动作 | 当前行为 |
| --- | --- |
| `info` | 返回线程、发布器、订阅者数量、已发布帧数、最近发布时间、采样与发布错误 |
| `get_state` | 应用卡片所选左右手到正在运行的采集流，并主动读取新快照 |
| `start` | 启动本卡片选择的目标，返回该目标的实际输出话题；重复启动不重复创建线程 |
| `stop` | 停止本卡片的数据流；其他卡片仍使用的目标继续采集，不控制关节电机 |
| `config` | 更新高级 CAN 接口及共享采集频率；保留旧版显式左右手配置调用兼容 |

卡片动作区只显示 `ACTION` 和 `BOUND_SIDE`。左右手只在动作区选择，齿轮中不重复出现；齿轮保留“高级：CAN 接口”（默认 `can0`，不是 CAN ID）及共享采集频率。触觉卡片的 CAN 接口按实例设置，与执行器共享 CAN 设置独立；调整执行器接口不会切换已有触觉流或改写其话题路径。`instance_id` 由 Core 自动传入，后端用于区分采集卡片及线程使用者，不作为用户输入项展示。

当前 Core 启动智能控制时只发送卡片 ID，不转发动作区选项。首次使用或切换左右手时，应启动智能控制，在卡片上选择 `BOUND_SIDE` 后点击一次“执行”，再打开“查看数据流”。该执行同时切换实际采集、JSON、热力图和返回的话题，不只是一次查询。已打开的抽屉仍订阅旧话题，切手后请关闭并重新打开。之后同一驱动进程内再次启停，会记住本卡片的选择；容器重启后需重新执行选择。本驱动不修改 Core 源码。

JSON 包含 `touch_scalar`（五指原始标量）、`touch_matrix`（每指 12×6 数组）、`matrix_sums`、`contact_flags`、设备目标、`sequence`、快照生成时间及分项采样时间。五指字典键固定为 `thumb/index/middle/ring/pinky`。每轮标量采集要求五指均收到新回复；阵列采集要求五指共 60 行全部更新，不用旧行补齐新一轮。

`scalar_valid`、`scalar_age_s` 表示标量有效性与数据年龄；`matrix_valid`、`matrix_fresh`、`matrix_age_s` 分别表示阵列仍在有效期内、本轮是否刚查询完成、距上次完整回复的时间。未到阵列查询间隔时可复用仍有效的完整阵列，并明确标记 `matrix_fresh=false`。超时、部分回复和过期数据对应字段返回 `null` 及 `errors`，不伪造为零压力或零矩阵。标量无效时 `state=error`；仅阵列无效时为 `degraded`；正常时为 `ok`。设备异常也继续发布错误 JSON，网页应能显示故障状态。

当前不主动查询未经确认的法向力、切向力和方向数据；兼容字段 `normal_force/tangential_force/tangential_dir` 返回五指 `null`，`force_fields_valid=false`。这些字段不等同于设备已装对应选件或已标定牛顿值。

默认 CAN 接口上的 JSON 话题为 `/{namespace}/hand/{side}/tactile`，热力图为相同路径加 `/heatmap`；左右手分别使用 left/right，具体以本卡片 `info.topic_out` 为准。其他 CAN 接口自动使用不同路径，避免同侧不同适配器混流。正常驱动启动时，命名空间依次取 `ROS_NAMESPACE` 环境变量、非空 `ros_namespace` 配置或容器主机名，并规范化为合法字符。JSON 使用 `std_msgs/msg/String`、`data/json`；热力图使用 `sensor_msgs/msg/CompressedImage`、`image/jpeg`，热力图排在第一个输出，供“查看数据流”直接显示。QoS 均为 BEST_EFFORT、KEEP_LAST、depth=1、VOLATILE。

热力图采用 Gemini 已加入的五指 12×6 阵列和指尖压力条，画面标明当前左右手。缺失／超时数据标为 `NO DATA`／`Tip: --`，不绘制成有效零压力。`info.heatmap_publisher_ready`、`heatmap_published_frames`、`heatmap_publish_error` 可诊断图像发布状态。渲染依赖 Pillow 和 NumPy，已列入依赖，Dockerfile 也包含 `tactile_heatmap.py`。

| `plugins.hand_tactile` 配置 | 默认值 | 含义 |
| --- | --- | --- |
| `enabled` | `true` | 注册传感器卡片；需要同时启用 `hand` 以提供共享控制器 |
| `poll_hz` | `20.0` | 后台循环期望频率，通信耗时较长时实际频率会降低 |
| `auto_start` | `true` | 驱动启动时先采集默认目标，卡片选择其他目标后停止无人使用的默认流 |
| `matrix_enabled` | `true` | 是否读取阵列；关闭后仍主动读取标量 |
| `matrix_request_interval_s` | `0.05` | 请求完整五指阵列的最小间隔 |
| `tactile_timeout_s` | `0.25` | 单轮标量查询等待新回复的超时 |
| `matrix_timeout_s` | `0.25` | 单轮阵列查询等待完整新行的超时 |
| `stale_after_s` | `1.0` | 判断已采样标量／阵列过期的时长 |
| `contact_threshold` | `8` | 标量大于等于该值，或阵列总和为正时，接触标记为 true |

发布器初始化失败、发布异常会记录到 `info.publish_error` 并重试，不再只在后台静默失败。`publisher_ready=true` 表示发布器已创建；`published_frames` 增长表示本地调用发布成功；只有 Core／网页实际收到递增的 `sequence`，才证明整条 DDS／WebSocket 链路打通。`state=running` 单独只表示采集线程在运行。

后台按 `(左右手, CAN 接口)` 分别采集和发布，卡片通过自动传入的实例 ID 选择目标。切换一张卡片不会改写其他卡片的目标；同一物理目标在驱动内复用一个采集线程，最后一个使用者停止后才关闭该线程。`poll_hz` 通过共享 `config` 同时更新各目标的期望频率；更换正在使用的 CAN 接口时同步迁移该卡片的采集和话题，不再固定使用默认左手。

Core 可先发送共享 `poll_hz`，再发送实例 `can_channel`。单独设置频率不要求左右手，也不覆盖实例选择；兼容调用中显式传入的 `bound_side` 仍必须是 `left/right`。启动采集后，通过 WebSocket 持续接收图像和 JSON，无需重复点击 `get_state`；JSON 的 `sequence` 和 `timestamp` 用于判断是否在更新。停止采集后，网页可能保留最后一帧，它不代表仍在更新。

## 3. 验证

在仓库根目录运行离线回归，不连接真实 CAN，也不驱动设备：

```bash
python3 -m pytest tests/test_linkerhand_l20.py tests/test_linkerhand_l20_binding.py tests/test_linkerhand_l20_grasp.py tests/test_linkerhand_l20_can_reconnect.py tests/test_linkerhand_l20_tactile.py tests/test_linkerhand_l20_configuration.py tests/test_linkerhand_l20_heatmap.py tests/test_linkerhand_l20_explicit_gesture.py -q
python3 -m pytest 'tests/test_dockerfile_copies.py::test_every_copied_path_exists[linkerhand/l20/Dockerfile]' 'tests/test_dockerfile_copies.py::test_every_imported_sibling_is_copied[linkerhand/l20/Dockerfile]' -q
python3 scripts/check_service_yml.py linkerhand/l20
```

硬件诊断脚本需能访问主机 SocketCAN，并安装运行依赖：

```bash
# 仅在 can0 尚未配置时启用接口
sudo ip link set can0 up type can bitrate 1000000

# 只读诊断：位置及触觉缓存
PYTHONPATH=. python3 linkerhand/l20/hardware_smoke.py --read-only

# 阵列缓存诊断：输出不代表已完成本轮全部行的新鲜度验证
PYTHONPATH=. python3 linkerhand/l20/hardware_smoke.py --tactile-matrix
```

实际抓握效果和各倍速的机械间隙需实机检查；离线模拟不证明抓住物体或传感器已标定。

## 4. ARM64 构建与容器更新

驱动端口 `15734`，L20 Lite 为 `15733`。部署服务见 `deploy/service.yml`：使用 host 网络、host IPC、`privileged: true`、`/dev` 和 DDS 配置挂载，ROS 域为 42。依赖为 `python-can`、`pyyaml`，ROS 2 及 `std_msgs` 由基础镜像提供。

`Dockerfile` 包含四张卡片的 Python 模块。完整构建时需把仓库 `common/` 加入构建上下文；`driver.yaml` 已通过 `build_context_extras` 声明该目录，并列出三张 actuator 和一张 sensor。以下命令在仓库根目录构建本地 ARM64 镜像，不推送镜像仓库：

```bash
l20_build_context=$(mktemp -d /tmp/linkerhand-l20-build.XXXXXX)
cp -a linkerhand/l20/. "$l20_build_context/"
cp -a common "$l20_build_context/common"
docker buildx build --platform linux/arm64 --load -t linkerhand-l20:latest "$l20_build_context"
```

运行中容器的开发更新可以只复制本次发生变化的文件并重启。纯位置抓握、触觉卡片及热力图同时上线时，至少需要同步以下文件，并确保容器已安装 Pillow、NumPy：

```bash
cd /home/anachronism/wangyunpeng2/phanthymotus-driver
docker cp linkerhand/l20/tactile_device.py embodied-linkerhand-l20:/work/tactile_device.py
docker cp linkerhand/l20/tactile_heatmap.py embodied-linkerhand-l20:/work/tactile_heatmap.py
docker cp linkerhand/l20/hand_device.py embodied-linkerhand-l20:/work/hand_device.py
docker cp linkerhand/l20/gesture_device.py embodied-linkerhand-l20:/work/gesture_device.py
docker cp linkerhand/l20/grasp_device.py embodied-linkerhand-l20:/work/grasp_device.py
docker cp linkerhand/l20/config.yaml embodied-linkerhand-l20:/work/config.yaml
docker restart embodied-linkerhand-l20
docker logs --since 1m --tail 60 embodied-linkerhand-l20
```

底层通信模块等文件也有变更时应一并复制。`docker cp` 修改容器可写层，不更新原镜像；以后删除并重建容器，应使用包含最新文件的 ARM64 镜像。重启会清空内存中的运行任务和抓握模式，不保留执行器绑定；刷新页面后应看到四张卡片，执行器按动作区选择左右手，`hand_grasp` 仅有三个公开动作。

## 5. 执行目标、共享 CAN 与停止行为

三张执行器的设备访问请求通过本次 `bound_side` 明确指定左右手，不要求 `instance_id`，也不保存卡片到左右手的绑定。动作区初始提供空选项；未选择、空值或非 `left/right` 值均不能访问设备。旧齿轮配置中残留的 `bound_side` 不作为后续动作的默认目标。平台的 `start/config` 可以不带左右手，仅完成生命周期或共享设置处理。

执行器使用 `hand` 侧栏齿轮中的共享 CAN 接口，默认 `can0`；三个执行器采用同一接口，不再按卡片保存。它不是左右手 CAN ID。运动或抓握保持期间不能切换该接口，应先停止任务并释放物体，再调整设置。触觉卡片继续使用自身的实例 CAN 设置。

每次接受运动前都探测所选侧的 CAN ID，要求全部手指的新位置回复；保留 USB-CAN 但断电或换手时会报告所选手未响应，历史缓存不能代替在线检查。同一设备的运动互斥，左右手的运行与抓握状态独立。

删除卡片不会留下执行器绑定记录，但也不能代替主动停止或释放。旧画布的执行请求可能不传 `instance_id`，而删除请求仅携带卡片 ID，此时无法可靠关联此前的匿名任务。驱动不会猜测设备或停止另一张卡片的任务：请先在 `hand/hand_gesture` 中选择对应侧执行 `stop`，抓握则先在 `hand_grasp` 中选择对应侧执行 `release`，确认完成后再删除卡片。能够关联到带 ID 原请求的停止调用只处理对应任务。

当前平台的通用运动中断钩子不会继承原请求的左右手参数，因此三张执行器不再声明无目标的 `on_interrupt_motion` 钩子。平台通用中断不能替代上述显式停止／释放流程。执行器的普通执行不依赖平台额外传入卡片 ID，驱动部署无需修改 Agent Core 文件；触觉卡片仍使用平台已有的传感器实例机制管理采集线程。

## 6. QEMU 断线重连

SocketCAN 的 QEMU 兼容钩子需在线程锁保护下只安装一次。重连不能继续包装进程级 `socket.socket.setsockopt`，否则包装层累积后会出现 `maximum recursion depth exceeded`，影响所有卡片及 HTTP 注册。

旧进程若已递归溢出，更新代码后还需重启容器以清除旧包装层。该问题与已实测的左右手 CAN ID 无关。离线回归包含重复安装、并发安装和多次 CAN 打开失败后恢复。

### QEMU 6.2 与 DDS 数据流

在本机 x86 虚拟机中，QEMU 6.2 的 ARM64 用户态仿真对 `setsockopt(IPPROTO_IP, IP_MULTICAST_IF)` 返回 `ENOPROTOOPT`。Fast DDS 因此无法正常创建跨参与者 UDP 发送通道：同一 ROS 上下文可自收，跨上下文或跨容器无法发现／收流。CAN 查询成功、HTTP 卡片返回正常、采集线程已启动，均不能排除这一独立问题。

[QEMU 上游问题 #1837](https://gitlab.com/qemu-project/qemu/-/issues/1837)记录了该限制；[QEMU 10.0 源码](https://github.com/qemu/qemu/blob/v10.0.0/linux-user/syscall.c)已实现该 socket 选项。x86 仿真部署需使用具备该支持的 QEMU，并对真实 DDS／WebSocket 收流做验证；真实 ARM64 主机不经过此用户态仿真路径。升级宿主机模拟器及重启依赖容器是单独的环境操作，设备驱动本身不执行这些操作。

2026-10-23 本机经授权升级至 QEMU 10.0.4，并重启 Core、L20 后，已在网页同一 WebSocket 通道连续收到递增序号的有效 JSON；五指标量及完整阵列均有效。安装来源为官方 `tonistiigi/binfmt` 的 `deploy/v10.0.4-59` 发布包，下载内容已按 GitHub 发布记录的 SHA256 校验。当前 ARM64 binfmt 注册保留 guest `argv[0]`（`P` 标记），宿主原有 `/usr/bin/qemu-aarch64-static` 启动路径也已更新；旧二进制备份为 `/usr/bin/qemu-aarch64-static.l20-before-6.2`。Core 源码、镜像及挂载配置未修改。
