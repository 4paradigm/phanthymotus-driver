# Dobot CRA joints, joint_move and teach

基于官方 V4 Python SDK 的 Jetson Docker 驱动。本版提供并列的 `joints`、`joint_move` 和 `teach`
三张可见卡片，URDF 由后台 `model` 资源提供。SDK 版本及源码校验值固定在 [requirements.txt](requirements.txt)。

## 卡片

| 名称 | 类型 | 内容 |
|---|---|---|
| `joint_move` | actuator | 绝对/相对关节运动、停止、暂停及继续 |
| `joints` | sensor | 六轴实际角度驱动的 CR3A 结构骨架、逐关节转角 JSON，以及共享连接配置 |
| `teach` | actuator | 驱动控制的拖拽录制、按名称管理轨迹、到起点运动、ServoJ 流式回放及暂停/恢复 |

`model` 为渲染器内部读取的 resource，不是可见卡片。
本版不注册 `safe_skin` 或 `safe_skin_control`，不发布安全皮肤 topic。
升级后请移除画布中的旧安全皮肤卡片及连线；不会自动修改已有画布。

## 关节输出

默认以 30Hz（约每 33ms）发布 `std_msgs/String` JSON，两个输出共用同一份反馈快照：

- `/dobot_cra/state/joints`，`sensor/skeleton`；仅包含 `timestamp_ms/available/joints`。
  每个关节包含 `name/q`，`q` 为弧度，关节名为 `joint1..joint6`。
- `/dobot_cra/state/joint_angles`，`data/json`；包含 `timestamp_ms/available` 和
  `joint1_deg` 至 `joint6_deg`，每个关节转角单独一行，单位为度。

JSON 输出示例（数值为示例）：

```json
{
  "timestamp_ms": 1793856449751,
  "available": true,
  "joint1_deg": 90,
  "joint2_deg": -30,
  "joint3_deg": 0,
  "joint4_deg": 0,
  "joint5_deg": 0,
  "joint6_deg": 0
}
```

断线或过期时 `available=false`，骨架 `joints=[]`，六个角度字段均为 `null`，不输出旧角度。
角度 JSON 不依赖 URDF 是否可用；型号不匹配时仅骨架停止提供关节位置。
输出不含 IP/端口、连接诊断、包计数、SDK 来源或冗余角度数组。
旧下游若依赖 `joint_names/angles_deg`，请改读 `joint1_deg` 至 `joint6_deg`。

## 卡片介绍

内部 `info` 只返回卡片介绍、参数 schema、配置 schema 和输入输出口定义。
`joints` 通过输出口持续发布数据，不提供业务 action；joint_move 按需建立 Dashboard 命令连接。

## 固定直角结构

反馈型号 113 自动识别 CR3A。保留上游原始 `resource/cr3_robot.urdf`，`model` 返回
从该文件及同版本 STL 派生显示骨架：两根长杆分别沿杆身中心面走线，再接回关节原点，
每根杆有两个固定直角拐点，共四处。J2 杆身中心面为局部 Z=0.1288m，J3 为 Z=0.0123m，
依据官方 CAD 中段截面确定，来源记录见 `resource/README.md`。
转角通过固定坐标系表示，末端补偿旋转使原有坐标变换不变。附加节点全部为 `fixed`，不接受控制角度。
原有六个可动轴的名称、轴向、限位及所有物理 link 的变换保持不变。

同时将原始复合 RPY 分解为单轴固定旋转，兼容平台现有 XYZ Euler 渲染顺序，
并保持标准 URDF 的 Rz*Ry*Rx 变换。测试对零位、当前样例姿态和 30 组随机角度逐一验证
全部物理 link 的位置与朝向，避免只画出直角却改变机械臂姿态。

模型只用于结构显示，不是运动规划模型或完整 CAD 外壳。未知/不匹配型号不会套用 CR3A。
先确认反馈有效再打开 `joints`，以便平台首次加载模型时取得正确型号。

## 网络与配置

Jetson 有线口接控制柜 LAN2：Jetson `192.168.200.10/24`，控制柜默认 `192.168.200.1`，
无需网关，Wi-Fi 可继续用于 SSH/互联网。

`DOBOT_CONTROLLER_IP`、`DOBOT_FEEDBACK_PORT`、`DOBOT_MODEL_NAME` 覆盖初始配置。
`config.yaml` 默认 IP 空，不配置时不联系控制器；部署片段默认 IP 为 LAN2。
`CONFIG_PATH` 可覆盖配置文件。`joints` 的共享配置提供 IP 和端口，变更时先关闭
旧反馈连接、清空缓存，再连接新地址。IP 声明为敏感字段，不打包到共享 Solution。
支持平台保存配置及启动前自动下发的 `config` 动作；返回 `state="configured"`。
随后 `start` 无需重复传入 IP/端口；重复应用相同配置不会断开现有连接，非法配置保留原连接。

反馈端口 `30004/30005/30006` 默认周期为 8/200/1000ms；接收和过期超时至少覆盖三个周期。
默认 `30004/30005` 为 1 秒接收、2 秒过期，`30006` 均为 3 秒；支持配置更大值。
SDK 清理异常不会中断重连，诊断字段 `cleanup_error` 单独记录。

## 构建与检查

```bash
bash build.sh --mirror none dobot/cra
```

SDK 及构建工具固定版本，源码校验 SHA-256；构建机需访问 GitHub codeload。
两阶段构建仅在 builder 中安装构建工具，运行镜像仍包含 SDK、numpy、requests、PyYAML 等必要第三方依赖。
这些运行依赖会增加镜像体积；本次未测量实际 ARM64 镜像体积。
部署使用 `/deploy/service.yml`、host 网络、平台 DDS loopback 配置，domain 42，
MCP 端口 `15746`，容器名 `embodied-dobot-cra`；无需主机 SDK 或 privileged。

```bash
python3 -m venv .venv
.venv/bin/pip install -r dobot/cra/build-requirements.txt
.venv/bin/pip install --no-build-isolation -r dobot/cra/requirements.txt pytest
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_dobot_*.py -q
.venv/bin/python scripts/check_service_yml.py
```

## PR 验收

1. 可见卡片为 `joints`、`joint_move` 和 `teach`；`model` 保留为后台资源。
2. 骨架使用六轴实际角度，默认 30 Hz；JSON 的六个角度字段分别占一行。
3. systemcontrol 不注册，joint_move 不提供独立使能或上电动作；运动前未使能时自动使能。
4. 反馈失效时骨架为空、角度为 null；`info` 只返回卡片介绍。
5. 本地模拟验证不操作真机，模型与实际机械臂的显示效果由现场验收。

## 关节运动

joint_move 与 joints 共用底层反馈服务和连接配置，均可独立启动。
move_j/rel_joint_mov_j 先 ClearError，再检查使能，未使能时发送一次 EnableRobot 并等待反馈确认。
然后按 detect_collision 配置安全皮肤：默认 false 关闭；true 开启并将四个区域设为 low。
新运动再发送 ClearError 和 MovJ/RelJointMovJ。
resume 只检查已使能、模式 10 和保护状态，然后复用命令连接发送 Continue；
不清错、不自动使能、不设置安全皮肤，保持暂停前配置。未使能或保护未解除时拒绝继续。
失败不继续也不重试。
皮肤设置由 detect_collision 显式选择。模式 11 等保护状态不会提前阻止准备命令；
新运动准备完成后必须确认模式 5、已使能且无保护触发，才发送位移命令。

五个运动 action 均返回 action_id，通过 ACP 完成回调报告真实终态；命令应答不等于运动完成。
暂停/停止也要等待新反馈确认状态与实际速度，缺失反馈不宣称停稳。
部署增加 Agent Core CA 证书只读挂载与主机名映射，保持 TLS 校验。

rel_joint_mov_j 新增相对关节偏移运动，单位为度，0 表示该轴不变；其余设置复用 move_j。
准备前及下发前校验实际角度加偏移后的最终目标，完成监测使用最终目标角度。

绝对运动的空白关节保持下发前实际角度，相对运动的空白偏移按 0；至少填写一个关节。

| action | 输入与行为 |
|---|---|
| `move_j` | `joint1_deg` 至 `joint6_deg` 为绝对目标角度，留空保持下发前实际角度 |
| `rel_joint_mov_j` | `joint1_offset_deg` 至 `joint6_offset_deg` 为相对偏移，留空按 0 |
| `pause` | 暂停控制器队列或工程，不适用于点动 |
| `resume` | 仅需手动 confirm_move=true，检查后直接 Continue，保持暂停前安全皮肤配置 |
| `stop_motion` | 停止控制器队列或工程，包括点动 |

两种角度运动至少填写一个关节，明确填入绝对角度 0 表示回到零度。
每次操作要求手动输入 `confirm_move=true`。速度和加速度为 1–100%，默认 10%，固定 cp=0。
目标行程为 J1/J2/J4/J5/J6 ±360°、J3 ±155°；实际还受控制器自定义软限位约束。
安全皮肤开关与灵敏度设置均等待新反馈确认对应队列完成，允许本次设置产生的短暂模式 7，
失败或超时不继续发送运动命令。暂停、继续和停止作用于整个控制器队列，不限于本卡片。

完成通知使用 `AGENT_CORE_URL` 和 `AGENT_CORE_CA_CERT`，默认地址为 https://phanthy-motus:15678，
CA 路径为 /opt/phanthy-motus/data/certs/cert.pem。部署支持 DOBOT_AGENT_CORE_URL、
DOBOT_AGENT_CORE_HOSTNAME、DOBOT_AGENT_CORE_ADDRESS、DOBOT_CA_DIR、DOBOT_AGENT_CORE_CA_CERT 覆盖。
TLS 保持校验；停止在反馈或证书配置异常时仍可下发，但不能宣称已停稳或已成功通知平台。

暂停完成以模式 10 及连续两包新反馈确认停稳为依据，不强制要求 pause_cmd_flag=1；
实测该字段可以为 0。resume 同样以模式 10 判断暂停状态，不依赖该标志。

## 示教录制与回放

`teach` 与 `joint_move` 共用 Dashboard 连接、反馈服务和机械臂占用管理。
录制、运动以及暂停的回放都保留机械臂占用；另一张卡不能插入新运动。
任一卡片的暂停、继续、停止请求转交给当前占用者，并以实际调用卡片的名称发送 ACP 结果。
`info` 仅返回卡片说明，没有业务状态查询 action 或新增 sensor 输出。

| action | 行为 |
|---|---|
| `start_trajectory` | 指定 `name`（已存在则报错，不覆盖）；ClearError、按需 EnableRobot、**关闭安全皮肤**（拖拽必然触发它），**由驱动发送 StartDrag 进入拖拽**并开始录制；完成通知表示机械臂已进入拖拽。录制自行结束（按拖拽键、达到上限、异常）时另有一条通知，见下文 ACP |
| `stop_trajectory` | 结束录制；驱动发送 StopDrag 退出拖拽，以新反馈确认模式 5 停稳后保存到本地。若操作者已提前退出拖拽，录制已自动结束，本操作返回其结果 |
| `list_trajectories` | 返回名称、录制时间、完整性、结束原因、时长、点数（按录制时间由新到旧） |
| `delete_trajectory` | 按名称删除本地轨迹；禁止删除正在录制、使用或暂停中的轨迹 |
| `play` | 先核验当前位置：不在起点（target_tolerance_deg 内）则 MovJ 到起点并以新反馈确认到位，已在起点则跳过；`pause_at_start=true` 时到位后停住等待 resume；否则复用新运动准备（清错、使能、皮肤，**安全皮肤默认开启**），由驱动以 50 Hz ServoJ 逐点发送录制关节角，等待实际结束；`return_to_start=true` 时回放结束后再 MovJ 回起点并核验。结果含 `moved_to_start`、`returned_to_start`、`elapsed_s` |
| `pause` | 回放中：沿轨迹在 0.5 秒内线性减速到停，停在路径上并以模式 5 停稳确认；到起点/回起点运动中：控制器 Pause（模式 10） |
| `resume` | 回放：确认停稳、无保护、仍在暂停位置后沿轨迹 0.5 秒加速继续；到起点/回起点：Continue，到位确认后接着开始回放（回起点则结束）；停在起点（pause_at_start）：开始回放。均不清错、不 Enable（到起点后的回放准备除外）、不重新设置皮肤 |
| `stop_motion` | 立即结束 ServoJ 流并发送 Stop，之后不会再发出任何目标点；录制时作为中断处理，StopDrag 并确认停稳，保留标为不完整的采集数据 |

`play` 参数：

| 参数 | 取值 | 说明 |
|---|---|---|
| `speed` | `0.25x` `0.5x` `1x`（默认） `1.5x` `2x` | 回放速度档位，1x 为录制原速；不再使用百分比或任意倍率，也没有加速度倍率 |
| `approach_speed` | `slow`（默认） `normal` `fast` | 运动到起点/回起点的 MovJ 速度档位，对应速度和加速度均为 10 / 25 / 50 |
| `pause_at_start` | 布尔，默认 false | 到达并核验起点后停住，占用保持，以 cancelled、`reason=paused_at_start` 通知（与 pause 一致），resume 开始回放，stop_motion 放弃 |
| `return_to_start` | 布尔，默认 false | 回放结束后运动回起点并核验；回起点失败时 play 以 error 结束并说明“回放已完成但回起点失败” |
| `detect_collision` | 布尔，默认 **true** | 仅控制安全皮肤：true 开启并将四个区域设为 low，false 关闭；不影响本体碰撞等级 |

除列表查询外，业务 action 都需要手动填写 `confirm_move=true`；该字段不设默认值或枚举。

### 命名与存储

轨迹只用 `name` 区分（NFC 规整、去首尾空白，1–80 个可见字符），没有 trajectory_id。每条轨迹一个目录，
目录名为名称的 SHA-256 前 32 位，所以任意名称都是安全路径，`mkdir` 本身就是原子的唯一性检查：
start_trajectory 遇到同名轨迹（含不完整的）直接报错，不扫描目录、不影响实时性。要重录请先 delete_trajectory。
没有采到任何点的失败录制会自动清除，不占用名称。清单（manifest）含 `recorded_at`（带时区的 ISO 录制时间）、
`created_at_ms`、时长、点数、状态和结束原因。早期以随机 id 命名的目录无法按名称寻址，不再列出也不能回放。

### 录制：驱动进入拖拽，点击 stop 退出

流程：start_trajectory（StartDrag）→ 操作者拖动机械臂 → stop_trajectory（StopDrag）→ 确认停稳并保存。

- **进入**：校验后先订阅反馈，再发送 StartDrag（要求机械臂静止、已使能、模式 5、无报警），然后等待首个拖拽包
  （模式 0 或 6）。`teach.drag_start_timeout_s`（默认 5 秒）内未进入拖拽，则中止、清除该录制并报错。
  StartDrag 被拒绝不再发其他命令；结果不明（超时）会按下面的退出规则确认并退出拖拽。
- **操作者意外结束拖拽**（按机械臂拖拽键）：模式 5 持续 ≥ 24 ms 即视为已退出拖拽，录制自动停止，
  状态为 `recorded`，`end_reason=drag_exited` 并带 `note`（“the arm left drag; recording stopped automatically and is usable”），
  可直接回放；驱动不再发送 StopDrag，确认停稳后释放机械臂。此时没有待完成的 action，驱动会主动发一条 ACP 完成通知
  （见下文 ACP）。
- **StopDrag 守卫**：StopDrag 只在请求之后收到的新反馈显示 `drag_status` 时发送，且只发一次。反馈过期/断线时不发送，
  待反馈恢复后再判断；模式 6 但 drag_status=0（控制器拖拽状态不一致）也不发送。现场曾在按钮拖拽中途收到 StopDrag
  后卡在带阻尼的模式 6，这些守卫用于避免再次触发。
- **上限**：录制时长从首个拖拽包起算，最多 `max_recording_s`（默认 500 秒）和 10000 点；达到上限后停止采集，
  发送 StopDrag 并按 stop_trajectory 的流程确认和保存，`end_reason=limit_reached`。
- **录制中允许的状态**：模式 6（进入时可短暂为模式 0）；模式 5 仅在刚退出拖拽的判定窗口内允许。
  模式 7/8/9/10/11 等其他模式、保护状态（报警、碰撞、皮肤、下使能、抱闸松开、型号）以不完整结束。
- **现场实测过渡态**（CR3A，30004）：进入拖拽时控制器先报约 24 ms 的模式 0（已使能、drag_status=1），再进入模式 6；
  退出拖拽后第一包模式 5 可能仍报 running_status=1，仅在最近一次拖拽后 100 ms 内且无点动/队列/暂停标志时容忍。
- **stop_trajectory 确认**：先停止采集，再按 StopDrag 守卫退出拖拽，只使用请求之后收到的新反馈，
  要求连续两包满足：模式 5、已使能、drag_status=false、无运行/点动/队列/暂停、六轴 |qd| ≤ 0.1°/s、无保护状态。
  等待上限为 `teach.drag_exit_timeout_s`（默认 10 秒，驱动停止时缩短为 enable_timeout）。超时、断线、StopDrag 被拒绝
  或保护状态时返回失败，数据标为不完整，机械臂继续被占用，可再次 stop_motion/stop_trajectory 重试；
  仍在拖拽时错误信息提示按机械臂拖拽键退出。
- **异常**：报警、碰撞、皮肤、下使能、抱闸松开、非法模式、运行/点动、断线/过期/溢出/时间戳缺口、坐标系变化、
  stop_motion、驱动停止/重启，均标为不完整并保留原始数据、轨迹点和明确原因；不自动清错、使能或 Continue。
  中断标记不可撤销，后续的 stop_trajectory 不会清除；退出确认期间收到的 stop_motion 或驱动停止，会在最终保存前同步核对。
- **占用**：start_trajectory 完成后到录制结束，teach 一直占用机械臂，joint_move 不能插入运动。退出无法确认时继续占用，
  直到后续 stop_motion/stop_trajectory 确认停稳；成功后才释放。旧任务不会释放新任务的占用。
- **ACP**（平台的 Action Completion Protocol：派发时立即返回 `action_id`，结束时驱动向 Agent Core 的
  `/api/acp/complete` 回调，Agent Core 据此唤醒 agent）：
  - 所有 action 都是“立即返回 + 结束回调”：start_trajectory 在机械臂进入拖拽、开始录制时回调；
    stop_trajectory 在退出确认和保存后回调；play/pause/resume/stop_motion 在真实终态后回调，
    status 为 completed / error / cancelled，`result.reason` 给出原因。
  - start_trajectory 若一直挂起到录制结束，会让 stop_trajectory 被同资源（arm）的屏障挡住，所以它在进入拖拽时即回调；
    录制**自行结束**（操作者按拖拽键、达到上限、异常中断）时没有待完成的 action，驱动改为主动发一条回调：
    `action_id=dobot_cra_teach_recording_<end_reason>_<随机串>`；成功（`recorded`，含 drag_exited、limit_reached）
    为 completed，不完整为 error，result 含 `name`、`state`、`end_reason`、`reason`、`note`、`duration_s`、`point_count`。
    平台对成功结果只保留耗时类字段，所以结束原因同时写在 action_id 里。
  - 由 stop_trajectory / stop_motion 结束的录制只由该 action 自己回调，不会重复发送。
  - 只有采样完整、至少 2 点、退出已确认且保存成功才为 `recorded`。
    `end_reason` 为 `stop_trajectory`、`drag_exited`、`limit_reached` 或 `interrupted`。
- 机械臂已处于拖拽、运行或暂停时 start_trajectory 被拒绝，且不发送任何命令。

采集从共享 30004 接收线程进入独立有界缓冲，不经过网页、ROS 发布或 joints 的 30 Hz 定时器。
原始数据逐包落盘（含模式）；按控制器时间戳线性重采样为 50 ms 一点，最多 10000 点、约 500 秒。
断线、缓冲溢出、超过 50 ms 的采样缺口、时间倒退、保护变化和用户/工具坐标系变化
均将录制标为不完整，不自动拼接。重启遗留的录制也标为不完整。坐标范围按 CR3A 限制校验。
小于 50 ms 的采样间隙才参与重采样；原始文件保留实际时间，不把重复旧包当新数据。

录制开始时关闭安全皮肤（拖拽会触发皮肤检测，EnableSafeSkin(0)），不改本体碰撞等级。
play 的 detect_collision 默认 true：开启安全皮肤并将四个区域设为 low；设为 false 则关闭。

### ServoJ 流式回放

不向控制器上传任何文件：录制只保存在本地（`data_dir`，50 ms 一点、单位为度的六轴关节角），
play 时由驱动通过 Dashboard 连接逐点发送 `ServoJ(J1..J6,t=0.020,aheadtime=50.0,gain=500.0)`。

- **时间比例**：每 20 ms 发送一个目标，按录制点线性插值；`speed` 为档位 0.25x–2x，默认 1x，
  即路径位置每 20 ms 前进 20×倍率 ms。不受全局速度比例控制。
- **每个目标发送前检查**：反馈新鲜；无报警、碰撞、皮肤触发、下使能、抱闸松开；未拖拽；模式为 5 或 7；
  实际关节角与最近 0.5 秒内已发送的某个目标相差不超过 3°（跟随偏离即中断）。发送比计划晚 100 ms 以上
  （控制器已断流）也中断。任一失败都发送一次 Stop 并以 error 结束。
- **完成**：最后一个目标发送后，以连续新反馈的模式 5、停稳及末点误差不超过 target_tolerance_deg 确认，最长 10 秒。
- **暂停**：沿路径把播放速率在 0.5 秒内线性降到 0（不离开录制轨迹），停发后以连续两包模式 5 停稳确认；
  play 以 cancelled、`reason=paused` 和 `position_s` 结束，pause 完成后 teach 保持占用。
- **继续**：要求停稳的模式 5、无保护、实际角度仍在暂停位置（target_tolerance_deg 内），否则拒绝且不发送任何命令，
  暂停上下文保留，可再次 resume 或 stop_motion。被手动挪动后须 stop_motion 再重新 play。
  继续时沿路径 0.5 秒加速，不清错、不 Enable、不设置皮肤（沿用暂停前的配置）。
- **停止**：stop_motion（任一卡片）、驱动停止都会取消流并发送 Stop；取消检查与 ServoJ 发送在同一命令锁内，
  Stop 之后不会再有目标发出。暂停减速过程中也可以直接停止。
- **到起点/回起点**：play 的到起点阶段和 return_to_start 的回起点阶段都是控制器队列中的 MovJ，暂停/继续使用控制器 Pause/Continue（模式 10）；
  play 在到起点阶段被暂停后，resume 先 Continue 并确认到位，随后接着做回放准备和 ServoJ 流（pause_at_start 则停在起点）。
  到位核验以新反馈为准，未到位不开始流。回起点被暂停后 resume 到位即结束。
- 通信中断不重发 ServoJ/MovJ/Continue；错误后尝试一次 Stop，无法确认停止则保留错误状态和占用。

现场实测（CR3A，经 Jetson 有线连接）：50 Hz ServoJ 发送间隔 19.97–20.03 ms，ACK 往返 p50 5.5 ms、最大 13.5 ms；
J6 0→+3°→0 的跟随滞后约 92 ms，去滞后后 RMS 误差 0.027°，过程中模式 5→7→5，无报警。

配置项（config.yaml 的 teach 或对应环境变量）：

| 配置/变量 | 作用 |
|---|---|
| data_dir / DOBOT_TEACH_DATA_DIR | 本地持久化录制目录；部署默认 /work/data/dobot-teach |
| drag_start_timeout_s | StartDrag 后等待机械臂进入拖拽的上限，默认 5 秒，最大 60 |
| drag_exit_timeout_s | StopDrag/操作者退出后等待模式 5 停稳的上限，默认 10 秒（原 60），最大 600 |
| max_recording_s / max_gap_ms | 录制时长上限（≤500）/ 允许的反馈间隔（≤50） |

早期版本录制目录中遗留的 `native.csv` 不再使用，delete_trajectory 时一并删除。不自动记录/回放 DO，也不改变工具负载设置。

本次新增代码仅使用标准库，requirements.txt 与 SDK pin 不变。
本地测试覆盖真实回环 TCP 的官方 SDK、1440 字节反馈、MCP HTTP、ACP、录制与 ServoJ 回放/暂停/继续/停止模拟；
模拟器不代表控制器固件行为。
