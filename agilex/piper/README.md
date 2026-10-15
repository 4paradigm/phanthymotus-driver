# PiPER Arm control and Joints

Place **Arm control** (`arm_control`) and **Joints** (`joints`) on the canvas.
Joints publishes measured PiPER skeletons independently of the control card.
The live camera remains a background stream. The current bundle does not declare
or publish a Parameters / `state/parameters` stream. Get status retains complete
feedback, limits, motor state, faults, capabilities and actual task outcomes.

| Action | Input / meaning |
| --- | --- |
| Move relative | `translation_mm=[dx,dy,dz]`; initial SDK end axes, pure translation while TCP is uncalibrated |
| Move to pose | Calibrated-only `pose_mm_deg=[x,y,z,roll,pitch,yaw]`, absolute base-frame tip target |
| Set joints | `joints_deg=[J1,J2,J3,J4,J5,J6]`, absolute or relative degrees |
| Reset | Fixed official zero-angle target `[0,0,0,0,0,0]` degrees; joint-space movement, gripper unchanged; physical path acceptance pending |
| Set gripper | `set_gripper_opening`, `width_mm`, effective SDK/deployment bounds, normally 0–70 mm; default 20 mm adjusted to bounds |
| Stop | `stop_motion`: cancel preparation/movement, best-effort measured hold, retain motor torque |
| Get status | Passive current feedback and per-action result/delivery records |

Set position, Open gripper and Close gripper are removed from the public action
list and rejected on direct old requests. The old `return-preset.json` file and
`preset.py` helpers are retained; current Reset neither reads nor overwrites the
file, and ignores legacy `intent_control.preset_joints_deg`.

Arrays accept native JSON arrays and strict JSON-array text. Absolute joints and
calibrated absolute Pose permit `null` to retain each fresh measured component;
at least one numeric target is required. Relative arrays require numbers and use
0 for no increment. Entirely zero relative requests return a successful no-op
without preparation or teaching takeover. Choose an action also does nothing.
Inactive declared fields retained by Core are ignored; invalid active values and
unknown fields fail. Defaults: Move `[1,0,0]` mm, speed 1%, range 1–10%, Move at
most 10 mm, calibrated relative rotation at most 5 degrees per axis.

TCP rotation/absolute Pose stay hidden and blocked until the installed tool's
`tcp_offset_mm_deg` is physically verified and `tcp_verified` is true. Uncalibrated
Move keeps the exact measured orientation and reports the SDK reference rather
than calibrated absolute tip accuracy. Rotation uses `Rz(yaw) Ry(pitch) Rx(roll)`.
Move remains linear. Point-to-point stays closed unless its existing
identity/firmware/source/configuration-bound acceptance is valid; see
[PATH_VALIDATION.md](PATH_VALIDATION.md).

Every nonzero control action, including Reset and Set gripper, uses the existing
guarded teaching exit/hold before resolving its target. Active recording,
unsupported firmware, faults, stale feedback and partial enabling reject the
action. A ready CAN arm is not prepared twice. Normal all-disabled states use
measured-pose preparation. One task covers preparation and execution; requests
are mutually exclusive and never queue. Start, Info, Joints and monitoring do not
perform takeover. The deployment gate `PIPER_MOTION_ENABLED` defaults to 0.

Reset uses the target in the pinned official SDK's `demo/V2/piper_ctrl_go_zero.py`.
It does not call ResetPiper, calibrate zeros, clear faults, change the gripper or
release motor torque. Default speed remains 1%, unlike the SDK demonstration.
An official zero-angle posture is not assumed to be vertical, reachable from
any state, or clear of obstacles. Validate the physical posture and return path
on arm 1 before claiming hardware acceptance.

Skeletons are published at 10 Hz in radians with official joint names. Joints and
Arm control share one receiver/cache. Stale feedback freezes the skeleton. Stop
on the control card leaves Joints and the background camera running; whole-driver
shutdown releases their resources. `model` is a plain `{"urdf":"..."}` resource.
The installed Core currently animates revolute joints only, so finger translation
is not animated. Actual gripper width is available through Get status. This
remediation does not modify Core or fake a finger angle. In installed official
Core release.261015.0897a8d, Joints' View data stream entry still uses the wrong
MCP ID `detail` and shows a fallback human. Use the top Monitor for the correct
PiPER model. This known Core issue is reproduced and recorded, not claimed fixed.

Each running action returns an `action_id` and eventually produces completed,
error or cancelled ACP status. Callback delivery checks `ok` and matching ID,
retries up to four attempts (2-second HTTP timeout, .25/.5/1-second backoff),
and keeps 64 per-action records. Retries never repeat hardware commands. Fast
terminal requests return their actual result without registering a pending ID.
Get status separates robot outcome from callback delivery failure. A delayed
callback cannot overwrite a newer result. See [UNIFIED_CONTROL.md](UNIFIED_CONTROL.md)
for Core registration/delivery limitations and examples.

## Validation and delivery

This review remediation is local and uncommitted. Simulated controls do not
constitute physical acceptance. ARM64 candidates are tested with no hardware
mapping, network disabled and a read-only root; existing services are unchanged.
Deployment and PR submission require the user's next explicit instruction.
Historical release `release.261015.0ae0a28` does not contain this remediation.

## Production passive diagnosis

`python3 agilex/piper/deploy/diagnose.py --output readonly.json` checks the five
runtime tools (`arm_control`, `joints`, `model`, `piper_photo`, `piper_camera_live`),
reads Info before/after, validates fresh feedback/model and queries camera Info.
It does not start, stop, reconnect, prepare or move the robot. A deployment gate
already at 1 is recorded without changing it. PASS is not motion acceptance.
`--photos 1` explicitly requests one capture; the default captures none.

## Historical implementations (superseded public contracts)

The following sections document retained regressions and older deployments.
Their Set position/preset-return and parameter-stream behavior is not the current
public contract; use the interface above and UNIFIED_CONTROL.md.

## 历史十卡实现（保留作回归，默认不注册）

同一条 CAN 连接、同一个操作锁，对外提供十张卡片。启动卡片、重连和服务重启不使能、不回放动作。
Ubuntu 的机械臂与夹爪均由操作人员通过卡片控制；运动许可默认关闭。

| 卡片 | 用途 | 主要提醒 |
| --- | --- | --- |
| 机器人状态 `piper_status` | 状态、固件查询、重连 | 模式、使能、故障分别反馈；重连关闭许可并清除预览 |
| 切换待机 `piper_standby` | 仅切换 STANDBY | 当前软件要求六轴全部失能；不自动失能或回零 |
| 切换电脑控制 `piper_can` | 仅切换 CAN | 确认其他控制程序已停止；切换后使能仍单独操作 |
| 电机使能 `piper_enable` | 选择六轴或夹爪使能 | 已经使能则返回已就绪；需要边界修正时明确展示并单独确认 |
| 电机失能 `piper_disable` | 选择六轴或夹爪失能 | 确认机械臂可靠支撑，或夹爪空载/夹持物已支撑；释放对应保持力 |
| 单轴运动 `piper_joint` | 一个关节的绝对目标角度 | 当前角度→目标角度→变化量，先预览再确认 |
| 夹爪开合 `piper_gripper` | 读取、张开、闭合、指定开度 | 先用独立卡片使能；open 到上限、close 到下限，小开度用 set_width |
| 相机与照片 `piper_photo` | 当前 RGB 拍照、文件导入 | 每次拍照明确确认；记录来源，不移动机械臂 |
| 操作许可 `piper_permission` | 本次允许/关闭控制 | 不使能；关闭时请求取消控制并尽力保持 |
| 停止动作 `piper_stop` | 机械臂/夹爪/全部动作取消 | 不排队，不失能；不是硬件急停，以实际反馈确认 |

## 日常顺序

读取状态 → 开启本次操作许可 → 切换 CAN → 使能六轴 → 按需使能夹爪 → 单轴/夹爪/拍照。
已经处于 CAN 或已经使能时，重复操作返回当前已就绪，不重复发控制指令。
收尾：停止动作 → 可靠支撑后分别失能 → 切换待机 → 关闭操作许可。
全局智能控制保持关闭即可手动操作十卡；界面自动将当前卡片置为就绪，不启动后台决策。
ACP 完成通知在智能控制开启时可能触发后台决策。

状态卡片提供 `summary`、实际角度/开度、模式/使能/故障、新鲜度、忙状态、完成结果和下一步。
`running` 仅表示已提交。实际完成/错误/取消通过 ACP 回报，同一卡片 `info.last_action`
或状态卡片 `completed_actions` 保留结果。

### 中文画布与确认流程

卡片和字段用 `title` 显示中文，`oneOf.const/title` 保留协议值同时提供中文选项。
只显示当前动作需要的字段；选择夹爪使能时不显示机械臂调整速度。
画布需搭配 Agent Core 的 card-controls 支持：点击执行先请求只读预览，展示目标/变化量，
操作人员确认后携带当前预览编号提交执行。预览编号、execute、接管字段不需手填。
取消确认不发送执行指令。拍照先询问本次授权，再调用一次 capture。
原始 MCP 调用仍需明确传入各确认字段，UI 隐藏参数不构成调用者身份鉴权。

### 直接 MCP 调用

模式卡片 `switch(execute=false)` 预览，再用相同条件和 preview_id、`execute=true` 执行。
CAN 预览/执行都需要 `other_controllers_stopped=true`。已经达到目标模式时只返回反馈。

使能卡片 `enable(object=arm|gripper, execute=false)` 预览。实际使能要求本次许可、CAN
和现场空间确认；六轴的边界修正还需 `allow_limit_adjustment=true`，夹爪使能需
`take_control=true`。夹爪使能保持当前开度，不自动张开或闭合；当前软件要求六轴已使能。
六轴部分使能/故障/示教中拒绝使能，并给出检查提示。

单轴卡片 `move(joint=1..6, target_deg=绝对角度, speed_deg_s=1, speed_pct=5, execute=false)`
预览；用相同参数、preview_id、`execute=true`、`workspace_clear=true`、`take_control=true` 执行。
关节编号必须是整数，不接受字符串。夹爪 open/close/set_width 采用相同预览/确认方式。

失能卡片 `disable(object=arm|gripper, supported=true)`，需要本次操作许可；已经失能时
只返回反馈。停止卡片 `halt(target=arm|gripper|all)` 不要求开启许可，不等待动作锁；
返回 operation_stopping=true 时，工作线程仍在执行取消/保持收尾，不能当作已经完成。
停止范围不匹配当前动作时，不取消其他卡片。停止卡片不发送急停、回零、复位或失能指令。

## 厂家协议与驱动策略

STANDBY/CAN 编码及电机使能/失能指令来自厂家协议。全失能再切模式、运动许可、
一次性预览、现场确认和速度/步长上限属于本驱动策略，提示中与厂家状态明确区分。
模式与使能各自独立；切模式不代替使能。使能不作为厂家零位校准。

待机↔CAN 要求六轴全部失能、反馈正常、示教/轨迹停止。切换后必须收到新鲜的目标模式
反馈才能报告 confirmed=true，超时不自动重试。
六轴使能前预置测得的当前姿态，防止继承旧目标。最多 3° 的 SDK 边界修正会在预览和
确认中逐轴列出，未明确允许则不执行。大于 3° 拒绝，要求现场检查实物姿态。
软件示教白名单默认空，当前十卡只显示实际示教状态、固件和机身按键说明，不发送未经
验收的录制/退出指令。旧固件的退出行为可能涉及回零/重启，应另行安排现场操作。

单轴速度 0.5–5°/秒，SDK 上限 1–10%，单次最多 12°；所有目标遵守 SDK 与部署限位交集。
三次平滑插值，20 Hz 发目标；到位要求 ±0.5° 内稳定 0.5 秒。
夹爪开度为 SDK 与部署范围交集（本部署 0–70 mm），力矩 0.001–1 N·m，默认 0.5。
预览仅一次有效，60 秒过期；角度偏移 >0.5°、夹爪开度偏移 >0.5 mm、状态变化、停止、
重连或新预览使旧编号失效。反馈超过 0.5 秒、故障或外部模式变化会拒绝/取消运动。
不同物理卡片共用互斥锁，忙时拒绝而不排队；状态读取及停止请求不等待该锁。

## 照片

`PIPER_CAMERA_ENABLED` 默认 0，部署使用 `${PIPER_CAMERA_ENABLED:-0}`；取得相机授权后设置 1。
`capture(capture_confirmed=true)` 拍当前 RGB 一张，保存 JPEG/JSON 并发布 DDS，不调整姿态。
不启动连续视频采集；相机权限与运动许可独立。
文件导入只接受宿主机 `/opt/phanthy-motus/data/piper/imports/` 下 JPEG/PNG 普通文件名，
拒绝路径、符号链接、损坏或超过 20 MiB/2000 万像素文件。记录 source=user_provided、
camera_accessed=false，不冒充实时拍照。

## Ubuntu 构建部署

Ubuntu 22.04 amd64、ROS2 Humble、can0 1 Mbps；host network、DDS domain 42、
FastDDS loopback profile 只读挂载沿用现场设置。

```bash
bash agilex/piper/build-local.sh agilex-piper:local-amd64
bash agilex/piper/deploy/check-image.sh agilex-piper:local-amd64
python3 agilex/piper/deploy/diagnose.py --output ubuntu-readonly.json
```

Bot 默认可能为 arm64，必须 inspect 后再部署到 amd64 控制器。固定实际镜像 ID、提交和
依赖版本，不将离线叠加构建冒充干净上游构建。基础镜像默认固定官方多架构索引
`sha256:860e53793ccf575e5369b5e9a19939de2bfcfb46a299954d06f54f2af54fed19`。
项目 ros-base:latest 核对时仅 arm64，因此现场 amd64 使用官方多架构 Humble 基础镜像。

| 依赖 | 用途 |
| --- | --- |
| PiPER SDK、python-can | 厂商协议和 CAN 通信 |
| NumPy、headless OpenCV | 图像数组、V4L2 单帧、JPEG；不安装 GUI OpenCV |
| PyYAML | 部署配置 |
| python3-pip | 构建时安装固定 Python 依赖 |
| sensor_msgs、FastDDS | 图像 DDS 发布与既定隔离配置 |

SDK 固定提交 8d4dc29bd6e753066dedee235c008e90ecb2ab4a；ZIP SHA256
`89dfa296331abc04b8d7ca5ebbc4dbe21bed44e521e85e8c4716a81e41305c29`，pip 安装前校验、禁用缓存。
Dockerfile 提供 PIPER_SDK_URL / PIPER_SDK_SHA256 构建参数，临时文件在同一 RUN 清理。
COS 归档发布目前缺少已验证对象链接及授权上传路径；具体依赖和迁移步骤见 [SDK_ARTIFACT.md](SDK_ARTIFACT.md)。
本轮拆分不增加第三方依赖，仅增加组件源码及少量无依赖前端代码。具体构建尺寸口径见 ACCEPTANCE。

## Mac

```bash
uv venv .venv-piper --python 3.12
uv pip install --python .venv-piper/bin/python -r agilex/piper/requirements-mac.txt
PIPER_MOTION_ENABLED=0 .venv-piper/bin/python -m agilex.piper.mac_main
```

打开 http://127.0.0.1:15741/。Mac 页面同样自动预览/确认，显示中文并跟踪实际完成结果。
Mac USB-CAN 还有主机门禁：启动时 PIPER_MOTION_ENABLED=0 只允许信息查询；现场确认后
由主机人员用 1 启动控制。人工许可卡片目前只支持 Ubuntu SocketCAN，不绕过 Mac 主机门禁。
只连一台 candleLight/gs_usb 适配器，停止其他控制程序。Mac 相机仅选 RealSense，不自动
选择内置摄像头；不启动 ROS，因此 dds_published=false。gs_usb/libusb/PyObjC 不进入 Ubuntu 镜像。

## 旧代码和验收

旧五卡片版本 e2c82e2 已保存到备份分支 codex/piper-five-cards-backup-20261012，并有本机
完整工作文件备份与 Git bundle（包括未提交的 smoke-test.py）。共享 runtime 保留既有
预览、控制、照片、异步与生命周期逻辑供回归验证；生产 device 使用本文顶部的用户意图合同，历史十卡只用于回归。
旧 piper_arm / piper_mode 不作为兼容入口暴露，旧画布需更新。此前已删除的五个动作不恢复。

```bash
python -m pytest -q tests/test_piper_prepare.py tests/test_piper_cards.py tests/test_piper_controls.py tests/test_piper_mac.py tests/test_piper_deployment.py tests/test_piper_ten_cards.py tests/test_common_lifecycle.py tests/test_dockerfile_copies.py tests/test_vendor_runtime_logging.py
```

共享 HTTP 日志的控制字符转义/200 字符上限有独立回归；Mac 入口先安装 logsafe。
当前软件、现场部署和历史真机记录见 [ACCEPTANCE.md](ACCEPTANCE.md)。历史五卡及已删除动作
的验收不代表十卡已完成全部真机动作验收。


### Capability-aware single card (2026-10-15)

`move` is displayed as **Move relative** and retains a linear path. `pose` is
**Move to pose**, a calibrated absolute base-frame tip target. Absolute
`pose_mm_deg` and absolute `joints_deg` accept `null` components, filled once
from fresh measured feedback at task start. At least one component must be a
number. Relative arrays accept numbers only; use zero for no increment.
A fully zero relative request returns `executed=false`, `No movement requested.`
without connecting, preparing or enabling the robot.

Without a verified TCP, the public schema omits Pose and the Move rotation
field. Without a configured six-joint preset it omits Return to preset.
Get status and the background parameters report `capabilities` and reasons.
Refresh the device/tool definitions after restarting with changed configuration;
old requests remain checked by the backend.

Pose has `motion_mode=linear` by default. The candidate `point_to_point` mode
uses the official pinned V2 MOVE P command (0x00 + EndPoseCtrl), never MOVE J
with an end-pose target. It is absent until an operator-reviewed acceptance
record matches the USB-CAN serial, observed firmware, Python driver-source
SHA-256, and motion configuration SHA-256. That serial identifies the adapter,
not the physical arm: replacing the arm/tool requires clearing the record and
repeating acceptance. Firmware not yet observed on this connection keeps the
candidate closed. No acceptance record is shipped or created automatically.
Errors never retry or change path. Stop/error recovery uses the existing linear
measured hold, never the candidate path. This is not obstacle avoidance.
See `PATH_VALIDATION.md` for the acceptance requirements and current blockers.


## Automatic teaching exit before control, 2026-10-15

Every nonzero Arm control request (Move relative, Move to pose, Set joints,
return preset and gripper actions) first checks the actual mode. In stationary
teaching it performs the existing guarded SDK exit-and-measured-joint-hold,
confirms CAN mode and stable feedback, and only then resolves the requested
movement/gripper target from post-exit feedback. Failure ends the same task;
no user target follows a failed takeover. No separate exit button is required.
This does not clear faults, reset, home, release torque, interrupt active
recording/playback, or bypass firmware/freshness/permission checks.

Core's lifecycle Start precedes Info and Stop as well as control calls, so it
remains passive. Status, monitoring, selection, zero relative requests and Stop
do not take over teaching. Actual control owns preparation, exit and execution
under one cancellable reservation. A repeated action already in CAN skips exit.


## Set position and Reset, 2026-10-15

`set_position` (**Set position**) first performs the same automatic teaching
exit/hold as control actions, then captures the stationary six measured joint
angles for 0.5 seconds (maximum drift 0.2 degree). It atomically replaces
`data_dir/return-preset.json`, bound to this USB-CAN serial/interface. No return
trajectory or gripper command is sent. Failed exit, stale/moving feedback,
cancellation, missing adapter identity or write failure retains the old preset.
The operator must re-save after replacing the physical arm on the same adapter.

`reset` is displayed as **Reset** when a valid saved/configured target exists.
It exits teaching/holds first, then returns all six joints via SDK MOVE_J at
1% by default (range 1–10%). The gripper remains unchanged. This is a planned
movement, not ResetPiper, fault clearing, zero calibration or motor disable.
The return path is a joint-space trajectory, not a straight TCP segment.
Effective limits, fresh feedback, cancellation, task serialization and stable
arrival checks still apply. The saved target survives driver/container restarts.
Invalid/mismatched stored data hides Reset and never falls back to an unrelated
configuration preset. Get status/background parameters show the saved position.
Refresh device/tool metadata after first save to make Reset visible in Core.
