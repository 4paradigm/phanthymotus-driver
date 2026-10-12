# PiPER 单臂十卡片与双臂遥操

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
两份桌面 landing 文档没有 COS 上传说明/SDK 地址，按用户要求暂不迁移 COS。
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
预览、控制、照片、异步与生命周期逻辑供回归验证；生产 device 只注册十张新卡片。
旧 piper_arm / piper_mode 不作为兼容入口暴露，旧画布需更新。此前已删除的五个动作不恢复。

```bash
python -m pytest -q tests/test_piper_prepare.py tests/test_piper_cards.py tests/test_piper_controls.py tests/test_piper_mac.py tests/test_piper_deployment.py tests/test_piper_ten_cards.py tests/test_common_lifecycle.py tests/test_dockerfile_copies.py tests/test_vendor_runtime_logging.py
```

共享 HTTP 日志的控制字符转义/200 字符上限有独立回归；Mac 入口先安装 logsafe。
当前软件、现场部署和历史真机记录见 [ACCEPTANCE.md](ACCEPTANCE.md)。历史五卡及已删除动作
的验收不代表十卡已完成全部真机动作验收。

## 双臂 VR 遥操与轨迹回放

默认配置仍使用原有单臂十卡片。配置 `arms.left` 和 `arms.right` 后，同一 Driver
管理两路独立 SocketCAN，并增加 `piper_teleop` 和 `piper_motion` 两张卡片。
左右臂各有一个 SDK 连接，十卡片、遥操和回放复用这些连接。双臂动作占用两个原有操作锁，
任一臂忙时立即拒绝，不排队。双臂模式启动时关闭两臂软件许可，即使环境变量为 1。

```text
PICO App → PICO teleop_device → /teleop/command → piper_teleop → IK 工作进程
                                                               ↓
轨迹回放 ─────────────────────────────────────────────→ 双臂限速执行 → 两路 CAN
                                                               ↓
                                               执行目标与反馈 → JSONL 录制
```

### 部署配置与机型标定

1. 核对两台机械臂、适配器与左右标签。两台 PiPER 必须使用两个独立 CAN 接口；不能把
   使用相同 CAN ID 的两臂挂到同一总线。`can0/can1` 只是示例，USB 重插后须重新核对映射。
2. 将 [双臂配置示例](config.dual.example.yaml) 放入现有 PiPER 数据目录，填写已核对的
   CAN 绑定。部署设置 `PIPER_CONFIG_PATH=/opt/phanthy-motus/data/piper/config.dual.yaml`。
   原单臂配置无须迁移；双臂模式下原机械臂动作必须显式选 `arm_side=left|right`。
3. 按 [标定模板](motion/calibration.example.json) 测量两臂安装坐标、TCP、手柄到 TCP
   的局部变换、跟踪坐标到各臂基座的旋转及允许的 TCP 工作区。位置单位为米，四元数
   顺序为 xyzw。`tracking_to_base` 的平移不参与相对映射；`base_to_world` 记录安装关系。
4. 模板故意使用 `hardware_verified=false` 和无效工作区。核对官方模型与实物、关节
   顺序和方向后才填写真实数据并确认。只把布尔值改成 true 不能完成标定。
   模型及来源校验见 [motion/NOTICE.md](motion/NOTICE.md)。标定或左右绑定改变后，旧录制拒绝回放。
5. 保留 PiPER MCP 15741、PICO MCP 15742。PICO WSS 默认也为 15741，同机必须改为
   15743，参考 [PICO 同机配置](deploy/pico-on-piper.example.yaml)。将该配置放入 PICO
   已有持久化目录，并设置其 `CONFIG_PATH=/var/lib/motus/pico/config.yaml`；保留配对状态。
   核对 mDNS、安装网址和配对邀请都使用新端口，不修改 App 或构建另一套输入协议。

两服务使用相同宿主机启动时钟、DDS domain 42 和既有 loopback DDS 配置。
当前固定 `/teleop/command` 只支持一组设备输入和一组双臂消费者；同域不要同时运行另一组
G1/双臂消费者。PiPER 沿用 SDK 直连 CAN，不启动 `piper_ros` 控制程序。
双臂功能不修改 Core/ActuCore；中文预览确认沿用 PiPER 十卡片配套 Core 能力。

### 操作与停止

- 用原有卡片分别开启两臂本次许可、切到 CAN、使能六轴和夹爪。新功能不代做这些步骤。
- 连接设备卡输出与 `piper_teleop` 输入。卡片 `start` 只准备 DDS 接收；`begin` 先预览，
  再由操作者确认。原始 MCP 执行须携带本次 `preview_id`、`execute=true`、
  `workspace_clear=true`、`take_control=true`。每臂预览复用原有一次性机制，60 秒失效。
- 松开双握把建立初始相对映射，再同时按下双握把跟随。任一握把松开时保持最后成功
  下发的参考；重握仍沿用本次初始映射。Trigger 相对本次初始值控制对应夹爪开度，首次不突跳。
- 关节参考最大变化率 1°/秒，单步时间额度最多 50 ms；SDK 速度 5%。夹爪参考最大
  变化率 10 mm/秒，力矩 0.5 N·m。限位取模型、SDK 与部署配置的交集，不代表实测电机速度保证。
- 输入最大年龄 300 ms，硬件反馈最大年龄 500 ms。断帧、跟踪丢失、连接/空间代次变化、
  IK 超时、CAN 错误、许可关闭或状态异常会结束本次会话。恢复后必须重新预览并启用，不自动恢复运动。
- `finish`、`piper_stop.halt`、关闭任一臂许可或停止相关卡片会取消整组双臂输出。
  保持只使用新鲜且有效的反馈，不失能、不回零。查看左右 `holds` 的发送结果及错误，
  软件发送成功不代表已物理停止。两路 CAN 不是原子同步；左侧下发后右侧可能失败。
- 实际反馈相对最后下发参考偏差超过 5°或夹爪 5 mm 时结束会话。新功能只检查 TCP 工作区，
  未做完整连杆、臂间或环境碰撞检测。必须先验证双臂安装和无碰撞活动空间。

状态卡 `read(arm_side=all)` 分别返回左右状态；一臂反馈失败不会隐藏另一臂结果。
机械臂动作选择具体左右臂。`piper_stop.halt(arm_side=all,target=all)` 停止整组，
许可 `block(arm_side=all)` 同时关闭两臂许可。相机仍是原有一张卡片，不做双相机扩展。

### 录制格式与回放

| 卡片动作 | 行为 |
| --- | --- |
| `piper_motion.record_start` | 在已启用的遥操会话中开始录制，不开始运动 |
| `record_stop` | 结束录制、写完整标记并原子保存；不结束遥操 |
| `list` / `inspect(filename)` | 列表或完整文件校验；inspect 不声称硬件条件通过 |
| `play(filename,execute=false)` | 检查文件、绑定、限位、起始姿态并生成一次性预览 |
| `play(...,execute=true,preview_id=...)` | 确认后单次 1 倍速回放，与遥操互斥 |
| `halt` / `stop` | 取消输出并尽力保持，不释放电机力矩 |

文件保存在 `data_dir/recordings/`。JSONL schema 为 `motus.piper.trajectory/1`：

- header 保存模型 SHA256、标定 SHA256、左右 CAN 绑定、关节顺序和单位。
- frame 保存相对单调时钟 `t_ns`、12 维实际下发目标 `target_q`（rad）、两夹爪目标
  `target_width_mm`、采样反馈、反馈 Unix 时间、两侧下发结果与各自耗时。
  SDK 发送返回仅证明主机下发，不能作为电机接收或到位确认。
- complete 保存帧数与整个 header/frame 内容的 SHA256。异常、队列满或写盘失败保留
  `.partial` 文件，不允许回放。故障不会阻塞遥操控制线程；卡片状态显示录制错误。

每次录制最多 10 分钟、12000 帧、64 MiB；每行最多 16 KiB，写队列最多 128 帧。
开始录制后需有至少两帧成功的双臂下发才可保存完整文件。不录图像或原始 VR 输入。
录制期间的松握保持也保存目标与反馈。停止录制后再发生的遥操故障不改变已完成文件。

回放只接受普通文件名，拒绝符号链接、截断、重复字段、NaN、时间倒退、过密帧、超过
300 ms 的采样间隔、过速轨迹、失败下发和不匹配绑定。首帧与当前姿态须在 0.05°、
夹爪 0.5 mm 内，否则拒绝，不自动回起点。回放使用同一限速执行通道，调度落后超过
100 ms 时取消，不突发补帧。结束后检查最终反馈；`running` 不代表回放完成。

### 软件验证与实体边界

```bash
python -m pytest -q tests/test_piper_dual.py tests/test_piper_trajectory.py tests/test_teleop_contract.py
```

同时运行既有十卡片回归和 Docker COPY/部署检查。数值测试使用真实 URDF、NumPy IK 与
隔离进程；硬件控制测试使用模拟 SDK。验证状态以当前任务和对应版本验收记录为准。
构建镜像后，可运行真实 DDS/MCP 的隔离检查。测试只挂载测试代码，不挂载硬件：

```bash
docker run --rm --platform linux/amd64 --network none --read-only \
  --tmpfs /tmp:rw,exec -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 \
  --mount "type=bind,src=$PWD/tests,dst=/tests,readonly" -w /work \
  agilex-piper:dual-teleop-20261012 bash -c \
  'source /opt/ros/humble/setup.bash && PYTHONPATH=/work:/tests:$PYTHONPATH python3 /tests/validate_piper_dds.py'
```

单臂十卡片的历史现场验收不能替代双臂标定、真实 DDS/头显、夹爪、停止距离与轨迹误差验收。
