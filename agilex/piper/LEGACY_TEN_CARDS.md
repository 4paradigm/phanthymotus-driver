# PiPER 十张独立卡片

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
