# PiPER 单臂与 D435：五张卡片

同一套驱动支持 Ubuntu SocketCAN 和 Mac candleLight USB-CAN。
启动、重连、容器重启均不使能、不重放动作。`PIPER_MOTION_ENABLED` 默认 `0`。
当前按用户要求在 **Mac 验收**；Ubuntu 镜像与 ROS 画布部署保留，尚未视为通过。

| 卡片 | 使用方式 |
| --- | --- |
| `piper_status` | `read` 看关节、夹爪、模式、使能、故障、新鲜度；`reconnect` 废弃旧预览后重连 |
| `piper_mode` | `read` / `preview` / `switch` / `stop_teaching`；未验证固件显示按键引导 |
| `piper_arm` | 初始化、单轴/六轴低速控制、保存/返回停放位、保持、停止、失能 |
| `piper_gripper` | 读取、开合、指定毫米开度、保持、停止、失能 |
| `piper_photo` | 当前姿态 RGB 拍照，保存 JPEG + JSON；Ubuntu 发布画布图像 |

## Mac 运行与验收

在仓库根目录使用 Python 3.10–3.12（现有 Mac 环境为 3.12）。
只连接一台 candleLight / gs_usb 适配器；停止此前的 CAN 监听或控制程序。

```bash
uv venv .venv-piper --python 3.12
uv pip install --python .venv-piper/bin/python -r agilex/piper/requirements-mac.txt
PIPER_MOTION_ENABLED=0 .venv-piper/bin/python -m agilex.piper.mac_main
```

打开 `http://127.0.0.1:15741/`，可直接操作同一 MCP 实现的五张卡片。
此页面仅监听本机；运动仍受门禁、预览、互斥和现场条件检查。
Mac 通过 AVFoundation 选择名称包含 RealSense 的设备，**不会自动选择内置摄像头**。
仅开放深度/红外、画面无彩色信息或相机权限不足时会报错。
Mac 不启动 ROS，照片在本地页面显示；`dds_published=false` 如实说明未发布 DDS。
Ubuntu 的 DDS 画布显示需要在部署时另行验证。
本次用户要求只提供自己拍摄的照片，已停止 photo 卡片；后续验收不调用相机。
已观察到部分 USB-CAN 关闭重开后需拔插 Mac 一侧 USB 才能恢复；重连稳定性尚未验收。

可传 `--usb-index 0 --port 15741 --data-dir /path/to/acceptance`。
Mac USB-CAN 还有第二层只读门禁：环境变量为 `0` 时只允许固件信息查询，拒绝所有控制帧。

```bash
python3 agilex/piper/deploy/diagnose.py --query-firmware --photos 3 --output mac-readonly.json
```

只读检查通过后，现场确认底座固定、空间清空、有人可及时停止，再关闭服务并用
`PIPER_MOTION_ENABLED=1` 启动。服务重启本身仍不使能。

## 日常操作

1. **读取**：状态应显示“电脑控制｜六轴已使能｜无故障”这样的三段信息。
2. **切换**：在模式卡片先预览，再按返回的缺少条件处理。切换不代替初始化。
3. **初始化**：`prepare` 默认预览；核对边界修正，再用相同速度执行。
4. **移动**：推荐 `move_joint` / `move_joints`，先 `execute=false`，再使用相同参数和
   `preview_id` 设置 `execute=true`、`workspace_clear=true`、`take_control=true`。
5. **停放位**：`save_default` + `confirm_save=true` 保存当前反馈，`return_pose` 预览/返回六轴。
6. **夹爪**：`open` / `close` / `set_width` 默认预览；确认开度、力矩后执行。
7. **停止**：本卡片的 `stop` 取消本卡片操作并尽力保持，不释放力矩。
8. **失能**：扶稳机械臂后单独调用 arm `disable` + `arm_supported=true`；
   夹持物已支撑后调用 gripper `disable` + `object_supported=true`。

预览编号可留空，此时使用最近一次匹配预览。预览只保存在进程内，**只能执行一次**；
新预览会替换旧预览，超过 60 秒、任一关节偏移超过 0.5°、模式/使能/示教/故障变化，
或重连、停止相关控制卡片后需重新预览。夹爪自身开度偏移超过 0.5 mm 或使能变化也需重预览。

## 模式切换与提醒

| 目标 | 要求 | 提醒 |
| --- | --- | --- |
| 待机 `standby` | 当前待机或 CAN，无其它动作，六轴全部失能 | 请先支撑机械臂并解除使能。 |
| 电脑控制 `can` | 当前待机，六轴失能，反馈正常，其它程序已停止 | 切换后仍未使能，请再执行初始化。 |
| 拖动示教 `teaching` | 当前 CAN，六轴已使能，静止 0.5 秒，人员扶稳、空间清空 | 扶稳后进入示教，轻推调整姿态。 |
| 示教转电脑控制 | 人员扶稳，停止录制/回放，固件流程已验证 | 先退出示教，再检查状态并初始化。 |

模式、使能、急停和故障分别处理。**待机不等于失能，停止不等于释放保持力。**
普通切换不会回零、复位、重启、自动失能或自动使能；3 秒内没有切换后的新鲜反馈，
返回“未确认切换成功”，不自动重试。以太网、Wi-Fi、遥控、主从和离线轨迹只显示状态。

### 示教固件能力

先用 `piper_mode.read(query_firmware=true)` 重新查询实际固件。此调用仅发 `0x4AF` 信息查询。
默认 `mode.verified_teaching_firmwares: []`，所有示教进入/退出只显示按键引导。
不能因 SDK 有函数或版本号较新，就视为软件示教已验证。

部署人员仅可把**现场验证过**的完整 SDK 固件字符串加入白名单。验证内容包括
`MotionCtrl_1(grag_teach_ctrl=1/2)` 的实际模式、录制、使能变化，且无回零、重启、
复位或意外失去保持力；验收记录必须注明硬件、固件与证据。
重新连接后须重新查询固件；软件进入示教指令也会开始录制。

`stop_teaching` 不检查停放位：对未验证固件和录制状态提供按键停止引导；
对已验证固件的回放发送暂停指令，并验证模式保持不变。
不会把“结束录制/退出示教”伪装成保留原模式的停止，不自动接管 CAN。

官方说明：[ROS 服务与固件差异](https://github.com/agilexrobotics/agx_arm_ros#服务调用)、
[SDK 维护者关于按键示教的说明](https://github.com/agilexrobotics/piper_sdk/issues/11)。
旧版退出流程可能涉及回零、重启；这些操作需单独安排。

## 运动边界与兼容

- 新六轴接口单位为度、度/秒：默认 `1°/秒`，允许 `0.5–5°/秒`，单次每轴最多 `12°`。
  三次平滑插值按峰值速度规划，20 Hz 发目标，调度中断不会直接跳到未来目标。
- SDK 速度百分比独立限制在 `1–10%`。全部目标遵守 SDK 和部署限位交集。
- 到位须在各轴 `±0.5°` 内稳定 `0.5 秒`；跟随偏差超过 `3°` 或超时会取消旧目标。
- 夹爪以 mm 和 N·m 为单位，开度取 SDK 与配置交集，默认 `0.5 N·m`、上限 `1 N·m`。
  确认到位后才报告完成，遇到夹持接触但未到指定开度不会虚报成功。
- 初始化最多 `3°` 的限位修正会明确预览，须单独 `allow_limit_adjustment=true`。
- 保留 `move_j1(target_deg, speed_pct, workspace_clear, take_control)` 旧接口：
  **直接执行**绝对 J1 目标，步长 `0.1–12°`、绝对角度 `±140°`，其余轴保持测量姿态。
  新操作推荐有预览的 `move_joint`。
- 保留 `return_default` 的 **J1-only** 含义：J2–J6 须已在保存值 `±1°` 内；
  新增 `return_pose` 才是六轴返回。超过任一轴 `12°` 会拒绝，需现场分段规划。
- 保存的停放位是部署本地文件，不是厂家零点；不提供 IK、自动避障、厂家零位校准。

arm / mode / gripper / photo 共用操作锁，忙时立即拒绝，不排队。status 在动作期间可读。
各卡片 `stop` 只取消自己的动作，返回 `state=idle`；`operation_stopping=true` 表示工作线程
仍在清理，期间新的物理操作被拒绝。清理完成前不会断开总线。
反馈超过 `0.5 秒`、某一关节/电机帧停更、故障、部分使能或外部模式变化会拒绝/终止动作。
同一部署数据目录的进程锁防止此驱动双开；仍需人工确认其它厂商程序已停止。

长动作返回 `action_id`，通过 ACP 回调完成/错误/取消；在卡片 `info.last_action` 可查看结果。
客户端超时或回调丢失不代表机械臂已停止，应调用对应卡片 `stop` 并检查反馈。
最大新六轴动作约 44 秒，ACP 等待上限 60 秒。所有返回都是普通 JSON 字典。

## Ubuntu 镜像与部署（保留，当前不作为 Mac 验收条件）

Ubuntu 22.04 amd64、ROS2 Humble，`can0` 由部署人员配置为 1 Mbps 正常模式。
Motion 默认关闭；保持 host network、DDS domain 42、FastDDS loopback profile 的只读挂载。

```bash
bash agilex/piper/build-local.sh agilex-piper:local-amd64
bash agilex/piper/deploy/check-image.sh agilex-piper:local-amd64
python3 agilex/piper/deploy/diagnose.py --query-firmware --photos 3 --output ubuntu-readonly.json
```

Bot 的仓库通用构建默认可能为 arm64，**必须 inspect 镜像为 amd64 后才可部署到该 Ubuntu**。
不要把旧 Bot 镜像号当成当前提交。依 landing 流程更新 PR #382 → 请求 Bot 镜像 →
核对提交/架构 → 现场验收 → Master/老师验收 → 合并与任务凭证。未验收不会自动合并或代发人员消息。

镜像沿用既有 ROS base 和依赖；新增 Python 控制/策略模块不增加第三方依赖。
Mac 的 gs_usb、libusb、PyObjC 只在 `requirements-mac.txt`，不进入 Ubuntu 镜像。
OpenCV 用于 RGB JPEG、NumPy 用于 SDK/图像、PyYAML 用于配置；保留 sensor_msgs 与 FastDDS
以支持画布图像。实际大小由 `check-image.sh` 输出，待镜像构建时记录，未虚构尺寸。

## 离线与真机验收记录

```bash
python3 -m unittest discover -s tests -p 'test_piper*.py'
```

测试覆盖参数、模式条件、一次性预览、过期/漂移/外部切换、取消/超时、门禁、
独立停止、动作中读状态、夹爪单位/限位、进程互斥、MCP/ACP、日志及部署约束。
模拟不满足条件时断言 **0 条控制命令**。

真机依次验收：状态与三次照片 → 待机/CAN → 初始化 → 逐轴及双轴小幅动作 →
空载夹爪 → 六轴停放位 → 示教进出 → 停止及支撑后失能。
重复点击、不支持固件、按键切换、切换超时与连接恢复分别记录。
历史单机的 J1 演示不算本版六轴/夹爪/模式切换的验收。
最新记录见 [ACCEPTANCE.md](ACCEPTANCE.md)。
