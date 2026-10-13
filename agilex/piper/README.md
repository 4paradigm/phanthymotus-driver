# PiPER 机械臂控制卡

当前驱动将操作合并到中文 `arm_control` 卡片，相机 `piper_photo` 与 `piper_camera_live` 保留为可选卡。
使用现有“选择操作＋执行按钮”及通用预览确认功能，参考 Q5 的动作与参数组织。
默认操作许可关闭；模式、电机使能与操作许可独立。

- [控制卡操作与参数](CONTROL_CARD.md)
- [共享只读快照接口](MONITOR_SNAPSHOT.md)：供计划二监控插件注入使用，共用原 CAN 连接。
- [两卡集成与画布迁移](INTEGRATION.md)：计划二完成后再注册监控与模型、安排现场核对。
- [历史十卡说明](LEGACY_TEN_CARDS.md)：旧源码、回归与现场备份保留，默认单臂不再注册旧工具；双臂模式继续使用十卡片。
- [历史验收记录](ACCEPTANCE.md)：原版本记录不能用作本控制卡的真机验收证据。

计划一与计划二从同一驱动提交建立独立分支。本分支交付控制卡、快照及测试；
单臂已保留现场独立 `piper_status` 骨架卡和 `model`，源码来源见 [部署兼容记录](DEPLOYMENT_COMPATIBILITY.md)。每台画布放控制与监控两张核心卡，
相机可选，后台 model 不摆成卡片。两台沿用各自 MCP、CAN 和话题绑定。

## 验证

测试使用假硬件记录 CAN 调用；生命周期和监控只读接口不发送控制指令。

```bash
python3 -m unittest discover -s tests -p 'test_piper*.py' -v
```

Ubuntu 运行入口仍为 `PYTHONPATH=. python3 agilex/piper/main.py`。
Docker 打包新增 `arm_control.py`，沿用原 ROS 与固定 SDK 依赖，不新增 agent core 修改。
默认 `PIPER_MOTION_ENABLED=0`、`PIPER_CAMERA_ENABLED=0`。

现场部署前必须核对额外限位实现、现有设备状态与备份。未核对额外功能前不得覆盖现场版本；
真机动作另行取得现场授权。当前实际 Bot Version：**尚未生成**。

## 双臂 VR 遥操与轨迹回放

默认单臂配置注册 `arm_control`、`piper_photo`、`piper_status`、`model` 和 `piper_camera_live`；控制许可与确认差异见部署兼容记录。配置 `arms.left` 和 `arms.right` 后，同一 Driver
管理两路独立 SocketCAN，使用原有十卡片并增加 `piper_teleop` 和 `piper_motion` 两张卡片。
切换双臂配置前，必须停用原来分别占用这两路 CAN 的单臂实例；不能并行抢占同一设备。
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
5. 保留两台 PiPER MCP 15741/15742。PICO MCP 使用 15744，WSS 使用
   15743；部署前重新检查三个端口的占用，参考 [PICO 同机配置](deploy/pico-on-piper.example.yaml)。将该配置放入 PICO
   已有持久化目录，并设置其 `CONFIG_PATH=/var/lib/motus/pico/config.yaml`；保留配对状态。
   核对 mDNS、安装网址和配对邀请都使用新端口，不修改 App 或构建另一套输入协议。

两服务使用相同宿主机启动时钟、DDS domain 42 和既有 loopback DDS 配置。
当前只支持一组设备输入和一组双臂消费者。PiPER 只订阅画布在 `start` 中传入的
`input_topic`，不回退到固定 topic；缺少连线输入时返回 `waiting_binding`，不能开始遥操。
更换输入 topic 前先停止卡片。设备当前通常发布 `/teleop/command`，同域不要同时运行另一组
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
