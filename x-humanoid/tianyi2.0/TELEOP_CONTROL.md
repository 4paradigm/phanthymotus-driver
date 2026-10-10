# 天轶双 Driver 遥操

## 使用

PICO Driver 的 `teleop_device` → 天轶 `teleop_control`。不修改 Core/ActuCore，不连接旧三卡链路。
安装机器人 Driver 后即有控制卡；齿轮只显示固定使用说明，画布不暴露 instance ID、标定路径、比例或模式。
从设备卡齿轮复制网址完成 App 下载、连接和配对。

1. Canvas 连接设备 Cmd 输出到控制卡，启动项目。
2. 正确佩戴头显、松开双握把；冷加载完成后以最新输入及实测 FK 建立一次映射。
3. 按住双握把跟随，松开任一握把保持；再次按住沿用原映射，不重新对齐实际手臂。
4. 停止项目结束遥操，不追加收臂。需要自然下垂时使用已有 `arm_gesture` 的 `reset`、`side=both`。
   reset 先停止遥操并确认释放，再使用原动作与反馈流程；失败返回原因，不强行抢占。
5. 空间重置出现 `needs_calibration` 时停止并重新启动项目；硬件故障先排除原因，再显式重启会话。

默认机器人预设为 Live，注册、配置、容器重启和启动项目均不自动运动；双握把才使能。
内部测试仍可注入隔离配置，但普通齿轮不提供 Shadow/Live。旧 UI 持久化运动参数不覆盖机型预设。

## 接口与线程

| 方向 | topic / format / schema |
|---|---|
| 设备 → 控制卡 | `/teleop/command` / `data/teleop-cmd` / `motus.teleop.command/1` |
| 控制卡 → Canvas监控 | `/teleop/state` / `data/teleop-state` / `motus.teleop.feedback/1` |

同机隔离 DDS domain42，JSON over std_msgs/String；控制侧接收 BEST_EFFORT、KEEP_LAST(1)、VOLATILE。
设备身份在消息中，不从 topic 派生；可选 controls 和 extensions 兼容，但不驱动新增动作。
设备只提供 input；旧 PICO begin/finish/stop/calibrate 操作包被拒绝，生命周期使用 MCP info/config/start/stop。
监控保留 schema 兼容及空 receipts，显示输入序号、映射代次、IK 决策、真实执行状态及 transport 恢复状态。
MCP 管理沿用普通 Core 的本机 loopback 调用，不新增宿主代理、鉴权或反馈接线。

天轶厂商 DDS 与本地 DDS 保持进程隔离，订阅创建、spin 和销毁由辅助进程单一线程负责。
异常退出由主进程有界重建；旧会话和过期帧拒绝，恢复不改映射，监控阻塞不拖住执行/停止。
IK 在独立工作进程中运行，输入/输出均有界并带原始截止时间；只消费最新有效结果。

## 模型、平滑与故障

内置 `tianyi_motion/tianyi2_dual_arm.json`：官方 URDF 固定 SHA、双臂各7关节、胸部参考系、
TCP与碰撞几何沿用冻结对照配置。静态预设不保存旧现场头腰腿角度，每轮按新鲜机器人反馈建立固定姿态基准。
范围仅双臂，hands_enabled=false；验收记录不参与启动门禁，不伪填物理验收标志。

相对位移比例0.5，位置目标使用既有 `/arm/cmd_pos`。不新增力矩、KP/KD或重力补偿。
以最后成功发布参考为起点做120ms一阶指数平滑，再裁剪每关节增量到1rad/s×dt，dt最多50ms。
断帧不累计追赶额度，发送失败不推进参考；该约束是命令位置变化率，不声称电机实际速度硬限制。
感谢 [PR #322 / @jsmy-CTH](https://github.com/4paradigm/phanthymotus-driver/pull/322) 的成功下发参考起点方法；
经 G1 #330 适配的120ms、1rad/s、50ms参数并非 #322 原始参数。

不可达、短暂IK失败或输入过期保持最后有效目标，继续检查新输入；回界后可以续接，不新增越界搜索。
普通松握恢复仍须收到新鲜反馈，但不等待位置/速度误差小于0.02的静止条件。
真正释放/停止仍需要后续反馈确认；急停、硬件错误、反馈有效性、URDF限位和原碰撞检查保留。
发生真实故障时明确显示原因，不伪造停止成功、不自动清错或改模式。

## 验证

使用锁定数值依赖的 Python：

```sh
python3 -m pytest -q x-humanoid/tianyi2.0/tests tests/test_teleop_contract.py
```

数值对照入口 `tests/compare_frozen_teleop_recording.py` 接收私有录制、冻结源码及模型路径；
不提交原录制、现场计划或凭据。冻结基线为 ActuCore unified r4 / Driver operator r16。
真实 DDS 隔离验证入口 `tests/two_driver_dds_smoke.py`（本目录内），要求 Linux、network none、
无设备挂载，连接真实 #329 DeviceRuntime/RosTransport、DDS、IK子进程及有限速度plant。
它不能替代现场反馈或真实机器人验收。开发真机通过后再请求BOT复审，最后验收BOT镜像。
