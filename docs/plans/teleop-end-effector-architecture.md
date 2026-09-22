# 天轶三段末端遥操架构实施计划

## 范围

用户已授权离线实现 `ActuCore teleop → Driver motion_control → Driver arm`。
本仓只修改天轶 Driver；用户确认设备空闲并开机后，补充机上只读预检、源码构建及禁网验证；不改变 Agent Core、G1、servo、旧动作或 v1
报文语义，不把此前现场效果视为本次迁移验收。提交与发布由主任务统一处理。

1. 将已验证天轶运动学、碰撞空间、可视化及官方模型迁入 `tianyi_motion`，保留来源和许可证；PICO 坐标映射留在 ActuCore。
2. 在同一 Driver 添加 `motion_control` 管理卡片及独立求解线程，读取执行器内部反馈快照。Preview 使用独立会话，只求解和显示。
3. 本机 domain 42 的 control/eef 输入经 IK 转成 control/joint，再由现有 arm 连续入口进入同一个 MotionGate；厂商发布仍只有原执行路径。
4. 显式 finish 在 Driver 内插值收臂、验证碰撞、等待真实停止与释放；异步进度可查询、失败可显式重试，取消能阻断未提交的后续目标。
5. `motus.control/2` 统一 boot/session/seq/source_seq/mapping_epoch/单调时效/动作空间/模型标定版本/HMAC。EEF 输入不超过 300 ms；关节目标不超过 100 ms，且不能晚于源输入期限。
6. 运行期速度沿用显式标定值，新入口未指定时为 1 rad/s；不新增累计行程或跟随误差门槛。原关节、碰撞、时效及停止检查保留。
7. motion_control 卡片通过现有 configSchema/config 机制设置 Driver 标定路径和运行速度；仅空闲时验证候选、原子应用并读回，错误不覆盖旧配置。标定文件内容不通过卡片编辑。
8. 对 bot 上重复而原生 ARM64 未复现的 ROS 消息 CMake 查库失败，保留失败诊断，并针对上游已调查的目录遍历 errno 污染增加构建期修正。只作用于 colcon 构建进程树，真实读目录错误和缺库仍失败；临时库在成功/失败后清理，最终运行环境不使用 LD_PRELOAD。

## 验证

- 真实 FK/IK 和共享 MotionGate 与独立有限速度、有限加速度 plant 组成离线闭环；发布目标不能直接变成实测位置。
- 覆盖 Preview 零输出、旧/新执行权互斥、签名/序号/版本/时效、最新帧、IK 失败恢复、取消、收臂、反馈失效与显式重试。
- 运行实际 DDS bus 入口的 ROS 替身，核对 domain、topic、两段转发和队列行为；不建立真实 DDS 或机器人连接。
- 回归旧 v1、管理请求、线程生命周期、日志、构建上下文及共享执行权；数值依赖锁定 ARM64 / Python 3.10，实际镜像构建另行记录。
- 2026-09-22：天轶专项（排除已有 servo 测试）、共享 ControlSink/生命周期、构建上下文及 Docker COPY 来源检查合计 **294 passed**（11.01 s）。新增 motion_control 与真实 bus 入口替身专项 **31 passed**。
- `test_servo.py` 为 **6 failed / 19 passed**；在未修改的基线 `c32c6cc44946ecc22ba76396fa6e859ea88bc3bc` 独立归档中复现相同六项失败，涉及旧中断钩子断言、ROS 消息替身缺失和旧 ControlSink 测试时钟，不改动 servo 来掩盖这些基线问题。
- 本机已有 ARM64 ActuCore 镜像中，对迁入的真实求解器执行 20 次微小目标对照：单线程总耗时中位数 2.35 ms、最大 9.63 ms；未显式设 BLAS/OMP 线程数的另 20 次中位数 2.45 ms、最大 6.02 ms，均无求解失败。原报告的慢例未复现，因此未猜测性优化算法或放宽 100 ms 生产预算。这是既有镜像上的离线数值证据，不是新 Driver 镜像构建或设备性能验收。
- 原始 `4be9446` 在本机原生 ARM64 完整构建成功，实际主入口和数值依赖导入通过；禁网、只读镜像 smoke 完成两轮启停，Shadow claim 被拒绝，硬件发布者为零。同 HEAD bot 两轮在 rcutils 查库处失败，未复现其原因；官方合并树的天轶构建上下文与 HEAD 一致，不把本机构建成功写成 bot 已通过。
- 构建诊断与上下文/COPY 回归 **35 passed**；禁网 ARM64 容器中用真实 CMake 注入缺失 rcutils，原退出码 1 保留，诊断记录缺库、NOTFOUND 与编译器/系统架构。只输出白名单字段，不输出完整环境。
- 加入 errno 构建修正后，宿主构建/诊断/COPY 回归 **36 passed、1 skipped**（Linux/glibc 专项不在 macOS 上假跑）；该专项在禁网 ARM64 容器内 **1 passed**，覆盖无污染成功、污染失败、修正成功、真实 EIO 失败及缺库失败。修正候选本机 ARM64 完整构建及真实 bundle smoke 均通过，最终镜像无 LD_PRELOAD，临时 C/so 均已清理；fc06778 的 bot/QEMU 构建已通过（8m36s），不再把此错误列作未解决构建阻塞。
- fc06778 已在天轶 Orin 从固定源码原生构建，通过消息编译和真实入口导入；禁网构建阶段实际 bundle 两轮启停通过，拒绝 claim，硬件发布者为零。运行中的 Agent Core、ActuCore、Driver 均未切换；只读 ROS 样本属于通信预检，物理动作和现场遥操验收尚未执行。
- bot 审查后的启停/输入边界修复已通过 **128 项定向回归**：卡片 ready 与执行状态分离，stop 取消求解且不能伪造释放，线程未退出时拒绝重启；无效输入后接收循环继续接收新帧。包括 16 项新增生命周期场景和 15 项异常输入场景。随后固定内容快照 `5657373f1c1c` 在天轶 Orin 原生构建成功，同样 **128 项在实际 Python 3.10 镜像的禁网构建阶段通过**，真实 bundle 两轮启停也通过（无硬件发布者）；未切换任何运行服务。数值栈体积按实际 manifest 记录在部署手册，不声称零增量。

## 文档联动

README、MOTION_CONTROL.md 和 deploy/TELEOP_RUNBOOK.md 描述新三卡路径。
TELEOP.md 明确为 v1 兼容说明，旧计划保留历史证据并指向本计划。公开文档不包含
现场认证信息、个人目录、实时占用或私有回放数据。
