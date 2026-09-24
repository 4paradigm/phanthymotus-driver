# Unitree G1 Driver

G1 Driver 通过 MCP 提供运动控制、机械臂动作、麦克风、扬声器、LED 与状态监控。启用配置见 [config.yaml](config.yaml)，构建入口见[仓库 README](../../README.md)。

现有 `arm.release` 使用厂商 action 99；连续控制入口包括 [servo.py](servo.py) 和 [servo_eef.py](servo_eef.py)。SDK 请求成功与实测动作完成是不同状态。

## 双臂遥操

`teleop_control` 接收设备 Driver 的输入，在 G1 Driver 内完成相对映射、独立进程 IK 和双臂位置下发。默认注册该卡，市场清单包含 `teleop_control`。随镜像提供原厂 G1_23（每臂 5 关节、固定假手）的完整固定几何 profile；不依赖现场临时标定脚本或用户填写路径。Core 和 ActuCore 无需遥操专用修改。

1. 安装 PICO Driver，在 Canvas 添加 `teleop_device` 和 `teleop_control`，连接命令输出与控制卡输入。
2. 在设备卡齿轮页查看安装和配对说明，连接 PICO。
3. 启动项目，先松开双握把；控制卡收到有效输入后建立初始相对基准。
4. 同时按住双握把控制双臂。松开任一握把暂停，重新握住沿用原基准；跟踪空间重置后需松握重新就绪。
5. 从 Canvas 停止项目；收臂复用既有 `arm.release`，它先取消遥操输入及待发目标再交还 SDK，之后需从 Canvas 重新启动遥操。Driver 重启后也需停止再启动项目恢复绑定，无需删线重连。

用户不需要填写实例 ID、模型路径、位移比例或 Shadow/Live。这些由框架和 Driver 预设管理；启动项目本身不发送运动目标。支持 FSM 500、801，不自动切换本体模式。

| 方向 | Topic | 格式 |
|---|---|---|
| 设备 → 控制卡 | `/teleop/command` | `data/teleop-cmd`，JSON schema `motus.teleop.command/1` |
| 控制卡 → Canvas 监控 | `/teleop/state` | `data/teleop-state` |

设备输入为左右控制器位姿、握把、跟踪状态、身份、代次、序号与时效。设备卡不订阅机器人反馈；PICO 只显示连接状态和握把提示。URDF、关节顺序、数值求解与执行均在机器人 Driver 内。

## 机型 profile 与每轮基准

`g1_motion/g1_23_fixed_hand.json` 固定关节映射、URDF SHA、双掌 TCP（腕轴前方 0.2 m）和手柄局部变换，沿用本轮 r11b 已实测的原厂固定手配置。该文件不包含某次现场腰腿角度，也不填写 acceptance 通过标记。每次初始化映射时，从同一份新鲜 LowState 采集双臂和 13 个腰腿关节，为数值工作进程生成本轮临时 profile；工作进程重启沿用同一份基准，新的遥操会话重新采样。缺失、过期或非有限实测值会明确报告，不用零值猜测机器人姿态。松握重握不重新标定。

适用范围是原厂 G1_23 固定假手；不宣称自动适配 G1_29、灵巧手或改装 TCP。附加碰撞检查仍关闭，旧现场预览空间盒未纳入默认 profile，不能将其当作碰撞验收结果。模型及上述 TCP 有本轮现场使用依据，但没有补造独立尺寸或长时机械验收记录。

卡片注册、info、默认启用及空闲 stop 不申请运动权、不创建 SDK 命令通道。`servo`、`servo_eef` 可以同时注册；本实现不新增统一控制器仲裁，也不在装载时因其他卡片 enabled 而拒绝遥操。旧会话/过期指令隔离与本遥操的停止取消仍保留。

## 执行与平滑

每个有效目标仅下发一次。以后端最后成功下发的位置为起点，做 120 ms 一阶指数平滑，再将各关节增量裁剪到 ±1 rad/s × dt；dt 最大 50 ms，断帧或重握不积累追赶额度。仅处理最新目标，不排队补发历史动作。这限制的是位置参考的变化率，不是实测电机速度的硬保证。

遥操肩肘增益为 kp=80/kd=3，腕部为 40/1.5；普通 servo 增益不变，不增加重力补偿。短暂 IK 失败丢弃失败结果及其滤波历史，后续有效新帧可续接。松握恢复不等待静止阈值；`continuation_ready` 表示可以继续接收目标，与物理保持确认 `hold_confirmed` 区分。

输入有效性、反馈时效与 URDF 关节限位仍校验；附加软件碰撞/工作区扫描默认关闭，不能据此认为任意姿态均无碰撞。原始电机遥测保留，厂商保护不修改。Driver 不自动停止其他控制程序。

## 状态与恢复

Canvas 监控的 `input_status` 区分 `waiting_binding`、`stopped`、`waiting_input`、`input_stale`、`transport_error` 和 `fresh`。`transport` 提供接收/接受序号、拒绝原因、订阅代次和执行器心跳；MCP 在线不等于 DDS 接收正常。

订阅创建、读取和销毁由同一专用 ROS 线程执行；设备身份补全不重建同 topic 订阅。IK 子进程故障分别报告 `ik_worker_timeout`、`ik_worker_exited` 或 `ik_worker_unavailable`，原始异常保留在日志中。

空闲且从未接管 SDK 时，重复停止返回 no-op；不创建释放任务，也不声称已验证物理停止。未知的厂商动作结果仍报告未知，不通过重复 stop 清除。Driver 不持久化或自动恢复运动租约。

### 平滑与轨迹衔接的参考来源

感谢 **@jsmy-CTH** 在 [PR #322：提供实际下发状态用于轨迹衔接](https://github.com/4paradigm/phanthymotus-driver/pull/322) 中的贡献。本实现参考其“以实际成功下发的关节参考作为后续轨迹衔接起点”的做法，避免反复以存在滞后的实测位置作为推进起点。请将 #322 作为这部分设计来源保留。120ms一阶指数平滑及本卡1rad/s、dt最大50ms的组合是本次G1适配追加的实现，不冒称为#322原有全部算法，也不将参考部分描述为本PR独立首创。


## 离线回归

常规测试位于 `unitree/g1/tests/`。`scripts/compare_g1_teleop_baseline.py` 使用明确 Git 基线、同一 profile 及已有录制，分别比较相对映射和固定时钟执行输出；它使用 SDK 替身，不是完整 IK 回放或真机验收。旧四卡专用、依赖外部 mapper 的 `validate_g1_offline.py` 已移除。真实模型 profile 对照见 `test_motion_control_numeric.py`。
