# Unitree G1 Driver

G1 Driver 通过 MCP 提供运动控制、机械臂动作、麦克风、扬声器、LED 与状态监控。启用配置见 [config.yaml](config.yaml)，构建入口见[仓库 README](../../README.md)。

现有 `arm.release` 使用厂商 action 99；连续控制入口包括 [servo.py](servo.py) 和 [servo_eef.py](servo_eef.py)。SDK 请求成功与实测动作完成是不同状态。

## 双臂遥操

`teleop_control` 接收设备 Driver 的输入，在 G1 Driver 内完成相对映射、独立进程 IK 和双臂位置下发。目前模型适用于原厂 G1_23（每臂 5 关节）。Core 和 ActuCore 无需遥操专用修改。

1. 安装 PICO Driver，在 Canvas 添加 `teleop_device` 和 `teleop_control`，连接命令输出与控制卡输入。
2. 在设备卡齿轮页查看安装和配对说明，连接 PICO。
3. 启动项目，先松开双握把；控制卡收到有效输入后建立初始相对基准。
4. 同时按住双握把控制双臂。松开任一握把暂停，重新握住沿用原基准；跟踪空间重置后需松握重新就绪。
5. 从 Canvas 停止项目；收臂复用既有 `arm.release`。Driver 重启后需停止再启动项目恢复绑定，无需删线重连。

用户不需要填写实例 ID、模型路径、位移比例或 Shadow/Live。这些由框架和 Driver 预设管理；启动项目本身不发送运动目标。支持 FSM 500、801，不自动切换本体模式。

| 方向 | Topic | 格式 |
|---|---|---|
| 设备 → 控制卡 | `/teleop/command` | `data/teleop-cmd`，JSON schema `motus.teleop.command/1` |
| 控制卡 → Canvas 监控 | `/teleop/state` | `data/teleop-state` |

设备输入为左右控制器位姿、握把、跟踪状态、身份、代次、序号与时效。设备卡不订阅机器人反馈；PICO 只显示连接状态和握把提示。URDF、关节顺序、数值求解与执行均在机器人 Driver 内。

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
