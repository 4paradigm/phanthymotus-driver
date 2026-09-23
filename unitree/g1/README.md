# Unitree G1 Driver

G1 Driver 通过 MCP 提供运动控制、机械臂预设动作、麦克风、扬声器、LED 与状态监控等现有能力。启用配置见 [config.yaml](config.yaml)，构建与开发入口见[仓库 README](../../README.md)和[贡献规范](../../CONTRIBUTING.md)。

现有 `arm` 支持预设手势与 `release`；`release` 使用厂商 action 99。现有连续控制入口为 [servo.py](servo.py) 和 [servo_eef.py](servo_eef.py)，其配置、动作维度、型号探测和运行前提仍以当前实现为准。SDK 返回成功不能替代动作完成的实测确认。

## G1 双臂遥操计划

遥操标定的 `safety.allowed_fsm_ids` 支持 `[500, 801]`，两种模式均按用户确认允许；Driver 只检查反馈，不自动切模式。旧标定没有该字段时沿用原 `fsm_id` 单值。

当前目标调整为 PICO Driver → G1 Teleop Control 两卡，Core / ActuCore 零改动。按[两卡验证计划](../../docs/plans/g1-two-driver-offline.md)检查兼容性和数值/执行链；下面的四卡计划是旧设计，不能当成当前产品接线方式。北京 r11b 已部署，用户本轮实体跟随与松握重握反馈正常；项目停止不发送动作，长时及全故障验收尚未完成。

[G1 双臂 motion_control / arm 契约与验收计划](../../docs/plans/g1-dual-arm-motion-contract.md)定义拟接入的 `ext_vr → teleop → motion_control → arm` 链路，目标为北京 G1_23、每臂 5 关节。该计划区分 EEF 14 个位姿数与 joint 10 个关节角，规定连续执行、内部重力补偿、唯一公开反馈和既有 release 交接，并提供分阶段验收表。

`teleop_control` 候选已完成隔离 DDS 输入/反馈检查与机上 IK 自检；具体边界见[验证记录](../../docs/validation/g1-two-driver-offline.md)。两卡执行复用 servo 的位置发布，当前 r11b 使用120ms一阶平滑与1rad/s指令限速，不做重力补偿；每个有效目标只发送一次，断流保持，恢复后接收新目标。完整目标仍需通过关节限位检查；附加碰撞策略见下文；开始/停止和既有 `arm.release` 交接保持独立。旧四卡文档的重力补偿与连续插值不是当前两卡契约。天轶 #321 保持独立，本轮不操作天轶。

### 两卡配置简化

安装机器人 Driver 后，在画布添加 `teleop_device` 并将命令输出连接 `teleop_control`。控制卡不要求用户填写实例 ID、模型路径、位移比例或 Shadow/Live；这些属于框架身份或 Driver 预设。生产预设 Live，启动画布只接通链路，收到有效双握把输入后才申请执行。旧保存的用户模式/标定字段不覆盖部署预设。

Live 不再要求填写模型/空间/PICO/外部控制验收布尔值、操作者、日期、证据 SHA 或停止/崩溃验收标记；仍检查实时反馈、故障和运动条件。实物验收结果另行如实记录，不作为人工填表启动门槛。当前运行镜像是否更新须以部署记录为准。

### 单向设备输入与监控

`teleop_device → /teleop/command → teleop_control`，设备端不订阅反馈、不发送操作请求。控制卡接到有效且双握把松开的输入后建立初始基准，重新握把不重标定；真实跟踪空间变化需先松握重新就绪。PICO只显示配对和握把提示。控制卡的 `/teleop/state` 供 Canvas 监控查看，带可读状态摘要，不需要连回设备。实例ID/固定说明项不属于卡片正面的动作输入，说明只在齿轮页。停止使用 Canvas 停止；收臂复用 arm release。

遥操不再因 DDS 手臂 topic 存在其他发布者而拒绝准备；不会自动停止其他控制程序。IK 进程通信故障分别报告 `ik_worker_timeout`、`ik_worker_exited` 或 `ik_worker_unavailable`，容器日志保留原始异常。此源码修正的部署状态以验证记录为准。

额外硬件状态门禁已在源码移除：电机 mode/motorstate、电压、温度、遥控器按键、mode_machine、腰腿相对旧标定偏差不再合并为阻塞遥操的 fault。`motor_telemetry` 保留 SDK 原始读数；反馈格式和时效仍校验，厂商本体保护不变。最新是否部署以验证记录为准。

G1 遥操默认停用附加软件碰撞/工作区包络扫描，`collision_checks_enabled=false` 可在 arm/motion 监控读回；URDF 关节限位仍有效。该配置不代表运动已通过碰撞验收。短暂 IK 失败后的新帧使用同会话续接，`continuation_ready` 与 `hold_confirmed` 分开：前者表示可接收新目标，后者才是物理保持确认。保持前旧帧拒绝；滤波历史不保留失败 IK。

### 平滑与 1 rad/s 限速（r11b 沿用）

参考 PR #322 从上一条实际下发位置接续的原则，G1 先做120ms时间常数的一阶平滑，再将各关节增量裁剪到 ±1 rad/s × dt。dt 使用成功下发间隔，最大50ms，避免断帧或重握积累大步长；基准不是每帧实测位置。只处理最新输入，不排队补发旧目标。此限制约束下发位置参考的变化速率，不是实测电机速度的硬保证。

遥操专用肩肘 kp=80/kd=3，腕部40/1.5；不修改普通 servo 的全局增益。松握重握在清空旧待发目标后可 resume，无需速度或位置静止确认；显式停止确认仍独立保留。r10 引入上述行为，r11b 已部署并获用户本轮实体跟随/恢复正常反馈；长期稳定性与异响的独立机械诊断仍未完成。


### ROS 接收与重启恢复（r11b，本轮现场反馈正常）

遥操节点由同进程内独立单线程 ROS 执行器接收；订阅创建、读取和销毁在同一线程完成。设备身份补全和重复启动不重建同 topic 订阅。公共执行器针对订阅销毁的 InvalidHandle 做有界恢复，未知异常保留堆栈并标记故障；监控看心跳，不以 MCP 在线代表接收健康。

`teleop_control.info.input_status` 区分 waiting_binding、stopped、waiting_input、input_stale、transport_error、fresh，带操作提示和输入年龄。`transport` 提供收帧/接受序号、拒绝原因、订阅代次及两个执行器健康。`/teleop/state` 增加同名诊断对象，原消息字段和topic不变。

普通Core在Driver重注册时恢复配置但不重放卡片start。重启Driver后请从Canvas停止再启动项目恢复输入绑定，无需删线重连；Driver不持久化或自动恢复运动租约。此候选不修改Core/ActuCore。

可回退运动基线为PR #330的 `8a0fe61`（r9，有异响及ROS问题未解决）；r11b 已独立部署，用户实体PICO跟随/恢复体验反馈正常；长期稳定性与独立异响诊断不在本轮结论内。

空闲且未接管SDK时重复停止返回no_op，不创建释放占用，也不声称物理停止已验证。无厂商action99的释放等待允许重复stop；未知厂商动作结果仍保持未知，不通过幂等处理清除。


### 平滑与轨迹衔接的参考来源

感谢 **@jsmy-CTH** 在 [PR #322：提供实际下发状态用于轨迹衔接](https://github.com/4paradigm/phanthymotus-driver/pull/322) 中的贡献。本实现参考其“以实际成功下发的关节参考作为后续轨迹衔接起点”的做法，避免反复以存在滞后的实测位置作为推进起点。请将 #322 作为这部分设计来源保留。120ms一阶指数平滑及本卡1rad/s、dt最大50ms的组合是本次G1适配追加的实现，不冒称为#322原有全部算法，也不将参考部分描述为本PR独立首创。
