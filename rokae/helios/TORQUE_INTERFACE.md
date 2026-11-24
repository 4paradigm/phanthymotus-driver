# 关节力矩来源复核（2026-11-18）

当前机器人运行版本1.0.9，RTE SDK0.1.10。读取运行进程 `/proc/*/maps` 确认实际加载的是 `libweb_ui_component-1.0.0.so`、`libdiagnostic_component-1.0.0.so`。

此前检查了磁盘上同时存在、但未被当前进程加载的1.1.0版Web组件，导致错误引用0x968字节消息布局；该结论已被本次实际加载版本检查替代。提供的客户端SDK1.0.3定义也不能按内存布局直接解码当前运行版。

## 映射证据

本机 diagnostic 配置 `scope_schema.yaml` 明确区分motor/sensor/gravity字段。进一步对照已加载诊断库的 `scope_build_native_message_layouts()`，确认编译时字段偏移；与已加载Web库 joint/scope 序列化读取偏移逐一一致。消息大小0x5e8（1512字节）。复核偏移见下表；原始库哈希和反汇编核查记录保留在开发工作区，不随运行镜像发布。

| WebSocket字段 | 编译时native字段 | 偏移 | 显示轴数 |
|---|---|---|---|
| waist | body_motor_torque | 0x58 | 4 |
| waist_gravity | body_gravity_torque | 0x88 | 4 |
| head | head_motor_torque | 0x148 | 2 |
| left_arm | left_arm_motor_torque | 0x208 | 7 |
| left_arm_gravity | left_arm_gravity_torque | 0x278 | 7 |
| right_arm | right_arm_motor_torque | 0x3f8 | 7 |
| right_arm_gravity | right_arm_gravity_torque | 0x468 | 7 |

`left_arm_sensor_torque`在诊断native布局偏移0x240，与WebSocket left_arm的0x208不同。因此不能将网页力矩称为传感器实测力矩。SDK同名motor_torque字段注释为电流环反馈、Nm（0x6077）；gravity_torque为RobotModel计算、Nm。当前网页设置脚本同样明确重力矩来自模型。

## 实现与限制

- 新增只读独立 `joint/scope` 订阅，按现有20轴名称映射 `motor_torque_nm`；双臂及躯干18轴另有 `gravity_torque_nm`，头部没有该值时保留null。
- 角度流和力矩流分别判断新鲜度，1秒力矩过期后清空；scope消息不能延长位置年龄。只有位置、力矩均有效才合入同一输出。分别记录接收时间，不宣称两路严格同步。
- `torque_status.feedback_source=motor_torque`、`gravity_source=robot_model`、`unit=Nm`；`physical_validity_verified=false`。
- 力矩随骨架消息逐轴发布（`joints[].motor_torque_nm` / `gravity_torque_nm`，单位 Nm），不再另设状态 topic；缺失值不补零。
- 静止样本电机反馈全为0，仅能证明上报内容如此；没有厂家逐轴有效标志，不代表真实受力为零或测量链正常。未进行运动、使能或加载校验。未额外启动或修改原厂诊断录波通道。
