# 双 Driver 遥操 DDS 契约 v1

用于 #329 PICO teleop_device 与 #321 天轶 teleop_control；公共验证器为 `common/teleop_contract.py`。不依赖 Core/ActuCore 的遥操专用功能。

## 绑定和传输

Canvas 正向连线传入 command topic；`topics(namespace, instance_id)` 返回 command/feedback 二元组，`binding_from_topic()` 从 command 解析 namespace 和标准化实例。实例中的连字符转为下划线，非法路径与跨实例消息拒绝。

topic 为 `/<namespace>/teleop/<instance>/command` 和 `/feedback`，format 分别为 `data/teleop-cmd`、`data/teleop-state`。JSON 使用 std_msgs/String；同机 ROS domain42，本机隔离。两端 RELIABLE / KEEP_LAST depth16 / VOLATILE 匹配。DDS 在隔离通信线程/进程内收发，不能阻塞执行或停止；回调只校验/入队。姿态应用缓存仅一个最新值，操作有独立有界待办，停止优先。DDS历史深度不是动作队列，不允许积压后补播旧轨迹。

## command

schema=`motus.teleop.command/1`，kind=`input` 或 `operation`。共有 instance_id、device_id、connection_epoch、space_epoch、sequence、received_monotonic_ns、clock_id。两种kind分别管理序号，操作不能淘汰姿态，也不能被姿态覆盖。

input沿用已验证XR字段：source_monotonic_ns、tracking_frame、head_reference、left、right。对象包含tracked/position/orientation_xyzw，左右增加grip/trigger。米、单位四元数xyzw，设备跟踪系X前/Y左/Z上；头显时间只追踪来源，不能用于机器人有效期判断。默认接收年龄300ms；转发、心跳、滤波不延长原始时效。无跟踪时pose为null，失效帧作为暂停信号可接收。可以附加由数值生成的安全text监控摘要，消费者不把摘要当控制字段。

operation包含request_id、action(begin/finish/stop/calibrate)、expires_monotonic_ns。默认请求入站期限5秒；重发原包，不改变身份、动作、时间或截止期。已受理的收臂不因入站期限届满中断。重复ID检查内容与代次并返回已记录状态，不能重复执行。连接变化取消旧pending begin；停止始终可由机器人卡本地MCP入口发起，不依赖头显姿态有效。

## feedback

schema=`motus.teleop.feedback/1`；含instance_id、control_instance_id、server_epoch、sequence、emitted_monotonic_ns、clock_id、source_sequence、connection_epoch、space_epoch、operator_session_id、mapping_epoch、state、reason、capabilities、execution、receipts。设备必须检查绑定、时效及反馈序号，旧反馈不能显示为当前执行成功。

execution保留实际Driver状态，包括armed（Canvas会话就绪）、started（操作者会话已建立）、mode；两者不能由ownership推导。实测q/dq、反馈时效、实际输出和保持确认按控制端真实信息输出，不要求所有机器人有同样自由度。capabilities首版为dual_arm。

receipts最多32条，字段为request_id、action、原操作的device_id/connection_epoch/space_epoch、status(accepted/completed/failed)、error(null或短原因)、result(object)。回执身份匹配原请求，不能使用最后姿态的顶层代次替代；因此RTC断流后的停止仍能收到正确回执。重复请求触发已有回执重发。accepted不是动作完成；completed必须有对应执行结果。显示拥堵可丢旧反馈快照，但不得丢失最终操作状态。

## 映射和恢复

开始完成冷模型准备后取新鲜输入和实测建立一次anchor。松握移动后重握使用原映射处理下一有效输入；执行租约更新、短暂保持不改变mapping_epoch。实测是IK初值和执行起点，不是每次重握的新映射原点。真实空间重置进入needs_calibration。

Canvas stop / PICO stop保持；PICO finish受控自然下垂且实测确认。短暂IK/输入失败保持，合法输入恢复后继续；真实硬件故障不得冒充恢复。

## 宿主核验记录

2026-09-23只读核实普通Core远端main=`9802eae43f158af75a9848909d7366d15c985e63`。源代码确认：data/*映射为String；正向连线传instance_id/input_topic；configSchema支持默认布尔和instance配置；活动监控优先显示text。已用原样导出的Core模块，在隔离Chrome/API/SQLite/MCP场景验证PICO配置、启动、监控和停止；另用真实DDS和有限速plant验证双向两卡流程。两者不代替完整生产Canvas或实机验收。

当前齿轮说明使用textContent、普通字符串为input，不支持可点击网址。用户已于2026-09-23接受复制网址入口，保持Core零改动，不注入HTML。配置保存由Core异步下发，必须以Driver info中的实际配置及错误核验，不能只信“保存成功”。
