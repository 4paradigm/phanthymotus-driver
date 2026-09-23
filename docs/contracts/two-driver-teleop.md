# 遥操设备输入契约

`teleop_device` 是设备输入源；机器人 `teleop_control` 负责映射、IK 和执行。设备 Driver 不加载机器人模型、不申请运动控制权，也不依赖 Agent Core 或 ActuCore 的专用改动。

## 接线与传输

Canvas 连接 `teleop_device` 输出到机器人 `teleop_control` 输入。设备只有一个输出：

| 项目 | 值 |
|---|---|
| Topic | `/teleop/command` |
| Format | `data/teleop-cmd` |
| Schema | `motus.teleop.command/1` |
| 消息 | `std_msgs/String` 中的 JSON |
| DDS | 同机 domain 42，部署加载本机隔离配置 |
| 当前 QoS | RELIABLE、KEEP_LAST(16)、VOLATILE |

当前同一 ROS domain 支持一个设备输入源。设备和实例身份放在消息中，topic 不拼接品牌、日期或实例 ID。应用层仅保留最新待发姿态；DDS 历史深度不代表允许排队执行历史动作。

## 输入消息

`kind` 固定为 `input`。主要字段：

- `instance_id`、`device_id`：设备卡实例和设备身份。
- `connection_epoch`、`space_epoch`、`sequence`：连接、跟踪空间代次及输入序号。
- `clock_id`、`received_monotonic_ns`：接收主机的时钟身份及接收时间。
- `source_monotonic_ns`：头显采样时间，只用于追踪来源，不能直接与机器人时钟比较。
- `tracking_frame`：`tracking_x_forward_y_left_z_up`，右手坐标系，X 前、Y 左、Z 上。
- `head_reference`、`left`、`right`：`tracked`、位置 `position`（米）、单位四元数 `orientation_xyzw`；控制器额外携带 `[0,1]` 的 `grip` 和 `trigger`。

未跟踪对象的位姿为 null。消费者校验格式、有限值、身份、代次、序号和时效；默认输入年龄上限为 300 ms。转发、心跳和滤波不得把旧采样时间刷新为当前时间。完整字段验证见 `common/teleop_contract.py` 的 `validate_input`。

## 生命周期与显示

设备配对只建立输入连接，不启动机器人运动。Canvas 启动和停止设备采集及机器人控制卡；握把是否使能由机器人控制卡处理。

PICO 显示连接状态、透视画面及握把提示，不发送 begin/finish/stop 操作请求，不订阅机器人执行反馈。机器人卡的 `/teleop/state` 用于 Canvas 监控，不是设备卡的输入或输出。共享模块中保留的历史操作/反馈辅助接口，不属于当前设备接线契约。

安装、配对与局域网发现的使用说明见 [PICO Driver](../../pico/pico/README.md)。
