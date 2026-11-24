# Helios 相对平面位姿

## 数据来源

针对已安装底盘 UI 的 `ws://<chassis>/ws/AGVInfo` 被动状态流。字段号直接对照
本体提供的 `/static/proto/uidata.proto`、`/static/proto/robottype.proto`：

| 消息 | 字段 | 类型 / 含义 |
|---|---|---|
| UiDatagram | current_state = 6 | RobotState 状态消息 |
| RobotState | pose = 1 | 原始定位平面位姿 |
| RobotState | timeStamp = 3 | 原厂报文时间，原样保留 |
| PlanarPose | x = 1、y = 2、theta = 3 | double；零坐标有效，非有限值拒绝 |

只提交上述只读 wire projection 和生成的 Python 绑定，不包含厂家 SDK、UI、
地图或实际轨迹文件。`control_speed` 不在投影中，也没有从它提取指令速度。
`chassis_pose_pb2.py` 可用 protobuf 4.25 系列 protoc 从 resource/chassis_pose.proto
重新生成；运行依赖沿用现有 protobuf>=4.21,<7，不增加运行时 SDK。

本体初次被动观察 8.1 秒接收 82 条 current_state，timeStamp 持续变化，位姿保持
不变。对应姿态也见于 RobotBase.locate 和底盘地图监控界面，因此按定位状态
处理，不据此宣称有轮编码器/惯导连续里程计。静止数据不构成运动精度验收。

## 原点与方向

源数据 `(x0,y0,theta0)` 的首个有效观测作为参考，随后计算：

```text
dx = (x - x0) * position_scale
dy = (y - y0) * position_scale
relative_x = cos(theta0)*dx + sin(theta0)*dy
relative_y = -sin(theta0)*dx + cos(theta0)*dy
relative_yaw = atan2(sin(theta-theta0), cos(theta-theta0))
```

不设置或修改底盘原点。停止卡片后再开始只清除本插件参考，不改变定位服务。
原始位置单位默认未确认，发布显示值时明确标注原始尺度。只有安装方确认 m/mm
后才换算标准米制 pose。输入缺失、断线、停止、过期时返回 null，不冒充零位。

## 平台格式

参考 [Driver README_dev 的 Robot Odometry](../../README_dev.md)
以及 [PR519](https://github.com/4paradigm/phanthymotus-driver/pull/519)
的 pose position/orientation、frame_id/child_frame_id 和独立时间戳结构。
统一输出调用 common.odom.build_interface/build_sample，不修改共享协议核心。

- 标准 schema 为 motus.odom/1，format 为 state/odom，载体为 std_msgs/String JSON。
- 没有速度能力：provides=[]，twist=[null,null,null,null,null,null]。
- frame=world 指本卡固定的初始参考系，pose.frame_id 为 <namespace>/relative_start。
- 已确认位置单位时 pose.position 为 {x,y,z:0}，orientation 为平面 yaw 的 XYZW
  四元数；pose_drift=unbounded，不为地图定位或尺度精度作有界误差保证。
- 原厂时间、Driver 接收时间分开保存。pose.stamp_ms 与顶层 stamp_ms 均为
  该观察的接收时间，info 声明 stamp_source=received，不伪造 source acquisition。
- 未确认单位时标准 pose=null，同时保留显示口上的原始相对数值。此处是生产者
  不把未知单位冒充米制数据。

## 验证范围

自动测试包括：真实 Protobuf 解析、未知字段兼容、零坐标、缺失 pose、非有限值、
初始朝向旋转、角度跨界、单位换算、未确认单位、停止/过期清空、重连、独立
生命周期、被动 WebSocket 无控制消息、标准 schema 和能力声明。

真机验收仍需分别记录：当前镜像版本、卡片发现和数据输出、停止再开始置零、
断线与恢复，以及在确认单位后的受控位移/转角对照。无受控运动数据时，不宣称
位移、角度精度或重定位连续性已验证。bot 镜像构建/正式部署和 Master 验收须
按仓库流程单独完成。
