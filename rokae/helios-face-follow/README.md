# Helios 单卡目标跟踪

`helios_face_follow` 通过独立服务注册到 Core。在画布把 Helios 的
`camera_rgb` 连到卡片的 JPEG 输入口。车身朝向模式使用胸部 RGB。启动后使用
`set_mode` 或 `set_target` 修改目标，`info` 查看状态。监控可以同时显示
原始相机和卡片的 `data/json` 目标输出。

- `person` 是默认模式，卡片启动即开始检测。同一 RGB 流上的 VOP 检测人体，
  `visual_depth` 估算人体区域的距离。画面只有一个人体时可直接跟踪。画面有多人时选取
  距离估计最小的目标，距离数据不足时等待，画布会显示 `waiting_for_person_depth`。
  单目深度默认使用模型估算，`distance_estimate_mm` 是估计值。此模式不加载人脸检测器。
- `nearest_face` 从人脸内区的深度样本取中位数，选择最近的人脸。输入必须是与
  640×480 RGB 对齐的 `image/depth-zlib`，共享光学坐标系，时间差不超过
  85 ms。通过 `depth_topic` 选择已配准的流。
- `speaker` 需要已校准的说话人数据源。默认输入为
  `/rokae_helios/speech/localized`，类型是可靠 QoS 的 `std_msgs/msg/String` JSON。
  字段为 Unix 秒 `timestamp`、非空 `utterance_id`、布尔 `active`，
  以及发声时的 `image_x`（图像右侧为正，范围 [-1,1]）。
  发声更新间隔须小于 500 ms。当前发言人保持锁定直到同一个
  `utterance_id` 的结束事件。声源断流或目标丢失时清除目标输出。
- `set_target(object_name="cup")` 选择 VOP 类别。VOP 必须用同一个 RGB
  相机，默认检测话题是 RGB 话题后缀 `/objects`。默认话题无发布者时，
  卡片请求 Perception 启动 VOP。大脑可用 `set_target` 切换类别。

卡片输出目标、归一化偏差、头部关节反馈及 `motion.state`。
`motion_available` 表示控制后端接受了当前运动指令。
在这台机器上，RGB 卡的默认预览报告 640×400，深度为 640×480 原生光学图。
两路没有可用的配准声明，最近人脸模式需要已验证的 RGB-D 配准输入。当前机器的麦克风服务尚未
确认能够发布逐句、相机坐标下的说话人位置。

独立镜像通过构建目标 `rokae/helios-face-follow` 生成。

## 车身朝向

在本机部署配置中将 `HELIOS_BASE_FOLLOW_ENABLED` 设为 `1`，重启卡片服务，在画布将
Helios `camera_rgb` 配置为「胸部 RGB（G300）」并连接到卡片。该设置优先选择底盘控制，
默认为 `0`。卡片使用现有 `base_drive` 的本机 MCP（15782）和 `controlled_spatial` 的
导航状态（15881）。各跟踪模式共用同一套转向逻辑，分别依赖各自的传感输入。

卡片运行期间一直跟踪，没有会话时长或累计转角限制。连续看到同一目标后，卡片按水平偏差
调整转速和每段转向时间（0.35–1.2 秒）。每段结束后根据新观测继续校正，靠近中心时减速。
`vx` 和 `vy` 始终为 0，非零偏航速率在 3.5–8°/s。
归一化水平偏差绝对值降到 0.07 时停转，离开中心超过 0.12 时恢复。
胸部相机声明不符、导航或其他底盘动作占用、底盘异常、里程计过期或底盘未停稳时，
卡片保留视觉输出。目标丢失、移入死区、换目标、切换模式或停止卡片时，卡片尝试取消
自身仍在运行的底盘动作。正常转向期间仍要求新鲜相机和目标数据。底盘在持续看到目标时有
最多 3 秒的额外启动时间，超出该时间与当段转向时间之和会请求取消并报告 `fault`。
监控中的 `motion` 显示状态、转向动作 ID 及请求角速度。

目前底盘驱动在车轮唤醒后才开始计算 `duration_s`，取消接口也不支持按 ID 原子取消。
卡片存活时会复核动作 ID 并监督取消。首次实车使用应由现场人员在开阔区域验证左右
转向和停转，并在测试期间独占底盘，保持遥控、导航和其他上位机底盘控制空闲。

## 头部控制接口

卡片最多每 100 ms 用一次新目标和最近的关节反馈生成双轴绝对位置。归一化偏差
在中心死区内时释放控制。出界反馈、目标丢失、RGB 或关节数据超过 350 ms、切换
模式或停止卡片时释放控制。软件速度上限为 0.15 rad/s，并且遵守适配器返回的
更严格的限速和软限位。适配器拒绝位置或无法确认停止时，`motion.state` 报告
`fault`。停止并重新启动卡片可清除已经确认释放的故障。

默认部署仅跟踪目标。完成 iNex SDK 适配后，在部署配置中只读挂载包含 socket 的
目录，并将 `HELIOS_HEAD_CONTROL_SOCKET` 设为容器内的绝对 socket 路径。例如将
`/run/helios-head-follow` 挂载到容器的同一路径，并设置
`HELIOS_HEAD_CONTROL_SOCKET=/run/helios-head-follow/control.sock`。
Socket 目录应由适配器管理，只允许授权的本机客户端连接。

适配器协议使用 Unix stream socket。每条连接接收一条以换行结尾的 UTF-8 JSON，
并在 50 ms 内返回一条以换行结尾、最多 4096 字节的 JSON。每条消息都带
`version: 1`。操作失败时返回 `ok: false`。

| `operation` | 请求字段 | 成功响应与语义 |
| --- | --- | --- |
| `acquire` | `lease`（卡片生成的唯一 ID）、`camera_topic`、`ttl_ms` | `ok: true`、`camera_topic` 和 `axes`。适配器核对已标定的相机话题、厂商控制权限及头部状态，保留独占租约。 |
| `position` | `lease`、`ttl_ms`、`valid_until_monotonic_ns`、`positions_rad`、`max_rad_s` | 适配器在 SDK 接受限速位置请求后返回 `ok: true`。使用同一 Linux 单调时钟，应在有效期前处理，并令控制在采样有效期和租约到期两者中的较早时刻失效。 |
| `halt` | `lease` | 停止该租约的运动并释放控制，返回 `ok: true`。已到期或已释放的租约也须能重复确认，且该操作不能影响其他租约。 |

`axes` 以 `head_joint_1`、`head_joint_2` 为键，每轴提供 `lower_rad`、
`upper_rad`、`max_rad_s`、`image_axis`（`x` 或 `y`）和 `image_sign`
（`1` 或 `-1`）。图像 x 向右、y 向下为正，适配器提供实机标定后的关节映射。
适配器必须独立执行急停、控制权、命令有效期、限位和失联停止，并保留厂商联锁。
当前真机只有已验证的 iNex 关节反馈，仍需用适用于该固件的厂商 SDK 实现并验证
上述适配器，才能启用电机随动。
