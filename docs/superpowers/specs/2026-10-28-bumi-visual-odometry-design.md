# Bumi 独立 RGB-D 视觉里程计（只读）

状态：实施设计。代码、打包与文档交付，**未构建 ARM64 镜像、未在真实 ROS 图中运行、
未部署、未做真机精度验收**。默认状态 `initializing`，不参与底盘控制。

## 基线与范围

基线提交 `e40ca380`（PR #408 合并后的 `upstream/main`）。本功能从已关闭的
PR #425 恢复，**只保留 RTAB-Map RGB-D 视觉里程计这一层**：`rtabmap_odom` 的
`rgbd_odometry` 是唯一的估计器，只读发布 `motus.odom/1` 位姿。

不恢复的部分：#425 中的机身 IMU 桥接、robot_localization EKF 松耦合、视觉与
gyro 的融合参数、IMU 零偏/重力初始化、读时延迟资格（host-read timing）、
距离反馈/保存图连接绑定补丁，以及 `loco_destination` 对该 sensor 的任何消费。
这些不是“以后再打开的参数”，而是本设计里不存在的数据通路和依赖包。

因此本设计的质量契约只有一句话：**相机跟踪产生了位姿，且它自己的诊断与位姿一致**。
没有第二路观测可以纠正它，也没有滤波器能在视觉中断后继续“预测”出位姿。

不含回环与全局定位，不承诺漂移有界；观测帧率约 1.8 Hz（历史记录，本轮未复测），
这一段位姿只适合查看、录包与后续离线评估，不适合作为运动闭环的唯一反馈。

## 与 #425 的取舍

| 项目 | #425 | 本设计 |
| --- | --- | --- |
| 估计器 | RTAB-Map + robot_localization EKF | 仅 RTAB-Map `rgbd_odometry` |
| 输入 | RGB、深度、内参、camera_meta、机身 IMU | 前四项，无 IMU |
| 时间 | 读时延迟资格、IMU 源时间近似 | 仅相机源时间域映射与传输年龄 |
| 输出 | EKF 融合位姿 + 视觉诊断 | 视觉位姿 + 视觉诊断 |
| 消费 | loco 距离反馈、保存图绑定 | 无消费者，只读查看 |
| 依赖 | rtabmap-odom、robot-localization、tf2 | rtabmap-odom、tf2、rclpy、numpy；消息包随 rtabmap 依赖，runtime 无 OpenCV/PyYAML |
| 标定 | 相机/IMU 外参 + IMU 约定 + 时间资格 | 仅可选相机安装（camera_to_body） |
| 质量状态 | 视觉 + 诊断 + IMU + 滤波联合判定 | 视觉 + 诊断联合判定 |

## 进程与安装边界

保留 Bumi bundle 的唯一 pyrealsense2 采集进程。bundle 侧新增**可选、只读**的原始
发布分支（标准 `sensor_msgs/Image` bgr8、32FC1 米制深度、`CameraInfo` 与
`std_msgs/String` 相机元数据），沿用原有 JPEG/压缩深度/内参/录像缓存；该分支由
`device.py` 的 `_camera_subprocess` 调用 `visual_inputs.py`，构造与每次发布都包在
try/except 里，故障只打日志，绝不影响媒体生产。原始分支与 sidecar 都不打开第二个
相机 pipeline。

视觉里程计 sidecar 是独立驱动目录 `noetix/bumi_visual_odometry/`：

- 独立镜像 `noetix-bumi-visual-odometry`，`dependencies` 阶段只安装视觉所需包，
  包集合锁定到 `/opt/bumi-visual-packages.txt`；应用层按已验证 digest 通过
  `VISUAL_BASE_IMAGE` 复用依赖层。
- 服务容器 `embodied-noetix-bumi-visual-odometry`：`network_mode: host`、
  `ipc: host`、只读挂载本机 DDS 隔离 profile，数据目录
  `/opt/phanthy-motus/data/bumi_visual_odometry`，`ROS_DOMAIN_ID=42`，
  `restart: always`。**不挂 USB、不 privileged、不挂 Docker socket**，与相机
  进程生命周期无依赖。
- 独立 MCP 端口 15720；bundle 仍为 15704。sidecar 停止/重置/崩溃只回收自己的
  RTAB-Map 子进程与内部话题。

## 输入

bundle 共享的只读输入（`NS` 为 bundle 实际 namespace，由 sidecar 自动发现或
`source_namespace` 指定）：

| 话题 | 类型/语义 |
| --- | --- |
| `/NS/odometry_input/rgb` | `sensor_msgs/Image`，原始 bgr8，光学坐标系 frame |
| `/NS/odometry_input/depth` | `sensor_msgs/Image`，与 RGB 对齐的 32FC1 米制深度 |
| `/NS/odometry_input/info` | `sensor_msgs/CameraInfo`，实时 K/D/R/P |
| `/NS/odometry_input/camera_meta` | `std_msgs/String` JSON：epoch、RGB/深度各自源时间/帧号/时间域、depth_scale、接收时间与时钟映射 |

接收侧门限（拒绝条件，不是精度保证）：

- RGB 与深度源时间差 ≤ 20 ms；RGB-D 与 CameraInfo 必须来自同一对齐帧。
- 设备时间域必须可识别（`hardware_clock`/`system_time`/`global_time`），映射到
  ROS wall time 后传输年龄 ≤ 250 ms；超过 500 ms 未更新即输入过期。
- 深度比例来自 SDK `depth_scale`，不假设单位恒为毫米；无法用深度像素占比过低即拒绝。
- 畸变当前支持无畸变或 `plumb_bob`（Brown-Conrady）：sidecar 成对去畸变、
  深度 nearest 重采样；未知模型拒绝，不把错误标签贴到数据上。
- 时间重置、epoch 变化、光学 frame 变化、内参变化都要求重置估计段（新 segment）。

bundle 的相机源时间与读时延迟限制保持公开：SDK 没有可确认的曝光时间，映射是
到达时间近似，不能证明 SDK 未返回过期缓存；本设计不因此引入 IMU 或滤波来“兜底”。

## 坐标与参考点

RGB 光学坐标系 C：x 右、y 下、z 前。默认没有实测安装外参，所以
`vendor.pose_reference = "rgb_optical"`，位姿直接表达在相机光学坐标系，参考点
就是相机原点——**不假设任何安装平移为零**，也不把未测量的相机到机身变换写成 0。

可选 `calibration` 只做一件事：把同一段视觉轨迹从光学参考点换算到机身参考点 B。
字段全部需要真实来源，解析失败则拒绝启动：

- `calibration_id`、`fixed_mount: true`（固定安装是本功能的使用条件）。
- `camera_to_body.translation`（三米制分量）与 `camera_to_body.quaternion`
  （归一化 xyzw，光学 C 到机身 B）。

这组值直接作为每段私有的 base→光学 frame 静态 TF 发布：RTAB-Map 在估计前用该 TF
把图像变换到 base 参考点，转动杠杆臂在估计器内部处理，runtime 不做二次变换，也不是
融合。配置有效时 `vendor.pose_reference = "body"`。没有标定时发布 identity TF
（base 与光学 frame 重合），并如实标 `rgb_optical`，不声称未测量的机身外参为零。
没有 IMU 字段、没有时间字段：本设计不接受“IMU 约定”或“读时资格”类配置。

`bumi_model.urdf` 存在 `waist_yaw_joint` 且未给出相机安装链，固定头部不自动证明
相机相对 base_link 刚性固定。B 是哪个实物参考点、是否跨腰关节必须现场确认；
跨关节时需锁定该关节，不能用渲染骨架 URDF 代替安装测量。

## 输出契约

固定输出 `/bumi_visual_odometry/state/odom`，`std_msgs/String`，port format
`state/odom`，负载复用 `common/odom.py` 的 `motus.odom/1`：

- `schema: "motus.odom/1"`、`stamp_ms`、`units`。
- `pose`：仅在 `tracking` 段存在，为
  `{position:[x,y,z] m, quaternion:[x,y,z,w] xyzw, frame_id, child_frame_id, covariance:[36]}`；
  `frame_id` 为内部段 odom（`bumi_visual_<uuid>_odom`），`child_frame_id`
  始终为段 base frame（`bumi_visual_<uuid>_base`）；无标定时 base 与光学 frame
  由 identity TF 重合（`pose_reference=rgb_optical`），有实测安装时该 TF 用实测值
  （`pose_reference=body`）。未 tracking 时 `pose` 为 `null`，不保留上一帧位姿。
- `twist`：**始终 `[null] × 6`**。RTAB-Map 的 odometry 消息给的是轨迹帧间的位姿
  变化，不是实测机身速度；补一个由位置差分算出的速度等于伪造测量，宁可为空。
  `frame` 按格式必填，取 `body`，但在没有 twist 的情况下它只是格式要求。
- `vendor`：`status`（`initializing`/`tracking`/`lost`）、`reason`、`segment_id`、
  `calibration_id`、`pose_reference`（`rgb_optical` 默认，`body` 仅在有实测
  `camera_to_body` 时）、`visual_diagnostic: {inliers, matches, lost}`、`input_age_ms`
  （按相机/视觉/诊断分项）、`visual_stamp_ns`、`camera_time_provenance`、
  `source: "rtabmap_rgbd_odometry"`、`accuracy_validated: false`、`twist_source: null`、
  `published_ns`。
- 状态卡 `info()` 的 `odom_interface` 为 `provides: []`（不测任何速度轴）、
  `pose_drift: "unbounded"`、`rate_hz: 20`，并回报 `pose_reference`、`segment_id`、
  `calibration_id` 与当前 `quality`。

## 质量状态与重置

- `initializing`：传感器停止、未发现唯一输入 namespace、等待 RGB-D 与诊断配对、
  或首个有效估计尚未产生。默认状态；不产出位姿。
- `tracking`：连续输入有效、视觉与 OdomInfo 按同一源 stamp 配对、inliers ≥ 10
  且 matches ≥ inliers、协方差合法（位置标准差 ≤ 0.2 m）、位姿跳变在
  `0.03 m + 1.5 m/s × dt` 以内。
- `lost`：视觉失跟（OdomInfo `lost` 或 inliers 不足）、输入过期、时钟回退或重置、
  光学 frame/内参变化、RTAB-Map 子进程退出、视觉接收 watchdog 超时。**lost 在本段
  永久锁存**：晚到的正常消息不能把同一段修回 `tracking`。

每段建立时向 `/tf_static` 发布以 segment 命名的 base→光学 frame 变换（identity 或
实测安装）；RTAB-Map 的 `publish_tf` 保持 false，不与其他 TF 发布者争用。

`reset` 顺序：先发布 `lost` 与原因 → 结束旧段并丢弃旧输入/旧估计 → 停止并重建
RTAB-Map 子进程与新内部话题 → 新 segment UUID → 重新配对后进入 `tracking`。
段间是坐标重置，不拼接偏移；跨段消息一律拒绝。旧段队列不会进入新估计器。

## 明确非目标

- 无 IMU 输入、无 robot_localization/EKF、无视觉-惯性融合、无加速度积分。
- 不用视觉里程计做距离反馈、不做保存图连接绑定、不改 `loco_destination`。
- 不改 `motus.odom/1` 格式，不新增第五个里程计方言；视觉专属字段全部进 `vendor`。
- 不接管相机所有权，不安装/启动 ROS RealSense 节点，不打开第二个 pipeline。
- 不含回环/全局重定位，不宣称漂移有界，不宣称 ≥5 cm 精度已达成。

## 验证与交付门槛

1. ROS-free 单测：健康状态机、相机时间映射与重置、安装外参换参考点、样本构造
   （`twist` 全 null、未 tracking 时 `pose=null`）、输入拒绝条件、子进程回收。
2. 打包一致性：Dockerfile 中每个 `COPY` 源文件真实存在、相邻模块都被复制、
   依赖检查只验证 `rtabmap_odom/rgbd_odometry`、无 robot-localization 安装；
   `deploy/service.yml` 为有效 YAML 且 `restart: always`。
3. 目标环境 ROS 集成（未执行）：`ros_probe.py` 渲染固定纹理墙的 RGB/米制深度
   与独立真值运动，真实启动 `rgbd_odometry`，比较静止、一米移动、断输入后的
   `lost`、重置段变化。只读取/估计，不发机器人命令。
4. 真机只读核查（未执行）：记录镜像 digest、固件、SDK、namespace、相机唯一
   所有者、topic/TF/QoS、CPU/RAM/温度/实际共享流帧率；确认媒体链路未受影响。
5. 真机精度（未执行）：validation 镜像录包，独立域用 `use_sim_time` 只回放输入，
   两次重算比较轨迹；再对同一机身参考点做留出试次。历史 5 cm 目标不因本设计恢复
   而自动成立，仍按独立测量与不确定度判定。

任何未执行项保持未验证，不把单测通过、镜像构建成功或探针渲染通过写成精度达标。

## 上游依据

- [RTAB-Map RGB-D 里程计节点与参数](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_odom/src/nodelets/rgbd_odometry.cpp)
- [OdomInfo 诊断定义](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_msgs/msg/OdomInfo.msg)
- [RealSense 投影与深度尺度](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0)

这些是 2026-10-22 查询的上游分支资料，不是已验证的部署锁文件；发布前必须锁定
实测兼容的包版本/镜像 digest，并用该版本重做同一套集成检查。
