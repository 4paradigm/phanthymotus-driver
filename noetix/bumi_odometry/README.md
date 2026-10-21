# Bumi 独立 RGB-D＋机身 IMU 融合里程计

实现提供 RTAB-Map `rgbd_odometry`、robot_localization EKF、只读 sensor
`rgbd_imu_odometry`，以及 loco_target 的连接选择。**代码与无硬件验证交付，
尚未构建 ARM64 镜像、运行真实 ROS 融合、部署或完成真机精度验收。**
外参/IMU 约定/读时同步资格尚待标定；新卡默认 initializing，不允许以其反馈开始移动。
纯 IMU 实验入口保留，不宣称一米 ±5cm 已达成。

## 安装分工

1. 更新 Bumi bundle 到本分支：它仍只有一个 pyrealsense2 pipeline，继续产生原 JPEG、
   压缩深度、内参和照片录像缓存。额外发布标准 raw Image/CameraInfo，机身 SDK 桥在
   `plugins.odometry_inputs.enabled` 控制下独立轮询。没有恢复 RTAB/EKF 安装到 bundle。
2. 单独安装 `noetix/bumi_odometry/driver.yaml` 对应的镜像/资源项，服务模板在
   `deploy/service.yml`。该服务无 USB、privileged 或 Docker socket 权限，只共享本机
   DDS domain 42 和既有本机隔离 profile。停止、重置、崩溃不会终止相机进程。
3. 应用 [平台补丁](platform/saved-feedback-binding.patch)，重新构建并部署平台。
   补丁基线为 phanthymotus main `6df601f612818716bb0dc7b180a9467e2c43172c`。
   本分支只交付补丁，没有改动其他平台工作区。应用前在独立平台分支执行
   `git apply --check`，通过后 `git apply`；版本不同必须解决适配后重跑平台测试。
4. 在画布中添加 sensor 并连 `state/odom` 到 `loco_target`。无连线用纯 IMU
   实验；连线后未就绪同步拒绝动作。旋转沿用既有算法和 ACP、急停、底盘互斥，
   距离控制直接调用 SDK，不经 loco_servo。

独立 sensor 的 MCP 端口是 15714，bundle 仍是 15704。服务通过原平台注册接口注册。
内部输入 namespace 在 DDS 中恰好一个时自动发现；多个来源必须用 sensor 的
`source_namespace` 配置选择。此发现仅用于 sensor 的原始输入，不用于决定距离反馈来源。

## 独立依赖与镜像构建

普通 Bumi 镜像不承担融合依赖。独立 Dockerfile 的 `dependencies` target 单独安装
Humble rtabmap-odom、robot-localization、TF、ROS 消息、OpenCV/NumPy 和 message_filters；
不安装 ROS RealSense、desktop 或 rviz。必要传递依赖保留。

`build-images.sh DEPS_IMAGE APP_IMAGE` 在有权限的 ARM64 buildx/镜像仓库环境执行，
会构建并推送依赖层，再按实际 digest 构建应用层，同时发布 registry cache。
本轮未执行该脚本或任何推送。也可通过仓库已有 build.sh 构建该独立 driver 目录，
Docker 层缓存可复用，应用源码变动不会使 apt 层失效。

发布需保存：依赖/应用 digest、`/opt/bumi-fusion-packages.txt`、构建元数据、冷/热耗时、
镜像大小、目标架构和 ROS 集成结果。当前没有可诚实填写的已验证 digest/ROS 包锁文件。
默认基础镜像 tag 仅供首次构建；正式发布把 FUSION_BASE_IMAGE 固定为已验证 digest。
任何首次安装成本均在独立镜像中完成，生产容器内不需手动 apt/pip 补包。

可选 `validation` target 额外包含 rosbag 记录/回放工具。默认 release 和 runtime
不包含这些记录工具；普通 Bumi bundle 不继承任一融合层。

## 输入与输出

bundle 共享输入（`NS` 为现有 bundle 的实际 namespace）：

| 话题 | 类型/语义 |
| --- | --- |
| /NS/odometry_input/rgb | sensor_msgs/Image，原始 bgr8 |
| /NS/odometry_input/depth | sensor_msgs/Image，RGB 网格的 32FC1 米制深度，使用实际 SDK depth_scale |
| /NS/odometry_input/info | sensor_msgs/CameraInfo，RGB 实时 K/D/R/P |
| /NS/odometry_input/camera_meta | std_msgs/String，RGB/depth 独立源时间、时间域、序号、接收时间、深度尺度、相机 epoch |
| /NS/odometry_input/imu | std_msgs/String，SDK 姿态/gyro/accel/原协方差、读取前后时间、poll 序号、source_stamp_ns=null |

RGB-D 按采集时间近似配对，差值不得超过 20ms。hardware_clock 使用显式设备到主机
近似映射，原时间和映射来源不丢失；重连/时间重置改变 epoch，要求重置估计段。
读取时间不是曝光时间。相机畸变当前支持 none/Brown-Conrady；未知模型拒绝融合，
不能贴上错误 plumb_bob 标签。sidecar 成对去畸变，深度 nearest 重采样。

融合只使用机身 IMU，不依赖旧容器中缺失的相机 Motion Module。SDK 没有已确认源时间，
读取耗时、poll 序号或数值变化都不能证明源采样年龄。本实现通过实测读时延迟资格配置
使用 host_read 近似，输出始终标记源时间不可用；无法消除“SDK 返回旧缓存”的固有限制。

RTAB-Map 输出机身参考点的三维局部位姿；同 stamp 的 OdomInfo 合格后，视觉 relay 才
送入 EKF。EKF 融合视觉六维 pose 和三轴 gyro，不双重使用其视觉差分速度，不用命令当
观测。加速度用于静稳重力初始化；本方案不启用未经验证的加速度双积分或把它用于失跟补走。
各段使用独立内部话题和 TF frame，仅 EKF 发布对应动态 TF，机身到光学/IMU 安装 TF
按实测值发布。`T_O_B = T_O_C inverse(T_B_C)` 在 RTAB 的 frame_id/TF 转换中完成，
不再重复手动修正视觉输出；数学杠杆臂回归作为其契约检查。

对外固定输出 `/bumi_rgbd_imu_odometry/state/odom`，类型 String、format `state/odom`。
复用 `motus.odom/1`：twist 在 body、SI 单位，pose 内明确局部 odom/机身 base_link。
`vendor` 同包携带 initializing/tracking/lost、segment/calibration、协方差、匹配/inlier、
输入年龄、时间来源、`accuracy_validated:false`。无有效估计时 pose=null、twist 各轴 null。
协方差是模型置信信息，不等于已验证的实际误差上界。

## 待标定配置

默认没有零外参示例文件，避免直接误用。可通过平台 sensor config 保存完整 `calibration`
对象；也可由部署配置管理将其放入挂载目录下 config.json 的 calibration 字段。
配置解析失败不启动融合。字段全部需要真实来源：

- calibration_id、fixed_mount=true。
- camera_to_body.translation（三米制分量）与 quaternion（归一化 xyzw）：**光学坐标 C 到 B**。
- imu_to_body.translation/quaternion：IMU I 到 B。
- imu_units 必须核实为 `m/s2,rad/s`，imu_quaternion 必须核实/转换为
  `xyzw_sensor_to_enu`，gravity_included=true。其他表示先在输入层适配，不能只改标签。
- gyro_bias（三轴，I 系）、gyro_variance（三轴，正数，有测量依据）。
- timing.mode=`host_read_qualified`、offset_s、uncertainty_s、qualification_id、max_age_s。
  corrected_stamp=读取区间中点-offset_s；当前不确定性上限 50ms、max_age 上限 0.5s。
  这两个上限是拒绝条件，**不是在此上限内就能达到 5cm 的保证**。

这里“sensor 到 body”明确指坐标数据变换：`p_B = R_B_S p_S + t_B_S`，
即 `T_B_S`；translation 是传感器原点在 B 中的位置，quaternion 将 S 中向量旋转到 B。
`header.frame_id=B`、`child_frame_id=S` 的 TF 直接存储这个值，不求逆。
TF 树的父子边方向不要与坐标数据变换方向混同；只有将 B 中数据转换到 S 时才使用
`T_S_B = inverse(T_B_S)`。例如相机绕 B 的 Z 轴 +90°、原点位于 `[0.4,0,0.2]`，
则 C 中点 `[1,0,0]` 在 B 中为 `[0.4,1,0.2]`。IMU gyro 是向量，只旋转、不加平移。
[ROS 2 Humble 的实际 PointStamped/Vector3Stamped 变换实现](https://github.com/ros2/geometry2/blob/humble/tf2_geometry_msgs/include/tf2_geometry_msgs/tf2_geometry_msgs.hpp)
将消息中的变换直接作用于数据，并将输出 frame 设为 header.frame_id。
`test_launch_tf_maps_nonidentity_sensor_coordinates_into_body` 检查 launch 发出的实际 TF；
这仍是替换 ROS 传输和子进程的无硬件回归，未代替真实 TF buffer/ROS 启动验证。

固定头部还不够：现有 URDF 有 waist_yaw_joint，却未给出真实相机/IMU 安装链。
现场必须明确 B 的实物参考点，确认是否跨可动腰关节。没有有效动态 TF 时须保持对应
关节固定；不能把骨架渲染模型当实测外参。

测量步骤：静止多姿态核对重力与 quaternion 正反方向；已知轴正负转动核对 gyro 单位、
轴向；60 秒静止估计零偏/方差；测量相机与 I 相对同一 B 的平移/旋转；比较转动时相机
圆弧与机身点实际位移。若今后启用线加速度，还需验证重力、振动、向心/角加速度杠杆臂
与姿态误差，不能直接勾选 EKF acceleration。

`timing.py` 可离线比较**原始视觉**体角速度与 IMU 读时角速度，搜索固定延迟并检查分段
一致性。静止、恒速、时间乱序或低激励记录会拒绝。输出不是源采样时钟，只是本次条件
下的读时近似资格材料，还需要现场复核；不能拿融合 EKF 自身输出反证输入延迟正确。

## 保存图连接契约

平台补丁读取持久化 connections，校验源卡是 rgbd_imu_odometry、端口是 state/odom，
将源卡/端口/连接 id/目标卡/话题编码进 revision。无边、无法解析和多边明确区分；
持久化旧话题与设备声明冲突时拒绝。启动、保存布局、应用方案、运行心跳走同一解析器。
复用现有 MCP 生命周期，无新增未鉴权接口。普通卡片没有 `x-feedback-binding` 声明，
不走该扩展。

连接租约 500ms，平台正常每 100ms 轮询（每次派发上限 200ms），驱动独立 20ms
watchdog 监控。正常换线派发立即撤销旧动作；派发/平台失联最迟在租约耗尽后请求零速度。
这些是软件检测预算，调度抖动和物理刹停时间必须另测，不能声称硬实时“瞬间停稳”。
每个非零距离命令都在同一绑定锁下重新检查来源/取消，已派发的旧订阅回调带 generation，
不能填入替换连接。每次动作固定 revision、source、segment、calibration 和相对起点。

随本分支交付的 config.yaml 默认 loco.require_feedback_binding=false，未连接时无需
本 sensor 或平台补丁即可使用纯 IMU 实验闭环。也可显式设 true，要求平台先确认空输入。
一旦收到保存图绑定（包括空输入）即启用租约；未托管模式不提供保存图保证。
给 start 手传一个 input_topic 不构成受支持
的连接选择，会拒绝。**新 sensor 距离反馈需要配套平台补丁**。已经进入托管连接模式
以后，缺绑定/失联会拒绝动作，不能偷偷退回旧模式。

## 无硬件验证

仓库根目录运行 `python3 -m pytest -q tests/bumi_fusion`。涵盖几何、静止窗口、延迟拟合、
原始输入尺度、质量锁存、初始化/重置、进程失败回收、保存图实际传递到消费者、旧回调、
来源锁定与故障制动。既有 Bumi/媒体/旋转回归命令与结果见 VALIDATION.md。

`python3 -m noetix.bumi_odometry.simulate_control --output report.json`
独立保存模型真值、估计和目标。nominal 模型和 15% 里程计尺度误差分别运行，后者可以
估计 done、实际终点偏离约13.5cm，刻意证明 done 不等于精度验收。该仿真不执行 RTAB。

实际 RTAB/EKF 无硬件集成命令（在构建好的镜像中、独立 domain）：

```
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=239 python3 /work/ros_probe.py --output /data/rendered-probe.json
```

探针渲染固定纹理墙的 RGB/真实米制深度，给出有非零安装平移的独立机身运动真值和
机身 IMU，真正启动 RTAB/EKF；比较静止与一米移动、断输入后的 lost、重置段变化。
只读取/估计，不发机器人命令。**本轮未执行该 ROS 探针。**

## 真机验证顺序

1. 只读记录当前镜像 digest、固件、SDK、namespace、相机唯一所有者、topic/TF/QoS、
   CPU/RAM/温度/媒体帧率。不打开第二个 pipeline，不为检查 Motion Module 改生产相机。
2. 部署新版 bundle 原始共享分支、独立 sidecar 和配套平台补丁；先不下发运动。
   缺标定显示 initializing、已连接 move 拒绝；没有连接仍走原 IMU 实验。
3. 记录相机源域、映射漂移、RGB-D skew、IMU 读取/到达延迟、连续更新与抖动。
   完成安装/IMU/时延资格测量并冻结 calibration_id。静止60秒、已知转动、手动直行对照。
4. 在 validation 镜像/独立记录服务录包，使用**实际 NS**替换正则中的 NS：
   `ros2 bag record --regex '^(/NS/odometry_input/.*|/bumi_fusion_.*/.*|/bumi_rgbd_imu_odometry/state/odom|/tf|/tf_static)$'`
   同时保存标定、依赖锁、配置和元数据。记录需完整覆盖结束后至少1秒的晚到诊断。
5. 在独立域回放时只回放原始五输入话题，不回放旧 visual/filtered/status/dynamic TF。
   新估计服务使用 use_sim_time=true（ROS参数）、相同标定，每次启动新进程、新段。
   自己重建对应静态安装 TF；两个重算不能并发使用同一段名字。重复两次并比较轨迹。
6. `bag_export.py BAG --poses trial.csv --visual-rates visual.csv --imu-rates imu.csv --calibration measured.json`
   在 validation 镜像导出。delay 分析用 `timing.py --visual-csv visual.csv --imu-csv imu.csv
   --data-source measured --output timing.json`。一次 bag 只算一次试次，输出拒绝覆盖。
7. 现场独立测量**同一机身 B 点**起止位移与不确定度，冻结参数后至少20个独立约1米
   留出试次、两个场景、不同朝向与拟用速度。失败/失跟全部计入。
   复用 experiments/bumi_odometry/evaluate.py 的 measured manifest 判定：
   abs(估计前向-独立实测前向)+测量不确定度≤0.05m。指令1米不是实测真值。
8. 单独进行自动距离制动验收，比较目标/估计/真实终点；中途遮挡、断输入、kill sidecar、
   reset、换线、取消和急停均应停车且报错，不自动切 IMU。记录零命令延迟和真实停止距离。
9. sidecar 停止/崩溃/重置期间逐项核查 JPEG/深度/内参、拍照录像、麦克风、声源方向、
   扬声器；再回归旋转、底盘互斥与一次 ACP 终态。记录新增原始流带宽和负载的实际影响。

任何未执行的项目均保持未验证，不将历史数学测试、启动成功或渲染探针通过写成硬件
精度达标。需要回滚时停用 sensor 连线并停止独立服务，保留原 IMU 实验；不修改相机所有者。


## Compatibility sync with PR #408 (2026-10-21)

This branch incorporates #408 commit `782255e8` and its predecessor direction/trace
and official-Ubuntu-archive changes, resolving shared files to the #408 superset.
Merge #408 first, then this PR; no #425 implementation was merged into #408.
The sensor stays independently startable without any destination card. The Bumi
bundle stays independently usable for unconnected experimental IMU distance.
Receiving a saved binding activates lease requirements; legacy topic-only
connections are rejected rather than silently selecting IMU.

The full platform patch/overlay here now includes the exact-lease/topic ACK,
config-time saved-graph resolution, live-port validation and failed-ACK stop
changes. Apply this full patch alone; the #408 supplemental patch is only for the
older #425 `c48692f8` artifact. Neither platform patch has been deployed.
See [shared interfaces and ownership](../../integration/bumi_odometry/README.md)
and [validation](../../integration/bumi_odometry/VALIDATION.md). Calibration gates,
RTAB-Map/EKF parameters and source-timestamp limitations are unchanged.
