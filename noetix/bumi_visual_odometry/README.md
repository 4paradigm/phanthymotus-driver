# Bumi 独立 RGB-D 视觉里程计（只读）

提供 RTAB-Map `rgbd_odometry` 与只读 sensor `rgbd_visual_odometry`。**只做视觉里程计：
无 IMU、无 robot_localization EKF、无视觉-惯性融合、无距离反馈/保存图绑定，也没有接入
`loco_destination`。** 代码、打包与文档交付，尚未构建 ARM64 镜像、未在真实 ROS 图与
真机运行、未做精度验收。新卡默认 `initializing`，位姿只供查看与录包，不参与任何运动。

## 安装分工

1. 更新 Bumi bundle 到本分支：它仍只有一个 pyrealsense2 pipeline，继续产生原 JPEG、
   压缩深度、内参和照片录像缓存。额外由 `device.py` 的 `_camera_subprocess` 调用
   **可选只读分支**（`visual_inputs.py`）发布标准
   `Image`/`CameraInfo`/相机元数据；该分支任何异常只打日志，不影响媒体生产，也不打开
   第二个相机 pipeline。没有恢复任何 RTAB/EKF 安装到 bundle。
2. 单独安装 `noetix/bumi_visual_odometry/driver.yaml` 对应的镜像/资源项，服务模板在
   `deploy/service.yml`。该服务无 USB、privileged 或 Docker socket 权限，只共享本机
   DDS domain 42 和既有本机隔离 profile。停止、重置、崩溃不会终止相机进程。
3. 本功能**不需要**平台补丁、不需要改动 `loco_destination`，也没有距离反馈消费者。
   在画布中添加 `rgbd_visual_odometry` sensor 只用于查看 `state/odom`；把它的输出接到
   任何底盘动作卡都不是本驱动支持的用法。

独立 sensor 的 MCP 端口是 15720，bundle 仍是 15704。服务通过原平台注册接口注册，
工具名为 `rgbd_visual_odometry`，支持 `start`/`stop`/`info`/`reset`/`config`。
配置保存在 `/opt/phanthy-motus/data/bumi_visual_odometry/config.json`（可用
`CONFIG_PATH` 覆盖），`MCP_PORT` 默认 15720。内部输入 namespace 在 DDS 中恰好一个时
自动发现；多个来源必须用 sensor 的 `source_namespace` 配置选择。

## 独立依赖与镜像构建

普通 Bumi 镜像不承担视觉里程计依赖。独立 Dockerfile 的 `dependencies` target 单独安装
Humble `rtabmap-odom`、`rtabmap-msgs`、TF、ROS 消息、`message_filters`、OpenCV/NumPy 和
YAML；**不安装 robot-localization，不安装 ROS RealSense、desktop 或 rviz**。依赖集合
锁定到 `/opt/bumi-visual-packages.txt`，构建时只验证
`/opt/ros/humble/lib/rtabmap_odom/rgbd_odometry` 可执行。

构建分两阶段：先构建 `--target dependencies` 并推送，再用其 registry digest 作为
`VISUAL_BASE_IMAGE` 构建 `runtime`/`release`。可按仓库已有 `build.sh` 构建该独立
driver 目录（`./build.sh noetix/bumi_visual_odometry`），Docker 层缓存可复用，应用源码
变动不会使 apt 层失效。该目录在 `driver.yaml` 里用可选的 `image_repository:
noetix/bumi-visual-odometry` 把自己的镜像与 bundle 的 `noetix/bumi` 仓库分开；否则同一
提交下两者会共用 `${REGISTRY}/${IMAGE_NAMESPACE}/noetix/bumi:<TAG>`，后构建的会覆盖前者。

本轮未执行构建或推送，因此没有可诚实填写的已验证 digest。默认基础镜像 tag 仅供首次
构建；正式发布把 `VISUAL_BASE_IMAGE` 固定为已验证 digest，并保存依赖/应用 digest、
`/opt/bumi-visual-packages.txt` 与构建元数据。可选 `validation` target 额外包含
rosbag2 记录/回放工具；默认 `release`/`runtime` 不包含，普通 Bumi bundle 不继承该层。

## 输入

bundle 共享输入（`NS` 为现有 bundle 的实际 namespace）：

| 话题 | 类型/语义 |
| --- | --- |
| `/NS/odometry_input/rgb` | `sensor_msgs/Image`，原始 bgr8，`NS_rgb_optical` 光学 frame |
| `/NS/odometry_input/depth` | `sensor_msgs/Image`，RGB 网格的 32FC1 米制深度，使用实际 SDK depth_scale |
| `/NS/odometry_input/info` | `sensor_msgs/CameraInfo`，RGB 实时 K/D/R/P |
| `/NS/odometry_input/camera_meta` | `std_msgs/String` JSON：epoch、RGB/深度独立源时间/帧号/时间域、`rgb_stamp_ns`/`depth_stamp_ns`、depth_scale、接收时间、`clock_mapping` |

没有 IMU 输入：`/NS/odometry_input/imu` 不被订阅，sidecar 甚至为不存在的 IMU 预留了
内部 remap，避免隐式订阅到同名活话题。RGB-D 按采集时间近似配对，差值不得超过 20ms；
设备时间域必须可识别，映射到 ROS wall time 后传输年龄不得超过 250ms，任一输入超过
500ms 未更新即过期。读取时间不是曝光时间：`hardware_clock→host` 映射是到达时间近似，
不能证明 SDK 未返回过期缓存，这个限制保持公开且不再用 IMU 或滤波掩盖。

RGB 与深度必须同尺寸、同光学 frame、同为对齐后帧；CameraInfo 与 RGB `stamp` 必须一致。
当前只支持无畸变或 `plumb_bob`（Brown-Conrady）：sidecar 成对去畸变、深度 nearest
重采样；其他模型拒绝启动该段，不能贴上错误标签。时间重置、epoch 变化、光学 frame 或
内参变化都要求重置估计段。

## 输出契约

对外固定输出 `/bumi_visual_odometry/state/odom`，类型 `std_msgs/msg/String`，port format
`state/odom`。负载复用 `common/odom.py` 的 `motus.odom/1`（示例为 tracking 段，
`covariance` 做了截断，其余字段完整）：

```json
{
  "schema": "motus.odom/1",
  "stamp_ms": 1758537600123,
  "frame": "body",
  "units": {"linear": "m/s", "angular": "rad/s", "length": "m"},
  "twist": [null, null, null, null, null, null],
  "pose": {
    "position": [0.12, -0.03, 1.02],
    "quaternion": [0.002, -0.71, 0.0, 0.70],
    "frame_id": "bumi_visual_<uuid>_odom",
    "child_frame_id": "bumi_visual_<uuid>_base",
    "covariance": [0.001, 0.0, "…其余 34 项…"]
  },
  "vendor": {
    "status": "tracking",
    "reason": null,
    "segment_id": "bumi_visual_<uuid>",
    "calibration_id": null,
    "pose_reference": "rgb_optical",
    "visual_diagnostic": {"inliers": 48, "matches": 90, "lost": false},
    "input_age_ms": {"camera": 63.2, "visual": 104.0, "diagnostic": 104.0},
    "visual_stamp_ns": 1758537600000000000,
    "camera_time_provenance": {"epoch": "<hex>", "clock_mapping": "device_to_host", "depth_scale": 0.001},
    "source": "rtabmap_rgbd_odometry",
    "accuracy_validated": false,
    "twist_source": null,
    "published_ns": 1758537663123000000
  }
}
```

契约要点：

- `pose` 可选；未 `tracking` 时为 `null`，**不保留上一帧**。`position` 米、`quaternion`
  xyzw、`covariance` 为 36 项 pose 协方差。
- `twist` **始终**为六个 `null`。RTAB-Map 的 odometry 消息是轨迹帧间位姿变化，
  不是实测机身速度；用位置差分补一个速度等于伪造测量。`frame` 按 `motus.odom/1`
  必填取 `body`，但没有 twist 时它只是格式要求。
- `child_frame_id` **始终**是本次估计段的 base frame（`bumi_visual_<uuid>_base`）。
  位姿参考点由 `vendor.pose_reference` 明确：默认 `rgb_optical`——没有实测安装外参时，
  该 base frame 与光学 frame 之间只发布一条 **identity 静态 TF**，两者重合，位姿就是
  相机位姿，而不是声称未测量的机身外参为零；配置了实测 `camera_to_body` 后，同一条
  base→光学静态 TF 使用实测值，`pose_reference` 变为 `body`。
- `vendor.status` 只有 `initializing`/`tracking`/`lost` 三态，`vendor.source` 固定
  `rtabmap_rgbd_odometry`，`accuracy_validated` 固定 `false`。
- 状态卡 `info()` 的 `odom_interface` 为 `provides: []`（不测任何速度轴）、
  `pose_drift: "unbounded"`、`rate_hz: 20`，并回报 `pose_reference`、`segment_id`、
  `calibration_id` 与当前 `quality`。`provides: []` 是刻意的：本 sensor 不报告任何
  速度轴，消费者不应把它当测速源。

## 质量状态与重置

- `initializing`：传感器停止、自动发现未找到唯一输入 namespace、等待 RGB-D 与
  OdomInfo 配对、或首个有效估计尚未产生。**默认状态**；不产出位姿。
- `tracking`：输入连续有效、视觉与 OdomInfo 按同一源 `stamp` 配对、inliers ≥ 10 且
  matches ≥ inliers、位姿协方差合法（位置标准差 ≤ 0.2m）、位姿跳变在
  `0.03m + 1.5m/s × dt` 以内。
- `lost`：视觉失跟或 inliers 不足、输入过期、时钟回退/重置、光学 frame 或内参变化、
  RTAB-Map 子进程退出、视觉接收 watchdog 超时。**lost 在本段永久锁存**，晚到的
  正常消息不能把同一段修回 `tracking`。

每段建立时向 `/tf_static` 发布一条以 segment 命名的 base→光学 frame 变换（无标定为
identity，有实测安装为实测值）；RTAB-Map 的 `publish_tf` 保持 false，不与机器人现有
TF 发布者争用，也不发布 odom→base 之外的东西。

`reset` 顺序：先发布 `lost` 与原因 → 结束旧段并隔离旧输入/旧估计 → 停止并重建 RTAB-Map
子进程与新内部话题 → 新 segment UUID → 重新配对后进入 `tracking`。段间是坐标重置，
不拼接偏移；跨段消息一律拒绝，旧段队列不会进入新估计器，也不会使旧动作在新段续走。

## 可选标定（仅相机安装）

默认没有零外参示例文件，避免直接误用。没有标定时位姿直接表达在光学坐标系，参考点就是
相机原点——**不假设安装平移为零**。如需把同一段轨迹表达在机身参考点，可通过 sensor
config 保存完整 `calibration` 对象，或由部署配置管理写入挂载目录下 `config.json` 的
`calibration` 字段。字段全部需要真实来源，解析失败不启动：

- `calibration_id`、`fixed_mount=true`。
- `camera_to_body.translation`（三米制分量）与 `camera_to_body.quaternion`
  （归一化 xyzw）：**光学坐标 C 到机身 B**。

这组值直接作为每段私有的 base→光学 frame 静态 TF 发布；RTAB-Map 用它把图像变换到
base 后再估计，所以含转动杠杆臂的换参考点在估计器内部完成，runtime 不做二次变换，
也不是融合。配置生效时 `vendor.pose_reference = "body"`。没有标定时发布的是 identity
TF（base 与光学 frame 重合），并如实标 `rgb_optical`。**本设计没有 IMU 外参、IMU 约定或时间资格字段**：
这些键不属于本 sensor 的配置契约，出现也不会被使用。

固定头部还不够：现有 URDF 有 `waist_yaw_joint`，却未给出真实相机安装链。现场必须明确
B 的实物参考点，确认是否跨可动腰关节；跨关节时须锁定该关节，不能把骨架渲染模型当实测
外参。

## 明确非目标

- 无 IMU 输入、无 robot_localization/EKF、无融合、无加速度积分。
- 不做距离反馈、不做保存图连接绑定、不改 `loco_destination`；画布把该输出接到底盘
  动作卡不是受支持用法。
- 不接管/不共享相机所有权，不安装或启动 ROS RealSense 节点，不打开第二个 pipeline。
- 不含回环与全局重定位，不承诺漂移有界，不宣称 ±5cm 精度已达成。
- 不修改 `motus.odom/1` 格式；视觉专属字段全部在 `vendor` 内。

## 无硬件验证

ROS-free 单测在仓库根目录执行：

```sh
python3 -m pytest -q tests/bumi_visual_odometry
```

另加打包一致性检查：

```sh
python3 -m pytest -q tests/test_dockerfile_copies.py tests/test_deploy_restart_policy.py
```

`ros_probe.py` 是真实 ROS 集成探针（不接机器人、不读 USB、无 IMU）：渲染固定纹理墙的
RGB 与真实米制深度，给出一米前向运动的真值，真正启动 `rgbd_odometry`，比较静止与一米
移动、断输入后的 `lost`、重置段变化。只读取/估计，不发机器人命令。**本轮未执行。**

```
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=239 python3 /work/ros_probe.py --output /data/rendered-probe.json
```

## 局限

- 未构建 ARM64 镜像、未部署、未在真机运行；本地没有 ROS，所以 ROS 参数、消息、QoS
  与发布包的兼容性都待目标环境验证。
- 没有实测相机安装外参，默认位姿在相机光学坐标系；`pose_reference` 会如实标注。
- 历史观测的共享流捕获约 **1.8 Hz**（本轮未复测），对运动估计而言采样稀疏，
  快速转动/低纹理/遮挡下失跟是预期结果，`lost` 锁存而不是降级。
- 相机源时间不可确认（SDK 无曝光时间），设备到主机映射是到达时间近似。
- 20ms 配对、10 inliers、0.2m 位置 sigma、250/500ms 时限都是**拒绝阈值**，不是精度
  认证；单测与渲染探针通过也不能写成硬件精度达标。

## 真机验证顺序（只读）

1. 只读记录当前镜像 digest、固件、SDK、namespace、相机唯一所有者、topic/TF/QoS、
   CPU/RAM/温度/媒体帧率与实际共享流帧率。不打开第二个 pipeline。
2. 部署新版 bundle 原始共享分支与本独立 sidecar；先不下发运动。无标定应为
   `initializing`，确认 bundle JPEG/深度/内参/拍照录像不受影响。
3. 记录相机源域、映射漂移、RGB-D skew、到达延迟、连续更新与抖动；如需要机身参考点，
   完成安装测量并冻结 `calibration_id`。
4. 在 validation 镜像/独立记录服务录包，用**实际 NS**替换正则中的 NS：
   `ros2 bag record --regex '^(/NS/odometry_input/(rgb|depth|info|camera_meta)|/bumi_visual_.*/.*|/bumi_visual_odometry/state/odom)$'`
5. 在独立域（`ROS_DOMAIN_ID` 隔离）回放时只回放原始四输入，不回放旧 visual/diagnostic
   话题；使用 `use_sim_time=true`、相同标定，每次启动新进程、新段，重复两次并比较轨迹。
6. 现场独立测量同一机身参考点起止位移与不确定度。任何未执行项保持未验证。
7. 回滚：停止该 sensor 连线并停止独立服务即可；相机所有者与媒体链路不变。
