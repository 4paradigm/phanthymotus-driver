# 本轮验证记录（2026-10-28）

功能代码基于 PR #408 合并后的 `upstream/main`（`e40ca380`），在
`feat/bumi-visual-odometry-port` 上从已关闭的 PR #425 恢复**仅视觉**部分（无 IMU、
无 robot_localization EKF、无融合、无距离反馈绑定）。设计见
[2026-10-28-bumi-visual-odometry-design.md](../../docs/superpowers/specs/2026-10-28-bumi-visual-odometry-design.md)。

## 与 #408 合并后基线的适配

`device.py` 的相机实现已回到主分支形态（没有 `camera_runtime.py`），因此原始
RGB-D 发布分支挂在 `_camera_subprocess` 里：`visual_inputs.py` 在同一条
pyrealsense2 pipeline、同一个 frameset 上发布，构造和每次 publish 都包在 try/except
中，失败只打日志，不影响 JPEG/Z16 媒体。sidecar、`common/visual_odometry.py`、
测试与文档从已验证的视觉里程计快照移植；`build.sh` 重新带上可选
`image_repository` 支持，避免 sidecar 镜像与 bundle 的 `noetix/bumi` 仓库互相覆盖。

## 已实际执行

- 视觉里程计、打包与相关回归（合并前最后一次执行）：

  ```sh
  python3 -m pytest -q tests/bumi_visual_odometry \
    tests/test_bumi_bundle.py tests/test_bumi_odom.py tests/test_odom_format.py \
    tests/test_bumi_imu_distance.py tests/test_bumi_leg_odometry.py \
    tests/test_bumi_loco_servo.py tests/test_bumi_camera_specs.py \
    tests/test_dockerfile_copies.py tests/test_deploy_restart_policy.py \
    tests/test_common_lifecycle.py noetix/bumi/tests
  # -> 461 passed in 58.60s
  ```

  其中本目录直接相关的打包/部署检查：

  ```sh
  python3 -m pytest -q tests/bumi_visual_odometry \
    tests/test_dockerfile_copies.py tests/test_deploy_restart_policy.py
  # -> 109 passed
  ```

- `tests/bumi_visual_odometry`（47 项）覆盖：健康状态机、输入/时间拒绝条件、安装
  外参换参考点、样本构造、launch 契约（只启动一个 `rgbd_odometry`、无 EKF/IMU0）、
  生命周期/reset、视觉-诊断配对与位姿跳变拒绝、每段 segment 命名的 base→光学静态
  TF、`driver.yaml` 的 `image_repository` 与端口、以及“设备相机子进程只共享一条
  pipeline、原始分支故障不影响媒体”的 bundle 钩子测试。
- `tests/test_dockerfile_copies.py` 覆盖 sidecar Dockerfile 的每个 `COPY` 源文件真实
  存在（`pipeline.py`、`runtime.py`、`service.py`、`ros_probe.py`、`common/`、
  `deploy/`）；`tests/test_deploy_restart_policy.py` 覆盖 `deploy/service.yml` 是有效
  YAML 且 `restart: always`。
- `bash -n build.sh` 通过；`driver.yaml` / `deploy/service.yml` 用 `yaml.safe_load`
  解析通过；单测加载 `device.py` 和 sidecar 模块不依赖本机 ROS。

## 未执行/未验证

| 项目 | 状态 |
| --- | --- |
| ARM64 Docker build、依赖/应用 digest、冷/热构建耗时、镜像大小 | 未执行；本机没有 Docker/Podman。没有捏造依赖版本锁或“镜像已发布”的结论。 |
| 真实 ROS 图集成与 `ros_probe.py` | 未执行；本机没有 ROS。RTAB-Map 参数、消息、QoS、TF 与发布包兼容性仍需目标环境验证。 |
| 真机相机共享流、实际帧率、RGB-D skew、延迟分布 | 未执行；`~1.8 Hz` 为历史观测，本轮未复测。 |
| 媒体链路回归（JPEG/压缩深度/内参/拍照/录像/音频）、CPU/RAM/温度 | 未执行；无硬件回归不能替代资源与物理行为检查。 |
| 平台注册、画布添加 sensor、部署/回滚 | 未执行；仅交付 driver.yaml 与 service.yml。 |
| 实测相机安装外参（camera_to_body） | 未执行、待测量；默认 `pose_reference=rgb_optical`。 |
| 位姿精度/漂移、录包两次重算、留出试次 | 未执行；不宣称漂移有界，也不宣称 ±5cm 达成。 |

任何未执行项保持未验证：单测通过、打包检查通过或渲染探针通过都不写成 ROS/硬件验收。
