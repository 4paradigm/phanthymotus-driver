# 纯 IMU 距离实验验证记录

日期：2026-10-21。本次指令积分交叉校验以 74cfae16 为基线，仍提交到 PR #408。

## 2026-10-21 指令积分交叉校验

- 根因：SDK `NingImuData` 没有源采样时间或序号；控制器与仅用于诊断的独立订阅器
  在相同时刻从同一 SDK 缓存读到的加速度最多相差数 m/s²（步态冲击），双重积分结果
  因此依赖轮询相位。已录 15:55:54 动作（请求 0.5 m，用户实测约 1.0 m）：控制轨迹
  估计 −0.450 m，独立订阅数据整段积分 +2.775 m（去趋势 +1.719 m）。两者都不是
  固定比例或固定符号误差。
- 同一批记录里，用 2026-10-09 标定的归一化指令积分：三次有实测值的试验为
  0.438 / 0.450 m（实测约 0.4 m）和 1.050 m（实测约 1.0 m），误差约 5–10%。
  因此它适合作为独立交叉校验，不适合替代 `accuracy_validated=false` 的结论。
- 实现：`imu_distance_control.run_distance()` 增加 `dead_reckoned_m` 累计和单向
  交叉校验；`loco_servo` 提供同一份标定的 `distance_full_scale()`，`loco_target`
  在 `loco_servo` 启用时传入，未启用时回退模块默认值。
- 新增测试 4 项：标定值校验、IMU 完全看不到运动时由指令行程停车、IMU 只报告一半
  加速度时由交叉校验停车、`loco_servo` 标定接线。原有 60 项 IMU 距离测试全过。
- 同一段已录数据的离线回放（`replay_crosscheck_trial.py`，不连机器人）：新代码在
  运动开始后 1.272 s、指令行程 0.420 m / IMU 估计 0.167 m 时停车报错；仅反向保护
  的回放在 2.316 s、估计 −0.052 m 时才停车。回放不能证明真机最终停在哪里。
- 回归：`tests/test_bumi_imu_distance.py` 64 passed；文档命令的 Bumi/历史实验/打包
  回归 481 passed；`noetix/bumi/tests` 96 passed（合并计入 481 中）。
- 未执行：真机部署、运动复测、镜像构建、ROS/平台集成。标定值仍是指令积分的模型参数，
  不是实测位移；`state=done` 仍只表示 IMU 估计满足条件。
- 仍应暂停纯 IMU 距离试验，直到用同一机身标记点取得新的实测值并确认交叉校验的
  触发点符合预期。

## 本次相机恢复与依赖撤回

- Bumi/历史实验/打包回归：**441 passed in 59.76s**。命令为下方初版命令再加入 `tests/test_bumi_camera_capture.py`。
- 公共生命周期、控制输入和子进程日志回归：**96 passed in 0.13s**，运行 `tests/test_common_lifecycle.py tests/test_control_sink.py tests/test_go2_logsafe_children.py`。
- 直接采集新增 8 项测试：原彩色/深度流规格、深度对齐、JPEG/Z16 格式与时间戳、内参、相机失败及 SIGTERM 清理、单采集者启停/恢复，以及真实拍照消费者复用缓存。边界以 SDK/ROS/编码器替身验证，未连接相机。
- 独立复核与融合改造前 cf8155cf^ 的采集实现对照，未发现明确的重要回归；Python 编译及差异检查通过。
- 当前 Dockerfile 的全部 apt/pip 安装指令与 cf8155cf^ **完全一致**，仅保留原有依赖。融合新增的 ROS 相机/RTAB-Map/EKF/显式 tf2 安装步骤、ROS apt 源/签名密钥及独立 udev 挂载均已撤回。
- 历史 ROS 采集/融合/距离反馈源码移入 `experiments/bumi_odometry/managed`，不在生产 COPY 清单中；旧 fusion_enabled 配置不能再启动融合。
- 本次完整 pytest 再次运行，仍为 **45 个收集错误、3 skipped**；错误路径见文末。
- 无 Docker/ROS 本地运行环境，远端 SSH 认证前断开；尚未执行 ARM64 镜像构建、相机实机回归或运动测试。不能宣称实际构建耗时或最终镜像包数量已经验证。

## 初版纯 IMU 本地验证（历史，基线 5d6352ce）

- 下述 Bumi 相关回归：**435 passed in 63.80s**，包括转角、距离、底盘互斥、相机、麦克风、声源方向、原视觉实验及打包路径检查。
- 新 IMU 模块：26 项测试，覆盖已知加速度轨迹、重力姿态补偿、匀速不清零、采样抖动、无效数据、取消、故障与制动。
- 独立代码复核发现“两秒窗口固定要求 100 样本”会因实际读取开销导致标定超时；四个不同采样周期的回归先失败，改为时间跨度加最低样本数后通过。复核未发现其他明确的重要回归。
- 修改过的 Python 源码编译和 git diff --check 通过。

```sh
python3 -m pytest tests/test_bumi_imu_distance.py tests/test_bumi_managed_fusion.py tests/test_bumi_camera_specs.py tests/test_bumi_odom.py tests/test_bumi_bundle.py tests/test_bumi_loco_servo.py tests/test_bumi_distance_control.py tests/test_bumi_odometry_pipeline.py tests/test_bumi_odometry_evaluate.py tests/test_dockerfile_copies.py noetix/bumi/tests -q --tb=short
```

## 尚未验证

- 本地没有 Docker/ROS 2，未构建完整 ARM64 镜像；远端审核构建结果待确认，不能保证已解决固定 30 分钟超时。
- 未部署或发送真机运动指令。SDK 加速度单位、重力约定、姿态方向、采样新鲜度及实际距离精度需要实测。
- SDK 没有源时间戳；软件保护只是故障启发式，无法提供独立位置或停车观测。
- 相机、语音等已做相关模拟回归，尚未做本次镜像部署后的实机回归。

## 全仓测试的环境阻塞

完整 python3 -m pytest -q 已执行，结果为 3 skipped、45 个收集错误。
环境缺少厂商 SDK，另有重复模块和本机 Python 3.9 与既有类型注解不兼容等问题；没有全仓通过声明。
完整错误路径如下：

```text
ERROR agibot/AimDK_X2/tests/test_device.py
ERROR brainco/revo2/tests/test_device.py
ERROR chasing/qianjiao_p200_pro/test_device.py
ERROR limx/tron2/tests/test_camera.py
ERROR limx/tron2/tests/test_completion.py
ERROR limx/tron2/tests/test_gripper_display.py
ERROR limx/tron2/tests/test_motion_readiness.py
ERROR limx/tron2/tests/test_sensors.py
ERROR noetix/bumi/noetix_sdk_bumi/examples_py/test_high.py
ERROR noetix/bumi/noetix_sdk_bumi/examples_py/test_low.py
ERROR noetix/bumi/noetix_sdk_bumi/examples_py/test_media.py
ERROR noetix/bumi/noetix_sdk_bumi/examples_py/test_policy.py
ERROR pndbotics/adam/test_arm_controls.py
ERROR pndbotics/adam/test_battery.py
ERROR pndbotics/adam/test_battery_status.py
ERROR pndbotics/adam/test_loco_contract.py
ERROR pndbotics/adam/test_state_flattening.py
ERROR pndbotics/adam/test_upper_body_cards.py
ERROR pndbotics/adam/test_vision_capture.py
ERROR robotera/q5_bundle/test_basic_sensors.py
ERROR tests/test_franka_discovery.py
ERROR tests/test_franka_driver.py
ERROR tests/test_piper_camera_live.py
ERROR tests/test_tianyi_bridge.py
ERROR ubtrobot/u1_pro/test_device.py
ERROR unitree/as2w/unitree_sdk2py/test/crc/test_crc.py
ERROR unitree/as2w/unitree_sdk2py/test/rpc/test_client_example.py
ERROR unitree/as2w/unitree_sdk2py/test/rpc/test_server_example.py
ERROR unitree/g1/test_slam_rpc.py
ERROR unitree/g1/tests/test_fsm_posture.py - TypeError: unsupported operand t...
ERROR unitree/g1/tests/test_speaker_pacing.py - TypeError: unsupported operan...
ERROR unitree/g1/unitree_sdk2py/test/crc/test_crc.py
ERROR unitree/g1/unitree_sdk2py/test/rpc/test_api.py
ERROR unitree/g1/unitree_sdk2py/test/rpc/test_client_example.py
ERROR unitree/g1/unitree_sdk2py/test/rpc/test_server_example.py
ERROR unitree/go1/tests/test_vision_capture.py
ERROR unitree/go2/unitree_sdk2py/test/crc/test_crc.py
ERROR unitree/go2/unitree_sdk2py/test/rpc/test_api.py
ERROR unitree/go2/unitree_sdk2py/test/rpc/test_client_example.py
ERROR unitree/go2/unitree_sdk2py/test/rpc/test_server_example.py
ERROR unitree/r1/tests/test_switch_mode_acp.py - TypeError: unsupported opera...
ERROR unitree/r1/unitree_sdk2py/test/crc/test_crc.py
ERROR unitree/r1/unitree_sdk2py/test/rpc/test_api.py
ERROR unitree/r1/unitree_sdk2py/test/rpc/test_client_example.py
ERROR unitree/r1/unitree_sdk2py/test/rpc/test_server_example.py
```
