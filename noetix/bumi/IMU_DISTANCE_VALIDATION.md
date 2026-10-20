# 纯 IMU 距离实验验证记录

日期：2026-10-20。基线提交：5d6352ce；本次为 PR #408 的纯 IMU 实验改动。

## 本地验证

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
