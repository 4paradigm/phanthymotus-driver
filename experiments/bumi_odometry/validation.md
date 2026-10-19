# 验证记录

日期：2026-10-16。当前交付为独立实验代码，尚未部署，尚未验证真机 1 米误差。

## 已执行

- `python3 -m pytest -q tests/test_bumi_odometry_evaluate.py tests/test_bumi_odometry_pipeline.py`：40 项通过；另加入 `tests/test_odom_format.py` 后共 58 项通过。
- `python3 -m compileall -q experiments/bumi_odometry`：通过，仅证明语法可编译。
- 评估 CLI 由测试以子进程运行：合成轨迹与独立手算结果一致，缺失试次保留为 invalid，synthetic 来源不能输出实测精度通过。
- 本地预检入口运行后返回 `ready: false` 和缺少 pyrealsense2；这是环境缺项，不是传感器枚举成功。
- 全库 `python3 -m pytest -q` 已运行：3 skipped，33 个收集错误，未进入完整测试执行。错误包括缺少厂商 SDK、重复测试模块名及 Python 3.9 不支持部分既有类型注解；不得宣称全库测试通过。

## 独立代码审查后的修复

- 一米验收限制真实前向位移在 0.9–1.1 米：新增 0 米、0.5 米、2 米回归案例，修复前均错误通过，修复后不再作为一米验收通过的依据。
- 晚到故障使整次记录失效：复现源时间 0.9 秒的失跟诊断在终点 1.0 秒之后、1.1 秒才到达的情况；现在立即持久化故障行，离线评估拒绝整份试验。即使没有新的滤波位姿，故障也刷盘保存。相应回归测试先失败、修复后通过。

## 未执行 / 完成前提

| 项目 | 状态与前提 |
| --- | --- |
| Humble 启动与 topic/TF 集成 | 未执行：本地没有 ROS 2、Docker、Podman 或 Colima。启动参数虽参考上游源码核对，仍须真实 Humble 运行验证。 |
| ARM64 依赖锁定 | 未完成：只有兼容依赖实际启动后才记录 runtime.lock.json，不以推测版本代替。 |
| IMU 缺失根因 | 未确定：此前只读查询发现宿主机可枚举 Motion Module，生产容器不可枚举；udev 可见性及 SDK 构建/版本差异待隔离验证。 |
| 安装外参 | 未取得：用户已确认固定相机、头部不能动；仍需测量 camera_link 到机身的旋转和平移及起始重力方向。 |
| 实际流采集与时钟 | 未执行：需要明确的相机空闲窗口及唯一采集所有者，不能并发打开生产相机。 |
| 原始录包两次回放 | 未执行：尚无真实 RGB-D/IMU 录包；不能将纯数学测试称为融合算法验证。 |
| 静止 60 秒及转动对照 | 未执行：待采集窗口与外参。 |
| 20 次约 1 米留出验收 | 未执行：需要现场人员控制现有手动移动并测量同一机身参考点的真实位移。 |
| 其他卡片恢复检查 | 本轮未改动线上容器、相机采集或卡片；后续切换采集窗口结束时须恢复并核验。 |

## 全库收集错误路径

下列为本次完整 pytest 的失败清单，均位于本次新增实验目录和测试之外：

```text
brainco/revo2/tests/test_device.py
chasing/qianjiao_p200_pro/test_device.py
noetix/bumi/noetix_sdk_bumi/examples_py/test_high.py
noetix/bumi/noetix_sdk_bumi/examples_py/test_low.py
noetix/bumi/noetix_sdk_bumi/examples_py/test_media.py
noetix/bumi/noetix_sdk_bumi/examples_py/test_policy.py
pndbotics/adam/test_arm_controls.py
pndbotics/adam/test_battery.py
pndbotics/adam/test_loco_contract.py
pndbotics/adam/test_state_flattening.py
pndbotics/adam/test_upper_body_cards.py
pndbotics/adam/test_vision_capture.py
robotera/q5_bundle/test_basic_sensors.py
tests/test_tianyi_bridge.py
unitree/as2w/unitree_sdk2py/test/crc/test_crc.py
unitree/as2w/unitree_sdk2py/test/rpc/test_client_example.py
unitree/as2w/unitree_sdk2py/test/rpc/test_server_example.py
unitree/g1/test_slam_rpc.py
unitree/g1/tests/test_fsm_posture.py
unitree/g1/tests/test_speaker_pacing.py
unitree/g1/unitree_sdk2py/test/crc/test_crc.py
unitree/g1/unitree_sdk2py/test/rpc/test_api.py
unitree/g1/unitree_sdk2py/test/rpc/test_client_example.py
unitree/g1/unitree_sdk2py/test/rpc/test_server_example.py
unitree/go2/unitree_sdk2py/test/crc/test_crc.py
unitree/go2/unitree_sdk2py/test/rpc/test_api.py
unitree/go2/unitree_sdk2py/test/rpc/test_client_example.py
unitree/go2/unitree_sdk2py/test/rpc/test_server_example.py
unitree/r1/tests/test_switch_mode_acp.py
unitree/r1/unitree_sdk2py/test/crc/test_crc.py
unitree/r1/unitree_sdk2py/test/rpc/test_api.py
unitree/r1/unitree_sdk2py/test/rpc/test_client_example.py
unitree/r1/unitree_sdk2py/test/rpc/test_server_example.py
```

当前没有实测精度结论。距离反馈控制尚未接入。


## 2026-10-19：合入现有 PR 并同步 main

按用户要求把实验合入原 loco_destination PR #408；同步上游 main 3ef6ee77，保留上游的 loco_servo、相机和声源方向实现。只推送 loco_destination 分支。loco_destination 自身闭环线程直接下发 SDK，不通过 loco_servo 执行；只与其共享底盘互斥保护。

- 合并后的 Bumi、loco_servo、相机规格、里程计及 ControlSink 测试：**362 passed in 58.28s**。
- 旋转预留状态阻止伺服 start/resume；状态查询和零角度请求不暂停伺服。
- 新回归复现并修复：前一 move 线程退出时会清除发布权，旋转必须等它退出后才接管；如果 join 超时仍未停止，拒绝启动旋转并保留停止信号。
- ACP action_id 改用 UUID4；固定 time.time_ns 后重复请求的回归测试先复现 ID 重复，修复后通过。独立合并复核未发现剩余阻塞问题。
- 相对合入的 main 检查，本 PR 不修改生产 Dockerfile、requirements、sound_direction.py 或 direction_card.py；实验仍未部署。
- 全库 pytest 重新执行，**45 个收集错误、3 skipped**，未宣称全库通过。当前失败路径如下：

```text
agibot/AimDK_X2/tests/test_device.py
brainco/revo2/tests/test_device.py
chasing/qianjiao_p200_pro/test_device.py
limx/tron2/tests/test_camera.py
limx/tron2/tests/test_completion.py
limx/tron2/tests/test_gripper_display.py
limx/tron2/tests/test_motion_readiness.py
limx/tron2/tests/test_sensors.py
noetix/bumi/noetix_sdk_bumi/examples_py/test_high.py
noetix/bumi/noetix_sdk_bumi/examples_py/test_low.py
noetix/bumi/noetix_sdk_bumi/examples_py/test_media.py
noetix/bumi/noetix_sdk_bumi/examples_py/test_policy.py
pndbotics/adam/test_arm_controls.py
pndbotics/adam/test_battery.py
pndbotics/adam/test_battery_status.py
pndbotics/adam/test_loco_contract.py
pndbotics/adam/test_state_flattening.py
pndbotics/adam/test_upper_body_cards.py
pndbotics/adam/test_vision_capture.py
robotera/q5_bundle/test_basic_sensors.py
tests/test_franka_discovery.py
tests/test_franka_driver.py
tests/test_piper_camera_live.py
tests/test_tianyi_bridge.py
ubtrobot/u1_pro/test_device.py
unitree/as2w/unitree_sdk2py/test/crc/test_crc.py
unitree/as2w/unitree_sdk2py/test/rpc/test_client_example.py
unitree/as2w/unitree_sdk2py/test/rpc/test_server_example.py
unitree/g1/test_slam_rpc.py
unitree/g1/tests/test_fsm_posture.py
unitree/g1/tests/test_speaker_pacing.py
unitree/g1/unitree_sdk2py/test/crc/test_crc.py
unitree/g1/unitree_sdk2py/test/rpc/test_api.py
unitree/g1/unitree_sdk2py/test/rpc/test_client_example.py
unitree/g1/unitree_sdk2py/test/rpc/test_server_example.py
unitree/go1/tests/test_vision_capture.py
unitree/go2/unitree_sdk2py/test/crc/test_crc.py
unitree/go2/unitree_sdk2py/test/rpc/test_api.py
unitree/go2/unitree_sdk2py/test/rpc/test_client_example.py
unitree/go2/unitree_sdk2py/test/rpc/test_server_example.py
unitree/r1/tests/test_switch_mode_acp.py
unitree/r1/unitree_sdk2py/test/crc/test_crc.py
unitree/r1/unitree_sdk2py/test/rpc/test_api.py
unitree/r1/unitree_sdk2py/test/rpc/test_client_example.py
unitree/r1/unitree_sdk2py/test/rpc/test_server_example.py
```
