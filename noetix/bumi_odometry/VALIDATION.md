## Deployment fragment packaging follow-up

Confirmed the driver deployer reads /deploy/service.yml from the image. The
sidecar runtime stage now copies deploy/ to /deploy/; release inherits runtime.
A new Dockerfile regression checks that mapping and release inheritance. It failed
before the COPY was added and passed afterward. Existing source-path tests also
verify deploy/service.yml exists. This is offline recipe validation, not inspection
of a built image.

- Packaging, deployment policy and fusion tests: **142 passed in 1.06s**.
- Both Bumi and odometry service.yml deployment contract checks passed.
- Build script shell syntax and Git whitespace checks passed.
- Full pytest remains **45 collection errors, 3 skipped**.
- No apt/pip dependencies or base image changed; only the small deployment YAML
  and Docker layer metadata are added. Image size/digest measurement, verified
  release base digest, Docker builds, ROS launch and deployment remain pending.
- The ordinary Bumi mirror comment remains aligned with PR408's actual official
  Ubuntu archive change; it is not changed by this follow-up. PR408 is unchanged.

## TF review follow-up

The proposed inversion is inconsistent with the data transform contract. A TF
parent B / child S stores the sensor pose T_B_S, mapping p_S to p_B. The launch
implementation is retained; its inline comment and README now state the equation
and distinguish frame-tree edges from data transforms. ROS Humble's official
PointStamped and Vector3Stamped doTransform implementations apply the supplied
transform and set the output frame to header.frame_id.

The new runtime launch regression inspects the emitted static TF messages using
nonzero translations and +90 degree camera/IMU rotations. It checks sensor-origin
positions, a transformed camera point and IMU gyro vector direction. Temporarily
inverting the runtime transforms makes this test fail; restoring them passes.
This mutation check uses only temporary local edits, restored before committing.

- Fusion tests: **83 passed**.
- Relevant regressions: **623 passed, 1 skipped** (see original command below).
- Full pytest: **45 collection errors, 3 skipped**, unchanged environment limits.
- Python compilation and Git whitespace checks pass.
- No dependencies, image layers, launch processes or production transform values
  changed. Image size/digest measurements and real ROS TF-buffer/robot validation
  remain unperformed. No image is published by this follow-up.

# PR408 compatibility integration update (2026-10-21)

- Syncs #408 commits `782255e8`, `b60b3d92`, `c27c2687` in an isolated worktree.
- Actual merge conflicts resolved to the #408 shared runtime superset; independent
  sensor implementation remains from #425.
- Relevant regressions: **622 passed, 1 skipped in 65.72s**. Fusion tests:
  **82 passed**, included in that count.
- Cross-interface tests feed actual RawCameraPublisher/imu_reading products into
  FusionRuntime and the actual make_sample output into DistanceFeedback.
- Full repository pytest: **45 collection errors, 3 skipped**.
- Both deploy/service.yml checks, COPY-path regressions, shell syntax and Python
  compile checks pass. Revised full platform patch applies and compiles at the
  fixed platform baseline; neither platform workspace nor deployment was changed.
- #408 standalone IMU needs no sensor/platform extension. This sensor needs no
  destination card, and remains initializing without measured calibration.
- Merge #408 first, then this synchronized PR. No PR was merged by this session.
- No image build, ROS launch, full platform integration, deployment or hardware
  validation was performed. The hardware/accuracy limits below still apply.

## Historical c48692f8 validation

# 本轮验证记录（2026-10-21）

功能代码基于 #408 最新 head 40ae7fe8，在 feat/bumi-rgbd-imu-odometry-sensor 开发。
设计提交 c0a6bfe7。原 PR 分支和其他平台工作区未修改。

## 已实际执行

- 新增测试集：`python3 -m pytest -q tests/bumi_fusion`，45 passed。
- 相关 Bumi/相机/音频/旋转/互斥/ACP 回归：下列命令（最终结果见本文件末尾）。
- Python 编译（pycache 指向 /tmp），构建脚本语法检查，Git whitespace 检查。
- 平台 patch 对 main 6df601f 的源码快照在 /tmp 执行 git apply --check、实际 apply 和编译；
  没有在用户平台工作区应用。保存图解析到真实反馈消费者的派发/确认逻辑也由无硬件测试执行。
- 独立审查复现两项 P1：首次 initializing 被锁存、换线前已派发回调能填入新订阅；
  添加失败回归并修复。另修复配置重放重置有效段、绑定错误不可恢复、视觉尚无配对诊断即进入
  EKF 的问题。复核未发现新的明确 P0/P1，但不等于真实 ROS/硬件验收。

相关回归命令：

```sh
python3 -m pytest -q tests/bumi_fusion tests/test_bumi_imu_distance.py tests/test_bumi_managed_fusion.py tests/test_bumi_camera_specs.py tests/test_bumi_odom.py tests/test_bumi_bundle.py tests/test_bumi_loco_servo.py tests/test_bumi_distance_control.py tests/test_bumi_odometry_pipeline.py tests/test_bumi_odometry_evaluate.py tests/test_bumi_camera_capture.py tests/test_dockerfile_copies.py noetix/bumi/tests
```

控制仿真实际执行，输出比较保存在 validation/control-model-summary.json。
这是 ROS-free 控制模型，不运行 RTAB-Map：

| 模型 | 模型真实终点 m | 估计终点 m | 实际终点相对1m目标误差 m |
| --- | --- | --- | --- |
| nominal | 0.999887 | 1.000413 | -0.000113 |
| 估计尺度+15% | 0.864577 | 0.993566 | -0.135423 |

第二项仍可返回 done，明确证明估计到达不能替代真实精度验收。失跟、数据停止、坐标重置、
换线和取消案例均以零命令退出，来源保持 rgbd_imu_ekf。不是物理停止距离测量。

## 未执行/未验证

| 项目 | 状态 |
| --- | --- |
| ARM64 Docker build、冷/热构建耗时、发布 digest | 未执行；本机无 Docker/Podman。没有捏造依赖版本锁或预构建镜像已发布的结论。 |
| 真正 RTAB-Map/EKF ROS 集成与 ros_probe.py | 未执行；本机无 ROS。参数、消息、TF 与发布包的兼容性仍需目标环境验证。 |
| 平台补丁完整后端/浏览器集成与部署 | 未执行；仅 scratch apply/编译和绑定模块行为测试通过。 |
| 真机 RGB-D/机身 IMU 采集、单位/姿态方向/外参/延迟资格 | 未执行、待测量；sensor 默认无法进入有效 tracking。 |
| 真实录包两次回放、协方差/负载与同步门限调参 | 未执行；20ms skew、10 inliers、0.2m position sigma 和时限是初始拒绝阈值，不是精度认证。 |
| 20次一米留出精度与自动制动终点验收 | 未执行；不宣称 ±5cm。 |
| 实机相机/拍照录像/音频/旋转无影响、故障停车延迟 | 未执行；已有无硬件回归通过不能替代资源负载和物理行为检查。 |

细节及独立真机步骤见 README.md。旧 IMU 实验仍在，融合依赖仅在独立镜像中。
机身 SDK 没有源采样时间；读时延迟拟合不能证明 SDK 未返回过期缓存，这个限制保持公开。

## 全库测试环境阻塞

实际执行 `python3 -m pytest -q`，在收集阶段停止：3 skipped、45 errors（2.43s），
主要为缺少厂商 SDK、重复模块/包导入及 Python 3.9 不支持现有类型表达式。
未声称全库通过，也未为修复这些环境问题改动无关机器人代码。

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

## 最终相关回归结果

`488 passed in 59.27s`。该次执行包含本轮最终产品代码；没有 ROS/硬件成功声明。
