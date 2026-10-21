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

## 2026-10-21 腿式里程计（第三轮）

真机数据说明纯 IMU 不可用：退级版 0.8 m 动作实际约 1.05 m，IMU 估计只有 0.075 m；
同一 0.6 指令的指令积分在两次动作间给出 0.2–0.35 m/s 的不同速度。按操作者要求，
把 #408 独立距离动作的距离来源改为**腿式里程计**。

- 新增 `noetix/bumi/leg_odometry.py`：从 `BUMI_V3.0_260119.urdf` 提取左右腿各 6 个
  关节的原点与轴，对 SDK 的 21 个关节位置做正运动学；脚底接触点取踝滚转关节原点
  投影到鞋底（z = −0.0511 m，来自 ankle_roll_link 网格包围盒）。
- 积分：支撑脚在世界中不动，因此每个采样机身位移 =
  `R_prev·p_prev − R_cur·p_cur`（p 为机体系脚位，R 来自 SDK 四元数）；两条腿隐含
  步长较小的是支撑脚，两者接近时按双支撑取平均；单步超过 0.05 m 视为噪声丢弃。
- `run_distance` 在 owner 暴露 `_read_distance_joints()` 时使用腿式里程计（生产的
  `LocoPlugin` 总是暴露），否则退回原 IMU 积分（单元测试用的 plant、旧 bundle）。
  结果新增 `distance_source`、`imu_position_m`、`imu_velocity_mps`；`feedback_source`
  仍保留实验入口标识 `imu_integrated_experimental`。
- 保留：SDK 四元数用于姿态和偏航/倾角检查；加速度/四元数/新鲜度/反向/超走/横向/
  速度/超时保护；加速度合理性上限 50 m/s²；指令积分只作交叉校验故障界限。
- 真机首测（2026-10-21 18:50，请求 0.5 m）：终态 done、`distance_source=leg_odometry`、
  `estimated_distance_m=0.462 m`，操作者实测约 0.7 m；腿式估计的竖直分量漂到 −0.105 m，
  同次 IMU 积分前向 0.478 m、指令积分 0.32 m。首测说明腿式估计偏小约三成，竖直漂移
  指向脚底接触点/脚掌滚动模型，而不是简单的比例误差。
- 第二次真机测试（2026-10-21 18:58，请求 0.5 m）：机器人**起步即停**，error
  “IMU estimated motion exceeds direction or speed limits”。轨迹显示标定结束后的第一个
  步态采样里腿式位置从 +0.004 m 跳到 −0.026 m、速度 −1.333 m/s：这是“鞋底最低角点”
  接触模型在脚掌从平踩转入滚动时的单样本跳变，不是真实运动。已改回**连续的鞋底中心**
  接触点，并把单步合理上限从 0.05 m 收紧到 0.02 m（1.0 m/s），标定结束时重置腿式
  起始位置，避免标定阶段噪声影响起点。
- 第三次真机测试（2026-10-21 19:36，连续鞋底中心模型，请求 0.5 m）：done，腿式估计
  0.483 m，操作者实测约 0.75 m；18:50 那次是 0.462 m vs 约 0.70 m，比值 1.51/1.55
  稳定。轨迹里脚只抬起约 1 cm、两脚几乎贴地，说明**脚在滑**：机体实际走了 0.70–0.75 m，
  而支撑脚运动学只解释了 0.46–0.48 m。`leg_joint_updates=275/333`，但关节值是状态量，
  采样重复不会系统性丢位移，因此主因判为打滑而非关节缓存。
- 第四次真机测试（2026-10-21 19:44，连续 CoP + 1.5 系数）：前进 0.5 m 请求下估计被
  放大后仍只有 0.394 m，机器人实际约 0.70 m，终态 error “stopped outside the estimated
  target band”；同批后退 0.5 m 被放大后的单步速度尖峰（1.32 m/s）触发速度保护。
  离线用 19:36 的关节数据对比接触模型：连续 CoP 只积出 0.180 m（脚掌滚动让接触点
  前移、抵消机体前进），鞋底中心 0.438 m、脚尖 0.469 m、脚跟 0.427 m、踝投影 0.349 m。
- 最终修正：接触点用鞋底中心（连续、不随滚动前移）；`DISTANCE_SLIP_SCALE=1.7`（0.75/0.438）；
  单步原始位移上限按滑移系数折算（0.02/1.7）避免放大尖峰触发速度保护；制动开始后不再
  更新腿式位移（速度置零），端点容差对腿式路径放宽到 0.10 m（制动阈值本身可达 0.10）。
  这是地面相关经验修正，换地面需重测并把系数改回 1.0；文档明确不把它当精度保证。
  若需要与地面无关的方案，应改用 RGB-D 视觉里程计（不需要相机-IMU 外参）。
- 诊断保留：轨迹记录 12 个腿关节角、两侧接触点；终态 `leg_samples`、
  `leg_stance_samples`、`leg_joint_updates`（判断 `get_joint_state()` 实际更新率）。
- 新增测试：`tests/test_bumi_leg_odometry.py` 10 项（站立脚位、平移积分、摆动脚不参与、
  偏航投影、SDK MotorState 兼容、异常步长丢弃、采样/关节数校验、四元数校验）；
  `tests/test_bumi_imu_distance.py` 增加 1 项“有关节接口时使用 leg_odometry”与
  `distance_source` 断言。
- 回归：IMU/腿式/广域 Bumi/实验/打包 **597 passed**；`noetix/bumi/tests` 96 passed
  （计入 597）。Python 编译与 whitespace 检查通过。
- **未做**：真机构建/部署/运动验证。需要确认：关节符号与 URDF 一致（走错方向先查符号，
  不要用系数补偿）、脚底接触点/双支撑阈值在真实步态下的表现、四元数 IMU 与 base_link
  的安装对齐。`accuracy_validated` 仍为 false。

## 2026-10-21 真机复测（第二轮，含审核修正）

操作者部署 `release.261021.30cf9db`（上一轮交叉校验）后先实测 5 次，再部署
`release.261021.9171626`（退级版）实测一次前进 0.8。

- 后退 0.5 那次按设计触发交叉校验：指令行程 0.478 m、IMU 估计 0.223 m，零速报错。
- 旧镜像前进 0.8：IMU 估计 0.406 m、横向 0.155 m；操作者实测实际 0.4 m、几乎无
  偏移。假横向 + 固定 0.15 m 限制是当时的直接阻断原因。
- 右移 0.3 ×2：四元数 yaw +9.5°/+8.2°，陀螺积分 +10.1°/+8.3°，两者一致，机身确实
  在向左转；20° 偏航限制终止动作。左移 0.3 反向转约 3.8° 并触发超走。
- **退级版前进 0.8（18:13）：实际约 1.05 m、轻微左偏，终态 error。** 轨迹显示
  指令积分只算出 0.458 m（去死区模型约 0.15 m/s），IMU 估计 0.075 m，退级在
  指令 0.426 m / IMU 0.166 m 触发，最后被 `IMU acceleration norm 30.078 exceeds
  experimental limit 30` 终止。也就是说：同一 0.6 指令下，实际速度在两次动作间
  从约 0.2 m/s 变到约 0.35 m/s；如果按退级继续走到指令 0.8，实际会到约 1.6 m。
- 结论：指令积分看不到打滑/实际位移，且标定速度本身不稳定，**不能用来降级完成动作**。
  交叉校验恢复为“发现矛盾立即零速报错”，终态不再有 `dead_reckoned_fallback`；
  0.15 m 完成带一并撤回（恢复 0.05 m）。这与 PR 审核的 High/Medium 意见一致。
- 同时把加速度合理性上限从 30 提高到 50 m/s²：正常步态冲击峰值实测 30.1–33.0，
  30 会把正常步态误判为传感器故障（本次 30.078 仅超 0.26%）。
- 保留：去死区速度模型（只用于校验，不作为完成依据）、横向限制
  `max(0.15, 0.5×distance)`、四元数/加速度/方向/速度/新鲜度/超时保护。
- 回归：`tests/test_bumi_imu_distance.py` 66 passed；文档命令的 Bumi/历史实验/打包
  回归 588 passed；`noetix/bumi/tests` 96 passed（计入 588）。
- 本轮修正**尚未构建镜像或部署**；真机复测待操作者确认。右移/左移时真实偏航
  8–10° 的问题本轮未处理（属于侧移步态/控制增益，需要单独真机试验）。
- 15:55:54 回放（旧退级版）只作为历史记录，不能用于当前行为的结论。

## 2026-10-21 前进修正与侧移（第五轮）

- 19:55 前进 0.5 m（固定鞋底中心 + 1.7 + 制动冻结 + 0.10 容差）：done，腿式估计
  **0.433 m**，操作者实测约 **0.45 m**——估计与实测只差约 2 cm，前进方向基本可用。
  这验证了固定鞋底中心 + 打滑系数 + 制动冻结的组合。
- 同轮右移 0.3 m：error “IMU estimated distance made no progress”，腿式侧向估计
  −0.027 m；轨迹显示机身 yaw 从 8.9° 增到 26.4°（左转 17.5°），机器人几乎没侧移。
  原因：侧移指令 0.65 仅略高于横移死区 0.60；右移步态本身使机身向左转，原偏航
  修正不足。
- 修正：侧移指令 0.65 → 0.9；偏航前馈 +0.12×body_y（右移右转、左移左转）；
  偏航修正限幅 ±0.4 → ±0.5。方向测试已更新为对侧移校验该前馈符号。
- 回归：IMU/腿式 78 项、Bumi 广域 600 项、插件 96 项；#425 组合 652 passed,
  1 skipped。侧移效果、侧移打滑系数**尚未真机验证**。

## 2026-10-21 侧移结果与分方向打滑（第六轮）

- 部署第五轮修正（侧移指令 0.9、偏航前馈 +0.12×body_y、限幅 ±0.5）后实测：
  右移 0.3 m 实际约 **0.4 m**（其中一次 error “IMU estimated motion exceeds
  direction or speed limits”），左移 0.3 m 实际约 **0.5 m**（done）。
  轨迹里左右移的 yaw 只变化几度，说明偏航前馈方向正确、不再明显跑偏。
- 但侧移的腿式原始估计只有约 0.12 m，实际 0.4–0.5 m：侧移脚底打滑的真实比值
  约 4.2–4.7，远大于前进的 1.7。全局 1.7 会让侧移动得远超目标。
- 修正：打滑系数改为分方向 `LEG_SLIP_SCALE = {forward/backward: 1.7, left/right: 4.4}`，
  `run_distance` 按方向构造 `LegOdometry(slip_scale=...)`；终态显示实际值。
  侧移 4.4 只由这两次标定，需要再测验证；若左/右差异大，再拆左/右系数。
- **侧移仍未验收**；本轮未做其它方向或不同距离的验证。

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
