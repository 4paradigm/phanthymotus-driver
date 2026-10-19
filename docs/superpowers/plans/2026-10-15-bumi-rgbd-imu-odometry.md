# Bumi RGB-D + IMU Odometry Implementation Plan

执行更新（2026-10-19）：用户指定将实验提交并入原 loco_destination PR #408，后续提交分支改为 feat/bumi-loco-destination；保留独立实验运行方式、声音分支隔离及所有真机验收前提。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 独立实现可回放的 RGB-D/IMU 里程计，用外部真值验证约 1 米移动的前向估计误差是否在 ±0.05 米内。

**Architecture:** RealSense ROS 2 驱动提供对齐 RGB-D、内参、TF 和内置 IMU，RTAB-Map 输出视觉位姿，robot_localization EKF 融合视觉位姿及三轴角速度。旁路质量节点拒绝断流和重置后的距离；离线评估程序比较固定起终点窗口与外部真值。先本地测试，再录包回放，最后在约定窗口做真机采集。

**Tech Stack:** ROS 2 Humble / ARM64，realsense2_camera，rtabmap_odom，robot_localization，Python 3.10（真机）；纯数学与评估工具保持 Python 3.9+，pytest。

**Spec:** `docs/superpowers/specs/2026-10-15-bumi-rgbd-imu-odometry-design.md`

## Global Constraints

- 工作目录 `/Users/4paradigm/Desktop/shixi/phanthymotus-driver-odometry`，分支 `feat/bumi-rgbd-imu-odometry`，基于 `origin/main`，不推送旋转或声音分支。
- 本阶段不发出运动命令、不修改旋转控制、不修改声音识别、不修改现有部署镜像。
- 只允许一个进程拥有相机；所有真机流采集在约定窗口内进行，本地开发和回放不得占用真机相机。
- 独立命名空间 `/bumi_odom`；不覆盖线上现有 TF。
- 第一版不进行加速度积分，保留 3D 位姿，前向距离使用起始水平前向投影。
- 安装外参、统一时间基准和 IMU 实际可读是采集前提；缺少标定不得填默认单位变换。
- 验收集至少 20 次，全部尝试计入，跟踪失败不能剔除；前向绝对误差 ≤0.05 米，跨越测量不确定度边界的结果记为不确定。

## Review Focus

- 相机杠杆臂与可动头部：相机旋转不能自动视为机身平移；任务 1 测试非零外参和纯转动。
- 不同时间域与回放：不能用接收时间替换采样时间；任务 2 检查时间戳，任务 3 对同一录包重复回放。
- 视觉失跟或重置后 EKF 仍预测：任务 2 必须令质量状态 lost，任务 1 禁止该段通过验收。
- 测量边界和失败样本：任务 1 测试 5 厘米边界、不确定度和缺失试次，任务 4 不允许只汇报成功试次。
- 线上相机与资源竞争：任务 2 默认不启动相机，任务 4 在约定窗口检查独占和负载，恢复原采集后核验。

---

### Task 1: 离线位移与误差评估

**Files:** Create `experiments/bumi_odometry/evaluate.py`, `experiments/bumi_odometry/README.md`; Test `tests/test_bumi_odometry_evaluate.py`。

**Interfaces:**
- `base_position(camera_position, camera_quaternion, camera_to_base_translation) -> tuple[float, float, float]`：输入为世界坐标中的相机位姿和相机坐标中的机身原点偏移；四元数顺序 xyzw。
- `project_displacement(start_position, end_position, start_quaternion) -> tuple[float, float]`：返回水平前向、横向位移（米）；起始 body X 轴在水平面归一化后投影。
- `evaluate_trial(samples: list[dict], truth: dict) -> dict`；`summarize(trials: list[dict]) -> dict`。
- CSV 列为 `stamp_ns,x,y,z,qx,qy,qz,qw,status,segment_id`，位姿必须已表示机身参考点，时间戳严格递增。真值 JSON 为 `trial_id,start_ns,end_ns,forward_m,lateral_m,uncertainty_m,scene,speed_m_s,split`，split 为 development/acceptance。start_ns 和 end_ns 各为 `[窗口起始纳秒,窗口结束纳秒]`，取窗口内有效位置均值；起始姿态使用起始窗口第一帧，起终点窗口必须静稳且不跨 segment。
- CLI：`python3 experiments/bumi_odometry/evaluate.py --manifest trials.json --output report.json`；manifest 列出全部 trial_id、CSV 和真值路径。报告逐次误差、横向误差、水平向量误差及均值/RMSE/最大绝对误差/有效率；无数据也保留失败记录。

- [ ] **Step 1:** 写测试：朝向 90° 时世界 Y 方向 1 米得到前向 1 米；垂直摆动不增加水平距离；0.2 米杠杆臂绕固定机身转动不产生机身位移；四元数 q/-q 等价；零范数/NaN/水平前向退化被拒绝。
- [ ] **Step 2:** 写评估测试：1.04 对 1.00 米为 0.04；1.06 为失败；误差加不确定度跨越 0.05 为不确定；lost、重置、重复时间戳、空起终点窗口和缺失文件均不能通过；19 次不足验收数量。
- [ ] **Step 3:** 运行 `python3 -m pytest -q tests/test_bumi_odometry_evaluate.py`，确认因缺少实现失败，再实现以上接口。用标准库完成，不引入 NumPy/SciPy。
- [ ] **Step 4:** 重跑同一命令应全部通过；运行 CLI 人工核算一组轨迹，报告必须标注合成数据，不能生成“实测达标”结论。
- [ ] **Step 5:** 提交 `feat(bumi): add independent odometry distance evaluation`。

### Task 2: 预检、隔离融合配置及质量输出

**Files:** Create `experiments/bumi_odometry/preflight.py`, `pipeline.launch.py`, `ekf.yaml`, `quality.py`; update实验 README；Test `tests/test_bumi_odometry_pipeline.py`。

**Interfaces:**
- `inspect_device(rs_module) -> dict`：只枚举设备、流能力、标定和 SDK 版本；不调用 pipeline.start，不更改设备选项。CLI 输出 JSON，缺少 gyro/accel 时非零退出。
- `TrackingQuality.update(kind: str, stamp_ns: int, received_ns: int, segment_id: str, valid: bool) -> str`，`TrackingQuality.state(now_ns: int) -> str`；kind 为 visual/imu，时间使用 ROS 时钟；等待两路有效数据后 tracking，任一路超过 0.5 秒或倒退/无效时 lost。本次 trial 失效锁存，不能在重连后悄悄继续。
- `quality.py` 订阅原始视觉里程计及质量、IMU、融合里程计；输出带 segment_id 的 JSONL/CSV 和 `/bumi_odom/status`。初始化状态 initializing；重启评估进程才建立新的 segment。
- `pipeline.launch.py` 提供 `use_sim_time`、`start_camera`（默认 false）、`calibration_file` 必填项；输出使用带 `bumi_odom_` 前缀的独立 TF frame，避免仅 ROS namespace 隔离但 TF 名仍冲突。标定文件必须声明机身安装旋转/平移及固定安装条件；验证后才启动融合。

- [ ] **Step 1:** 先写预检测试，断言只有 RGB/depth 时拒绝；模拟 rs API 对 pipeline.start/选项写操作直接报错，确保预检不占相机。写质量状态测试覆盖等待、断流、时间倒退、视觉失败、重置、恢复后仍失效及回放时钟暂停。
- [ ] **Step 2:** 运行 `python3 -m pytest -q tests/test_bumi_odometry_pipeline.py` 确认失败，再实现纯逻辑及 ROS 适配层，ROS 导入放在入口，纯逻辑可在无 ROS 的笔记本测试。
- [ ] **Step 3:** 配置 RTAB-Map 只使用视觉输入，关闭其 odom TF，EKF 独占实验 odom TF；EKF 配置 3D 视觉位置/姿态和三轴角速度，禁用加速度。RealSense 启用源时间、对齐深度、CameraInfo、gyro/accel，录制原始流；不启动 SLAM 或闭环地图。
- [ ] **Step 4:** 重跑任务 1/2 测试；在隔离的 Humble 环境用 `ros2 launch ... --show-args` 和回放启动验证参数、topic、TF及消息类型。缺少 ROS 环境时明确标记未运行，不能用配置文本断言替代集成测试。
- [ ] **Step 5:** 提交 `feat(bumi): add isolated RGB-D IMU odometry pipeline`。

### Task 3: 固定依赖、录包及离线回放验证

**Files:** Create `experiments/bumi_odometry/runtime.lock.json`, `validation.md`; update README、必要的任务 1/2 代码与测试。

**Interfaces:** runtime.lock.json 记录 Ubuntu/ROS 发行版、ARM64 架构、精确包版本或源码提交、SDK/固件及完整配置哈希；validation.md 区分本地单元测试、合成轨迹、录包回放、真机测量的证据。

- [ ] **Step 1:** 在隔离运行环境查询并解析可用版本，安装仅在该环境发生。依赖能实际启动且互相兼容后填写 lock；不得写假版本或修改生产容器。无可用隔离环境则记录缺口，继续可执行的本地验证。
- [ ] **Step 2:** 编写明确 topic 列表的 rosbag2 录制命令，包含 RGB、对齐深度、内参、原始 gyro/accel、融合 IMU、TF/静态 TF、视觉里程计及质量、融合里程计和状态；保留相机 metadata，记录时间域、传感器时间戳和序号。启动时记录设备标定与安装标定。
- [ ] **Step 3:** 回放使用 `/clock` 和 use_sim_time，不启动相机。原始传感器重算与已记录估计输出必须 remap/分开运行，禁止同 topic 双发布；连续两次回放比较轨迹与报告。回放中插入/裁剪断流段，确认 lost 后试次失败。
- [ ] **Step 4:** 运行任务 1/2 全部测试，并检查纯数学工具与 ROS 环境下录包导出工具使用完全相同的单位和坐标。真实录包尚未取得时保留回放项“待采集”，不得标为已通过。
- [ ] **Step 5:** 提交 `docs(bumi): document reproducible odometry capture and validation`；只有实际验证的运行时版本才提交 lock。

### Task 4: 约定窗口内的真机采集与一米精度验收

**Files:** Update `experiments/bumi_odometry/validation.md`、必要配置；录包及现场测量放独立数据目录，不提交大型录包或凭据。

**Interfaces:** 使用任务 1 的 manifest/真值 schema；报告记录场景、速度、试次、标定及配置哈希，开发集与验收集分开。

- [ ] **Step 1:** 与用户约定不测试视觉卡片的采集窗口及现场测量人员。只读保存当前相机所有者、容器配置和资源基线，列出恢复命令；先确认相机固定安装与外参。
- [ ] **Step 2:** 在实验环境解决 IMU 枚举缺失：分别核验 udev 可见性与 SDK 版本/构建差异，先枚举、再静止读流；不改生产容器、不更新固件。每项改动独立验证并记录根因证据。失败则不执行移动验收。
- [ ] **Step 3:** 按约定释放原相机所有者并启动唯一实验采集，录 60 秒静止及人工转动对照；确认时钟、标定、质量状态和资源占用，完成后恢复原采集并检查相关卡片可用。
- [ ] **Step 4:** 少量开发集完成调参并冻结配置，然后采集至少 20 次独立约 1 米验收，覆盖至少两个室内场景、不同朝向及拟使用速度范围。现场使用现有手动方式移动，同一机身参考点做独立起终点测量；实验进程不发送运动命令。
- [ ] **Step 5:** 用任务 1 CLI 输出全部试次报告。依据规范给出通过、未通过或不确定；报告不等同于自动距离控制能力。只有无失败且精度达标才提议下一阶段距离反馈控制。
- [ ] **Step 6:** 重跑变更涉及的测试，记录真机证据和恢复结果；提交 `docs(bumi): report measured one-meter odometry accuracy`。没有实测数据时不创建宣称完成的提交。

## 执行与完成边界

建议采用 Native：由当前会话逐项实现和验证，最后做一次独立代码审查。任务 1–3 的本地部分可先进行；任务 4 需要单独约定采集窗口和现场真值测量。无法访问真机或未取得真值时，只交付已验证工具与明确的待测项，不声称 ±5 厘米目标完成。
