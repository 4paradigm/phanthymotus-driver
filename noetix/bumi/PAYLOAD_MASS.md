# Bumi 手臂载荷估重 Card（静态第一版）

本卡 `payload_mass` 通过已有 HighController 关节状态做**固定挂点、固定姿态的经验估重**。
实现了静态采集、空载去皮、独立已知质量试次、拟合/验证、kg 输出、取消、数据存档和回放。
`slow_probe` 返回 `UNSUPPORTED`：目前没有已验证的、保留整机稳定控制的单臂运动后端。
本卡不发送运动、使能、失能、模式切换或低层电机命令；不会替人把手臂抬到测量姿态。

代码和 mock 通过不等于真机验收。没有预设手臂安全载荷、称重精度或默认可用姿态。
几何 Jacobian / N·m 物理模型、主动往复及动态辨识不在本实现内。

## 源码与版本边界

- 开发基线：上游 `8a80d835`。新增功能独立分支，不依赖此前的动作编排或站台 Skill。
- 仓库绑定：`noetix_sdk_bumi/src/highcontrol_pybind.cpp` 暴露
  `get_joint_state/get_imu_data/get_mode`；MotorState 含 `pos/vel/tau/motor_id/error/temperature`。
- [厂商网页](https://web.noetixrobotics.com/docs/) 当前展示
  `subscribe_robot_hardware_status`，与随仓库绑定有差异。本卡只接已有 StatePlugin getter
  路径，既不覆盖厂商回调，也不另建 SDK 实例。若升级到只有回调的 SDK，先适配共享状态
  分发层再接入；不能直接根据网页替换运行中的 ABI。
- 默认 `tau` 单位为 `sdk_raw`；不乘减速比、扭矩常数，也不宣称它是独立力矩传感器。
  只有查证具体设备后才可设置 `Nm` 及 `units_evidence`；仍使用经验模型。
- SDK/ROS 只在驱动集成层加载。纯算法、mock、回放和 HTTP 诊断均可在 x86 Ubuntu 上运行。

## 开发机验证

在 Driver 仓库根目录执行（需要现有 Python 3、pytest、PyYAML；不自动安装系统包）：

```bash
bash scripts/check_bumi_payload_mass.sh
```

它执行完整 mock 和新增/Bumi/打包相关测试。单独保留一组可回放的模拟档案：

```bash
python3 noetix/bumi/payload_mass_cli.py mock --output-dir /tmp/bumi-payload-demo
```

模拟构造了带挂具零点、多轴正负响应、不敏感轴和噪声，完成两次空载、两种质量各两次
训练、两种质量各一次独立验证，再用另一质量的反馈估计。输出 `simulation_only=true`。
合成系数、质量、姿态和阈值只属于模拟，不能复制成真机配置或作为承载能力。

用 mock 输出中的 `trial_id` 回放，`estimate` 试次不含待测真值：

```bash
python3 noetix/bumi/payload_mass_cli.py replay \
  --data-dir /tmp/bumi-payload-demo --profile-id sim_left --trial-id REPLACE_TRIAL_ID
```

回放读取原试次关联的模型及历史基线，不使用当前重新去皮后的模型，不连接机器人。

## 只读真机诊断（阶段 A）

先由现场确认目标机器人身份、系统、架构和容器版本。以下命令在已确认的 Bumi 宿主上执行，
不含 SSH 密码，不要求另起 LowController。部署新驱动后，现有 StatePlugin 必须保持启用。

```bash
docker inspect --format '{{.Config.Image}}' embodied-noetix-bumi
docker exec embodied-noetix-bumi python3 -c 'import platform; print(platform.platform(), platform.machine())'
docker exec embodied-noetix-bumi python3 -c 'import hashlib, pathlib, highcontrol_py; p=pathlib.Path(highcontrol_py.__file__); print(p, hashlib.sha256(p.read_bytes()).hexdigest()); print([n for n in ("get_joint_state","get_imu_data","get_mode","subscribe_robot_hardware_status") if hasattr(highcontrol_py.HighController,n)])'
docker exec embodied-noetix-bumi python3 /work/payload_mass_cli.py diagnose --seconds 10
```

诊断只调用 `payload_mass.info`，不初始化 SDK、不取控制权、不改变机器人状态。
JSONL 含原始反馈、实际 motor_id 归一顺序、主机接收间隔、读错误、重复源标记计数及变化计数。
需要保留时在宿主将输出重定向到本卡数据目录内的新文件。

**新鲜度限制：**当前 getter 没有源时间戳/序号。`host_observation_hz` 是主机读取频率，
不是设备独立反馈频率；数值不变可能是静止，也可能是缓存，诊断不自动判为死帧。
`freshness_status=host_receipt_only`，`effective_sample_count=null`；不输出伪造 95% 置信度。
有真实源标记的输入会拒绝重复/回退标记，但这不能自动证明端到端延迟或统计独立性。

先确认实际 ID 与 URDF 名称/符号关系、真实反馈更新/延迟、tau 是否随外载可分辨变化、
零点噪声/温漂/饱和情况。不要用增加 Python 轮询频率代替这些核验。tau 恒零或不响应时停止
真实估重路径；本卡拟合会拒绝无可辨识响应，但程序本身无法证明设备的力矩语义。

## 工程配置与允许范围（阶段 B 前）

配置源是独立的工程 JSON 文件，卡片普通测量参数不能扩大其限制。
模板：`resource/payload_mass_profiles.example.json`；模板中的物理量为 `null`、确认位为 false，
故意不能直接测量。复制到宿主挂载目录后，根据现场测量填写：

```bash
mkdir -p /opt/phanthy-motus/data/bumi/payload_mass
# SOURCE_DIR 为已核对版本的源码目录
cp "$SOURCE_DIR/noetix/bumi/resource/payload_mass_profiles.example.json" \
  /opt/phanthy-motus/data/bumi/payload_mass/engineering_profiles.json
```

必须填写/核实：

| 字段 | 含义 |
| --- | --- |
| hardware_id / sdk_version | 该设备唯一标识、实际固件/安装绑定版本或 SHA256；不是本机架构名 |
| hook_id / control_config | 同一固定单点柔性挂具、固定控制模式/增益条件的标识 |
| motor_ids / joint_names | 已核对的该侧肩/肘 SDK ID 与 URDF 名称；不按数组顺序猜测 |
| mapping_verified / mapping_evidence | 实际电机映射的确认及依据 |
| pose_q | 按 ID 0..20 排列的完整 21 关节姿态；同时约束另一侧、腿和腰 |
| quaternion_xyzw / workmode | 核实的躯干姿态、可静态保持的高层模式 0/1/2 之一 |
| pose_tolerance_rad / body_tolerance_rad | 关节姿态和 IMU 姿态容差，分别与配置及基线比较 |
| max_velocity_rad_s / max_gyro_rad_s | 全身关节/躯干静止阈值 |
| max_torque_span / noise_floor_raw | 连续窗口力矩跨度、实测噪声下限；单位与 tau 一致 |
| torque_abs_limit | 工程人员确认的测量饱和/异常阈值，不等于推算的载荷能力 |
| max_temperature_c / max_temperature_drift_c | 温度门槛、与空载温度的允许变化 |
| max_mass_kg / load_range_confirmed | 现场确认的实验上限及确认位；无默认安全质量 |
| feedback_verified / feedback_evidence | 载荷响应、更新率/延迟核验依据 |
| accept_host_only_freshness | 明确接受 getter 来源活性无法在线证明的限制；默认 false |
| max_calibration_error_kg | 独立试次验收误差上限；不是本卡保证的精度 |

用 getter 采空载也须明确接受 host-only 限制；`tare` 不要求提前确认载荷响应，但要求完整
测量条件和映射。`calibrate/fit/estimate` 还要求载荷范围和反馈验证已完成。
`max_mass_kg` 是实验限值；最终可输出的范围还会被独立验证范围收紧。

`config.yaml` 的 `payload_mass` 下配置实际 `hardware_id/sdk_version`；也可在目标设备
`deploy/service.yml` 的 environment 中设置 `BUMI_PAYLOAD_ROBOT_ID` 和
`BUMI_PAYLOAD_SDK_VERSION`。它们必须与 profile 一致。该身份是部署声明，SDK getter 未提供
可自动核对的设备序列号；部署人员须核对实物，不能把同一配置搬到其他机器人。

可从卡片左侧配置入口修改 `profiles_file` 的绝对路径。完整配置变更会改变指纹，旧基线和
模型随即不再适用。不设置挂点、姿态、限值和现场确认，就只显示诊断，不自动允许称重。

左臂候选 ID 0/1/2/3 对应 `l_arm_pitch_joint/l_arm_roll_joint/l_arm_yaw_joint/l_elbow_pitch_joint`；
右臂 10/11/12/13 对应 `r_*`。这只是仓库基线，`mapping_verified` 必须来自实物核对。
左右侧、挂点、姿态、模式、硬件或控制条件不同，必须另建 profile 并独立标定。

## 一张卡的实际使用

卡片类型为 `processor`：它处理数据并写入标定档案，不驱动电机；使用单实例、一个
`/<namespace>/payload_mass/result` 的 `data/json` 输出。参数随 action 展示。
启动项目只使卡片就绪，不自动去皮、标定或测量。可在输出监测窗口看异步最终结果；立即
返回的 `running` 只表示任务已创建。

先由现场放好手臂、挂好固定挂具并确认能稳定保持。物体不得碰地、靠躯干、被桌面或人托住。
程序不能自动辨别所有额外支撑、挂点滑移、控制器增益变化或悬挂物摆动。

每次点击执行对应一次独立试次；采样时间默认 5 秒，允许 1–30 秒，必须实际达到
`min_samples` 且覆盖整个连续稳定窗口。它不会把多个短暂停顿拼成一个稳定窗口。

以下 JSON 是 `tools/call` 的 arguments，工具名均为 `payload_mass`。

1. **空载**：卸掉物体、保留同一挂具。至少做两次独立卸载/确认。

   ```json
   {"action":"tare","arm":"left","mode":"static","profile_id":"left_hook_A","sample_duration_s":5,"confirm_empty":true,"trial_note":"独立空载第1次，同一挂具"}
   ```

   第二次改试次说明，再执行。两次空载响应若显著不一致则拒绝，不用新零点掩盖问题。
   每次成功 tare 都创建新的 baseline_id 并使旧模型失效，因此已知质量采集必须在最后一次
   tare 之后进行。失败采集不会覆盖有效基线或模型。

2. **已知质量训练**：使用现场确认范围内至少两个不同的正质量，每种至少两次独立重新挂载。
   真值来自独立电子秤。下面用占位说明，实际 JSON 中填有限数字，不能填字符串。

   ```text
   action=calibrate, arm=left, mode=static, profile_id=left_hook_A
   known_mass_kg=<电子秤真值>, role=train
   independent_trial_confirmed=true, sample_duration_s=5
   trial_note=<该次重新挂载及电子秤真值来源>
   ```

   零质量也可作为额外校准试次，但不能替代两次 tare 与两个正质量训练点。

3. **整次独立验证**：重新挂载两种质量，`role=validation`，其质量范围覆盖训练端点。
   不能把同次采集相邻帧拆成训练和测试。确认位只记录人的操作声明，程序无法证明是否
   实际重新挂载；必须遵守现场试验记录。

4. **拟合**：

   ```json
   {"action":"fit","arm":"left","mode":"static","profile_id":"left_hook_A"}
   ```

   仅训练试次拟合 signed `Δtau ≈ k*m`。根据响应/噪声选择有效轴，剔除不敏感轴；
   独立验证检验质量误差和力矩向量残差。响应不可辨识、不单调、误差/残差超限则拒绝。
   只有拟合和验证成功才原子替换当前模型指针。

5. **未知质量估计**：在相同条件下独立悬挂待测物体并等待稳定。

   ```json
   {"action":"estimate","arm":"left","mode":"static","profile_id":"left_hook_A","sample_duration_s":5}
   ```

   estimate 拒绝 `known_mass_kg`。当前 Core 的通用表单可能发送隐藏字段：先回到 calibrate
   清空真值输入框，再切换 estimate；其他无关已声明字段会被忽略。不要把答案留在表单里。
   成功输出 `valid=true,mass_kg,unit=kg`；分辨率取窗口噪声与独立验证误差，按它选择显示小数位。
   噪声尺度不除以 sqrt(帧数)，不会把相关帧算成独立样本。

6. **停止**：卡片停止按钮或 `{"action":"stop"}`。仅取消本卡任务，不发失能/零力矩/回零。
   正在采集再次启动返回 BUSY。驱动销毁取消并清理线程、ROS 节点，不自动恢复任务。

耗时动作按 ACP 返回 `action_id`，MCP 成功写出并 flush 后才允许 worker 继续并报告一次终态；
未交付的响应取消任务而不注册伪完成。静态结果不代表硬件动作完成回执。

## 结果、失效与故障

成功字段包括质量、手臂/模式/profile/calibration ID、质量范围、分辨率、样本数、原始单位、
pose_match、freshness_status、每轴 MAD 离散程度、归一力矩残差、独立试次验证误差和限制说明。
`effective_sample_count=null`，不宣称统计独立样本数。

- `BELOW_RESOLUTION`：含轻微负信号，返回 null，不强行截为“0.000 kg”。
- `OUT_OF_RANGE`：超过验证范围或显著负响应，返回 null，不做大载荷外推。
- `POSE_MISMATCH/BODY_MISMATCH/MODE_MISMATCH`：与配置或基线条件不一致。
- `MOTOR_FAULT/OVER_TEMPERATURE/SATURATED/TEMPERATURE_DRIFT`：异常/温漂，不恢复保护状态。
- `UNSTABLE/INSUFFICIENT_SAMPLES/STALE_SOURCE/DATA_ERROR`：未得到连续有效窗口。
- `NOT_CALIBRATED/PROFILE_UNVERIFIED/IDENTITY_MISMATCH`：缺少对应条件或现场证据。
- `CONFIG_CHANGED/BASELINE_EXPIRED/STORAGE_CORRUPT`：需要检查工程文件、重新标定或恢复档案。
- `UNSUPPORTED`：slow_probe 未实现运动后端，未发送任何运动命令。

任何失败会清空当前有效质量。成功结果默认最多保留 30 秒，可由工程配置的
`result_max_age_s` 调整（上限 300 秒）；检测到接收中断、读取错误、姿态/力矩条件变化、
基线过期或配置变化，会由 ROS 定时器更新为 null，info 调用也执行相同检查。
getter 静默返回旧缓存仍可能无法发现，界面和日志应保留该限制。

## 数据与追溯

宿主 `/opt/phanthy-motus/data` 已挂载到驱动相同路径，无需更改其他功能目录：

```text
bumi/payload_mass/
  engineering_profiles.json           # 现场工程配置
  profiles/<profile_id>.json          # 当前基线/模型指针
  profiles/tare_<uuid>.json           # 不可覆盖的历史基线
  trials/<profile_id>_<uuid>.json     # 原始 q/dq/tau/温度/故障/源标记/主机时间、条件、摘要
  calibrations/<profile_id>_<uuid>.json # 不可覆盖的模型、训练/验证试次 ID、指标、配置快照
```

文件有版本和 SHA256 校验。原始试次与模型用独占新文件名，不覆盖已有档案；指针使用临时文件、
fsync、原子替换。损坏文件报错，不悄悄换成模拟或零点。每 profile 最多读取 1000 个试次，
需要归档时只处理本卡目录。不要删除音视频、动作编排或站台 Skill 数据。
原始试次不是硬件准确度证据；控制配置/挂具确认等人工条件也需随试验记录保存。

## 构建、PR 与部署

Dockerfile 已包含四个新 Python 模块；`resource/` 包含未验证模板。
`main.py` 注册插件、`driver.yaml.cards` 声明 processor、`config.yaml` 默认启用卡片。
没有工程配置时可发现并查看诊断，但不会估重或动作。

提交 feature 分支 PR 后按团队流程评论 `/request_bot_review` 获取实际镜像和部署指令。
`build.sh` 在仓库凭证齐备时会推送镜像并登记市场，不能把它当成只做本地验证的命令。
需要人工本地构建时，从包含根 `common/` 的临时上下文构建，与 `build_context_extras` 约定一致：

```bash
# 在仓库根目录；这里只构建带本功能的本地镜像，不推送/登记市场
PAYLOAD_CONTEXT=$(mktemp -d /tmp/bumi-payload-build.XXXXXX)
cp -a noetix/bumi/. "$PAYLOAD_CONTEXT/"
cp -a common "$PAYLOAD_CONTEXT/common"
docker build --platform linux/arm64 -t bumi-payload-mass:local "$PAYLOAD_CONTEXT"
```

这会构建真实 ROS/SDK ARM64 驱动，可能下载大依赖。x86 mock 不需要该步骤。
在已确认的目标设备上使用实际构建得到的镜像；先保存当前镜像引用与 compose 配置，只替换
`noetix-bumi` 服务镜像，保留现有挂载与 DDS 环境，不重建其他服务或整机控制器。
重启会中断该 Driver 的所有能力，必须安排现场维护窗口。

部署后核对 tools/list 中有且仅有一张 payload_mass，检查现有音视频/遥测/移动卡未退化，
再按阶段 A/B 做真实诊断、标定与独立测试。不能将容器 running 当作称重功能验收。
回退时换回记录的旧镜像；本卡档案独立保留，不覆盖/恢复整个共享数据库。

## 真机尚待验证与验收记录

本轮不连接或操纵真机，尚无硬件称重精度结论。现场依次记录：

1. 具体 SKU/设备 ID、绑定/固件版本、单位及信号来源、实际 ID/符号、反馈活性/延迟。
2. 允许实验范围、固定挂点/挂具、静态姿态与控制条件、温度及噪声阈值。
3. 独立空载、训练和验证试次；另用未参与拟合的物体做盲测。
4. 电子秤真值对比：MAE、最大绝对误差、同质量重挂重复性、包括不稳定/超时在内的
   **全部尝试失败率**。模型里的 validation 指标只覆盖被接纳的稳定校准试次，不能代替该失败率。
5. 左/右臂分别验收；画布 kg/无效状态截图、取消/故障/断联结果、现有卡片回归。
6. 将实际证据交对应 Master 验收；此前其他功能的验收不适用于本卡。

阶段 C 主动“掂一下”保持未开放：还需验证运动后端、带载稳定/余量、A/B 各自标定、运动入口
仲裁及停止收尾。不能调用 LowController 给其他电机填零，也不能直接复用未确认示教编号。

## 原始 DDS 反馈诊断（不改动运行中的 Driver）

当 getter 读到零力矩或未解释的错误码时，可单独使用
`scripts/bumi_payload_raw_probe.py` 检查原始 `RobotStatus::StatusData`。
它不导入或初始化 HighController/LowController、不注册SDK回调、不发运动指令，
不修改 Driver 文件、不重启容器；只创建一个原生 DDS participant 和状态订阅。

工具复用镜像内已有的 g++、离线 idlc 和 DDS 动态库，不安装软件包、不升级 SDK。
从厂商头文件复原状态 IDL，用厂商离线编译器生成缺失的类型实现；编译前逐字节核对
全部六种结构的12份 XTypes map/info 元数据。任何不一致报 SCHEMA_MISMATCH，
不会尝试连设备。不会为了链接方便引入包含控制器代码的 libdriver.a。

将脚本复制到机器人宿主的 `/tmp/bumi_payload_raw_probe.py` 后执行：

```bash
docker exec -i embodied-noetix-bumi python3 - --domain 0 --seconds 6 --summary-only \
  < /tmp/bumi_payload_raw_probe.py
```

默认SDK根目录为 `/work/noetix_sdk_bumi`。domain 0 只是厂商原生域的候选，不等于
ROS_DOMAIN_ID=42；未发现发布者时应核对域、网卡、peer、协议和 QoS，不能据此认定
“设备力矩为零”。只接纳一个状态发布者，多来源则报 AMBIGUOUS_WRITER。

输出包括安装绑定 SHA256、类型元数据校验、实际 DDS 消息数、原始电机时间戳变化数、
左右手臂各轴力矩 min/max、实际 motor_id 映射检查和**未经SDK uint8截断的原始错误码**。
错误码只保留原值，不推测1或其他未说明代码的含义；质量始终为null。
不加 `--summary-only` 可同时查看发现的话题/类型和每秒原始关节快照。

这些数据用于排查，而不是称重或运动：

- SDK getter 与原始DDS明显不一致：继续检查协议/SDK解析、位宽、时序和版本。
- 两者都为零：仍需核对姿态、设备是否真实上报力矩及固件语义，不能直接归因固件。
- DDS计数增加：仅证明收到了DDS消息，不证明每个电机测量独立刷新。
- 自然下垂、空载零值不能单独证明称重不可行；也不能在零响应上拟合有效模型。

原生集成测试仅在开发机回环接口、独立域173中运行，发布的是测试状态数据，不连接机器人。
需要已有原生运行库时，可显式提供路径：

```bash
BUMI_PROBE_TEST_LIBRARY_DIR=/path/to/dds/lib \
BUMI_PROBE_TEST_IDLC=/path/to/dds/bin/idlc \
python3 -m pytest tests/test_bumi_payload_raw_probe.py -q -p no:cacheprovider
```

未提供路径时，纯Python检查仍运行，原生DDS测试标记skip；不能将skip写成通过。
原生测试验证带符号力矩、零数据、无发布者、重复来源、ID错误、NaN、原始错误码保留和取消。
这些是测试数据，不代表真机反馈已验证。
