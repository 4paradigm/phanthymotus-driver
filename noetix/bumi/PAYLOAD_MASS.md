# Bumi 固定曲肘估重 Card（待真机验证）

`payload_mass` 提供三个业务操作：**曲肘 prepare → 计算重量 weigh → 测量完毕 finish**。
结果以 `mass_kg`、`unit: kg` 显示，并通过 `/<namespace>/payload_mass/result`
的 `data/json` 输出供画布监视。`start/stop/info` 为生命周期和诊断操作。

## 实现原理

固定相同姿态、挂点、控制参数与机身方向，关节重力矩差近似为
`Δτ = J(q)^T · m · g`。SDK只声明了 `MotorState.tau`，没有证明其为经过
标定的 Nm，也没有证明反馈的零值、误差码和刷新语义。因此本版不假设力臂或
直接除以9.81，而是用已知质量拟合各关节的有符号响应 `k`：

`m = Σ(w_i k_i Δτ_i) / Σ(w_i k_i²)`。

prepare 每次在曲肘后采集新的空载基线（挂具保留），weigh 采集挂载数据并与
本次空载相减。采用中位数及MAD噪声、关节筛选、独立留出试次验证、响应向量
残差及量程/分辨率检查。读数低于分辨率、不在标定范围或不稳定时不显示kg。
这是一种固定实验条件下的净质量估算，不保证尚未实测的准确度。

## 手臂动作的边界

现有高层SDK没有任意手臂关节位置接口。低层 `set_joint()` 要求全身21个关节
命令，不能假设只改肘关节就不会影响站立。本版使用现有 `action_recording`
的两个**现场已录制并验证**的编号：曲肘保持、双臂下垂归位。
程序不创建、不覆盖示教槽，也不发送低层电机命令。

曲肘与归位均检查实际21关节姿态、机身方向、关节速度和工作模式，再采集一个
完整稳定窗口；收到PLAYTEACH命令回执不等于到位。曲肘动作必须保持到下一次
操作，不能是短暂经过目标角度或自动循环摆动。若固件不能提供稳定保持，本方案
会超时，须先解决保持后端，不能仅扩大姿态容差。
mode 23仅在profile明确提供 `playback_hold_verified: true` 和 `hold_evidence`
时允许；归位的 `home_workmode` 可为2或23。进入新示教前使用已有停止回放接口，
只从walking模式2播放；不自动使能、站起或解除保护。

称重会话独占本驱动的身体动作；移动/姿态/示教及导航流被阻止，停止操作可取消
称重。失败时仍保留占用，允许finish或stop处理。stop不会自动让挂物的手臂下落；
它调用已有回放停止接口，未确认停止时保留占用。外部厂商App或其他进程不受此锁
控制，现场测试不能让它们同时控制机器人。

## Ubuntu 24.04本地检查

在仓库根目录执行（算法与模拟测试无需SDK/ROS硬件）：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  noetix/bumi/tests tests/test_bumi_bundle.py tests/test_bumi_loco_servo.py \
  tests/test_bumi_odom.py tests/test_bumi_camera_specs.py -q
git diff --check
```

关闭pytest自动插件加载是为了避免开发机ROS Jazzy与Conda Python插件互相加载。
机器人部署沿用已有ROS Humble容器和SDK构建，Ubuntu24.04宿主不必直接运行SDK。

## 首次真机配置

本PR尚未进行机器人测试。配置模板
`resource/payload_mass_profiles.example.json` 中没有预填真实姿态、示教编号、质量
上限或标定系数；直接复制模板不能开始动作。完成以下现场工作后才可称重：

1. 核对机器人/SDK固件身份、实际SDK motor ID映射、力矩反馈与error语义。现有
   共享StatePlugin提供10Hz原始getter观测；getter没有源时间戳，读取成功不是源
   仍在刷新的证明。默认拒绝host-only；只有现场核实刷新与载荷响应后，才能设置
   `accept_host_only_freshness: true` 并记录 `feedback_evidence`。有效独立样本数
   始终为未知，不按轮询次数宣称测量置信度。非零错误不被静默忽略。
2. 通过现有action_recording录制并验证曲肘保持和双臂下垂动作，使用空闲编号；
   核实重放与停止行为，读取目标姿态与模式，填写 `motion`，保存实际验证证据。
3. 保持同一挂点/挂具与姿态，填写profile中实测的姿态、温度、抖动、超时、实验
   质量范围及误差验收条件。identity须与`config.yaml`或部署环境变量
   `BUMI_PAYLOAD_ROBOT_ID`、`BUMI_PAYLOAD_SDK_VERSION`匹配。
4. 放置配置于持久目录 `/opt/phanthy-motus/data/bumi/weighing.json`。
   卡片齿轮可更改该绝对路径；活动会话中禁止改配置。初始 `model` 为null。

## 现场标定（卡片仍保持三步界面）

无model时prepare可采空载；weigh保存挂载数据并返回`NOT_CALIBRATED`，不伪造kg。
从prepare/weigh结果的`trial_file`取得文件：

- 至少2个独立空载prepare试次，每次卸载后重新开始完整会话。
- 至少2种不同的正质量，每种至少2个独立重新挂载的训练试次。
- 另采独立留出验证试次，覆盖训练质量范围两端。用电子秤提供真值。
- 同一个完整试次不能重复、复制或拆分到训练和验证。每次使用相同挂具与姿态。

将文件复制到开发机，创建manifest（实际至少8条）：

```json
[
  {"file":"/absolute/empty1.json","mass_kg":0,"role":"train","independent":true,"note":"第一次独立空载"},
  {"file":"/absolute/load1.json","mass_kg":0.3,"role":"train","independent":true,"note":"独立挂载；电子秤真值"}
]
```

在本地运行离线工具，输出为新文件，避免直接覆盖活动配置：

```bash
python3 noetix/bumi/calibrate_payload_mass.py \
  --config /absolute/weighing.json --manifest /absolute/manifest.json \
  --output /absolute/weighing-calibrated.json
```

工具按整试次验证，反馈不可辨识、误差超阈值或数据重复时拒绝生成模型。
返回的验证误差仅针对已接受的稳定试次，不表示现场总体成功率。
成功后结束卡片会话，再将输出安装为新的配置；prepare重新记录空载。

## 日常三步操作与验收

1. **曲肘**：先保持空载且保留挂具，点击执行。等结果`ready`，确认已到位并
   稳定，程序已记录空载力矩；这一步不能带着未知物体去皮。
2. **计算重量**：挂好物体，物体不接触地面、身体或人手，待稳定后执行。
   结果有效才显示kg；每次重新计算会先清掉旧值。准备有效期最多5分钟，反馈
   中断、姿态/模式/配置变化或温度漂移会使基线失效，结果也会过期。
3. **测量完毕**：先取下物体，再执行；仅核验目标下垂姿态后报告归位完成。
   会话结束/重启不保留本次空载基线。

盲测时不向卡片提供待测物体真值，用独立电子秤比较。记录所有失败尝试，验证
空载漂移、不同重量、物体晃动、反馈断开、保护模式、取消、并发动作及收臂行为。
只有实际完成这些检查后，才能给出该机器人/挂具/姿态下的有效量程和误差。

代码PR待真机验收；按入职流程请求bot构建镜像，在目标Bumi部署测试后再验收。
