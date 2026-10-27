# Bumi 实时称重 Card（待真机验证）

该实验卡默认关闭。上机测试需显式启用；普通部署不会注册该卡或启动称重DDS进程。
测试部署时，在PR构建结果给出的镜像启动命令中使用：

```bash
curl -fsSL https://motus.phanthy.com/run-pr-image.sh | \
  sudo env BUMI_ENABLE_EXPERIMENTAL_PAYLOAD_MASS=1 bash -s -- <PR构建结果中的镜像地址>
```

也可在自己的部署配置中显式设置 `plugins.payload_mass.enabled: true`。
修改卡片齿轮参数不能启用一个未注册的卡片。启用后按下述五分钟流程测试。

本版已改为直接读取机器人原始DDS状态：同一帧的关节力矩、姿态、IMU、模式及
`motorstatearray.timestamp`一起接入。无需更换SDK或单独运行采集脚本。
驱动启动时自动发现唯一状态源；默认原生DDS域为0，独立于ROS域42。
源时间戳重复时不产生新样本；冻结超过0.5秒清空重量并终止测量。
缺少可验证源时间戳的数据仍返回 `FRESHNESS_UNVERIFIED`，不使用getter缓存替代。

画布操作顺序：

1. **曲肘**：空载并保留挂具，执行后等待 `ready`。机器人应保持曲肘，程序记录空载力矩。
2. **测量重量**：先点击执行，看到“实时测量已启动”后，再轻轻挂上物体。
   监视窗口中的 `mass_kg` 持续变化，`unit` 为 `kg`；`stable=true`、`valid=true`
   时读取重量。`stable=false` 是变化中的临时读数。
3. **取下物体，再点击本卡的画布停止**：结束实时采样，退出回放，核验手臂自然下垂。
   只有确认下垂后返回 `finished / home_confirmed=true`。不需要单独录制收臂动作。

不要先挂物再点击测量；程序会检查点击时是否空载，且不会用挂载数据重新去皮。

## 五分钟快速版：直接操作

启用后的默认模式为 `setup_mode: quick`。不需要填写工程JSON、机器人身份或人工关节角度。

1. 用现有action_recording录制：双臂下垂 → 曲肘 → 原姿态停留约5分钟，保存一个编号。
2. 在payload_mass卡片齿轮里填写该 `recording_id`，选挂载侧 `arm`（默认left），保存。
   `hold_duration_s`默认300秒，与录制的总时长对应。
3. 空载、保留挂具，选择 **曲肘** 执行，等 `ready`。程序自动记录起始下垂姿态、
   检测实际曲肘，并等待约3秒连续稳定的姿态与空载力矩后记录基线，无需手填角度。
   动作仍在进行时重新等待稳定，不把刚开始弯曲的过渡角度当成最终曲肘目标。
4. 选择 **测量重量**，先点击执行，再挂一个电子秤称过的参考物体（不超过0.5kg）。
   等约3秒稳定后，选择 **校准参考重量**，在 `known_mass_kg` 填实测kg，执行。
   例如电子秤读数200g，填写0.200。
5. 不停止测量，取下参考物体，换挂待测物体；监视窗口中的 `mass_kg` 持续刷新，
   `stable=true`后读取。取下物体后数字回到接近0。
6. 在五分钟回放到时前，取下物体，点击本卡画布 **停止**，程序退出回放并检查自然收臂。

每次重新开始曲肘会话，会重新采空载并清除上次快速校准；本次会话中只需校准一次。
快速版使用一个参考物体形成实验性比例换算，输出标记 `experimental=true`、
`quality=experimental_single_reference`、`validation_status=not_validated`，不等同于精度验收。
未校准时只显示采集状态和稳定数据文件，不生成虚假的kg。

`remaining_s`显示本次示教剩余时间。到时或实际姿态改变后，旧重量清空；程序不会循环
重新播放示教。当前版本沿用示教中记录的五分钟保持段，不切换全身低层控制。

快速版沿用motion_state的错误语义：已记录的故障码会终止测量；未说明的状态值
保留在 `unknown_status_codes` 和原始反馈中，不被标成“已验证无故障”。

## 工程配置模式（后续精度优化可选）

齿轮中选择 `setup_mode: profile` 后，使用 `profiles_file` 指向原工程配置。
`resource/payload_mass_profiles.example.json`为此模式的模板，需要实测姿态和参数。
身份仍与config.yaml或部署环境变量 `BUMI_PAYLOAD_ROBOT_ID`、`BUMI_PAYLOAD_SDK_VERSION`
匹配；mode 23需要原有保持证据。该模式不使用单参考快速校准。

工程配置模式的安装位置默认 `/opt/phanthy-motus/data/bumi/weighing.json`。
默认快速版无需创建这个文件；不需要单独录制收臂动作。

## 实时读数与稳定状态

- 原生只读DDS程序接收连续源帧，取消40Hz抽帧；机器人此前原始DDS实测约400Hz。
  数字仍最多约5Hz更新，接收帧数不等于独立传感器样本数。
  不新建HighController/LowController，不修改现有动作控制或StatePlugin的显示。
- 空载基线在曲肘阶段记录；点击测量后只读取新反馈，不重新去皮。
- 快速版使用全部接收帧（保留零值）的力矩均值，避免稀疏反馈被中位数抹成0。
  0.5秒时间块用于检查波动，完整窗口至少含4块；误差尺度取分块波动并设置
  单帧贡献下限，不用接收帧数开平方虚构精度。工程模式仍使用原中位数/MAD。
  新旧统计基线不能混用，更新后必须重新空载曲肘。参考校准尝试会保存原始记录。
- 温度与空载基线的比较统一使用窗口中位数，温差门槛保持10°C；绝对温度和故障
  仍逐帧检查。温漂错误会显示具体电机、当前/空载温度及阈值，不据一次过期快照
  推断先前是否真正过热。
- 变化中以最近约0.3秒的数据平滑显示，`provisional=true`、`valid=false`。
- 完整窗口（默认3秒）满足观测数、力矩波动、姿态、速度和机身稳定条件后，
  使用整个窗口得到稳定读数。物体变化后重新等待稳定。
- 快速版关节速度门槛为0.2rad/s；准备及实时称重都检查完整窗口内挂载臂的角度跨度
  不超过0.03rad。该门槛属于实验性测量容差，不改变机器人底层控制或硬件限值。
- 快速版固定角度参考为双臂与腰部；腿部调整后的角度不必与空载时相同。
  固定角度、空载基线和完整窗口的角度跨度使用相同范围；腿部角度不重置窗口。
  腿部速度、全部关节故障/温度、机身朝向/陀螺仪和源新鲜度仍检查。
  工程profile模式继续检查全部21个关节与基线的差异。
- 低于模型分辨率时显示0 kg并标记 `below_resolution=true`，不表示精确零质量。
  快速版分辨率为经验性检测尺度，尚未通过实物误差验证。
- 超量程或异常力矩响应不输出有效重量。反馈中断、保护、姿态/模式或配置变化会清空
  读数并终止测量；取下物体后仍可点击停止退出回放、核验归位。
- 测量启动的ACP完成事件只表示流已启动，不等待用户未来挂载，也不阻塞到点击停止。

称重会话保留身体动作控制权，阻止本驱动内的其他姿态、示教和底盘流式命令。
外部程序不受此互斥控制。停止时，已发出的回放命令若还未报告启动，会继续等待
启动反馈后停止；确认不了时保留占用并返回 `STOP_UNCONFIRMED`，不提前放行。
驱动关闭/其他动作的紧急停止可中断任务；与本卡画布停止并核验自然收臂不同。

## 工程模式多试次标定（可选）

以下离线工具针对工程模式的数据；快速版按上面的单次参考校准操作即可。

工程模式没有model时仍可采集：实时读数为null、状态 `NOT_CALIBRATED`；稳定窗口自动保存
原始数据，并返回 `trial_file`。同一连续稳定区段不会每次刷新都重复存盘。

准备电子秤和同一挂具，采集：

| 试次 | 要求 |
|---|---|
| 空载prepare | 至少2次独立会话 |
| 质量A、B训练weigh | 两种不同正质量，各至少2次独立重新挂载 |
| 质量A、B验证weigh | 另各1次独立重新挂载 |

每次曲肘后先点击测量，再挂物；保存稳定窗口的trial_file。取下物体，停止并收臂。
独立试次不能用同一连续读数拆分或复制替代。
将文件复制至Ubuntu，创建manifest，填写各文件的电子秤真值：

```json
[
  {"file":"/absolute/empty1.json","mass_kg":0,"role":"train","independent":true,"note":"独立空载1"},
  {"file":"/absolute/load1.json","mass_kg":0.3,"role":"train","independent":true,"note":"独立挂载；电子秤真值"}
]
```

实际manifest至少8条。开发机执行：

```bash
python3 noetix/bumi/calibrate_payload_mass.py \
  --config /absolute/weighing.json --manifest /absolute/manifest.json \
  --output /absolute/weighing-calibrated.json
```

工具拒绝重复、不可辨识、噪声/残差或独立验证误差超阈值的数据。成功后先停止会话，
再安装新配置并重新曲肘采空载。未知物体测量接口不接受真值参数。

固定姿态力矩差近似 `Δτ = J(q)^T · m · g`。本版不假设SDK tau为Nm，使用已知质量
拟合有符号每kg响应k，以 `Σ(w k Δτ)/Σ(w k²)` 估算净质量。精度与量程须通过实物
盲测确定，不能把模拟测试结果当成机器人精度。

## Ubuntu 24.04离线检查

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  noetix/bumi/tests tests/test_bumi_bundle.py tests/test_bumi_loco_servo.py \
  tests/test_bumi_odom.py tests/test_bumi_camera_specs.py \
  tests/test_common_lifecycle.py tests/test_control_sink.py -q
git diff --check
```

关闭自动插件加载以避开ROS Jazzy/Conda插件冲突。机器人继续使用原ROS Humble容器；
新增原生只读程序使用镜像已有的DDS运行库与构建工具，无新增apt/pip依赖。
构建时校验生成状态类型的全部12份XTypes元数据与仓库SDK一致；不链接动作控制库。
原生程序strip后保留，CMake/编译中间产物在同一构建层删除；build目录只保留两份Python
绑定和原生程序。镜像内 `/work/payload_mass_build_size.json` 记录实际原生程序字节数、
六个运行时Python模块字节数和清理的构建字节数；这些文件字节数不等于完整镜像压缩大小。
2026-10-27读取注册表manifest测得：原生接入前的 `release.261026.1825579` 压缩层合计
501,662,250字节，原生接入但尚未清理的 `release.261026.d65e752` 为503,452,880字节，
增加1,790,630字节（约1.71MiB）。清理后的新镜像实际对比记录在PR正文。
离线标定工具和测试发布器留在开发仓库，不打入运行镜像。

## 无机器人演示

```bash
python3 scripts/demo_bumi_scale.py --output /tmp/bumi-scale-demo
```

运行真实快速版采样/拟合/滚动显示代码，关节反馈和动作由模拟器提供。
演示参考物体0.20kg、未知物体0.35kg、卸载0kg和停止归位；报告带
`simulation_only=true`，不能作为真机精度结果。

原生DDS联调测试位于 `tests/test_payload_mass_feedback.py`。额外设置
`BUMI_TEST_DDS_READER`（已编译的payload_mass_dds路径）与
`BUMI_TEST_DDS_PUBLISHER`（已编译的tests/native_payload_mass_publisher.cpp路径）后，
同一pytest命令会运行真实DDS传输测试，覆盖参考校准、未知重量、卸载、源冻结清数及归位。
测试发布器仅运行在回环网卡、独立域和测试主题，不放入运行镜像。
