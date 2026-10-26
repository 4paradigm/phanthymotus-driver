# Bumi 实时称重 Card（待真机验证）

画布操作顺序：

1. **曲肘**：空载并保留挂具，执行后等待 `ready`。机器人应保持曲肘，程序记录空载力矩。
2. **测量重量**：先点击执行，看到“实时测量已启动”后，再轻轻挂上物体。
   监视窗口中的 `mass_kg` 持续变化，`unit` 为 `kg`；`stable=true`、`valid=true`
   时读取重量。`stable=false` 是变化中的临时读数。
3. **取下物体，再点击本卡的画布停止**：结束实时采样，退出回放，核验手臂自然下垂。
   只有确认下垂后返回 `finished / home_confirmed=true`。不需要单独录制收臂动作。

不要先挂物再点击测量；程序会检查点击时是否空载，且不会用挂载数据重新去皮。

## 首次真机准备

本PR尚未部署或验证机器人。先由Master验证以下事项，再填写
`resource/payload_mass_profiles.example.json`：

- 一个曲肘示教编号 `motion.bend_recording_id`，动作后能够持续保持，直到停止。
- 停止回放后手臂自然回到下垂姿态；记录 `motion.home_q` 和现场证据，确认
  `motion.return_on_stop_verified`。归位通过已有停止回放接口完成，不新建收臂示教。
- 真实机器人和SDK/固件身份、motor ID映射、姿态、机身方向、温度与载荷实验范围。
- 力矩反馈的刷新、载荷响应和错误语义。SDK getter没有源时间戳，读取成功不能
  证明传感器刷新；默认拒绝host-only反馈，核实后才可设置
  `accept_host_only_freshness=true` 并填写 `feedback_evidence`。非零error不被忽略。
- 需要在mode 23保持时，profile必须提供 `playback_hold_verified=true` 和 `hold_evidence`。

**录制动作会自动回到下垂姿态，并不能证明它能保持曲肘。必须先验证保持能力。**
不能保持时，本卡会报姿态/模式错误，不循环重放，不下发全身低层关节命令来补保持。

配置保存于持久目录 `/opt/phanthy-motus/data/bumi/weighing.json`。卡片齿轮可更改
该绝对路径。`hardware_id`、`sdk_version`须与config.yaml或部署环境变量
`BUMI_PAYLOAD_ROBOT_ID`、`BUMI_PAYLOAD_SDK_VERSION`一致。
模板未填真实参数时不能开始动作；初始model为null。

## 实时读数与稳定状态

- 沿用已有StatePlugin的原始约10Hz反馈，不新建SDK控制器。数字最多约5Hz更新。
- 空载基线在曲肘阶段记录；点击测量后只读取新反馈，不重新去皮。
- 变化中以最近约0.3秒的数据平滑显示，`provisional=true`、`valid=false`。
- 完整窗口（默认3秒）满足观测数、力矩波动、姿态、速度和机身稳定条件后，
  使用整个窗口得到稳定读数。物体变化后重新等待稳定。
- 低于已验证分辨率时显示0 kg并标记 `below_resolution=true`，不表示精确零质量。
- 超量程或异常力矩响应不输出有效重量。反馈中断、保护、姿态/模式或配置变化会清空
  读数并终止测量；取下物体后仍可点击停止退出回放、核验归位。
- 测量启动的ACP完成事件只表示流已启动，不等待用户未来挂载，也不阻塞到点击停止。

称重会话保留身体动作控制权，阻止本驱动内的其他姿态、示教和底盘流式命令。
外部程序不受此互斥控制。停止时，已发出的回放命令若还未报告启动，会继续等待
启动反馈后停止；确认不了时保留占用并返回 `STOP_UNCONFIRMED`，不提前放行。
驱动关闭/其他动作的紧急停止可中断任务；与本卡画布停止并核验自然收臂不同。

## 已知质量标定

没有model时仍可采集：实时读数为null、状态 `NOT_CALIBRATED`；稳定窗口自动保存
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
新增实现仅有Python源码，无新增apt/pip依赖。离线标定工具留在开发仓库，不打入运行镜像。
