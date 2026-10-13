# PiPER 计划二开发验证

日期：2026-10-13。分支：`codex/piper-monitor`。基线：`f5ca598278a3f015cd9503d674a81d2d75b122fa`。

## 结果

- 监控测试 22 项：通过（含现场快照命名适配新增 2 项）。
- 最新指定回归集合：187 passed、114 subtests passed（含 5 项部署适配测试）。
- 实际 Edge headless + 未修改 Core 渲染器：通过；528 个矩阵值对比，无页面脚本错误。
- 浏览器场景：三组实测角度模拟、过期冻结、读取失败提示、恢复、双设备隔离。
- ROS transport 使用假 ROS 对象验证 Core context、QoS、消息序列化、重复关闭及部分初始化失败释放。
- 官方源 URDF 与许可证原始字节 SHA256 核验：通过。
- 监控生命周期中假硬件记录的控制命令数：0。
- 以上开发模拟验收未访问真实反馈。后续现场缓存测试见下节；均未执行机械臂动作、
  模式切换、使能/失能或拍照。随后用户明确授权部署，记录见下节。

Python 3.12 回归命令（pytest、PyYAML、OpenCV 为测试环境依赖，不是本提交新增的驱动依赖）：

```bash
python -m pytest -q \
  tests/test_piper_monitor.py tests/test_piper_monitor_bundle.py tests/test_piper_ten_cards.py \
  tests/test_piper_cards.py tests/test_piper_controls.py tests/test_piper_prepare.py \
  tests/test_piper_deployment.py tests/test_piper_mac.py tests/test_common_lifecycle.py \
  tests/test_dockerfile_copies.py tests/test_vendor_runtime_logging.py
```

初次回归环境缺少 PyYAML/OpenCV，补齐隔离测试环境后通过。独立阶段模型导出合并到 `monitor.py`。
现场部署阶段新增 `monitor_bundle.py`，补充 Docker COPY 与模型资源打包；最终打包回归通过。

## 浏览器模拟证据

- 本机输出目录：`/Users/lei/Documents/ChatGPT/第四范式实习/acceptance/20261013_piper_monitor/`
- `monitor-simulation.png`：两台骨架及中文参数（明确标注模拟反馈）。
- `result.json`：场景、矩阵对比数、模型请求和脚本错误记录。
- 骨架渲染器 SHA256：`2f1f9f7856c36c1a019ad0cf46666fac41ed720b695c2be3d3aabac4967b14f4`。
- 测试中仅调用两次 loopback 假 MCP 的 `model`，没有请求现场 API。
- 证据截图及 Third-party JS 仅在本机，未提交到 Git；可用浏览器脚本重新生成。

## 待集成与用户测试

### 2026-10-13 现场缓存只读测试补充

- 用户要求真实场景测试；通过 SSH loopback 转发复用机械臂 1 现有 `piper_zero_check.info`
  的实时缓存（约 5 Hz），`piper_status.info` 仅补充操作许可/忙碌信息。不调用 `status/read_state`
  或控制动作，不创建第二个 SDK/CAN 连接，不改现场镜像或配置。
- 实际本地 Core 画布顶部「监控」显示官方 PiPER 模型，反馈角度曾读到
  `[22.575, 11.656, -2.914, 16.939, 22.019, -16.416]` 度；时间戳持续更新。
  这是静止姿态与通道验证，未验证真实运动跟随或物理尺寸/方向。
- 现场 SSH 随后发生超时：验证 `skeleton=null`、参数显示读取失败且保留最后有效时间。
  不回退模拟反馈；本地 SSH 转发自动重连。测试期间现场服务被另一项工作切换为
  `arm_control` 版本，旧缓存工具移除；旧缓存恢复验证记录为 `blocked-by-network`，不作为通过证据。
- 随后核对现场构建 `cb368189` 源码，确认 `arm_control.info` 调用无控制副作用的
  `read_monitor_snapshot()`，不连接、不发送命令，不等待动作锁。临时桥接改为仅调用该 info。
  监控适配 `fresh/faults/last_actions` 命名，22 项监控测试与 182 项回归通过。
- 新快照验证通过：两帧真实 CAN 时间戳推进、六轴度/弧度一致、PiPER 六关节模型正确、
  参数显示「新鲜」，记录在 `live-runtime-result.json`；截图 `monitor-live.png`。
  六轴使能、夹爪开度/使能、有效限位、故障及最近动作结果均来自该现场快照。
  Mac/Ubuntu 时钟存在约 0.209 秒偏差，临时桥接将验证时钟对齐服务器，保留原始 CAN 时间戳；
  同主机生产插件不需要这种桥接。时钟探测最小往返约 7 毫秒，记录在 `clock-offset.json`。
- 机械臂 2 暂未接入现场测试页；完整双设备验收、ROS2 发布链路仍待最终集成。
- 用户发现卡片「查看数据流」显示人形。原因是 Core 详情面板传入固定 MCP ID `detail`，
  模型查询失败后回退默认人形。独立渲染器测试漏掉了此入口；不把它记为通过。
  用户明确选择保持不改 Core，仅使用顶部「监控」入口。本次未修改 Core。
- 测试桥接脚本和证据在本机 `acceptance/20261013_piper_monitor/`，不属于生产注册或部署。

### 2026-10-13 12:04 现场部署

- 用户明确要求部署供检查。分别从现场原镜像叠加监控文件、模型、只读注册适配器；
  保留机械臂 1 `arm_control/piper_photo`，机械臂 2 旧十卡，以及各自 CAN、相机、配置和数据目录。
- 新镜像：`agilex-piper:local-monitor-ae45f64d-arm1` 与 `agilex-piper:local-monitor-ae45f64d-arm2`。
  Core 镜像 ID 部署前后完全一致，Core 未改动或重启。
- 原 compose、两台运行中驱动源码、镜像身份、画布前后布局备份：
  `/opt/phanthy-motus/backups/piper-monitor-20261013-120447/`。
- 镜像构建检查禁用网络、未映射设备，验证保留原工具、六轴官方模型、未创建硬件连接。
  两台 MCP 健康检查通过，监控 `running/publishing=true`，发布错误为空，独立话题注册成功。
- 两台参数话题通过实际 DDS → Core → WebSocket 到达 Edge 画布；切到「最新」后分别显示正确设备名、
  未连接、动作许可关闭和缺失字段不可用。现场截图 `server-monitor-deployed.png`。
- 驱动重启后尚未建立共享 CAN 连接；监控不负责连接，因此骨架尚无实测帧。用户需在原控制卡
  手动读取一次状态再检查姿态。此阶段不宣称完成两台真实运动跟随或物理尺寸验收。
- 未发送运动、模式、使能/失能命令，未查询固件、未拍照。画布仅新增两个监控节点，未删旧卡。
- 照片参考的粗细外壳是标准 URDF visual，随镜像发布；当前 Core 忽略外壳，仍显示线条骨架。
  卡片详情入口的既有模型寻址问题仍保留，请使用顶部「监控」。
- 部署脚本在服务器 `/home/agilex/piper-monitor-deploy-20261013/deploy_monitor.py`。
  回退命令（只恢复驱动，不动 Core）：
  `sudo python3 /home/agilex/piper-monitor-deploy-20261013/deploy_monitor.py --rollback /opt/phanthy-motus/backups/piper-monitor-20261013-120447`

用户要求先测试再提交 PR，因此本阶段保留本地独立提交，不创建或更新 PR。
最终目标：[驱动 PR #382](https://github.com/4paradigm/phanthymotus-driver/pull/382)。
本次监控版本：**尚未生成**。
查询时该 PR 历史最近成功 Bot Version 为 `release.261011.ec99be9`，对应历史构建，
不代表本次监控提交或当前 PR HEAD；提交 SHA 不作为 Version。
