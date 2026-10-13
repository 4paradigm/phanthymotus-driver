# PiPER 计划二开发验证

日期：2026-10-13。分支：`codex/piper-monitor`。基线：`f5ca598278a3f015cd9503d674a81d2d75b122fa`。

## 结果

- 监控测试 23 项：通过（含现场快照适配与固定轮廓跟随验证）。
- 最新指定回归集合：204 passed、118 subtests passed（含 8 项自动骨架卡与 8 项示教控制测试）。
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
  tests/test_piper_status_card.py tests/test_piper_teaching_control.py \
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

### 2026-10-13 12:27 仅机械臂 1 两张卡

- 用户要求先仅操作机械臂 1，把读取状态与控制拆成两张卡，移除其余画布卡片。
  当前布局精确包含 `piper_status`、`arm_control` 两个节点，无数据或执行连线。
- 新增独立 `status_card.py`；手动读状态复用现有连接和安全缓存快照，不发送控制命令。
  控制卡移除读取状态选项，保留原预览/确认控制流程，默认「请选择控制动作」为空操作。
- 仅重建机械臂 1：`agilex-piper:local-arm1-status-78b61ded`。
  核对机械臂 2 镜像与启动时间均未改变；Core 镜像未变，不修改或重启 Core。
- 原布局与部署备份保留。最初完整多卡备份：
  `/opt/phanthy-motus/backups/piper-arm1-two-cards-20261013-122517/`；
  最终部署备份：`/opt/phanthy-motus/backups/piper-arm1-two-cards-20261013-122741/`。
  迁移仅更新已有画布配置，避免 Core 删除节点时自动调用机械臂 2 的停止方法。
- Edge 实际服务器页面刷新验证通过：两卡可见，读取状态独立，控制默认未选动作。
  现场截图 `arm1-two-cards.png`。侧栏注册工具仍保留，其他画布节点已移除。
- 本次仅做软件与界面验证，没有执行真机动作或手动读取；真实控制由用户自行测试。
  状态卡测试覆盖安全连接、独立生命周期、拒绝控制参数与控制卡动作隔离；最终回归 190 项通过。
- 部署脚本：`/home/agilex/piper-monitor-deploy-20261013/split_arm1_deploy.py`。

### 2026-10-13 12:38 骨架读取卡纠正

- 用户明确「读取状态」指单独读取骨架位置，而不是仅列参数。
  将 `piper_status` 改为 sensor「读取骨架位置」，接管原采样器与模型，避免重复插件。
  画布仍仅两张机械臂 1 卡；监控只展示一个骨架话题。
- 模型用 `display_*` 固定分支补上主要底座、关节、臂杆及双指轮廓。
  沿用 Core 的固定骨架段渲染，Core 无修改；不是实体网格。
  原始六轴仍为唯一活动关节，运动链的 528 个矩阵对比通过，新分支随所属链接移动。
- 193 项回归与 114 项子测试通过，模型浏览器模拟无脚本错误。
  骨架卡启停、info、定时采样不会连接或发送控制命令；只有显式手动 read 可建立共享连接。
  隔离镜像检查补充验证 sensor 的 info 不误入连接流程。
- 机械臂 1 新镜像：`agilex-piper:local-arm1-pose-24cb589e`。
  备份：`/opt/phanthy-motus/backups/piper-arm1-pose-20261013-123759/`。
  机械臂 2 镜像和启动时间、Core 镜像均未变化。
- 手动读取一次机械臂 1 骨架，实际服务器顶部「监控」显示新轮廓。
  10 Hz 发布正常，两个缓存快照反馈时间推进且显示新鲜；角度曾为
  `[2.926, 0, 0, 5.176, 24.156, -14.121]` 度，动作许可关闭。
  截图 `arm1-full-skeleton-live.png`，记录 `pose-live-result.json`；未执行运动、模式或电机命令。
  真实运动跟随仍由用户自行验证。
- 脚本：`/home/agilex/piper-monitor-deploy-20261013/deploy_pose_arm1.py`。

### 2026-10-13 12:56 自动检测与示教入口

- 用户要求骨架卡没有业务按钮、自动检测动作；公开 schema 改为仅框架生命周期，
  确认画布不再出现读取/执行表单。Core 固定的「查看数据流」按钮仍保留，
  用户再次选择不改 Core，骨架使用顶部监控。
- 自动初始化共享 runtime 的 CAN 监听（`piper_init=False`），采样回调不做连接操作。
  无第二个 SDK/CAN 连接，不发送运动、模式或电机控制指令。
- 控制卡增加进入/退出拖动示教菜单，带一次性预览、支撑和模式/固件检查。
  本阶段无真实软件示教动作验证。现场验证固件列表为空，软件入口提供按键指引；
  不自动查询或把现场固件加入验证列表。
- 模拟 SDK 验证开始/结束录制只发送对应指令，结束后的待机反馈不会被强制切回 CAN；
  无权限、过期或已变更预览、未知固件、回放中均不发模式或电机指令。
  自动连接覆盖锁忙、失败、重启恢复和停止重试的资源释放。完整回归 204 项、118 项子测试通过。
- 初次部署检查发现智能控制正在运行，因此未改现场；用户关闭后明确要求继续，
  重新检查通过后仅重启机械臂 1 驱动。机械臂 2 镜像/启动时间及 Core 镜像均未改变。
- 镜像：`agilex-piper:local-arm1-auto-teaching-bbb7c2a5`。
  备份：`/opt/phanthy-motus/backups/piper-arm1-auto-teaching-20261013-125617/`。
- 不调用手动 read，仅查询监控 info，确认自动反馈新鲜且时间推进。
  状态曾为示教，角度 `[2.662, 35.149, -32.335, 6.977, 4.461, -14.111]` 度，许可关闭。
  记录 `auto-pose-result.json`，画布截图 `auto-detector-teaching-form.png`，
  自动骨架截图 `automatic-skeleton-live.png`。菜单选择仅做表单验证，未点击示教执行。
- 脚本：`/home/agilex/piper-monitor-deploy-20261013/deploy_auto_teaching_arm1.py`。

用户要求先测试再提交 PR，因此本阶段保留本地独立提交，不创建或更新 PR。
最终目标：[驱动 PR #382](https://github.com/4paradigm/phanthymotus-driver/pull/382)。
本次监控版本：**尚未生成**。
查询时该 PR 历史最近成功 Bot Version 为 `release.261011.ec99be9`，对应历史构建，
不代表本次监控提交或当前 PR HEAD；提交 SHA 不作为 Version。
