# PiPER 计划二开发验证

日期：2026-10-13。分支：`codex/piper-monitor`。基线：`f5ca598278a3f015cd9503d674a81d2d75b122fa`。

## 结果

- 监控测试 22 项：通过（含现场快照命名适配新增 2 项）。
- 最新指定回归集合：182 passed、114 subtests passed。
- 实际 Edge headless + 未修改 Core 渲染器：通过；528 个矩阵值对比，无页面脚本错误。
- 浏览器场景：三组实测角度模拟、过期冻结、读取失败提示、恢复、双设备隔离。
- ROS transport 使用假 ROS 对象验证 Core context、QoS、消息序列化、重复关闭及部分初始化失败释放。
- 官方源 URDF 与许可证原始字节 SHA256 核验：通过。
- 监控生命周期中假硬件记录的控制命令数：0。
- 以上开发模拟验收未访问真实反馈。后续现场缓存测试见下节；均未执行机械臂动作、
  模式切换、使能/失能、拍照或现场部署。

Python 3.12 回归命令（pytest、PyYAML、OpenCV 为测试环境依赖，不是本提交新增的驱动依赖）：

```bash
python -m pytest -q \
  tests/test_piper_monitor.py tests/test_piper_ten_cards.py \
  tests/test_piper_cards.py tests/test_piper_controls.py tests/test_piper_prepare.py \
  tests/test_piper_deployment.py tests/test_piper_mac.py tests/test_common_lifecycle.py \
  tests/test_dockerfile_copies.py tests/test_vendor_runtime_logging.py
```

初次回归环境缺少 PyYAML/OpenCV，补齐隔离测试环境后通过。首次打包检查发现新增模块间依赖需要
提前更改 Docker COPY，随后把模型导出合并进独立 `monitor.py`；最终打包检查通过，未修改 Dockerfile。

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

按 `MONITOR_INTEGRATION.md` 由计划一提供无控制副作用的 `read_monitor_snapshot()`、注册插件、
复制模块和资源、调整画布布局。此独立分支尚未把插件注册进现场服务；假反馈验收不能替代
集成后的 ROS2 联调或真机模型方向/尺寸核对。

用户要求先测试再提交 PR，因此本阶段保留本地独立提交，不创建或更新 PR。
最终目标：[驱动 PR #382](https://github.com/4paradigm/phanthymotus-driver/pull/382)。
本次监控版本：**尚未生成**。
查询时该 PR 历史最近成功 Bot Version 为 `release.261011.ec99be9`，对应历史构建，
不代表本次监控提交或当前 PR HEAD；提交 SHA 不作为 Version。
