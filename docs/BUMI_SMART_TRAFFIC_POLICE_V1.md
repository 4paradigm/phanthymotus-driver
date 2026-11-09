# Bumi 智能交警 V1.0：接口与主机验收

## 范围与仓库审查

实现只在 `phanthymotus-driver` 的 `feat/bumi-driver` 工作区中进行；未提交、推送、部署或连接机器人。

第一轮视觉实现开始时 Driver 起始 HEAD：`814b3ca953d33eba0aca9a61dcefa773f5da483b`，起始工作区干净。
此前手势扩展及本轮P1-1修复均基于已有未提交工作继续进行；本轮只修改视觉打包、注册和生命周期接入。
只读参考主仓库 HEAD：`3a3567a75e4255afe616d87ee77d14321052b70e`。主仓库的 VOP 帧来源扩展与交通灯原型是用户在途修改，不是本次新增或已部署能力。

实际审查的关键入口：

- Driver `README_dev.md`：工具字段、生命周期、普通 dict 返回、ACP 与 hooks。
- 主仓库 `perception/main.py::PerceptionBundle`：`_plugin_for/get_all_tools/dispatch/shutdown` 与原有 MCP handler。
- 主仓库 `perception/plugins/vop.py`：`_FrameMetadata/_image_cb/_extract_objects/_publish_objects/VideoObjectPerceptionPlugin.dispatch`。
- 主仓库 `perception/plugins/traffic_light/{color_classifier.py,schemas.py,__init__.py}`：既有 HSV 门限、红色色相环绕和主导颜色原型。
- Bumi `device.py::LocoPlugin`：目标动作预约、`_rotate_preflight/_do_destination/_do_rotate_worker/_rotate_status/_stop_move`。
- Bumi `main.py::BumiDeviceBundle` 与 `config.yaml`：已有 LocoPlugin 加载方式。
- Bumi `loco_servo.py`：控制仲裁及启动提交点。
- 主仓库 `agent-core/src/api/skills.py::validate_local_skill/import_local_skill`、`mcp_client.py::_to_openai_schema/present_to_llm`：真实 Skill 格式及 processor 系统动作过滤。

**当前生产路径**：`common/traffic_light/` 是唯一实现，由 `noetix/bumi/main.py` 按配置注册；通过已有 `build_context_extras` 和Dockerfile的整目录common COPY进入标准Bumi镜像。Camera和VOP继续使用原能力，主仓库仍只读。

原Driver根目录perception替代入口不属于标准发布链路，P1-1修复后退役；不再动态导入主仓库PerceptionBundle或utils。完整变更、隔离测试及剩余问题见[P1-1接入说明](BUMI_TRAFFIC_LIGHT_P1_1.md)。

## 调用关系

```text
现有 Camera JPEG ──→ 现有 VOP ──→ objects JSON ──→ traffic_light
       └───────────────── 自动推导并订阅 RGB ──────────┘
                                                   ↓
                               独立源帧、逐灯泡分析、3/5投票
                                                   ↓
                         智能交警 Skill：绑定具体目标方向，逐灯报告
                            ↓ 质量不足且允许有限观察旋转
              loco_relative_move.observation_* → 原旋转 worker / ACP / stop_move
                            ↓ 停稳后新帧复看；结束并释放观察会话
               方向/任务语义明确 → traffic_police_gesture.stop/go(execute=true)
                            ↓
               共享 _body_action → 原 action_recording 播放/停止路径
                            ↓ 人工观察效果，cancel确认退出与停稳
```

Camera、Depth、VOP、Vision Capture 和底盘控制器均未重写。没有新增深度学习模型、训练、第三方运行依赖或 Agent Core 修改。

## 标准Driver入口与生命周期

沿用 `noetix/bumi/main.py` 和原MCP端口15704。`plugins.traffic_light.enabled=true`使插件默认注册；`driver.yaml`声明processor，实际工具来自BumiDeviceBundle.get_all_tools。

Python plugin.start是bundle准备阶段，不创建视觉实例/订阅；画布提供VOP输入后，Card dispatch(start)才使用已有executor创建两个订阅。Python plugin.stop调用同一shutdown路径停止全部实例。Card单实例stop、缓存隔离和失败重试语义不变。

不需要另一份源码checkout、新镜像/服务/端口、源码volume覆盖或替代Perception入口。原VOP缺少源帧字段时仍输出未知，不用发布时间或最近RGB伪造帧关联。现有画布若绑定旧Perception MCP，需要重新绑定Bumi MCP；Skill工具名不变。

## traffic_light Card

- 名称：`traffic_light`；类型：`processor`；多实例：支持。
- 画布输入：一个 `data/json` 端口，连接 VOP objects。
- 输入示例：`/ubuntu/camera/color/objects`，ROS `std_msgs/msg/String`。
- 自动推导 RGB：`/ubuntu/camera/color`，ROS `sensor_msgs/msg/CompressedImage`，JPEG。
- 输出：`<VOP input>/traffic_light`，例如 `/ubuntu/camera/color/objects/traffic_light`，ROS `std_msgs/msg/String`，JSON。
- namespace 不硬编码；输入必须为合法绝对 `/.../objects` Topic。
- 图像与 VOP 订阅及 JSON 发布 QoS：BEST_EFFORT、KEEP_LAST、depth=2、VOLATILE。
- 每个实例一个 ROS node、两个订阅、一个 publisher 和失效检查 timer；没有第二个相机 opener、相机采集进程或私有图像分发服务，也没有额外推理线程。

### Actions

| Action | 参数 | 行为 |
| --- | --- | --- |
| `start` | `instance_id`；首次提供 `input_topic`，或单个 `input_topics` | 建立两个订阅；已有同一绑定时幂等。首次输入来自画布连线 |
| `stop` | 可选 `instance_id`；省略停止全部实例 | 先停回调，再从 executor 移除 node，随后销毁；清空缓存与投票。失败保留清理句柄 |
| `info` | `instance_id`；可选 `input_topic` | 只读返回状态、Topic、最近来源时间、同步状况、缓存计数及错误 |
| `latest` | 可选 `instance_id` | 返回完整最新结果和 `runtime`；过期时清除灯色/方向；未启动时正常返回 waiting |
| `config` | `instance_id`，配置字段或 `config` 对象 | 仅停止时修改实例配置，严格校验 |
| `observe` | 可选 `instance_id` | 在已由画布保存的绑定上开始/恢复观察，不接受猜测的新输入 |
| `finish_observation` | 可选 `instance_id` | 结束本实例观察、释放订阅和缓存，保留画布绑定供复看 |

多实例时业务调用必须指定实例；无实例的系统 `stop` 停止全部。本插件最多8个活动实例，保存的绑定和配置各最多32个。

Core 实际隐藏 processor 的 `start/stop/info/config`。因此 Skill 使用正式公开的 `observe/finish_observation/latest`，内部调用同一套生命周期；不修改 Core 的过滤机制。`latest.runtime` 返回与 `info` 相同的运行信息。

`on_interrupt_all` 和 `on_interrupt_motion` 绑定系统 `stop`，由 Core 直接触发，不依赖取消后的 LLM 回合继续清理。

### 配置、同步与识别

- 缓存默认64帧，另有16 MiB总字节上限；单个 JPEG 默认最多1 MiB。
- `max_age_s=1.0`；按当前 ROS 时钟和本地单调接收时钟共同限制有效期。
- pending 检测最多8份；跟踪默认最多32个，消失1秒后移除，单目标历史最多5次。
- JPEG 在解码前检查 SOF 尺寸，最大3840×2160；不存在压缩图像无限扩张路径。
- 同步严格匹配 VOP `source_stamp={sec,nanosec}` 与 JPEG `header.stamp`。`timestamp`、框颜色和最近一帧都不是替代来源。
- 支持检测先于 RGB 到达：有界等待相同 stamp。重复帧不增加票数；相同 stamp 不同 JPEG、过期/未来来源、退役 stream、时钟回退均返回未知。
- start 建立新的来源时间屏障并清空历史。Skill 在运动前停止 Card，反馈停稳后重新 observe，只接受新屏障之后的数据。
- 复用原型的 HSV 色相范围、饱和度门限和红色环绕思想；改用独立连通发光区域、亮度对比、尺寸与边界检查，不使用整个框均值。亮度门限调整为90，属于保守初值。
- 每个灯泡独立输出；红绿同时亮时保留两个 bulbs，灯组顶层 color=unknown，不强选主色。
- 轮廓圆度/凸性判断圆灯；发光轮廓模板比较左、直行、右箭头。小、模糊、截断、方向相似的图案返回 unknown；不按灯珠位置推方向。
- 每个灯组及灯泡独立3/5投票。当前帧未知、改变颜色/方向时，不延续旧值。颜色与方向置信度、稳定性分别计算；两项均稳定时整体 stability=stable。
- P1-3：仅整帧IoU候选图中互为唯一的一对一匹配继承历史；歧义组件的旧轨迹退休，当前框保留但使用新ID重新累计。歧义前stable历史不能在下一帧复活；signal_id不代表已验证的物理身份。规则、复现及边界见[P1-3说明](BUMI_TRAFFIC_LIGHT_P1_3.md)。
- 置信度是保守几何/像素证据分数，不是经过现场数据校准的概率。

现有 Bumi JPEG header 是编码后的 ROS 发布时间，**不是已经验证的曝光时间**。这里能保证现有消息身份的精确对应与接收缓存隔离，曝光时间及底层采集队列延迟仍需真机验证。

### 结果示例

以下为格式示意，数值不代表现场测量：

```json
{
  "state": "ok",
  "timestamp": 1800000000.25,
  "frame_timestamp": 1800000000.20,
  "frame_stamp": {"sec": 1800000000, "nanosec": 200000000},
  "frame_timestamp_semantics": "ros_header_stamp",
  "stream_id": "example",
  "frame_seq": 5,
  "reason": null,
  "signals": [{
    "signal_id": "signal_001",
    "bbox": [148, 34, 226, 270],
    "position": {"x": 0.1, "y": 0.2},
    "detection_confidence": 0.92,
    "color": "red",
    "color_confidence": 0.91,
    "symbol": "arrow_left",
    "symbol_confidence": 0.86,
    "color_stability": "stable",
    "symbol_stability": "stable",
    "stability": "stable",
    "observations": 5,
    "bulbs": [{
      "bbox": [163, 48, 207, 93],
      "color": "red",
      "color_confidence": 0.91,
      "symbol": "arrow_left",
      "symbol_confidence": 0.86,
      "color_stability": "stable",
      "symbol_stability": "stable",
      "stability": "stable",
      "pixels": 620
    }],
    "reason": "emitting_regions"
  }]
}
```

状态包含 `ok/waiting/no_signal/stale/error`。`ok` 表示当前配对数据已处理，不保证每个维度已识别；仍需检查每个目标的字段。没有检测目标时 signals=[]、state=no_signal。无法同步时逐目标颜色/方向未知；stale 不携带有效旧灯色。`latest` 额外包含 `runtime`，Topic 消息不重复携带运行配置。

## 安全观察旋转

工具仍为 `loco_relative_move`，不是新底盘 Card。

| Action | 参数 | 返回与约束 |
| --- | --- | --- |
| `observation_begin` | 无 | 检查模式、故障、关节/IMU静止、其他运动及servo占用，预约会话并返回 session_id；不移动 |
| `observation_rotate` | `session_id/request_id/angle_deg` | 复用 `_rotate_preflight/_do_destination` 和原旋转 worker；返回 running、原 action_id；请求必须唯一 |
| `observation_status` | `session_id` | 状态、预算、动作编号、stop_confirmed、observation_after_ns、错误及释放情况 |
| `observation_end` | `session_id` | 取消锁存、请求停止、等待原 worker、连续反馈停稳、等待monitor退出，然后释放；不能确认则 locked |
| `stop_move` | 原接口 | 始终可调用；活动观察会话额外执行结束与反馈核验，失败返回 STOP_UNCONFIRMED |

集中常量：单次请求≤10°，最多3次，累计请求绝对值≤30°。正值左转，负值右转。预算在接受前预约；失败不退款；实测净转角超出请求的部分补扣。它们是软件请求预算，不是对机械误差的保证。

观察会话与原目标动作共用 chassis RLock，整个会话阻止原始 rotate、目标平移、loco.move、其他身体动作和servo非零命令。JSON不能伪造内部调用 token。servo启动在提交点重查预约，防止检查与提交之间竞争。会话结束后原操作恢复；不改原旋转控制律。

使用原 action_id、ACP和状态接口；原 status 增加当前 action_id 与 last_action_id。观察 monitor 在ACP终态之后继续核对同一动作和连续关节/IMU停稳，故 ACP completed 本身不等于会话 ready。

20秒动作监视预算；停止检查预算2秒。连续关节速度≤0.10、三轴角速度≤0.10、连续yaw变化≤0.3°后才反馈确认。任何非零电机错误、非法模式、不完整或非有限反馈都拒绝。

超时、线程异常、编号缺失/错配、状态查询失败、取消或停止失败都请求原 stop_move，锁存取消，不自动继续或重开。Core中断hooks走原停止入口并自动清理观察会话。确认失败保留预约；不会虚报正常 ended。重复 end 幂等。

**必须保留的安全边界**：公开SDK没有来源采样时间和独立硬件停止确认。因此 stop_confirmed 表示SDK连续停稳读数通过检查，不是物理急停认证；冻结/陈旧SDK数据的独立检测能力仍不足。硬件急停、停止延迟与真实反馈更新必须现场验证。`loco_servo.dry_run` 不是观察旋转的全局空跑开关。本轮所有运动测试都用Mock，没有真实动作。

## traffic_police_gesture Card

Bumi actuator，业务动作仅 `stop/go`，复用同一 LocoPlugin 的 `_execute_body_action`、`_do_teaching_action` 和 `stop_playback`，播放期间保留原 `_body_action`。两个 recording ID 来自 `plugins.traffic_police_gesture`，默认null，不在Skill里指定。

业务调用必须为 `stop(execute=true)` 或 `go(execute=true)`；不带execute的stop是Core项目停止所需的安全取消，不播放停车手势。`info`只读，`cancel`退出并核验SDK停稳，失败locked。SDK无物理完成事件，running/accepted及模式回到walking都不能视为手势完成；不能伪造completed。手势期间禁止观察旋转、平移及其他身体动作。

P1-2将Card生命周期和业务状态分开：start后的info为空闲时 `state=ready, gesture_state=idle, busy=false`；播放期间gesture_state保留真实状态，取消后仍ready；生命周期stop后idle，退出未确认则state=error/gesture_state=locked。详见[P1-2契约与测试](BUMI_TRAFFIC_POLICE_GESTURE_P1_2.md)。Skill只同步了两处info字段检查，没有改变决策、工具或旋转预算。

P1-4：bundle保持逆序停止，对各插件独立隔离普通异常并在全部尝试后汇总上抛；Camera等失败不能再跳过LocoPlugin原gesture shutdown链路，signal/finally的外层关闭也保持执行。只保证软件清理机会，不声称物理停止，详见[P1-4说明](BUMI_BUNDLE_SHUTDOWN_P1_4.md)。

P2-2 Driver部分：手势先本地停止、验证、提交状态并释放operation_lock，再由单个有界通知worker发送ACP；网络结果通过info.acp_notifications独立观察，不改变安全结果或阻塞shutdown。通知无重试、进程退出best-effort；当前Core的early callback/pending回收仍是独立P2-1问题，声明timeout不保证自动回收。原安全验证和Core 5秒期限均未改，不承诺所有stop小于5秒。详见[P2-2说明](BUMI_GESTURE_NOTIFICATION_P2_2.md)。

完整接口、为什么需要execute标记、ACP限制、20项需求覆盖和现场待验证项见[交警手势说明](BUMI_TRAFFIC_POLICE_GESTURE_V1.md)。

## Skill 安装与完整定义

文件：[`skills/bumi-smart-traffic-police.json`](skills/bumi-smart-traffic-police.json)。

- slug：`bumi-smart-traffic-police`
- 名称：智能交警
- requiredTools：`traffic_light`、`loco_relative_move`、`traffic_police_gesture`，均以真实插件get_tools输出校验；业务stop/go的execute必填项也经真实Core展开校验。
- 格式：现有 Agent Core Skill JSON，不是 Codex SKILL.md，也没有新的Python执行引擎。
- 通过现有 `POST /api/skills/import` 提交文件JSON；成功后使用现有Skill激活机制。相同slug不同内容的导入会409，不能覆盖用户已有Skill。
- 安装后先在画布连接 VOP→traffic_light 并成功启动一次，保存输入绑定；连接所需工具到决策核心。不要在本轮连接真实机器人或导入运行账户。

例如未来安装时，将完整文件内容作为 `/api/skills/import` 请求体，按现有Core登录机制提供认证。本轮没有调用该接口。

完整 JSON 和 instruction 附在文末，来源与可导入文件保持一致。

## 文件变更清单

新增：

| 文件 | 用途 |
| --- | --- |
| `common/traffic_light/__init__.py` | 共享插件与算法导出 |
| `common/traffic_light/schemas.py` | 原配置、证据契约及stamp/bbox校验，字节保持不变 |
| `common/traffic_light/color_classifier.py` | 原颜色及箭头分析，字节保持不变 |
| `common/traffic_light/processor.py` | 原缓存、同步、tracking及投票，字节保持不变 |
| `common/traffic_light/plugin.py` | 通用ROS/MCP runtime及bundle生命周期桥接 |
| `common/traffic_light/_ros_lifecycle.py` | 包内最小节点清理、失败保留和重试 |
| `tests/test_bumi_traffic_light_packaging.py` | 标准context/COPY树、隔离导入及真实Bumi集成 |
| `docs/BUMI_TRAFFIC_LIGHT_P1_1.md` | P1-1发布接入、测试和退役记录 |
| `noetix/bumi/observation_safety.py` | 会话预算、请求去重、监视与停稳核验 |
| `noetix/bumi/traffic_police_gesture.py` | stop/go示教语义薄封装、取消、watchdog及标准ACP |
| `tests/test_bumi_traffic_police_gesture.py` | 真实共享动作路径、互斥/取消/并发/超时与内存MCP测试 |
| `docs/BUMI_TRAFFIC_POLICE_GESTURE_V1.md` | 手势接口、安全边界、需求测试映射及现场清单 |
| `docs/skills/bumi-smart-traffic-police.json` | 完整可导入Skill |
| `docs/BUMI_SMART_TRAFFIC_POLICE_V1.md` | 本交付说明、接口、测试与现场清单 |
| `tests/test_traffic_light.py` | 真实OpenCV、同步、多目标、生命周期及真实Bumi MCP handler接口测试 |
| `tests/test_bumi_observation_safety.py` | 真实LocoPlugin与Mock控制器的安全及并发测试 |
| `tests/test_bumi_traffic_skill.py` | 真实Core校验函数、工具名称、processor可见性和完整instruction检查 |

修改：

| 文件 | 用途 |
| --- | --- |
| `noetix/bumi/device.py` | 原工具schema/dispatch、动作编号查询、会话门控；共享身体动作helper、手势注册及shutdown |
| `noetix/bumi/main.py` | 原手势配置转交；本轮注册独立于运动Controller的TrafficLightPlugin |
| `noetix/bumi/driver.yaml` | 声明gesture actuator及traffic_light processor |
| `noetix/bumi/loco_servo.py` | 启动提交点再次检查底盘预约 |
| `noetix/bumi/config.yaml` | 原观察/手势配置不变；本轮增加traffic_light默认开关和原保守参数 |
| `noetix/bumi/Dockerfile` | COPY观察安全及手势模块，仅修改源码清单，未构建镜像 |
| `noetix/bumi/tests/test_loco_destination.py` | 更新新增动作schema、hooks和action_id终态契约断言 |
| `noetix/bumi/README.md` | 链接本说明 |

## 主机测试与未覆盖项

测试环境为 `/tmp/bumi-traffic-v1-tests`，使用原系统numpy/pytest，并仅在该临时环境安装项目已有依赖 `opencv-python-headless==4.11.0.86`。没有修改依赖文件或系统Python。

所有命令使用 `PYTHONDONTWRITEBYTECODE=1`、`-B`、禁用pytest缓存，避免向只读参考仓库产生字节码或缓存。真实Bumi MCP handler以内存HTTP输入调用；未监听网络端口。Skill校验抽取真实Core纯函数，不导入其数据库或启动Core；仅其异常类型使用测试替身。

上一轮视觉/观察实现完整验收为361项通过。手势扩展阶段完整验收为422项通过（56.42秒），后续只读审查重跑422项通过（56.56秒）。本轮P1-1删除旧入口后的最终完整组合为**440 collected、440 passed、0 failed、0 skipped，57.11秒**；新增模块单独16项通过（0.77秒）。隔离范围与剩余问题见[P1-1接入说明](BUMI_TRAFFIC_LIGHT_P1_1.md)，不能将主机通过视为完整V1已可部署。

后续P1-2仅修复手势Card启动握手，按手势→bundle/Skill→Core集成→完整组合顺序验证。最新完整结果为 **492 collected、492 passed、0 failed、0 skipped，58.66秒**；增加30项手势用例与22项真实Core契约/集成用例，详见[P1-2说明](BUMI_TRAFFIC_POLICE_GESTURE_P1_2.md)。以下命令已包含新增Core测试。

后续P1-3只修复tracking歧义历史继承，新增18项跟踪测试；最终统一结果为 **510 collected、510 passed、0 failed、0 skipped，58.85秒**。先视觉/跟踪79项、再packaging/bundle 22项、最后完整组合，全部通过。详见[P1-3说明](BUMI_TRAFFIC_LIGHT_P1_3.md)；以下命令已包含tracking测试。

后续P1-4修复bundle停止异常短路，新增27项shutdown测试；最新统一结果为 **537 collected、537 passed、0 failed、0 skipped，59.49秒**。先新增模块27项、再手势/bundle/Skill 126项、最后完整组合，全部通过。详见[P1-4说明](BUMI_BUNDLE_SHUTDOWN_P1_4.md)；以下命令包含新增shutdown模块。

后续P2-2 Driver部分将gesture网络投递移出本地安全停止关键路径，新增19项测试；最终统一结果为 **556 collected、556 passed、0 failed、0 skipped，60.28秒**。按新增通知→原gesture→bundle shutdown→Core lifecycle顺序的组合158项通过（6.26s），再执行完整组合。详见[P2-2说明](BUMI_GESTURE_NOTIFICATION_P2_2.md)；以下命令包含新增通知模块。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
/tmp/bumi-traffic-v1-tests/bin/python -B -m pytest -v -p no:cacheprovider \
  tests/test_traffic_light.py tests/test_bumi_traffic_light_packaging.py \
  tests/test_traffic_light_tracking.py \
  tests/test_bumi_observation_safety.py \
  tests/test_bumi_traffic_skill.py tests/test_bumi_traffic_police_gesture.py \
  tests/test_bumi_gesture_notification.py \
  tests/test_bumi_gesture_core_lifecycle.py \
  noetix/bumi/tests/test_loco_destination.py \
  tests/test_bumi_loco_servo.py tests/test_bumi_bundle.py \
  tests/test_bumi_bundle_shutdown.py \
  tests/test_bumi_camera_specs.py tests/test_bumi_imu_distance.py \
  tests/test_bumi_leg_odometry.py tests/test_bumi_odom.py \
  'tests/test_dockerfile_copies.py::test_every_copied_path_exists[noetix/bumi/Dockerfile]' \
  'tests/test_dockerfile_copies.py::test_every_imported_sibling_is_copied[noetix/bumi/Dockerfile]' \
  'tests/test_dockerfile_copies.py::test_every_sibling_file_named_by_shipped_code_is_copied[noetix/bumi/Dockerfile]'
```

还执行了 AST/Skill JSON解析与 `git diff --check`，均通过。开发中先单独运行安全模块、真实OpenCV、Card/Skill和原Bumi控制测试，再执行上述完整组合。早期发现的测试图左右方向标注、Topic边界校验和旧schema断言已修复；Core校验改为抽取真实纯函数，避免为测试安装无关FastAPI服务依赖。没有遗留主机测试失败。

结束复核：主仓库623个tracked/untracked文件的聚合SHA256为 `c11944f4e940604a701e5c652e5cdf9c67f1e140df6e81594f56b1f5e982d645`，与开始一致；Git status和索引哈希亦一致。Driver仍在 `feat/bumi-driver`、原HEAD，最终状态见本轮P1-1接入说明；未暂存、提交或推送。


覆盖：红黄绿、灯灭/黑场/白反光、背景干扰、红色环绕、低分辨率、模糊/不完整/截断图案、bbox边界、同时亮灯、左直右箭头、逐目标遮挡、3/5与重复帧、突变和消失重获、旧/未来/错配来源、时钟/stream变化、缓存帧数与字节限制、创建/清理异常及重启、MCP工具调用、会话限额/并发/故障/超时/取消/停止与编号问题、原Bumi控制回归。

未覆盖：真实ROS2 DDS发现与BEST_EFFORT丢包行为、Jetson性能/GPU推理、真实交通灯样本精度、闭源SDK的陈旧反馈、物理停止及真实机器人完整Skill执行。合成图通过不等于现场箭头识别准确率达标。

## 后续现场测试清单（本轮不执行）

1. 核对部署VOP确有source_stamp/stream_id/frame_seq，确认traffic light类别与bbox。
2. 测量RGB/VOP实际帧率、端到端帧龄、精确stamp匹配率、丢帧及重启。
3. 核对JPEG header相对曝光时间、采集/编码队列延迟，尤其旋转后第一批帧。
4. 红黄绿、反光/逆光/夜间/频闪、全灭与遮挡，记录unknown及误判率。
5. 左转、直行、右转及其他形状：不同距离、角度、灯珠尺寸和模糊程度。
6. 多灯组/同灯组多个灯泡同时亮、跨帧身份、消失重获与视角切换。
7. 固定观察先验收，再由现场授权测试有限小角度、三次/30°预算、底盘互斥与机械误差。
8. 独立核对SDK反馈更新、软件停止延迟、关节/IMU停稳门限和硬件急停。
9. 用户中断、通信/SDK故障、停止未确认锁定及资源回收。
10. 用原action_recording录制停车/放行，核对编号及内容不含平移，再填入Driver配置；核验正常站立、双脚着地、肢体安全范围及人工观察动作效果。
11. 完整Skill：固定观察、有限旋转、新帧复看、明确车道/目标对应关系、逐灯报告、stop/go手势、cancel核验退出及异常清理；确认无平移、俯仰、导航或车辆控制。

## 完整 Skill JSON

```json
{
  "slug": "bumi-smart-traffic-police",
  "name": "智能交警",
  "oneLiner": "观察多个机动车信号灯，必要时有限原地旋转，仅对明确目标方向执行已配置的停车或放行手势。",
  "description": "Bumi智能交警V1.0：复用Camera/VOP和traffic_light逐灯识别，经受限观察会话改变视角；目标方向和任务语义明确时，通过traffic_police_gesture播放现场配置的stop/go示教动作。",
  "category": "robotics",
  "version": "1.0.0",
  "requiredTools": [
    "traffic_light",
    "loco_relative_move",
    "traffic_police_gesture"
  ],
  "compatibility": {
    "hardware_provider": "noetix",
    "hardware_model": "bumi",
    "ros": "humble",
    "perception": "traffic_light V1.0; VOP stream with source_stamp",
    "motion": "loco_relative_move observation_* V1.0",
    "gesture": "traffic_police_gesture V1.0; configured and site-verified stop/go recordings; manual physical verification"
  },
  "instruction": "你正在执行Bumi智能交警V1.0：观察、必要的有限原地旋转、逐灯判断，以及仅在任务语义明确时执行停车/放行手势。禁止巡逻、身体平移、抬头低头、自动通过道路、车辆控制和其他交警业务动作。禁止调用loco.move、loco_servo、loco_relative_move.move或原始rotate。手势内不得夹带底盘移动。\n准备与绑定：工具使用当前MCP注册名称；动作通常展开为mcp__<mcp_id>__<tool>__<action>。画布须连接VOP输出到traffic_light并至少成功start一次；不硬编码/ubuntu或猜测Topic，不重复启动Camera/VOP。所有视觉调用携带同一个画布instance_id。使用observe、finish_observation、latest作为业务观察接口；info运行信息也包含在latest.runtime中。\n1. 检查loco_relative_move.status与traffic_police_gesture.info；已有手势busy、其他动作或故障时，不抢占、不启用机器人。调用observation_begin检查工作模式、故障、SDK关节/IMU停稳与底盘占用，保存唯一session_id。begin不移动；拒绝时报告原因，不能通过重开会话绕过限制。\n2. 调用traffic_light.observe并核对latest.runtime。至多等待3秒、以不小于0.2秒间隔读取latest，重复同一stream_id/frame_seq不能增加证据。整次任务最多120秒。必须分别保留每个signal_id、bbox/position、color、symbol、detection_confidence、color_confidence、symbol_confidence、observations和稳定性；同一灯组多个bulbs分别保留。\n3. 只在state=ok、未过期且至少3次独立有效观测时检查color_stability和symbol_stability；对应维度stable才称已确认。颜色和方向独立，unknown不得补成旧值。waiting、no_signal、stale、error均据实报告；不能从框均色、color_name或灯珠位置推灯色/箭头。不能把任何红灯解释为全局停车，也不能把任何绿灯解释为所有方向放行。\n4. 质量不足且本次任务允许改变水平视角、现场已确认可安全原地转动时，先调用traffic_light.finish_observation，确认idle且cleanup_pending为空。仅使用observation_rotate(session_id,request_id,angle_deg)：单次绝对角度不超过10度，最多3次，累计绝对角度不超过30度。request_id唯一，重试保留原编号。正角左转、负角右转；不能把图像偏移直接当作转角。禁止交替正负或重开会话绕预算。若原因是垂直盲区、分辨率不足、来源缺失或故障，则报告unknown，不以转动代替修复。\n5. running/action_id仅表示旋转请求接受。等待同一action_id的ACP终态，再检查observation_status；只有ready、stop_confirmed=true且编号一致才可复看。locked/error/cancelled、丢失编号、查询失败或超时均停止后续旋转。停稳确认仅来自连续SDK读数，不是独立硬件急停确认。随后traffic_light.observe重建缓存，只接受frame_stamp晚于observation_after_ns的新有效图像，再采集至少3次观测。跨视角signal_id会重置，不能按列表顺序或同一编号猜测物理目标。\n6. 决策必须绑定用户任务或现场已验证的具体车道/车辆方向和对应signal_id。目标方向不明确、symbol为unknown、缺少可靠目标对应关系、存在适用于该方向的冲突/未知信号时，优先逐灯报告unknown，不做手势。circle也不自动代表所有车道；不得自行推断未提供的车道适用关系。仅在明确需要目标方向停车时选择stop；明确允许目标方向放行且箭头/目标方向一致时选择go。黄色或其他不明确情况不自动选择go。\n7. 执行手势前，先结束观察会话：必要时stop_move，再observation_end(session_id)，确认stop_confirmed=true、released=true且不是locked/error。必须完全释放观察预约后才能手势。调用traffic_light.finish_observation释放视觉订阅并保留报告快照；随后检查traffic_police_gesture.info的state为ready、gesture_state为idle、busy=false，所选stop_configured或go_configured为true。机器人须已正常站立、双脚着地，且现场已确认相应示教不含底盘平移、摆动空间安全；SDK不能验证这些物理条件，不得把模式值当作现场确认。会话中已有同次确认时不重复询问。\n8. 手势只能调用traffic_police_gesture.stop(execute=true)或traffic_police_gesture.go(execute=true)。recording ID完全由Driver配置决定，Skill不得指定ID、改配置、借用其他预设动作或在gesture_not_configured时改做另一个动作。缺少哪个recording ID按返回missing_config逐项报告。stop是停车手势，绝不是Bumi底盘急停；用户要求停止任务或出现异常时使用cancel，不能调用业务stop。Core项目生命周期不带execute的stop只用于取消，不会播放手势。\n9. 手势返回running或accepted/action_id只表示提交或进入play_teach，不能称物理动作完成。手势期间禁止观察旋转、平移、其他身体动作或第二个手势；连续stop、stop→go、go→stop也必须先退出并核对空闲。通过info检查活动手势和错误；SDK没有可靠物理完成事件，不能用固定等待时间、模式回到walking或ACP状态猜测手势已完成。需要现场人员观察动作效果；确认已结束后调用cancel复用stop_playback返回walking并检查停稳。未确认效果时只能报告手势效果尚未确认，必要时cancel中断。\n10. ACP只跟踪原action_id的操作生命周期：cancel/超时/故障会给cancelled或error终态，不伪造completed；这与手势效果是否成功是两回事。播放看似结束也要cancel退出并检查playback_exit_confirmed=true，再读info确认state=ready、gesture_state=idle且busy=false。锁定、停止未确认、播放异常或watchdog超时后不自动重放、不切换另一个手势；仅请求cancel并报告原错误，无法释放时保留安全锁定。\n11. 无论正常完成、用户中断、异常或超时，都清理：traffic_police_gesture.cancel；若观察会话尚未结束，stop_move和observation_end并核对stop_confirmed/released；traffic_light.finish_observation并确认idle、cleanup_pending为空。框架hook也会执行手势取消和视觉停止，但不能只因工具返回就声称已物理停稳或恢复安全空闲。清理失败明确报告，不继续动作。\n12. 最终逐灯列出signal_id、视角/时间、位置、color、symbol、两类置信度、稳定性和观测数，多bulbs逐个展示；未知写未知，不合并多个灯。单独说明任务目标方向、为什么选择stop/go或不动作、实际返回的gesture/recording_id/action_id、是否经人工观察效果、ACP退出结果、旋转次数与累计角度，以及是否核验恢复空闲。不得把示教播放请求成功当作停车/放行效果已成功。"
}
```

### 完整 instruction（便于审阅）

你正在执行Bumi智能交警V1.0：观察、必要的有限原地旋转、逐灯判断，以及仅在任务语义明确时执行停车/放行手势。禁止巡逻、身体平移、抬头低头、自动通过道路、车辆控制和其他交警业务动作。禁止调用loco.move、loco_servo、loco_relative_move.move或原始rotate。手势内不得夹带底盘移动。
准备与绑定：工具使用当前MCP注册名称；动作通常展开为mcp__<mcp_id>__<tool>__<action>。画布须连接VOP输出到traffic_light并至少成功start一次；不硬编码/ubuntu或猜测Topic，不重复启动Camera/VOP。所有视觉调用携带同一个画布instance_id。使用observe、finish_observation、latest作为业务观察接口；info运行信息也包含在latest.runtime中。
1. 检查loco_relative_move.status与traffic_police_gesture.info；已有手势busy、其他动作或故障时，不抢占、不启用机器人。调用observation_begin检查工作模式、故障、SDK关节/IMU停稳与底盘占用，保存唯一session_id。begin不移动；拒绝时报告原因，不能通过重开会话绕过限制。
2. 调用traffic_light.observe并核对latest.runtime。至多等待3秒、以不小于0.2秒间隔读取latest，重复同一stream_id/frame_seq不能增加证据。整次任务最多120秒。必须分别保留每个signal_id、bbox/position、color、symbol、detection_confidence、color_confidence、symbol_confidence、observations和稳定性；同一灯组多个bulbs分别保留。
3. 只在state=ok、未过期且至少3次独立有效观测时检查color_stability和symbol_stability；对应维度stable才称已确认。颜色和方向独立，unknown不得补成旧值。waiting、no_signal、stale、error均据实报告；不能从框均色、color_name或灯珠位置推灯色/箭头。不能把任何红灯解释为全局停车，也不能把任何绿灯解释为所有方向放行。
4. 质量不足且本次任务允许改变水平视角、现场已确认可安全原地转动时，先调用traffic_light.finish_observation，确认idle且cleanup_pending为空。仅使用observation_rotate(session_id,request_id,angle_deg)：单次绝对角度不超过10度，最多3次，累计绝对角度不超过30度。request_id唯一，重试保留原编号。正角左转、负角右转；不能把图像偏移直接当作转角。禁止交替正负或重开会话绕预算。若原因是垂直盲区、分辨率不足、来源缺失或故障，则报告unknown，不以转动代替修复。
5. running/action_id仅表示旋转请求接受。等待同一action_id的ACP终态，再检查observation_status；只有ready、stop_confirmed=true且编号一致才可复看。locked/error/cancelled、丢失编号、查询失败或超时均停止后续旋转。停稳确认仅来自连续SDK读数，不是独立硬件急停确认。随后traffic_light.observe重建缓存，只接受frame_stamp晚于observation_after_ns的新有效图像，再采集至少3次观测。跨视角signal_id会重置，不能按列表顺序或同一编号猜测物理目标。
6. 决策必须绑定用户任务或现场已验证的具体车道/车辆方向和对应signal_id。目标方向不明确、symbol为unknown、缺少可靠目标对应关系、存在适用于该方向的冲突/未知信号时，优先逐灯报告unknown，不做手势。circle也不自动代表所有车道；不得自行推断未提供的车道适用关系。仅在明确需要目标方向停车时选择stop；明确允许目标方向放行且箭头/目标方向一致时选择go。黄色或其他不明确情况不自动选择go。
7. 执行手势前，先结束观察会话：必要时stop_move，再observation_end(session_id)，确认stop_confirmed=true、released=true且不是locked/error。必须完全释放观察预约后才能手势。调用traffic_light.finish_observation释放视觉订阅并保留报告快照；随后检查traffic_police_gesture.info的state为ready、gesture_state为idle、busy=false，所选stop_configured或go_configured为true。机器人须已正常站立、双脚着地，且现场已确认相应示教不含底盘平移、摆动空间安全；SDK不能验证这些物理条件，不得把模式值当作现场确认。会话中已有同次确认时不重复询问。
8. 手势只能调用traffic_police_gesture.stop(execute=true)或traffic_police_gesture.go(execute=true)。recording ID完全由Driver配置决定，Skill不得指定ID、改配置、借用其他预设动作或在gesture_not_configured时改做另一个动作。缺少哪个recording ID按返回missing_config逐项报告。stop是停车手势，绝不是Bumi底盘急停；用户要求停止任务或出现异常时使用cancel，不能调用业务stop。Core项目生命周期不带execute的stop只用于取消，不会播放手势。
9. 手势返回running或accepted/action_id只表示提交或进入play_teach，不能称物理动作完成。手势期间禁止观察旋转、平移、其他身体动作或第二个手势；连续stop、stop→go、go→stop也必须先退出并核对空闲。通过info检查活动手势和错误；SDK没有可靠物理完成事件，不能用固定等待时间、模式回到walking或ACP状态猜测手势已完成。需要现场人员观察动作效果；确认已结束后调用cancel复用stop_playback返回walking并检查停稳。未确认效果时只能报告手势效果尚未确认，必要时cancel中断。
10. ACP只跟踪原action_id的操作生命周期：cancel/超时/故障会给cancelled或error终态，不伪造completed；这与手势效果是否成功是两回事。播放看似结束也要cancel退出并检查playback_exit_confirmed=true，再读info确认state=ready、gesture_state=idle且busy=false。锁定、停止未确认、播放异常或watchdog超时后不自动重放、不切换另一个手势；仅请求cancel并报告原错误，无法释放时保留安全锁定。
11. 无论正常完成、用户中断、异常或超时，都清理：traffic_police_gesture.cancel；若观察会话尚未结束，stop_move和observation_end并核对stop_confirmed/released；traffic_light.finish_observation并确认idle、cleanup_pending为空。框架hook也会执行手势取消和视觉停止，但不能只因工具返回就声称已物理停稳或恢复安全空闲。清理失败明确报告，不继续动作。
12. 最终逐灯列出signal_id、视角/时间、位置、color、symbol、两类置信度、稳定性和观测数，多bulbs逐个展示；未知写未知，不合并多个灯。单独说明任务目标方向、为什么选择stop/go或不动作、实际返回的gesture/recording_id/action_id、是否经人工观察效果、ACP退出结果、旋转次数与累计角度，以及是否核验恢复空闲。不得把示教播放请求成功当作停车/放行效果已成功。
