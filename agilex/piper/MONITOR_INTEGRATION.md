# PiPER 计划二：姿态与参数监控集成

本分支提供 `PiperMonitorPlugin`、模型和测试。原约定由计划一负责最终集成；
用户于 2026-10-13 明确要求现场部署后，新增 `monitor_bundle.py` 和 Docker COPY，
基于两台现有镜像叠加监控，并在现场画布添加监控卡。PR 仍等待用户测试后更新。

## 共同约定

- 原计划每台机械臂两张核心卡为 `arm_control` 与 `piper_monitor`；用户最新要求先仅操作机械臂 1，
  当前现场画布只保留 `piper_status`（读取状态）和 `arm_control`（机械臂控制）。其他画布节点移除，
  后台模型、监控及相机工具保留注册，机械臂 2 服务不改动。
- 使用现场已有 Core（含 PR #305 的中文标签和手动执行支持），本次不修改 Core。
  通用表单采用“选择操作＋执行按钮”；骨架和参数由监控区域的两个输出展示。
- 已核实 Core 的卡片「查看数据流」入口向骨架渲染器传递固定设备 ID `detail`，模型查询失败后
  回退为人形。用户明确选择不改 Core，当前使用顶部「监控」入口；该入口携带真实 MCP ID。
  独立渲染器验收通过不能代表卡片详情入口通过，不应把人形回退当作 PiPER 展示结果。
- 每台继续使用原有 MCP、CAN、数据目录和相机绑定。权限、预览、动作结果和话题按设备隔离。
- 监控复用共享 runtime/CAN，只接受一个只读快照函数；不创建 SDK 或第二个 CAN 连接。
- 保留旧十卡源码、未提交的 `smoke-test.py`、现场额外关节限位功能及部署备份。
  现场额外功能尚未核对，不能用本地旧驱动覆盖现场。
- 用户已授权只读部署与现场测试；仍不执行真机动作、模式切换、使能/失能或拍照。

## 构造与注册

```python
from agilex.piper.monitor import PiperMonitorPlugin

runtime = ...  # 计划一已有的共享控制对象
monitor = PiperMonitorPlugin(
    runtime.read_monitor_snapshot,
    namespace="agilex/piper",  # 第二台用 piper2/piper
    device_name="机械臂 1",     # 第二台用 机械臂 2
    ros2=ros2,                # 现有 DualDomainROS2，不创建新的 DDS 域
    publish_hz=10,
)
plugins = [runtime, monitor]  # 返回给现有 DriverBundle
```

保留既有工具注册的部署接线：

```python
from agilex.piper.monitor_bundle import add_monitor
plugins = add_monitor(existing_build_plugins(config, namespace, ros2), config, ros2)
```

适配器优先调用计划一安全快照。机械臂 2 的旧十卡镜像没有该接口，适配器仅在现有
非阻塞连接锁内读取共享 driver 的缓存 getter；未连接时直接返回不可用，不调用
`connected/status/read_state`，不连接、不获取动作锁、不改变许可或预览。
现有配置、原控制工具、相机和命名空间保留。不会用本地旧 runtime 覆盖现场。

机械臂 1 最新部署再用 `status_card.split_status_card(plugins)` 拆出读取状态：
`piper_status.read` 显式手动读取时可建立同一 runtime 的 CAN 连接，然后读取安全缓存快照。
它不调用 `status/read_state`，不获取动作锁、不发送运动、模式或电机命令。
状态卡启停仅改变自身状态；监控采样仍不会主动连接。
控制卡移除 `read` 操作，其他控制流程保持原实现；默认「请选择控制动作」为空操作，
必须由用户选择具体控制动作才执行。部署仅叠加独立文件，不修改 runtime 或 Core 源码。

`PiperMonitorPlugin` 提供 `get_tools/start/stop/dispatch`，适配 `DriverBundle`。
其中 `piper_monitor` 为 sensor，动作是 `start/stop/read/info`，允许智能控制关闭时手动操作。
`read` 返回 `{"skeleton": dict或None, "parameters": 中文字典}`。
`model` 为 resource，调用返回普通字典 `{"urdf": "<robot …>…</robot>"}`，不可预包装 MCP content。
`model` 留作骨架渲染器自动调用的后台资源；Core 可能仍在资源列表中列出它。
最新两卡现场布局不再摆放监控节点，后台发布仍保留。

共享 bundle 启动先启动 runtime、再启动监控，关闭时顺序相反。构造和 `start` 均不采样；
只有显式 `read` 和运行中的定时回调调用快照函数。`stop` 不调用任何控制停止方法，允许再次启动。
不提供 `ros2` 时仍可手动 `read`，但 `info.publishing=false`，不会伪称正在发布 DDS。

## 快照接口：交给计划一实现

函数签名 `read_monitor_snapshot() -> MonitorSnapshot`，返回一份独立、可稳定遍历的字典快照。
所有时间为 Unix 秒，所有关节角度为度。字段在 `monitor.py` 的 `MonitorSnapshot` 定义：

| 字段 | 类型 / 含义 |
| --- | --- |
| `device_name` | 设备名称字符串 |
| `connected` | 当前共享 CAN 连接是否可用，必须是真正 bool |
| `feedback_time` | 六轴角度/模式/使能等必要反馈中的最旧接收时间 |
| `feedback_fresh` | 提供方是否确认该反馈有效且新鲜，必须是真正 bool |
| `joints_deg` | J1–J6 的六个有限数值，不得用命令目标代替 |
| `joint_limits_deg` | 六组 `[下限, 上限]`，须已取 SDK 与现场配置的有效交集 |
| `control_mode` / `arm_status` | 现有厂家状态字符串，例如 `CAN_CTRL(0x1)` / `NORMAL(0x0)` |
| `enabled` | 六个 bool，各轴独立使能状态 |
| `gripper` | `width_mm`、`enabled`；缺失或过期时用 `{"available": false}` |
| `motion_enabled` | 操作许可 bool |
| `error_code` / `motor_faults` | 故障码整数 / 故障名称列表 |
| `busy` / `active_card` | 是否执行动作 / 当前动作卡名称或 None |
| `completed_actions` | 按公共工具名保存的动作结果字典，缺失不得冒充成功 |

2026-10-13 计划一现场实现使用 `fresh`、`faults`、`last_actions`。
监控也接受这些字段：`fresh` 对应 `feedback_fresh`；`faults` 内取 `arm_status/error_code/motor_faults`；
`last_actions` 对应 `completed_actions`。若同时给出表中字段，则表中字段优先；不修改提供方快照。
生产插件与共享 runtime 同主机使用同一时钟；跨主机临时测试须核验时钟差，不能把接收时间当作 CAN 时间。

缺失字段按“不可用”显示，禁止填零、false 或空成功结果来假装设备反馈。夹爪反馈须单独检查有效性。
提供方应尽快返回缓存反馈；连接、断线重连和 SDK 查询归共享 runtime 管理，不在此回调中执行。
快照不得持有动作锁，不得使预览失效、改变许可、取消运动或发送控制命令。
现有 `status()` / `read_state()` 有控制副作用，**不能直接作为该回调或在回调内调用**。
计划二的假快照已覆盖接口，不依赖计划一完成即可测试。

## 发布与模型

| 设备 | 骨架 `sensor/skeleton` | 中文参数 `data/json` |
| --- | --- | --- |
| 机械臂 1 | `/agilex/piper/state/joints` | `/agilex/piper/state/parameters` |
| 机械臂 2 | `/piper2/piper/state/joints` | `/piper2/piper/state/parameters` |

命名空间必须显式传入并符合 ROS 命名，不自动替换非法字符，以免两个设备被规范化成同一话题。
参数是顶层中文键，现有 KV 渲染器可直接展示；六轴骨架格式为
`{"timestamp_ms": ..., "position_unit": "rad", "joints": [{"idx": 0, "name": "joint1", "q": ...}, …]}`。
这里的 q 是反馈角度转换的弧度，无预测或目标动画。

插件同时检查提供方的新鲜标记、连接状态、六个有限角度以及反馈年龄 `0 <= age <= 0.5` 秒。
无效/过期/断线不发送骨架帧，画面停留在最后实测姿态；参数仍发布反馈状态、时间和错误。
第一次有效反馈前最后有效时间显示不可用。停止后旧定时回调不能再发布或覆盖新周期状态。
采样重叠时跳过定时回调，手动读返回“采样正忙”，不排队。DDS 发布异常由 `info.publish_error` 显示，下一周期重试。
发布使用现有 Core DDS context/executor、`std_msgs/String`、BEST_EFFORT/KEEP_LAST(1)/VOLATILE。

模型详情见 `resource/SOURCE.md`。模型在构造时校验并导出，缺失/损坏时显式失败，避免 Core
悄悄使用人形骨架回退。六轴支持旋转；夹爪手指在 q=0 处固定，开度只显示数值。

用户照片对应的简化外壳已加入 `resource/monitor_shapes.json`，导出为标准 URDF `<visual>`。
底座/关节较粗、臂杆/腕部较细、夹爪为双指固定示意。用户明确保持 Core 不改；当前 Core
忽略这些外壳，因此只准备模型及独立预览，不宣称现有卡片已显示新外形。

镜像需复制 `monitor.py`、`monitor_bundle.py`、`status_card.py` 和整个 `resource/` 到 `/work/agilex/piper/`。
使用现有 ROS/std_msgs 依赖，不新增 Pinocchio、网格、SDK 或前端库。不要直接修改本阶段以外的 shared runtime。

## 验证与交付边界

Python 无硬件测试：

```bash
python3.12 -m unittest discover -s tests -p test_piper_monitor.py -v
```

浏览器模拟测试使用实际 Core 源文件与离线 Three.js 测试素材，见 `tests/piper_monitor_browser.mjs`。
准备 Three.js 0.164.1 的 `build/three.module.js` 与 `examples/jsm/controls/OrbitControls.js`，
放入同一测试目录；无需添加到驱动依赖或提交这两个文件。

```bash
PIPER_CORE_WEB=/path/to/agent-core/web \
PIPER_THREE_DIR=/path/to/offline-three \
PIPER_PYTHON=/path/to/python3.12 \
PIPER_PLAYWRIGHT=/path/to/node_modules/playwright \
PIPER_BROWSER=/path/to/chromium-executable \
PIPER_MONITOR_EVIDENCE=/path/to/evidence \
node tests/piper_monitor_browser.mjs
```

它只启动临时 loopback HTTP 测试服务，直接加载未经修改的 Core 渲染器；不访问现场服务、不连接 CAN。
真机模型方向/物理尺寸最终核对及 ROS2 现场联调由计划一集成后安排。

最终集成目标：[驱动 PR #382](https://github.com/4paradigm/phanthymotus-driver/pull/382)。
本监控提交不更新该 PR；计划一接入后更新。每次正式交付记录 Bot 表格实际 Version，
没有生成写“尚未生成”，提交 SHA 不作为 Version。
