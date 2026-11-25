# region_presence 区域目标在场基础卡

### 普通目标的位置调整与画质检测

将 VOP 接到本卡并设好目标类别、区域和置信度。VOP 应使用同一台 Bumi 彩色相机。
调用 `target_guidance`，一次返回位置、整帧质量和可供 TTS 使用的中文提示。
位置取最新有效 VOP 检测，画质取相机最近接收的 JPEG，两项分别判定时效，未进行逐帧同步。
仅需位置时调用 `check`，读取 `position_guidance`。

单个目标以所设区域中央三分之一为对齐范围。画面里仍看得见但处于区域外的目标也能获得移动提示。
多个合格目标同时出现时提示单独展示。画质复用 image_quality 的亮度、过亮/过暗像素比例和清晰度算法，
默认采用共享画质算法的阈值，无需在画布单独连接图像质量卡。低纹理画面也可能导致清晰度指标偏低。

现场可依次将目标置于区域左上方、移至中央、遮挡镜头，并观察位置提示和亮度指标的变化。

### 区域人数超限提醒

将 `target_name` 设为 `person`，将 `capacity_limit` 设为 1。先让一人站在区域内，第二人进入并保持超过 `enter_hold_s` 后，事件口发送一次 `capacity_exceeded`。一人离开且人数持续回落至上限以内，经过 `leave_hold_s` 后发送一次 `capacity_restored`。两种变化都至少需要两帧不同时间戳的观测。

`check.detected_count` 给出该帧区域内符合置信度要求的检测数量，`over_capacity` 给出最近确认的超限状态。断流、初始待确认或关闭功能时为 null。默认上限 0 关闭此功能。上限同样适用于其他目标类别。

### 等候区停留提醒

在卡片配置中设置 `dwell_alert_s=5`，将目标类别设为 `person`，并连接 VOP 检测流。
确认区域有人后开始计时。`check` 返回的 `occupied_s` 只随新检测帧更新；
首次达到 5 秒时，事件口输出一次 `dwell_reached`。继续站在区内不会重复提醒。
短暂漏检进入离开确认期时暂停显示时长，恢复后沿用本次在场起点。确认无人或断流后重新计时。
计时针对整个区域，区域内换人且始终有人时沿用同一次计时。默认阈值 0 表示关闭提醒，仍可查询时长。

扫码辅助动作 `scan_guidance` 可直接读取同一 Bumi 驱动最近两秒内收到的彩色 JPEG。
定位到二维码时，以画面中央三分之一区域为目标，提示持码人向上、下、左或右移动。
亮度取二维码外接框内的平均灰度，无法定位时取全图。超过 220 提示调暗，低于 45 提示补光。
在活动签到画布中，将本卡绑定到决策核心，并让 barcode 接入同一 Bumi 的 camera。
更新签到技能后，未读到码时会查询本卡，返回文字并通过已连接的 TTS 和扬声器提示。

### 现场验收扫码引导

直接调用本卡的 `scan_guidance` 可以逐步验收取景，即使扫码器已经能读出凭证。依次展示：

| 操作 | 预期观察 |
| --- | --- |
| 把完整二维码举到画面上方 | 已定位，提示往下移 |
| 居中后向后移，直到仍能定位但码框短边低于画面短边的 15% | 提示靠近镜头，`size_ratio` 给出实测比例 |
| 靠近并保持居中、亮度适中 | `framing_ok=true`，提示“二维码已对齐，请保持不动” |
| 让镜头看到过亮的空白画面 | 提示避开反光或调暗屏幕 |

15% 是取景提示的经验阈值。签到流程每次读码失败只播报一项调整，下一次以新画面为准。
取景合适后，是否成功识别凭证仍由扫码卡判定。

**把现有 VOP 的逐帧识别结果，变成一个稳定的“区域内有没有目标”状态。**
确认出现、确认离开时各输出一次事件，持续在场不重复输出。
可复用到门口迎宾、工位有人提醒、指定物品在位检查等场景。
它不识别身份、不累计进出人数、不自动播报或控制机器人动作。

## 为什么单独做成 Card

VOP 负责“这帧识别到了什么”；本卡负责区域筛选、出现/离开确认和断流处理。
不同 Skill 可以共享这一个状态，不必各写一套计时和漏检处理。
它不加载新模型、不开第二路相机、无新增 Python 依赖，构造时不启动 ROS。
VOP 自己仍需正常运行；未准备好的模型不能靠本卡替代。

算法与 ROS 包装在 `common/region_presence*.py`，不依赖 Bumi SDK。
当前在 Bumi Bundle 显式注册；其他驱动可用相同构造接口接入。
每个 Bundle 支持一张卡、一个区域、一个目标类别、一路 VOP 输入；
同一结果可以供多个消费者使用。

## 输入与输出

```text
camera ── image/jpeg ──► vop ── data/json ──► region_presence
                                                │
                                       data/json（变化事件）
                                                ▼
                                           decision_core
```

业务动作 `check` 读取当前结果，`target_guidance` 返回位置与画质提示，`scan_guidance` 返回扫码取景提示；`start/info/stop/config` 是系统动作。
`start()` 注册钩子不订阅数据。画布启动项目时解析实际连线，向系统动作
`start` 传入一路绝对 `input_topic` 后才运行。

输入为 `std_msgs/msg/String` 中的 VOP JSON：

```json
{
  "timestamp": 1794000000.1,
  "count": 1,
  "objects": [{"name": "person", "position": [0.1, -0.2], "confidence": 0.9}]
}
```

- `position` 是目标框中心的 VOP 坐标：画面中心 `(0,0)`，左上 `(-1,-1)`，
  右下 `(1,1)`，不是像素或米。目标中心在区域内才计入，边界包含在内。
- `name` 精确匹配类别名（忽略大小写和首尾空格），`confidence` 范围 0–1。
  先用 VOP 的 `list_recognizable_objects` 确认模型支持的准确类别名。
- 合法的 `objects: []` 表示这帧没有目标；错误 JSON、缺字段、模型错误、
  错连电量/pose 数据不会被当作无人。
- 必须有 `count == len(objects)` 和数值 `timestamp`；每帧至多 256 个检测，
  JSON 至多 256 KiB。重复或倒序的发布时戳不会刷新有效期或帮助确认状态。
- 输入订阅为 best effort、volatile、depth=1，与现有 VOP 输出兼容。

接入持续识别这一路相机的 VOP 实例。现有 VOP 的 `detect_photo` 和
`detect_photo_url` 也会向同一结果话题发布；区域监测期间不要在该实例混入
其他照片检测。上游 JSON 不携带可验证的采集来源，本卡不能替你判别图片来源。

事件输出话题为 `/<namespace>/region_presence/events`，同样为 String JSON。
事件只有变化时才输出，不是持续心跳；应用启动时应先调用 `check`。
事件携带 `event_id`、`config_revision`、`priority: 1`、当前目标数和输入话题。
`priority` 遵循当前 Agent Core 的 JSON 事件优先级约定，接到决策输入后可触发处理。
消息仍受 DDS 交付条件影响；发布报错可在 `info.publish_error` 查询。

## 参数怎么填

第一次全部保留默认值，先验证全画面中 person 的出现与离开。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `target_name` | `person` | VOP 中的准确目标类别名，可换成受支持的物品名 |
| `min_confidence` | 0.5 | 只接受置信度达到此值的目标 |
| `x_min / x_max` | -1 / 1 | 区域左/右边界 |
| `y_min / y_max` | -1 / 1 | 区域上/下边界 |
| `enter_hold_s` | 0.6 | 连续有目标多久才确认在场 |
| `leave_hold_s` | 1.5 | 连续没有目标多久才确认离开 |
| `stale_after_s` | 3 | 多久没有新发布的合法识别帧就失效 |

例如画面中央门口先尝试 `x_min=-0.6, x_max=0.6, y_min=-1, y_max=1`，
再按实际画面调整。它是二维画面区域，不对应真实地面距离。
确认时长为 0 仍需至少两帧；数据帧间隔应小于超时时长。
配置会全部校验后一次生效，成功更新后清空旧状态，重新等待观测。

## 查询与事件的含义

| 查询状态 | present | 含义 |
| --- | --- | --- |
| `ready` | true / false | 已确认在场 / 未检测到目标 |
| `stabilizing` | null | 正在确认，不能据此判定有人或无人 |
| `no_data / stale / invalid_data / idle` | null | 未收数据 / 断流 / 数据错误 / 已停用 |

`detected_count` 是最新有效帧中匹配区域和类别的框数，不是累计人流量。
`stable_state` 是最近一次确认的状态，过渡期间可能是历史值；业务应以
`state == ready` 和 `present` 判断当前结果。

| 事件 | 含义 |
| --- | --- |
| `initialized` | 启动、配置或数据恢复后首次确认状态；不是新来客证明 |
| `appeared` | 已确认从无目标变为有目标 |
| `disappeared` | 已确认从有目标变为无目标 |
| `unavailable` | 断流、错误、配置变化或停止；不是目标离开 |

一次遮挡短于 `leave_hold_s` 不会产生离开再出现事件。区域一直有人时，
另一个人进入不会再产生 `appeared`：本卡判断整体在场，不做个体追踪。
检测不到目标也不证明物品实际丢失，物品场景需 Skill 进一步确认。

新鲜度依据“收到新的 VOP 发布帧后的本地单调时间”。VOP 的 timestamp
是发布时刻，不是相机采集时刻，因此 `capture_time_verified=false`。
如果上游时钟被调回，倒序消息会被忽略；确认上游恢复后停止/启动本卡，
重建发布时戳基线。断流、坏帧恢复后重新确认，不沿用断流前的计时。

## 明天部署与真机验收

1. 本地文件已接入 `noetix/bumi/main.py`、默认配置和 `driver.yaml`。
   **当前还没有构建或安装包含本卡的新镜像**，刷新旧镜像页面不会出现它。
   把本次代码随当前 `feat/bumi-driver` 分支提交到自己的 fork，按项目
   PR 流程请求构建（`/request_bot_review`），使用构建返回的实际镜像版本。
2. 部署新测试镜像后刷新网页，在 BUMI-DEVICE-BUNDLE 的 processor 列表
   找到 `region_presence`。画布连线由人操作，首次只做检测验收即可。
3. 拖入相机、VOP、本卡及 decision_core；按上图连数据线。
   添加 decision_core 到本卡的执行绑定，才能让 Skill 调用 `check`。
   原有挥手/语音执行线按对应 Skill 需要连接。
4. 保留默认配置并启动项目。先确认 VOP 数据流确实有 `objects` 和
   `timestamp`，本卡 `info.input_topic` 是 VOP 输出而不是相机话题。
5. 空画面保持超过 1.5 秒：`check` 应为 `ready, present=false`，出现
   一次 `initialized`。人走入画面保持超过 0.6 秒：一次 `appeared`。
6. 站在画面里 10 秒：`check.present=true`，不反复发 appeared。
   人离开超过 1.5 秒：一次 `disappeared`；短暂漏检不触发离开。
7. 停止 VOP 或断开识别流超过 3 秒：`present=null`，事件为 unavailable，
   不得报 disappeared。恢复后先重新初始化，不能把恢复当作新来客。

迎宾 Skill 可以在 `appeared` 时调用既有 `semantic_action(wave)` 和播报。
如果希望启动时画面中已经有人也打招呼，需要 Skill 明确处理
`initialized + present=true`；本卡不自动执行动作。物品在位 Skill 则读取
同一格式的 present 状态，再按场景决定是否提示复查。

## 本地验证

```powershell
python -m pytest tests/test_region_presence.py tests/test_region_presence_card.py -q
python noetix/bumi/tools/region_presence_demo.py --output-dir D:/jiqiren/region-presence-demo
```

测试覆盖状态转换、区域/类别筛选、短暂漏检、断流、重播、坏数据、配置原子性、
启停、重绑定旧回调、部分初始化失败、ROS 工厂与 Bundle 注册。
演示使用合成 VOP JSON，不代表模型准确率、真实 DDS 或真机视觉验收。

2026-11-07 本地验证：本卡 47 项测试通过；连同图像质量与 Bumi 相关回归
共 262 项通过。实际 Bundle 工具导出通过框架的 Schema 转换检查，模型只需
看到业务动作 `check`。另外提取参考框架源码中的 VOP `_extract_objects`
和 `_publish_objects` 函数执行，所产 JSON 可被本卡接受；没有加载模型或启动 ROS。
尚未构建 ARM64 镜像，尚未进行真实 DDS、VOP 准确率或真机验收。
