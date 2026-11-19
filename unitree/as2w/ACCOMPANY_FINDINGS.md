# As2-W 伴随模式：取证记录与继续追踪控制路径的方案

> 本文档记录「能不能用软件启停伴随模式」这一问题的全部证据。
> 当前状态：**未找到控制路径，但已排除大半搜索空间，并把剩余三条假设做成了可在机器人上执行的实验**。
> 配套工具：`unitree/as2w/accompany_probe.py`（机器人上取证，默认只读）、
> `unitree/as2w/accompany_verdict.py`（笔记本上离线判读取证 JSON，纯标准库）。

---

## 1. 结论摘要

| 阶段 | 问题 | 结论 | 证据 |
|---|---|---|---|
| A | 公开 SDK 有没有伴随接口 | **没有** | AS2 sport 服务 21 个 api id 全列，无 follow/accompany/ISS；`sport_api.py` |
| A | 别的服务有没有 | **没有**（已列全） | 服务清单：`sport` / `slam_operate` / `audio` / `audio_led` / `videohub` / `motion_switcher` |
| A | 上游新版 SDK 有没有 | **没有**，我们不是版本落后 | 与 GitHub master 的 `as2/sport/sport_api.py` 逐行一致 |
| A | 手册怎么说 | 进入靠**人按遥控器 M 键**，退出 5 种全是物理操作 | 手册 p20 |
| B | 固件里到底有没有这个行为 | **有**，头部指示灯紫色常亮（手册 p14）就是固件在跑 ISS | 手册 p14 |
| B | 能不能从软件触发 | **未确定，剩三条假设** | 见 §4，工具已就绪 |

一句话：**行为在固件里，入口没公开。** 下一步不是再读文档，而是上机取证。

---

## 2. 已排除的路径（不要重复查）

| 路径 | 怎么排除的 | 结论 |
|---|---|---|
| AS2 sport 服务 | `unitree_sdk2py/as2/sport/sport_api.py` 全文，21 个 id（1001–1041） | 无 follow/accompany/ISS |
| 借用 B2 的 `TRAJECTORYFOLLOW(1018)` | B2 的 id 表与 AS2 **冲突**：AS2 的 1018 是 `HANDSTAND` | 机型家族不同，id 不可迁移 |
| 借用 Go2 的 2041–2058（FreeWalk/FreeAvoid 等） | AS2 id 表里不存在该区段 | 同上 |
| `slam_operate` 服务 | `controlled_spatial.py:21` 已列 1102/1201/1202/1801/1802/1804/1901 | 只有建图与导航，无跟随 |
| 上游 SDK 已支持 | 与 GitHub master 对比 | 我们的副本就是最新公开版 |
| 手册里另有启用方式 | 30 页逐页读 | 只有 M 键 / App 开关（App 与遥控器互斥） |

---

## 3. 本轮新增的四条线索（都可核对）

### 3.1 官方示例给出了 `mode` 的权威枚举（14 项）

出处：`unitree_sdk2_python` 仓库 `example/as2/sport/as2_sport_state.py`，订阅 `rt/lf/sportmodestate` 后打印 `FSM_STATE_STR[int(mode)]`：

```
PASSIVE, STAND_DOWN, STAND_UP, DEFAULT_MODE, RUNNING_MODE, CLIMB_MODE,
LEFT_SIDE_GAIT, RIGHT_SIDE_GAIT, HANDSTAND, BIPED_STAND, FRONT_FLIP,
BACK_FLIP, RECOVERY, BASE_HEIGHT_CTRL
```

**里面没有伴随态。** 但这份表的用处是：`mode` 一旦出现 **≥14 的值**，就一定是厂商未公开的模式——这比「猜名字」强得多，是可以直接判读的信号。已实现进工具的 `mode_name()` / `undocumented_mode()`。

### 3.2 本仓库早已观察到 `AI_*` 状态族

`device.py:737-748` 里，`_STANDING` / `_MOVING` / `_DOWN` 三个集合同时包含普通族与 `AI_` 族：

```
AI_STAND_UP, AI_BALANCE_STAND, AI_RECOVERY_STAND, AI_FREE_WALK, AI_WALK,
AI_RUN, AI_STAND_DOWN, AI_DAMPING, AI_FALL, AI_FALLEN
```

这说明 AS2 固件存在**第二套运动模式族**（对应 Go2 的「AI 模式」）。伴随是 ISS（激光雷达 + 感知）驱动的行为，落在 AI 族里的可能性最高。**这是目前最有价值的推断**。

### 3.3 `motion_switcher` 服务是切族的公开入口

`unitree_sdk2py/comm/motion_switcher/` 提供 5 个 api：`CHECK_MODE(1001)` / `SELECT_MODE(1002)` / `RELEASE_MODE(1003)` / `SET_SILENT(1004)` / `GET_SILENT(1005)`。外部资料（Go2 文档「AI 运动服务接口」）明确写着：Go2 的 AI 模式就是**先用 motion switcher 切到 AI 模式**，再调 sport 接口。

`CheckMode` 是只读的，返回机器人当前模式与可用模式。**如果 AS2 的模式列表里出现伴随相关名称，那就是控制入口。**

### 3.4 服务枚举的可行做法（工具已实现）

每个 Unitree RPC 服务就是一对话题 `rt/api/<service>/request|response`（`unitree_sdk2py/core/channel_name.py:14`）。所以「枚举服务」= 枚举 DDS 端点。

这里有个坑必须先说清，否则会白跑：

| 坑 | 事实 | 后果 |
|---|---|---|
| 容器里 ROS 用 FastDDS | `Dockerfile:67` `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` | **`ros2 topic list` 看不到 `rt/*`**，两种 DDS 实现互不可见 |
| 镜像的 CycloneDDS 没开 topic discovery | `Dockerfile:42` 编译时未加 `-DENABLE_TOPIC_DISCOVERY=ON` | `BuiltinTopicDcpsTopic` 大概率不可用 |
| 端点发现不需要那个开关 | DCPSPublication/Subscription 是标准能力 | **用订阅端点反推服务名，这条路可用** |

工具里 `discover_endpoints()` 走的就是端点发现（`BuiltinTopicDcpsSubscription` + `BuiltinTopicDcpsPublication`），复用 SDK 已建好的 participant，保证网卡与 domain 一致。

---

## 4. 三条待验证假设（按性价比排序）

### H1（最高优先）：伴随态在 `fsm_name` / `mode` 里可见，但名字未被登记
- **判别**：`--watch 60`，中途短按 M 键。看 `fsm_name` 是否变出一个新名字（如 `AI_ACCOMPANY`），或 `mode` 是否跳出 14 项枚举。
- **风险**：无（纯订阅 + GetState）。
- **若成立**：`config.yaml` 填 `accompany.fsm_names`，卡片立刻从 `inferred` 升为 `confirmed`。**但这只解决「看见」，不解决「控制」。**
- **若 mode 出现 ≥14 的值**：说明伴随是一个独立模式，那么按 §3.3，`motion_switcher` 很可能能选中它 → 转 H3。

### H2：存在未公开的 sport api id
- **依据**：AS2 的 id 表有 20 个空号：`1010, 1012, 1014, 1022–1033, 1035–1039`（工具 `sport_api_gaps()`）。
- **判别**：`--probe-ids` 逐个发空参数，看返回码。
- **风险**：**高**。未知 id 可能对应空翻、关节运动等。因此工具要求三道开关：`--armed --robot-on-stand --remote-in-hand`，且逐个发送、间隔 1 s、只发空参数。
- **注意**：返回 4xxx（不存在/无权限）说明该 id 不通；返回 0 **必须立刻看机器人有没有动**。

### H3：存在未公开的独立服务
- **判别**：`--services`，看是否有名称含 accompany/follow/iss 的 `rt/api/*`。
- **风险**：无。
- **若找到**：用 `CheckMode` 的思路先找只读 api，再谈写入。

---

## 5. 上机取证协议（一次做完，约 10 分钟）

前置：机器人架起（四轮离地）、遥控器在手、周围 2 m 无人、电量 > 40%。

```bash
# 1. 只读侦察：服务清单 + 运动模式清单
docker exec -it <container> python3 /work/accompany_probe.py --services --modes --json /tmp/step1.json

# 2. 标定观测：按提示按 M 键 / 拨摇杆
docker exec -it <container> python3 /work/accompany_probe.py --watch 60 --json /tmp/step2.json

# 3. 仅在 1、2 均无线索时才做，且必须架起机器人
docker exec -it <container> python3 /work/accompany_probe.py \
    --probe-ids 1022,1023,1024 --armed --robot-on-stand --remote-in-hand --json /tmp/step3.json
```

判读不必人工比对：把 JSON 拷回笔记本（或仍在容器里）后运行判读脚本，它按下面的判读表自动输出每条现象的结论与下一步（纯标准库，无 DDS 依赖）：

```bash
python3 /work/accompany_verdict.py /tmp/step1.json /tmp/step2.json [/tmp/step3.json] --json /tmp/verdict.json
```

判读表：

| 现象 | 含义 | 下一步 |
|---|---|---|
| 步骤 1 出现可疑服务 | H3 成立 | 对该服务做只读 api 探测 |
| 步骤 1 的模式列表里有伴随相关名称 | H3 成立（更省事） | `SelectMode(<name>)` 验证 |
| 步骤 2 中 `mode` 跳出 14 项 | 伴随是独立 mode | 用 motion switcher 反查该模式名 |
| 步骤 2 中 `fsm_name` 变新名 | H1 成立 | 填 `fsm_names`，仅获得观测能力 |
| 步骤 2 两者都不变 | 伴随态对两个通道不可见 | 只剩 H2 与厂商渠道 |
| 步骤 3 某 id 返回 0 且机器人动了 | H2 成立 | **记录该 id，但先别写进卡**：需确认真机语义与可重复性 |

---

## 6. 红线

1. **不要在未架起的机器人上跑 `--probe-ids`。** 未知 id 的行为未知，这是本方案唯一可能造成物理损坏的动作。
2. **不要把「探测到某个 id 返回 0」当成「能启停伴随」。** 返回 0 只说明服务端认识这个 id，语义必须真机复现 3 次以上才能写进卡片。
3. **即便拿到控制路径，手册的限制仍然成立**：伴随遥控器与 App 不能同时控制机器人，SDK 抢占底盘与「人握着遥控器」本身冲突。上线前必须重新评估互斥与接管策略。
4. **卡片描述里的安全信息不得因追到接口而删减**：`test_accompany.py` 里有断言守着这一条。

---

## 7. 结论回填位置

| 若得到 | 回填到 |
|---|---|
| 伴随态 fsm 名称 | `config.yaml` 的 `accompany.fsm_names`（数组，可多个） |
| 伴随态 mode 值 | `accompany.py` 的判定分支 + 新增测试 |
| 可用服务名 | `rpc_proxy.py` 的服务表 + `accompany.py` 新动作 |
| 可用 api id | 同上，且**必须**先补真机复现记录 |
| 全部失败 | 本文档 §1 表格追加一行「结论 C 最终确认」，PR 标题维持「观测」表述 |
