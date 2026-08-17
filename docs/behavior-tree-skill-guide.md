# 基于 Behavior Tree 编排 MCP 卡片开发 Skill 技能 — 协作指南

> 适用仓库：phanthymotus-driver
> 状态：首个落地案例（G1 迎宾 `g1-welcome`）已真机跑通并提交审核
> 技术栈与机型无关：MCP 协议统一，本指南的流程可推广到 T800、tianyi2.0 等所有机型

本指南面向仓库协作者，介绍如何用 **Behavior Tree（行为树）** 组合 driver 中已有的
MCP 卡片，开发出可被 agent-core 调用、可上传资源中心的**组合动作 Skill**。
读完本指南 + 跟随示例，即可开发你的第一个 BT 技能。

---

## 目录

1. [为什么用 Behavior Tree 编排 Skill](#1-为什么用-behavior-tree-编排-skill)
2. [Behavior Tree 原理](#2-behavior-tree-原理)
3. [对照本项目卡片开发 BT 技能](#3-对照本项目卡片开发-bt-技能)
4. [用 BT 编排 MCP 卡片：架构与实现](#4-用-bt-编排-mcp-卡片架构与实现)
5. [Skill 的部署与上传流程](#5-skill-的部署与上传流程)
6. [实战经验与常见坑](#6-实战经验与常见坑)
7. [附录：开发一个新技能的最小步骤清单](#7-附录开发一个新技能的最小步骤清单)

---

## 1. 为什么用 Behavior Tree 编排 Skill

### 1.1 现状问题：组合逻辑无处安放

MCP 卡片已经覆盖了单点能力（动作、感知、语音、灯效……），但"如何把它们组合成
一个场景动作"目前没有标准答案，存在三个痛点：

1. **LLM 是唯一的运行时编排器**：`agent-core` 中"先检查再动作"的组合逻辑每次由
   LLM 临时生成——不可复用、不可测试、不可可视化；同样的"先看 FSM 状态再转向"
   逻辑每个场景都要重新发明一遍。
2. **ACP barrier 只有线性串行**：异步动作完成后才放行下一个 actuator 调用，没有
   分支 / 并行 / 重试 / 超时 / fallback 语义。
3. **组合逻辑散落在硬编码里**：`switch_mode` 的 FSM 序列、`_do_auto_mapping` 状态
   分支、`smart_motion` 跨卡协调都写死在 driver 内，无法被场景层复用。

一句话：**卡片能力齐备，缺的是把它们编排成"技能"的标准化方法。**

### 1.2 BT 的优势：把组合语义标准化

Behavior Tree 把"顺序、备选、并行、重试、超时、条件、打断"这些组合语义**显式化为
一棵树**，相比硬编码顺序代码 / LLM 临时编排，本质优势是：

| 维度 | 硬编码 / LLM 临时编排 | Behavior Tree |
|---|---|---|
| **可复用** | 每次场景重新写一遍 | 子树可嵌入任意更大的流程（技能 → 场景技能） |
| **可测试** | 靠真机手测 | 无硬件单测（FakeMcpClient 断言调用序列） |
| **可可视化** | 不可见 | Groot2 直接导入/监控（BT.CPP 兼容 XML） |
| **可打断** | 手动处理中断 | halt 级联 + 打断原语是语言内置语义 |
| **容错完备** | 自己写 try/retry/fallback | Retry / Timeout / Fallback / Parallel 阈值开箱即用 |
| **可演进而非推翻** | 改行为 = 改代码 | 改行为 = 改 XML 声明，执行器不变 |

特别是**容错与优雅降级**：真机场景中卡片偶发失败（语音合成超时、手势参数不匹配、
FSM 状态不对）是常态。BT 的 `Fallback`（备选）、`Parallel` 阈值（部分成功即继续）、
`Retry`、`Timeout` 让"失败时怎么办"成为树声明的一部分，而不是散落的 if 分支。

### 1.3 为什么本项目适合：映射零概念缺口

这不是"引一个新框架进来"，而是**本项目的卡片契约恰好就是 BT 的三态模型**——
可行性研究（逐条对照 BehaviorTree.CPP v4 源码）确认了这一点：

| 本系统已有 | 恰好对应 BT 概念 | 说明 |
|---|---|---|
| 卡片 `tools/call` 同步返回 dict | Action 节点 SUCCESS/FAILURE | 无 error → SUCCESS；`{"error":...}` → FAILURE |
| ACP 异步完成（`action_id` + 完成回调） | 异步节点 RUNNING 状态 | 首 tick 发起，后续 tick 感知完成 |
| `smart_motion.interrupt_all` / `stop_move` | halt 打断级联 | 打断原语已内置，无需新造 |
| 状态卡查询（fsm_id、电量、位姿） | Condition 节点 | 读卡 + 阈值判定即条件 |
| 画布 / `requiredTools` 机制 | 技能卡的依赖声明 | 树工具卡直接进现有注册链路 |

四个具体原因：

1. **原语已齐备，零硬件改动**：G1 拥有动作卡（loco/arm/led/tts）、状态卡
   （state/fsm）、统一打断（smart_motion）、异步完成（ACP）——BT 需要的全部原语
   都已存在，编排层只是把它们组织起来；
2. **零侵入**：纯 stdlib Python 轻量执行器（`bt_runtime`，零第三方依赖），树定义用
   BT.CPP 兼容 XML，叶子只做 `tools/call`——**编排层永远位于 driver 安全边界
   （SmartMotion / N5 门禁）之上**，不改变底层实时控制与安全逻辑；
3. **与平台机制无缝对接**：每棵树 = 一张 MCP 工具卡 = 技能卡 `requiredTools` 的
   一个依赖——直接融入现有"注册 → 部署 → 上传 → 审核"流程，LLM 只调用树根，
   树的内部对 LLM 隐藏但描述可见；
4. **已获真机验证**：M0 迎宾技能（问候 + 挥手 + LED 闪烁）已真机跑通并提交审核，
   本指南的每一步流程都有实战背书。

### 1.4 一句话总结

> **卡片是能力，行为树是技能**。BT 把组合逻辑从"LLM 的临场发挥"和"driver 的
> 硬编码"中抽出来，变成可复用、可测试、可可视化、可打断的树声明——而本项目的
> MCP 契约与平台机制让这一步的映射成本趋近于零。

---

## 2. Behavior Tree 原理

### 2.1 核心思想

行为树是一棵**有向树**：根节点按固定周期（tick）向下传播一个 tick 信号，每个节点
执行自己的逻辑后向父节点返回状态。树不会"自己长出新行为"——**行为全部定义在叶子，
组合语义定义在内部节点**。

节点状态只有四种：

| 状态 | 含义 |
|---|---|
| `IDLE` | 未开始（节点初始/复位状态） |
| `RUNNING` | 正在执行中（异步任务进行时返回） |
| `SUCCESS` | 执行成功 |
| `FAILURE` | 执行失败 |

### 2.2 节点分类

```
                     ┌─────────────┐
                     │   根节点      │  ← tick 从这里开始，每周期一次
                     └──────┬──────┘
                        Sequence       ← 控制节点：决定孩子的执行逻辑
              ┌────────────┼────────────┐
          Condition     Action       SubTree   ← 叶子：读卡判定 / 调卡动作 / 复用子树
           (读状态)      (发指令)
```

**控制节点（内部节点，决定"怎么组合"）：**

| 节点 | 语义 |
|---|---|
| `Sequence` | 顺序执行：**任一孩子失败 → 整体失败**；全部成功 → 成功 |
| `Fallback` | 备选执行：**任一孩子成功 → 整体成功**；全部失败 → 失败 |
| `Parallel` | 并行执行全部孩子，按 `success_count` / `failure_count` 阈值定整体结果 |
| `ReactiveSequence` | 响应式顺序：每个 tick 从头检查，条件变化时**立即打断正在 RUNNING 的兄弟** |
| `ReactiveFallback` | 响应式备选：同上，用于恢复类逻辑 |
| `IfThenElse` | `if 条件 → then 分支，else 分支` |

**装饰器（包装一个孩子，改变其行为）：**

| 装饰器 | 语义 |
|---|---|
| `Retry` | 孩子失败时重试 N 次（BT.CPP v4 的 while 语义） |
| `Timeout` | 孩子运行超过 N 毫秒 → 打断它并返回失败 |
| `Repeat` | 孩子成功则重复 N 次 |
| `Inverter` | 反转孩子结果（成功 ↔ 失败） |
| `RunOnce` | 孩子只执行一次 |
| `ForceSuccess` / `ForceFailure` | 强制孩子结果 |

**叶子节点（本系统落地为 MCP 卡片调用）：**

| 叶子 | 语义 |
|---|---|
| `McpAction` | 同步调用一张 actuator 卡（`tools/call`），返回 dict 无 error → SUCCESS |
| `McpCondition` | 读卡判定（`tools/call` 查询 action），结果与 `check` 表达式比对 → SUCCESS/FAILURE |

### 2.3 黑板（Blackboard）

节点间共享数据用**黑板**：一棵树一个 `dict`，任何节点可读写，子节点（SubTree）
拥有继承父树数据的独立作用域。树参数（如问候语文本）经黑板注入，端口引用
`{key}` 语法解析——`text="{greeting}"` 表示运行时取黑板的 `greeting` 值。

### 2.4 三个关键机制

- **halt（打断）**：节点收到 halt 信号时，向所有 RUNNING 的孩子**级联传导**，直到
  叶子执行清理（停止动作）。这是"中断"语义的统一出口。
- **记忆性**：Sequence/Fallback 会记忆上次执行的进度（`Sequence` 从失败的下一个
  孩子继续）；`Reactive*` 则每个 tick 从头重评估。
- **tick 循环**：`while status == RUNNING: tick_once(); sleep(interval)`——树在
  异步任务（RUNNING）期间周期性地"醒来"继续推进。

> 一句话：行为树 = 把 `if/else + 顺序 + 并行 + 重试 + 超时 + 打断` 写成一棵可
> 复用、可测试、可可视化的树，而不是散落在代码里的面条逻辑。

---

## 3. 对照本项目卡片开发 BT 技能

开发技能的第一步不是写 XML，而是**看清 driver 里有哪些牌、每张牌在树里扮演什么
角色**。本章以 G1 真机 `tools/list`（28 个工具在线，2026-08-07 实测）为准，
介绍卡片体系与 BT 编排的对应关系。换机型（T800 / tianyi2.0）时方法相同——
`tools/list` 拿到卡片清单后，按本章的四类角色归类即可。

### 3.1 卡片全景（G1 真机）

卡片 = driver 暴露的 MCP 工具，分三类：`actuator`（可调用动作）、`sensor`
（数据流/状态）、`resource`（静态资源）。与技能编排直接相关的卡片：

| 卡片 | 类型 | 能力（action 枚举） | 在树中的角色 |
|---|---|---|---|
| `tts` | actuator | speak / get_volume / set_volume | 动作叶子：语音问候 |
| `arm` | actuator | execute（手势）/ release / list | 动作叶子：挥手等姿态 |
| `led` | actuator | set / off / effect | 动作叶子：灯效 |
| `loco` | actuator | move / stop_move / get_fsm_id / wave_hand / ... | 动作叶子 + 状态门禁 |
| `switch_mode` | actuator | 安全模式切换（lie2standup 等） | 动作叶子（置于门禁之后） |
| `controlled_spatial` | actuator | navigate_to_tag / navigate_to_pose / list_tags / ... | **异步叶子**（ACP RUNNING） |
| `smart_motion` | actuator | interrupt_motion / status | **打断兜底**（halt） |
| `loco_state` / `loco_motion_state` | sensor | 运动模式/速度/位置状态流 | 条件数据源 |
| `battery` / `imu` / `joints` | sensor | 电量 / 姿态 / 关节状态 | 条件数据源（门禁） |
| `motion_events` | sensor | SmartMotion 事件流 | 事件感知（进阶） |
| `mic` / `camera_*` / `lidar_cloud` / `ext_*` | sensor | 音视频/点云数据流 | **不进树**（topic 消费） |

### 3.2 四类卡片在树中的用法

**① 同步动作卡 → `McpAction` 叶子**

`tts` / `led` / `arm` / `loco`（move 等）调用后**阻塞至动作完成**才返回：
成功 → SUCCESS；返回 `{"error": ...}` 或调用异常 → FAILURE。

```xml
<McpAction tool="tts" action="speak" text="欢迎来到范氏集团"/>
```

注意：叶子超时（McpClient timeout）必须大于卡片最坏阻塞时长——真机 `arm` 手势
约 10s、`tts` 播完才返回，树内统一配 30s（见 §6.2）。

**② 状态查询卡 → `McpCondition` 门禁**

`loco` 的 `get_fsm_id` 等只读 action、`battery` 等状态卡：读卡结果与 `check`
表达式比对（值是列表 → 结果在列表内通过；是标量 → 结果相等通过）。

```xml
<McpCondition tool="loco" action="get_fsm_id" check='{"fsm_id": [500, 501, 801]}'/>
```

典型用法是**动作门禁**：先确认状态满足条件，再执行动作（§3.3 模式 1）。

**③ 异步动作卡 → ACP RUNNING 语义**

`controlled_spatial` 的导航类 action（`navigate_to_tag` / `navigate_to_pose`）
返回 `{"state": "navigating", "action_id": "nav_..."}`，完成/失败经 ACP
（`/api/acp/complete`）异步通知——正好对应 BT 的 RUNNING 状态与完成感知：

- **M0 现状**：叶子以同步方式调用，阻塞至导航结束（简单可靠，代价是树无法感知
  中间状态、不能中途重评估条件）；
- **M2 规划**：`AsyncActionNode`（onStart/onRunning/onHalted 接口已预留，
  `bt_runtime/actions/async_action_node.py`）——首 tick 发起导航返回 RUNNING，
  后续 tick 轮询完成，halt 时打断。

```xml
<!-- 异步叶子（M2 语义；M0 下同样写法，内部同步阻塞） -->
<McpAction tool="controlled_spatial" action="navigate_to_tag" tag_name="P3"/>
```

**④ 打断卡 → halt 兜底**

`smart_motion`（`interrupt_motion`）是统一打断原语。树内一般不主动调用它，而是
由技能的 `interrupt` 路径使用（SkillPlugin → 置位 stop_event → `halt_tree()` →
`smart_motion` 兜底）——保证中断安全收敛，不绕过 SmartMotion 安全层（§6.6）。

### 3.3 从卡片到树的常见编排模式（模板库）

以下模式全部用真机卡片示例，直接改参数即可套用到你的技能。

**模式 1 · 动作门禁（先检查再动作）** —— 带状态的机器人技能必备

```xml
<Sequence name="stand_gate">
  <McpCondition tool="loco" action="get_fsm_id" check='{"fsm_id": [500, 501, 801]}'/>
  <McpAction tool="loco" action="move" vx="0.5" duration="2"/>
</Sequence>
```

FSM 不满足（未处于站立态）→ 门禁 FAILURE → 整个 Sequence FAILURE，绝不带着
错误状态执行动作。

**模式 2 · 优雅降级（失败不阻塞整体）** —— 非核心步骤用

```xml
<Fallback name="polite_turn">
  <Sequence name="safe_turn">
    <McpCondition tool="loco" action="get_fsm_id" check='{"fsm_id": [500, 501, 801]}'/>
    <McpAction tool="loco" action="move" vx="0" vy="0" vyaw="0.8" duration="1.5"/>
  </Sequence>
  <AlwaysSuccess/>
</Fallback>
```

门禁或转向失败 → 跳过转向，迎宾整体仍 SUCCESS（迎宾技能"无转身"新需求即由
删除该 Fallback 段实现——改行为就是改 XML）。

**模式 3 · 重试（短暂失败自愈）** —— 偶发失败的手势/动作

```xml
<Retry num_attempts="3" name="wave_retry">
  <McpAction tool="arm" action="execute" gesture="face wave"/>
</Retry>
```

**模式 4 · 并行（两个动作同时发起）** —— 问候 + 挥手

```xml
<Parallel success_count="1" failure_count="2" name="greet">
  <McpAction tool="tts" action="speak" text="欢迎来到范氏集团"/>
  <McpAction tool="arm" action="execute" gesture="face wave"/>
</Parallel>
```

任一成功整体即成功（语音失败不阻塞挥手，反之亦然）；注意 M0 叶子同步阻塞，
实际仍是串行发起，真实并行待 M2 异步叶子（§3.2 ③）。

**模式 5 · 循环（周期性动作）** —— LED 闪烁

```xml
<Repeat num_cycles="3" name="blink">
  <Sequence name="once">
    <McpAction tool="led" action="set" r="0" g="255" b="0"/>
    <McpAction tool="led" action="off"/>
  </Sequence>
</Repeat>
```

**模式 6 · 超时保护（防止卡死）** —— 包裹不可靠调用

```xml
<Timeout ms="30000" name="speak_guard">
  <McpAction tool="tts" action="speak" text="欢迎来到范氏集团"/>
</Timeout>
```

超过 30s 未完成 → 打断叶子并 FAILURE。

### 3.4 卡片编排硬性规则

1. **参数必须对照真机 schema**：XML 引用的卡片/action/参数必须在 `tools/list`
   真实枚举内（最贵的一课，见 §6.1）——开发新技能前先 `curl tools/list` 盘点；
2. **sensor 数据流卡不进树**：`mic` / `camera_*` / `lidar_cloud` 等是 topic 数据流
   （start/stop + 订阅），树里读状态只走查询型 action（`loco get_fsm_id`、
   `battery` 等）；
3. **安全边界不可破坏**：运动卡（`loco` / `switch_mode` / `controlled_spatial`）
   路由 SmartMotion 安全层；打断走 `smart_motion`，树不得绕过（§6.6）；
4. **阻塞型卡配足超时**：叶子 McpClient timeout ≥ 卡片最坏阻塞时长（真机统一
   30s，§6.2）；
5. **高频控制不进树**：loco 100Hz 运动发布、joint PD 属 driver 底层，树只做
   任务级编排（调用频率 ≤10Hz）。

### 3.5 树格式选型：BT.CPP 兼容 XML

树定义采用 **BT.CPP 兼容 XML**（`BTCPP_format="4"`），执行器为 Python 轻量实现
（`bt_runtime`，纯 stdlib 零依赖）。三个收益：

1. **Groot2（BT.CPP 官方可视化工具）可直接导入/监控我们的树**；
2. 树定义与官方生态互通，**未来若需迁移原生 BT.CPP，XML 是现成契约，零改写**；
3. 语义与官方逐条对齐，社区文档、语义表全部适用（语义基准见文末"技术来源"）。

---

## 4. 用 BT 编排 MCP 卡片：架构与实现

### 4.1 双轨机制（先理解全貌）

一个技能由**两层**组成，缺一不可：

```
┌─────────────────────────────────────────────────────────────┐
│ 层 2 · 技能卡（平台入口/门面）  —— agent-core 侧              │
│   data.db config 表 skills 键  {"installed": [{slug, name,   │
│   instruction, requiredTools, configSchema, ...}]}           │
│   → 画布激活 → LLM activate_skill() → instruction 注入 prompt│
└──────────────────────────┬──────────────────────────────────┘
                           │ instruction 引导 LLM 调用
┌──────────────────────────▼──────────────────────────────────┐
│ 层 1 · 执行层（树引擎）  —— driver 侧                         │
│   SkillPlugin（MCP 工具：welcome_skill）                      │
│   └─ bt_runtime 执行器 加载 welcome.xml（BT.CPP 兼容 XML）    │
│      └─ 叶子 = MCP tools/call → led / tts / arm / loco 卡片  │
└─────────────────────────────────────────────────────────────┘
```

**BT 树是技能的执行引擎，技能卡是平台的入口**：LLM 只调用 `welcome_skill` 这一个
工具，树的内部组合对 LLM 隐藏，但行为描述可见。

### 4.2 代码结构

```
unitree/g1/
├── bt_runtime/                  # BT 执行器（纯 stdlib Python，零第三方依赖）
│   ├── status.py                # NodeStatus: IDLE/RUNNING/SUCCESS/FAILURE
│   ├── blackboard.py            # 黑板：dict + RLock + parent 链
│   ├── mcp.py                   # McpClient: MCP JSON-RPC 2.0 over HTTP
│   ├── fakes.py                 # FakeMcpClient：无硬件测试用
│   ├── tree_node.py             # 节点基类：tick/halt 级联
│   ├── controls/                # Sequence/Fallback/Reactive*/Parallel/IfThenElse
│   ├── decorators/              # Retry/Timeout/Repeat/Inverter/RunOnce/Force*
│   ├── actions/                 # McpActionNode（同步叶子）/ AsyncActionNode（M2）
│   ├── conditions/              # McpConditionNode（读卡判定）
│   ├── xml_parser.py            # BTCPP_format="4" XML 解析
│   ├── tree.py                  # Tree: tick_once/tick_while_running/halt_tree
│   └── runner.py                # CLI：--tree <id> [--mcp-url <url> | --fake]
├── skills/
│   ├── welcome.xml              # 技能树定义（BT.CPP 兼容 XML）★ 你主要写这里
│   └── registry.py              # 树注册表：id → xml 路径 / 工具名 / 描述
├── skill_plugin.py              # SkillPlugin：每棵树 = 一张 MCP actuator 工具卡
├── config.yaml                  # plugins.skills.enabled / timeout
├── main.py                      # SkillPlugin 装配（传 smart_motion 插件引用）
└── scripts/
    ├── install_welcome_skill.py   # 技能卡写入 agent-core data.db
    └── uninstall_welcome_skill.py # 按 slug 卸载
```

### 4.3 叶子节点：MCP 卡片如何被调用

**`McpAction`（动作叶子）**——XML 中除保留属性 `tool / action / name / description`
外的所有属性，自动作为 `tools/call` 的参数透传（字符串值自动转 int/float/bool）：

```xml
<McpAction tool="arm" action="execute" gesture="face wave"/>
<!-- 等价于：tools/call {name: "arm", arguments: {action: "execute", gesture: "face wave"}} -->
```

**`McpCondition`（条件叶子）**——`check` 属性为 JSON 表达式，逐 key 判定读卡结果：

```xml
<!-- 读 loco 卡 get_fsm_id，若结果 fsm_id 在 [500,501,801] 中 → SUCCESS -->
<McpCondition tool="loco" action="get_fsm_id" check='{"fsm_id": [500, 501, 801]}'/>
```

判定规则：`check` 的值是**列表** → 结果在列表内即通过；是**标量** → 结果相等即通过。
调用失败或卡片返回 `{"error": ...}` → FAILURE。

### 4.4 树定义：以迎宾技能为例（真实代码）

`unitree/g1/skills/welcome.xml`——组合 `tts`（语音）+ `arm`（挥手）+ `led`（灯效）
三张卡片，实现"同时问候 + 挥手，然后 LED 闪烁 3 次"（无移动）：

```xml
<root BTCPP_format="4" main_tree_to_execute="Welcome">
  <BehaviorTree ID="Welcome" description="迎宾：语音问候 + 挥手 + LED 闪烁（无移动）">
    <Sequence name="welcome_root">
      <!-- 两个动作都执行（问候 + 挥手），任一失败可容忍；success_count 仅决定
           整体结果（有意偏离 v4：不提前 halt 未执行的兄弟） -->
      <Parallel success_count="1" failure_count="2" name="greet">
        <McpAction tool="tts" action="speak" text="欢迎来到范氏集团"/>
        <McpAction tool="arm" action="execute" gesture="face wave"/>
      </Parallel>
      <!-- LED 闪烁 3 次（绿 → 灭），无需移动/转身 -->
      <Repeat num_cycles="3" name="led_blink">
        <Sequence name="blink_once">
          <McpAction tool="led" action="set" r="0" g="255" b="0"/>
          <McpAction tool="led" action="off"/>
        </Sequence>
      </Repeat>
    </Sequence>
  </BehaviorTree>
</root>
```

逐节点解读：

- 根 `Sequence`：先"问候段"，后"LED 段"，顺序完成；
- `Parallel success_count=1 failure_count=2`：问候与挥手**两个动作都执行**，
  任一成功整体即成功（语音失败不阻塞挥手，反之亦然）——注意本实现有意偏离 v4
  默认"达到阈值即 halt 未执行兄弟"的行为，注释已在 XML 中说明；
- `Repeat num_cycles=3`：`set(绿) → off` 循环 3 次实现闪烁。

> 参考：最初版本是"先 LED 变绿 → 问候+挥手 → 转身"，真机验收后按业务需求调整为
> 上面的最终形态。**树是声明式的，改行为就是改 XML**——这是 BT 相比硬编码的
> 最大优势。

### 4.5 注册树

把树登记到 `unitree/g1/skills/registry.py`——SkillPlugin 与 CLI runner 共用：

```python
SKILLS = [
    {
        "id": "welcome",                          # CLI --tree welcome
        "xml_path": str(_SKILLS_DIR / "welcome.xml"),
        "main_tree": "Welcome",                   # XML 中 BehaviorTree ID
        "tool_name": "welcome_skill",             # MCP 工具名（LLM 可见）
        "description": "迎宾：同时语音问候与挥手 + LED 闪烁（行为树驱动，无移动）",
    },
]
```

### 4.6 工具卡：SkillPlugin 自动暴露

`skill_plugin.py` 按现有 Plugin 规范实现（`get_tools`/`start`/`stop`/`dispatch`，
`dispatch` 返回纯 dict，处理 `start/stop/info/run/interrupt/status`）：

- 每棵树生成一张 **actuator** 工具卡，`action` 枚举 `[run, interrupt, status]`，
  带 `x-completion`（`run` 声明异步，timeout 取 `config.yaml` 的 `plugins.skills.timeout`）；
- `run`：后台线程构建树 → `tick_while_running()` → 完成/失败 POST `/api/acp/complete`
  （复用 `controlled_spatial._acp_notify` 模式）；
- `interrupt`：置位 stop_event（**防重放**，见 §6.4）→ `halt_tree()` → 运动卡兜底
  `smart_motion.interrupt_all`（经既有安全层，不绕过）→ ACP 回 `cancelled`；
- `status`：返回 `{"state": "running"|"idle"|"cancelled", "last_status": ...}`。

装配走既有模式：`main.py` Bundle 按 `config.yaml` 的 `plugins.skills.enabled` 加载，
Dockerfile 显式 COPY 新增文件。`config.yaml`：

```yaml
plugins:
  skills:
    enabled: true
    timeout: 120        # x-completion 默认超时
```

### 4.7 测试（无硬件可全量跑）

三层测试，共 82 个用例（unittest，与仓库既有测试同模式）：

```bash
cd unitree/g1
python3 -m unittest discover -s bt_runtime/tests -t .   # 执行器节点语义（60 个）
python3 -m unittest discover -s skills/tests -t .       # 树集成 + schema 快照（17 个）
python3 -m unittest discover -s scripts/tests -t .      # 部署脚本（5 个）
```

- **执行器测试**（`bt_runtime/tests/`）：用 `FakeMcpClient` 对照官方语义表逐节点
  验证——Sequence 复位/记忆、Parallel 阈值、Retry/Timeout/Repeat、halt 级联等；
- **树集成测试**（`skills/tests/`）：`FakeMcpClient` 断言调用序列（问候→挥手→
  LED×3），以及**真机 schema 快照测试**——把真机 `tools/list` 的枚举值硬编码到
  测试（如 `ARM_GESTURES = {"face wave", "high wave", ...}`），XML 引用的参数
  必须存在于快照内，防止"参数名与真机 schema 不一致"类缺陷（§6.1 的教训来源）；
- **部署脚本测试**（`scripts/tests/`）：临时 sqlite 库验证 install/uninstall 幂等。

### 4.8 本地运行验证

```bash
cd unitree/g1
# 无硬件：FakeMcpClient，打印卡片调用序列与树结果
python3 -m bt_runtime.runner --tree welcome --fake
# 真机直连（G1 MCP 端点）
python3 -m bt_runtime.runner --tree welcome --mcp-url http://10.110.12.110:15701/mcp
```

---

## 5. Skill 的部署与上传流程

一条技能从开发到上线走四步：**执行层部署 → 技能卡部署 → 画布激活 → 上传审核**。

### 5.1 部署执行层（driver 容器）

```bash
# 1. 构建增量镜像（真机上用 Dockerfile.bt，基于现有已发布镜像 + COPY 新文件）
#    首次构建亦可直接用 g1/Dockerfile 全量构建
cd ~/phanthymotus-driver   # 注意：tar 内含 unitree/g1/ 前缀，须在仓库根解压
docker build -f unitree/g1/Dockerfile.bt -t phanthy-g1-driver:bt-<skill> .

# 2. 真机加载并重启驱动容器
cd /opt/phanthy-motus && docker compose up -d unitree-g1

# 3. 验证新工具已注册（tools/list 应出现 welcome_skill）
curl -X POST http://<机器人IP>:15701/mcp \
  -H "Content-Type: application/json" \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}'
```

### 5.2 部署技能卡（agent-core data.db）

技能卡字典写入 agent-core 的 SQLite（`config` 表 `skills` 键，`{"installed": [...]}`），
字段：`slug / name / description / oneLiner / instruction / category / version /
author / installedAt / active / requiredTools / configSchema / icon`。
**`instruction` 是最重要字段**——它是注入 LLM prompt 的指令，需要引导 LLM 优先调用
树工具，并给出树不可用时的手动降级步骤。

`scripts/install_welcome_skill.py` 为参考实现（幂等：按 slug 去重，`UPDATE` 影响
0 行时回退 `INSERT`，防止静默假成功）：

```bash
# 本地开发库
python3 scripts/install_welcome_skill.py --db-path agent-core/resource/data.db
# 真机
scp unitree/g1/scripts/install_welcome_skill.py unitree@<机器人IP>:/tmp/
ssh unitree@<机器人IP> "echo 123 | sudo -S python3 /tmp/install_welcome_skill.py"
ssh unitree@<机器人IP> "echo 123 | sudo -S docker compose -f /opt/phanthy-motus/docker-compose.yml restart agent-core"
```

卸载：`scripts/uninstall_welcome_skill.py`（按 slug 删除）。

### 5.3 画布激活

重启 agent-core 后，前端画布「设计」→「技能」中应出现该技能卡，激活后 LLM 调用
`activate_skill("g1-welcome")`，`instruction` 注入 prompt，LLM 即可按指令调用
`welcome_skill` 工具完成迎宾。

### 5.4 上传资源中心（提交审核）

技能开发 + 真机验收通过后，上传到资源中心（motus.phanthy.com）：

```bash
export RESOURCE_CENTER_API_KEY=rc_<你的API Key>
curl -X POST https://motus.phanthy.com/api/skills/mine \
  -H "X-API-Key: $RESOURCE_CENTER_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "slug": "g1-welcome",
    "name": "G1迎宾",
    "description": "迎宾场景：同时语音问候与挥手、LED 闪烁（行为树驱动，无移动）",
    "oneLiner": "G1迎宾 - 问候+挥手+LED闪烁",
    "instruction": "你是 G1 迎宾助手。当用户要求执行迎宾时：\n1. 优先调用工具 `welcome_skill`，action=\"run\"……",
    "category": "robot",
    "icon": "👋",
    "version": "0.1.0",
    "requiredTools": ["welcome_skill", "led", "tts", "arm", "smart_motion"],
    "configSchema": {}
  }'
```

要点：

- **payload 字段**：`slug / name / description / oneLiner / instruction / category /
  icon / version / requiredTools / configSchema`（`configSchema` 可选，无持久化配置
  需求时留 `{}`，官方示例同样为空，不影响审核）；
- 上传成功后返回 `{id, slug, status: "draft"}`，进入**草稿**状态；
- 到前端 https://motus.phanthy.com/zh/skills 的技能广场提交审核；
- `requiredTools` 声明技能依赖的 MCP 工具（含树工具自身），供审核方核对。

### 5.5 验收检查清单

```
□ python3 -m bt_runtime.runner --tree <id> --fake        # 本地树逻辑 OK
□ 三层测试全绿（执行器 / 树集成 + schema 快照 / 脚本）
□ 真机 CLI 跑通（--mcp-url 直连）——实际动作正确执行
□ 新镜像部署后 tools/list 出现 <tree>_skill 工具
□ install 脚本写库成功，重启 agent-core 后画布可见、可激活
□ 前端/LLM 调用 welcome_skill 完成真实场景
□ 上传资源中心成功（status=draft）并已提交审核
```

---

## 6. 实战经验与常见坑

以下全部来自 G1 迎宾技能真机验收时踩过的坑，写树时请对照检查。

### 6.1 参数必须对照真机 schema（最贵的一课）

现象：手臂动作"没有触发"，静默失败。根因：`gesture="wave"` 不在真机枚举里
（真机是 `"face wave"`），而 `Parallel success_count=1` 的容错把失败吞掉了——
树返回 SUCCESS，人眼只见"没挥手"。

**对策**：schema 快照测试。把真机 `tools/list` 的枚举硬编码进 `skills/tests/
test_welcome_tree.py`（`ARM_GESTURES`/`LED_ACTIONS`），XML 引用的任何参数必须在
快照内。**新树引用某张卡之前，先 `curl tools/list` 查它的真实 schema。**

### 6.2 叶子超时必须大于卡片阻塞时间

默认 `McpClient` timeout 3s，而真机 `arm.ExecuteAction` 阻塞约 10s、RPC 延迟
6-10s——5s 超时会中断树内调用导致动作未触发（RPC 错误码 3104
`RPC_ERR_CLIENT_API_TIMEOUT`）。**所有树执行相关客户端用 30s**：runner 与
`skill_plugin._default_client` 均 `McpClient(..., timeout=30.0)`；真机
`loco.SetTimeout(30.0)`、`arm_client.SetTimeout(30.0)`。

### 6.3 注册表漏注册直接报错

初版 XML 用了 `Repeat` 节点但未实现注册 → `ValueError: unknown node type: Repeat`。
新增节点类型需在 `bt_runtime/decorators/`（或 `controls/`）实现并在
`BehaviorTreeFactory._register_builtins()` 注册。

### 6.4 interrupt 防重放

`interrupt` 若只调 `halt_tree()`，tick 循环醒来后**会从头重放整棵树**（动作重复执行
+ 双重 ACP 回调）。**主防线是 `stop_event`**：`tick_while_running` 在每次 tick 前
（含首 tick）检查；interrupt 先置位 stop_event，再 halt_tree，再 `smart_motion.
interrupt_all` 兜底，ACP 回 `cancelled`。

### 6.5 部署后必须进容器核对文件

tar 包内容带 `unitree/g1/` 前缀，解压到错误目录会让所有修复"从未进入容器"——
症状是改了代码但真机行为不变。**部署后 `docker exec` 进容器核对文件内容**（如
welcome.xml 是否含最新注释/节点），再重启验证。

### 6.6 安全边界不可破坏

- 树的叶子只做 `tools/call`；运动卡仍路由 SmartMotion 安全层，N5 门禁
  （`velocity_proposal`）不受影响——**编排层永远在 driver 安全边界之上**；
- 打断走 `smart_motion.interrupt_all`，不绕过安全层；
- 高频控制（loco 100Hz、joint 500Hz PD）不进树——树只做任务级编排
  （调用频率 ≤10Hz）。

---

## 7. 附录：开发一个新技能的最小步骤清单

```
□ 1. curl tools/list 盘点可用卡片与真实参数枚举（对照 §6.1）
□ 2. 在 unitree/g1/skills/ 新建 <skill>.xml，用 Sequence/Fallback/Parallel/
       Retry/Timeout/Repeat 编排动作（仿 welcome.xml）
□ 3. registry.py 注册（id / xml_path / main_tree / tool_name / description）
□ 4. skills/tests/ 写集成测试：FakeMcpClient 断言调用序列 + 真机 schema 快照
□ 5. python3 -m bt_runtime.runner --tree <id> --fake 本地验证
□ 6. 真机 CLI 验证：--mcp-url http://<IP>:15701/mcp
□ 7. 构建增量镜像 → 部署 → tools/list 确认新工具
□ 8. scripts/install_<skill>.py 写技能卡（仿 install_welcome_skill.py）
       → 重启 agent-core → 画布激活 → LLM 调用验证
□ 9. 上传资源中心（§5.4）→ 前端提交审核
```

---

## 相关文档


- 驱动开发规范：仓库根 `README_dev.md`（工具 schema / x-action-params / configSchema）
- 平台技能规范：飞书《新同学 landing 文档》§4（技能卡字段与上传流程）
- 技术来源（语义基准，对照开发/阅读时以此为准）：
  - BehaviorTree.CPP 源码：https://github.com/BehaviorTree/BehaviorTree.CPP
  - BehaviorTree.ROS2 源码：https://github.com/BehaviorTree/BehaviorTree.ROS2
- BT.CPP 官方文档：https://www.behaviortree.dev/
