# Generic Simulator — 虚拟具身卡片 bundle

一组**虚拟 plugin 卡片**，在 Agent Core 画布上与真驱动卡片同等使用：拖到画布上、
连到 `decision_core`、看渲染、被 LLM 调用。用来在**没有硬件**的情况下跑通并断言
agent + VLA 回路。

它不是传统意义的物理仿真器。见下面「这个后端不做什么」。

| | |
|---|---|
| 端口 | 15711（15712–15714 预留给其他 simulator 变体） |
| 镜像 | `simulator-generic` |
| 构建 | `./build.sh simulator/generic` |
| 测试 | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_sim_*.py -q` |

## 为什么它能冒充驱动

驱动就是一个 MCP HTTP 服务器 + 每 30 秒往 `/api/mcp` 注册一次。agent-core 的
`ros2_bridge.py` 和 `topic_subscriber.py` 是**纯订阅方**，分辨不出仿真与真机 ——
所以这个 bundle **不需要改 agent-core 一行**。

## 卡片

**Sensor**：`odom` `imu` `laser_scan` `battery` `map`
**Actuator**：`loco` `nav` `switch_mode` `arm` `led` `tts` `speaker` `sim_scenario`
**Resource**：`model`（URDF）`sim_report`（事件、播报记录、断言判定与得分）

所有传感器都从**同一份世界状态**派生（`sensors.py` 是纯函数），不是各自造随机数 ——
激光雷达与里程计不会互相矛盾，这是仿真值得信的前提。

几个刻意的选择：

- **locomotion 卡叫 `loco`，语音卡叫 `tts`**，动作分别是 `stop_move` / `interrupt`。
  `llm.py` 的打断兜底恰好找这两个名字。详见 `cards_motion.py` 顶部与
  `tests/test_sim_interrupt_naming.py`。
- **`model` 和 `sim_report` 是 `resource` 类型**。`_needs_barrier` 豁免
  `sensor`/`resource`，所以能在一段 90 秒导航 pending 期间读进度；写成 actuator
  则每次状态查询都排队在导览后面。
- **`sim_scenario` 的动作是 `run`/`abort`，不是 `start`/`stop`** —— 后者是框架发给
  每张卡的生命周期动词，会被基类先拦截，而且 agent-core 启动项目时就会自动开跑一趟导览。
- **`x-completion` / `x-resource` / `x-hooks` 写在 `inputSchema` 里面**。agent-core
  只从那里读；放在工具字典顶层会静默失效，任何日志都不报错。

## 跑一趟导览

画布上：加载场景 → `run` → 看 `map`、`sim_scenario` 的状态面板、活动流与
`performance.js` 的分步耗时；跑完 `sim_report` 给判定与得分。

命令行：

```bash
curl -s localhost:15711/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call",
       "params":{"name":"sim_scenario","arguments":{"action":"load","scenario":"exhibition_tour"}}}'
curl -s localhost:15711/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":2,"method":"tools/call",
       "params":{"name":"sim_scenario","arguments":{"action":"run"}}}'
# 跑着的时候随时可以查（sim_report 是 resource，不受 barrier 阻塞）
curl -s localhost:15711/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"sim_report","arguments":{}}}'
```

## 场景

`scenarios/*.yaml` 是**唯一真相**：`sim_scenario` 卡在真机上跑它，pytest 用假时钟
重放它，两边过同一份 `assertions.py`。否则 CI 绿和真机绿就不再是同一件事。

新增场景不用重建镜像 —— 丢进 `/opt/phanthy-motus/data/sim/scenarios`（已 bind-mount），
刷新画布，卡片配置里的下拉框就有了（`configSchema` 在调用时扫描目录生成）。

打断事件优先用 `after_arrival` + `delay`，不要用绝对的 `at:` —— 真机上 LLM 往返
3–48 秒，绝对偏移在假时钟下落在某一段中间，在真机上可能已经过了两个航点。

## 这个后端不做什么

**没有路径规划。** `LocalBackend` 直线驱向目标；真机的 Slamtec 底盘会规划。所以两个
航点之间横着一堵墙，那一段会撞停 —— 而失败原因与被测的编排无关。
`Scenario.validate()` 在 load 时把所有「直线不通」的航点对报出来，别等跑一遍才发现。

**没有动力学、没有接触、没有渲染。** 要回答的是「这条链路对不对」，不是「这个策略行不行」。
后者需要物理后端，接口（`WorldBackend`）第一天就留好了：`reset/apply/step/state/sense`，
`config.yaml` 里 `world.backend` 一个键切换，卡片不知道自己在跟谁说话。计划是
Stage 1 只把碰撞/FK 换成真的（仍在机器人上），Stage 2 整个后端搬去
`phanthymotus-cloud` 做流式远端。

**吞吐层测不了感知。** `perception/` 只有 `Dockerfile.jetson`，TensorRT engine 由镜像
的 JetPack 版本决定，x86 上没有等价物。

## 目录

```
world.py        VirtualWorld：导航任务、语音、事件日志，一把 RLock
backend/        WorldBackend 接口 + LocalBackend（独轮车积分 + 栅格射线）
clock.py        注入时钟（RealClock / FakeClock）
geometry.py     Pose、OccupancyGrid
sensors.py      从世界状态派生传感器读数的纯函数
card_base.py    卡片基类：生命周期、发布器、权威 info；以及格式→渲染器对照表
cards_*.py      各卡片
scenario.py     场景定义与发现
assertions.py   裁判：七项断言 + 五维评分
acp.py          /api/acp/complete 回调
plugins.py      build_plugins：构造一个世界，注入每张卡
```

`world.py` 的并发规则逐条对应 `perception/README.md` §「Plugin Concurrency」——
`dispatch()` 跑在 `ThreadingHTTPServer` 线程上，而 tick 线程改同一份状态。改这个文件前
先读它的模块 docstring。
