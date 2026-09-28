# Go1 新卡 loco_confirmed：设计与测试说明

更新：2026-09-28。分支：`feat/go1-loco-confirmed-card`。
新增卡已部署；最新姿态控制权修订有 59 项离线回归通过。下述 Bot 版本的单次短移/停止测试通过，供 Master 评审；新提交需重新构建，不覆盖所有姿态、长动作或平台中断链路。

## 最新：保持姿态的控制权修复（未真机验证）

- 修复 Bot 对 `2d74268` 的审查问题：姿态达到反馈判据返回 completed 后，SDK 仍持续下发姿态，因此新卡继续持有控制权，不再在动作完成时自动释放。
- 保持期间旧 `loco` 的移动、姿态及普通 stop 写入均被 SDK 仲裁拒绝；新卡后续动作也返回 RESOURCE_BUSY。应先显式调用新卡 `stop_move`，或停止新卡生命周期；原 stop 不是本卡的接管/急停入口。
- 新卡显式停止先设置独立 stop latch，再释放持有者；释放 RPC 失败保留本地持有记录，禁止新动作并允许再次显式停止。`start` 不释放姿态、不清除停止锁存；新的显式动作才会重新获得控制权。
- `status` 的 `holding_control` 表示该历史动作是否仍由本卡作为保持姿态持有；completed 只是历史反馈已满足，并非控制权已释放或持续监测健康的保证。
- 新增6项离线回归覆盖：成功姿态后真实旧卡通过生产SDK仲裁请求移动/姿态/停止、显式停止后释放及重新获权、生命周期停止、释放失败重试、停止异常、停止与完成竞争。全部59项通过；没有运行真机姿态测试。
- `release.260928.9ae4d5b` 是修复前 `2d74268` 的构建版本。此次修复需要新Bot构建与后续受控验收，不使用旧版本号代表修复版。

## 最新：实际 SDK 属性读取检查与 Bot 镜像证据

- 当前 C++ 源码已有 `py::class_<UDPState>` 和七个字段绑定，无需重复添加。Bot 提到的未绑定类型错误在 `3a92d79` 官方 ARM64 镜像中未复现。
- 新增 `check_sdk_binding.py`，构造仅使用 loopback 和临时本地端口的 UDP 实例，实际读取 `udp.udpState`，校验类型及七个初始整数计数。任何类型转换或字段读取异常均令构建失败；不调用 Send/Recv、不启动 SDK 循环。
- Dockerfile 用 `RUN --network=none python3 /work/check_sdk_binding.py` 替换只检查 `hasattr` 的弱验证。新增小型脚本，不新增依赖；网络隔离构建步骤需要 BuildKit。
- 新增 5 项回归：正常读取、属性存在但实例转换报错、缺失字段、计数异常、Docker 构建接入。与现有测试共 53 项通过。
- 新脚本已在现有官方 ARM64 镜像 `release.260928.6f8a95d` 中以 `--network none` 隔离运行通过；这不是本次新提交的完整镜像构建结果，新 Bot 构建仍需单独确认。

### 已验证版本：3a92d79 / release.260928.6f8a95d

镜像 ID：`sha256:31283aab27fccce748e32abb0e80fc3c467e7bc9fde759897dcaf17ae69fcb94`。

- 拉取 Bot Try it 指定镜像后，为保留原卡使用现场 `main.py/config.yaml/sensors.py/camera_snapshot.py` 兼容层，保留 27 卡。不是原样执行 Try it 脚本后的部署；新卡、SDK client/proxy、controllers 和 SDK 二进制来自官方镜像。
- 2026-09-28 动作前电量93%、最高电机温度47°C、遥测新鲜、静止站立。一次 `vx=0.1 m/s, duration=0.5 s` 返回 completed、ok=true、stop_confirmed=true，完整调用1.998秒。静止stop_move、非法参数拒绝和历史查询通过。
- command_id：`go1_loco_confirmed_d19ce61e79d445ee9d3b16e89a314cca`。运动结果样本vx=0.1575 m/s；停稳样本vx=0.0007、vy=0.0017 m/s、yaw=0.0019 rad/s。设定速度不是实测速度上限，0.5秒不代表整个停稳耗时。
- 现场用户确认前移后自行停稳，无外部介入。测试后恢复control_enabled=false，电量92%、最高温度48°C、遥测新鲜、静止站立。
- 画布测试由现场用户操作并提供返回JSON：move返回CONTROL_DISABLED（timestamp_ms=1790575990465），status返回idle/NO_ACTION/control_enabled=false（timestamp_ms=1790576301519）。证明画布手动调用及返回、禁用保护有效，不证明decision_core连线或画布运动测试通过。
- 开机后曾收发成功计数为0，重启唯一driver恢复；未根治启动遥测恢复问题。自动注册曾超时，Core日志亦有发现驱动及订阅主题记录，不据此推定所有平台链路通过。
- 本次提交只改构建检查、回归和文档，没有追加真机动作；原样Try it、长动作、姿态、平台中断和旧卡混用仍未完整验收。

以下为历史修订记录，部署状态以本节标注的版本为准。

## PR #348 生命周期与 DDS 审查修订

- `start` / `stop` 是框架生命周期入口，正常返回 `{"state":"ready"}` / `{"state":"idle"}`；默认禁用控制时也可调用，不读取遥测、不启用控制、不发送机器人指令。
- 启用控制时，生命周期 `stop` 关闭新动作入口，取消本卡活动任务并请求停止，不等待物理反馈；`idle` 只表示卡片生命周期状态，绝不是停稳证明。如果停止请求自身异常，返回 `state=idle` 并保留 `ok=false`、错误码与接管提示。
- 真机运动停止及确认只使用 `stop_move`，中断钩子仍映射到它。**迁移注意：`stop` 不再是 `stop_move` 的兼容别名。** `stop_move` 在生命周期 idle 时仍可用于停止确认，但仍受 control_enabled 开关约束。
- `start` 不清除旧任务取消信号或 SDK 停止锁存；旧任务退出前不能接收新动作。只有新的显式动作才可重新获得控制权。
- 删除 Dockerfile 中禁止设置的 `FASTDDS_BUILTIN_TRANSPORTS=DEFAULT`；保留 service.yml 里的 DDS profile 挂载及 `FASTRTPS_DEFAULT_PROFILES_FILE`，不新增包或基础镜像。这项源码修复不等同于已经验证发布镜像及现场 DDS 通信。
- 新增 7 项生命周期/配置回归，包含真实 bundle dispatch、禁用/启用、无遥测、停止后重启、异常和零速度路径；与日志及原回归组合 **48 项离线测试通过**。

当时的 Bot 构建 `release.260928.87f7036` 对应 `22ffc77`，不包含本节修复。后续生命周期修复已随 `3a92d79 / release.260928.6f8a95d` 测试，见文档顶部；文末早期硬件记录仍只对应 `84e5234`。

## PR #348 日志审查修订

针对 Bot 审查，本次修复 SDK spawn 子进程在导入 SDK 前调用 `logsafe.install(check_fd=False)`；UDP 循环与状态解析错误按路径限频，首次立即记录，此后每 5 秒最多一条（包括恢复日志）。错误消息变化或反复断连不会重置限频窗口。诊断计数、停止锁存和遥测新鲜度逻辑不因日志限频而跳过；日志保护只覆盖 Python 输出，不宣称拦截原生 SDK 的所有输出。

新增 `tests/test_go1_sdk_logging.py`，以假 SDK 验证真实 spawn worker 入口、持续错误、恢复、反复断连、诊断计数及解析失败不刷新遥测。`python3 -m unittest -q tests.test_go1_sdk_logging tests.test_go1_loco_control`：**41 项离线测试通过**。

以下真机证据对应修订前提交 `84e5234`，不能充作日志修订版的真机验证。日志修订提交 `22ffc77` 的 Bot 版本为 `release.260928.87f7036`；该日志修订版未真机部署。

基础设施修改仅为集成所必需：Dockerfile 增加 `loco_control.py` 复制并验证重编译绑定含 `udpState`；driver.yaml 增加新卡目录元数据。不新增 apt/pip 依赖、基础镜像或服务配置；会增加少量源码/绑定字节，不宣称镜像大小完全不变。本次日志修订不再修改这两个基础设施文件。

## 目的与功能

保留原 `loco`，新增独立 `loco_confirmed`，用于有人看护环境下的短距离位置调整、转向和动作中断。它不是新的机械运动原语，而是增加反馈、错误传播、时限和停止确认，让 Agent 区分“请求已接受”与“观察到动作结果”。

- `move`：有限时前后、横移、偏航及组合运动，检查请求轴的同向响应，到时请求停止，不自动延长或重试。
- `stop_move`：独立停止信号、旧命令代次失效、持续中性命令及停稳检查。
- `start/stop`：生命周期就绪/停止，返回 ready/idle，不代表真机完成动作或已停稳。
- 姿态：站起、趴下、平衡站立、恢复站立、阻尼；后两项要求 confirm=true，不作自动停止兜底。
- `status`：查询最近 32 条动作。参数 action_id 可填写返回的 action_id 或 command_id；重启不保留。
- 短移动（≤2 秒）同步返回结果；长移动和姿态先返回 accepted、executed=false、action_id，再通过 ACP 回报最终结果。回调失败保留状态，不盲目重发。

## 与原卡对比

| 维度 | 原 loco | 新 loco_confirmed |
|---|---|---|
| 定时移动返回 | 启动后台任务即返回 ok | 区分接收与最终结果；短动作等待反馈 |
| 运动确认 | 卡片不检查实际运动响应 | 检查新鲜遥测及请求轴的响应方向 |
| 运动停止返回 | 调用 stop_move 后返回 stopped | `stop_move` 用多个新样本证明停稳，否则明确报错；生命周期 `stop` 不作停稳证明 |
| 异常定位 | 参数校验及原有调用返回 | 遥测过期、发送失败、未观察到动作、停止未确认等 |
| 追踪 | 无本卡动作历史查询 | 动作编号、最近历史、异步 ACP 回报 |
| 冲突处理 | 原有控制逻辑 | SDK 所有权与停止代次；明确拒绝控制权后不误停原卡 |

原 controllers.py 与主仓库一致，新旧卡共享一个 SDK/UDP 客户端。底层代理/SDK有改动，因此保留原卡源码不等于底层行为完全不变。原返回缺口不证明历史未执行都由它造成；多次开机收包为零、重启原驱动后恢复是另一个问题，本卡不声称永久修复启动联网时序。

## 判据与安全边界

默认 enabled=true、control_enabled=false：可展示/查询，控制请求返回 CONTROL_DISABLED，不注册中断钩子，不在此模式生命周期中发停止。启用前确认看护、遥控器、平整场地、网线余量及单一控制源。

新卡占用 SDK 时其他卡的控制写入会被拒绝；停止锁存后需新的新卡动作解除。禁止新旧两套控制同时驱动，不能阻止外部电脑或遥控器控制。旧卡脚本/姿态混用需专项验证。

- accepted / ok=true / executed=false：仅接收。
- completed / ok=true：符合软件反馈判据；move 还需停稳确认。
- error 或 cancelled / ok=false：失败、无法确认或停止中断。
- STOP_UNCONFIRMED：无法证明停稳，需遥控器接管；不等于必然仍在运动。

遥测年龄 ≤0.5 秒且有独立新收包序号。停稳需停止请求后的至少 3 个不同样本，持续 ≥0.15 秒满足平面速度 ≤0.03 m/s、偏航 ≤0.08 rad/s，默认等待 3 秒。

速度范围：vx ±1 m/s、vy ±0.6 m/s、vyaw ±90°/s；非零线速度至少 0.05 m/s，非零偏航至少 6°/s。duration 省略或 0 为 0.5 秒，默认上限 10 秒。

这是依赖 SDK/UDP 的软件停止，不是硬件急停或精确位移控制。普通调用可能受平台 ACP barrier 影响；中断钩子声明不等于已验证平台端到端中断。

## 部署与测试

复用现场镜像增量集成，保留温度、motion_feedback、camera_snapshot 等全部 26 张原卡，新增后共 27 张；原工具定义逐项对比一致。原容器另存并停止，未并行启动第二个 SDK。

镜像：go1_bundle:loco-confirmed-test-20260924。
镜像 ID：sha256:fae932695d6b5ce27ab8c812a2f3b69e7780b8c54c357d0732f155497ad23572。它不是 PR Bot Version。本 PR 不混入现场其他分支的卡片源码。

| 项目 | 结果 |
|---|---|
| 离线回归 | python3 -m unittest -q tests.test_go1_loco_control，36 项通过；覆盖反馈/停止故障、取消、RPC/UDP、姿态判据、ACP、双卡注册和默认禁用 |
| ARM64 构建 | 新绑定暴露 udpState，导入及增量镜像构建通过 |
| 隔离注册 | 无网络、假客户端验证 27 张卡；原 controllers.py/sensors.py 哈希不变 |
| 真机部署 | 2026-09-28 加载成功，旧卡保留，遥测与温度持续可读 |
| 默认保护 | 实际 MCP move 调用返回 CONTROL_DISABLED |
| 静止停止 | completed、stop_confirmed=true |
| 参数拒绝 | vx=2 返回 INVALID_ARGUMENT，未执行越界运动 |
| 单次短移 | vx=0.1 m/s、duration=0.5 s，completed；观察到 mode=2、vx=0.1177 m/s，之后 stop_confirmed=true |
| 停稳样本 | vx=0.0053、vy=0.0068 m/s、yaw=0.0018 rad/s，sample_seq=4073 |
| 历史查询 | 按 command_id 查得一致的最终结果 |
| 现场确认 | 用户确认小范围前移并自行停稳，无遥控器或其他电脑介入 |

动作编号：go1_loco_confirmed_d5188bde17844ebba63554f78c41a675。测试前电量 86%、最高电机温度 45°C；临时限制最大移动时间 0.5 秒。

完整同步调用约 **1.97 秒**，包含运动、减速与停稳确认，不是停止请求到停稳的单独测量，也不意味着 0.5 秒内物理停稳。采样中存在速度峰值及横向偏移，不能宣称精确速度或精确 5 cm 位移。

只执行一次授权短移，没有自动重试。结束后恢复默认 control_enabled=false 并重新加载，新卡保留展示，未再发动作。原始 JSON 已保存。画布首页 HTTP 200，但未经登录的 /api/mcp 返回 401，不能据此宣称已完成画布端到端连线验收。

## 后续验收边界

本次以部署、短移和停止证据提交评审。尚未真机验证长移动 ACP 回报、全部姿态、运行中平台中断、旧卡脚本混用及物理停止距离/时延标定；这些不能算已通过。

创建 PR 后记录实际 PR 号及 PR Bot 表格 Version，不用镜像 ID、分支名或 Git SHA 代替。最终验收与合并由 Master/维护者决定。
