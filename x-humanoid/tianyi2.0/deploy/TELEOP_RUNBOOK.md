# 天轶遥操候选：构建与部署准备

首测 Live 已配置，首次试验未见有效跟随并锁存反馈故障；尚未真机跟随验收。切换前仍需真实停止确认和释放执行权，不能通过重启清除锁存。以下通用构建示例保留初始候选标签，实际更新版本以实施计划的日期记录为准。

## 机上构建

只上传两个候选仓的源码；不传输本地 Docker 镜像。先重新确认天轶身份、`/data/hanzebei` 的属主与挂载、Docker 数据根和资源余量。建议本事项源码放在 `/data/hanzebei/pico-tianyi/src/driver`、`src/core`，缓存和日志放在同事项目录。上传前查看 rsync dry-run，排除 `.git`、凭据及本地虚拟环境；不使用 --delete。

源码落位后，从事项目录执行的构建命令如下。这些路径是待准备的部署布局，不表示已经在机器上创建。

```sh
cd /data/hanzebei/pico-tianyi
mkdir -p build-tmp evidence
export TMPDIR="$PWD/build-tmp"
bash src/driver/x-humanoid/tianyi2.0/deploy/build_teleop.sh \
  local/phanthy-motus/tianyi-driver:teleop-20260920-r2 > evidence/driver-build.log 2>&1
bash src/core/deploy/build_tianyi_actucore.sh \
  local/phanthy-motus/actucore:tianyi-execution-20260920-r1 > evidence/actucore-build.log 2>&1
```

两个脚本只构建，不启动服务、不推送。Driver 临时构建上下文复用 `common` 和仓内 `robotera/q5_bundle/vendor/audio_msgs`，并编译厂商消息；缺失依赖或主入口导入失败即构建失败。公共 apt 源选择仅作用于构建容器，保留签名验证。机上重建镜像 ID 可能不同，应记录实际 ID 和源码哈希。

## 隔离软件验证

从候选 Core 仓根执行：

```sh
bash deploy/test_tianyi_teleop_isolated.sh ../driver \
  local/phanthy-motus/actucore:tianyi-execution-20260920-r1
```

使用真实 ROS2/MCP、镜像中的 ActuCore 和只读 Driver 源码，模拟模型及本体。Docker network none，无设备和凭据。PASS 必须包含 hardware_writes=0 和 stop_confirmed=true。它不证明 PICO、现场模型或真实跟随效果。

Driver 镜像检查脚本为 `tests/image_shadow_smoke.py`，需在 network none、只读根、可写临时 /tmp 的容器内，加载 `/opt/ros/humble/setup.bash` 和 `/tianyi_ws/install/setup.bash` 后运行。实际加载双臂、双手和遥操插件，两轮启停均不得产生硬件发布器或取得租约。

## 切换前必须生成实际差异

每次切换前重新读取当前 Compose、服务镜像和挂载，确认无人占用及无任务/租约，再生成只影响目标服务的差异、备份与回滚命令。既有 Shadow 部署不能替代下一次的预检。

- Driver 保留现有所有插件配置，仅增加顶层 `teleop.enabled: true`、`live_enabled: false` 和只读标定路径；不替换整份正式配置。
- ActuCore 使用随仓 `config.example.yaml`，明确 `robot_profile=tianyi2`、`mapping_version=relative_v1`、`mode=shadow`，namespace 与 Driver 一致；MCP 15740、WSS 15741 先检查冲突。
- 两服务同机运行，共用单调时钟。连续目标 domain 42 限本机；Driver 厂商 domain 0 仍使用现有已核验网络配置，不替换为回环 profile。独立总线子进程负责回环隔离。
- 配对/WSS/RTC 由 ActuCore 卡片托管，不另建框架外服务。证书、配对状态、管理密钥只放个人目录并按最小权限挂载。Agent Core 与 ActuCore 使用同一管理密钥文件；Core 的 `TELEOP_MANAGEMENT_URL` 明确指向同机回环 `/mcp`。Agent Core 接入补丁需单独构建评审，不能假定 ActuCore 镜像自动更新 Core。
- Core 候选已迁入原生 APK 和 Canvas 专用连接面板；原生 V040 已安装并从实机截图确认绿色双臂/白色躯干，服务端显示镜像已切换，Core 面板 overlay 仍待独立部署。现场部署必须核对既有 PICO 客户端协议及中文配对入口，补齐 UI 后再做用户流程验收，不能把 MCP 可调用当作界面已完成。

Shadow 回查：实际镜像、MCP 工具列表、十四关节反馈、PICO 连接、IK 方向；Driver `ownership_held=false`、`output_active=false`，遥操未创建硬件发布器。应用启动不自动接管。

## Live 前置项与停止

模型顺序/方向、手掌变换、躯干固定关节、碰撞尺寸、手部张开闭合范围需现场记录。按用户决定，厂商电源／急停与停止契约资料缺失不再阻塞接管；如实保留 subscription_arrival_only，不声称已验证硬件采样新鲜度。不能把模板 null 替换成猜测值，不能伪填 acceptance。运行时急停、上电、关节故障和反馈超时检查仍执行。

双臂-only 阶段使用同一标定哈希及显式 `hands_enabled: false`，手部开合端点可留空，Driver 不发布手部命令。ActuCore 胸部相对 IK 不要求腿部模型角度，但 Driver 固定体位反馈仍保留。Shadow 预览配置的 workspace/controller 约定不能直接标为 Live 已验收。

普通 Live 仍要求停止/断流及 Driver 崩溃保持证据。首次验收候选增加显式 prepare_first_acceptance 和 60 秒进程内窗口，完整配置与独立证据字段见 [执行契约](../TELEOP.md)。本地测试、机上 ARM64 构建和隔离网络的实际 bundle Shadow 启停已通过，r2 已部署且首测 Live 配置已就绪、idle/无执行权，未实物验收；现场前置证据缺失时仍禁止接管，不能将 stop_verified 等标志设真以绕过前置条件。

暂停应看到新反馈确认 hold_confirmed；显式 stop/release 后确认 ownership_held=false 才可切换版本。停止未确认时保留日志和控制权锁存，先现场确认并处理，不自动重启清故障。未接管的 Shadow 才能按已核验原镜像和备份配置回退；其他服务保持原状。


## 2026-09-21 Yoga 跳转构建记录

沪京直连异常时，本轮经 Yoga SSH -W 转发恢复访问，未更改全局网络/SSH 配置。updates/latest-input 中候选 latest-input-20260921-r1 已机上构建并通过禁用网络、只读根的 socket smoke；未部署。当前 Driver 经此前重启为 idle/无执行权，ActuCore 仍保留 fault；须核对新 boot 与旧会话再恢复，不能把 idle 当作物理停止验收。完整镜像 ID 和证据见仓根实施计划。


候选切换脚本位于 `/data/hanzebei/pico-tianyi/updates/latest-input/switch.py`。无参数只做预检及生成 before/preview；`--apply` 更新 Driver 镜像并重启无执行权的 ActuCore 卡片；`--rollback` 在相同空闲预检下恢复备份镜像。均不调用遥操 prepare/claim/start。sudo 认证由现场终端完成，回读 SWITCH PASS 后仍需核对镜像和无执行权状态。完整 ROS/MCP 隔离链路已通过，证据 ros-e2e.log；实际动作验收未通过。


同日后续：用户已完成切换，实际 Driver 为 latest-input-20260921-r1，卡片已重建并标定；只读回查双方 idle、无执行权、Driver 无输出、PICO 已连接。以上“未部署”为构建时历史状态。最新 Canvas 出现编辑锁，未重复切换或启动动作。Mac 的 tianyi-bj 别名当前不可解析；临时采用已核验的显式目标地址经 Yoga 跳转，不修改用户 SSH 配置。现场命令应从当前设备记录核验完整入口，勿直接重用失效别名。后续编辑锁释放后，日志改进已备份并同步现场 trial.py，机上隔离验证通过；未启动动作。现场运行 --start 后会分别显示 card_state/card_reason 和 driver_state/driver_reason，以及会话和序号。详见实施计划。

启动脚本的零输出标定若遇反馈暂缺/过期，最多重试三次，每次仍核验空闲及原始新鲜度；持续失败则退出并显示具体错误。prepare/start 不重试，不能靠重复启动绕过执行故障。

现场 trial.py 的 Core/MCP 查询明确直连 127.0.0.1，不使用环境代理；Core 只读预检遇拒绝连接最多等待 5 秒，持续失败显示目标地址。停止及运动写操作不重试。该处理不等于已确认拒绝连接根因。

现场可先运行同目录 preflight.py，一次查看只读检查的失败清单；不会启动或准备遥操。fixed_body 不匹配时先读取头/腰/腿实测与标定差值，不能只凭报错重写基准。当前已确认头部俯仰偏移约21°，重新标定须确认现场姿态；头显断开须恢复连接。

用户已要求每次按实际姿态建立会话基准；session-baseline 候选因此从新鲜静止机器人反馈采集头/腰/腿位置，暂停恢复期间不重设，VR 放置姿态不参与。旧 fixed_motor_positions_rad 留作历史来源，不需每次重写文件。session-baseline 已由用户完成应用，现场试验使用新基准；启动动作仍是独立步骤。

按现场用户既有授权，session-baseline/switch.py --apply 可在确认遥操无执行权、Canvas 无编辑锁时调用 stop-project，读回停止后才更新镜像；有锁或控制占用仍拒绝。无参数只读预检、--rollback 不自动停止运行项目。此入口不 prepare/claim/start 遥操。


此前阶段（2026-09-21）：先验收 PICO Shadow IK 显示，再接真机跟随。本地 command_expired 恢复及 0.3.11 显示候选未部署，不运行切换或启动命令来替代离线验证。现有 power_ns_stale 故障及租约不可凭显示正常自动清除。

后续状态：用户已确认 0.3.13/continuous 的 PICO 显示基本正常；12:41:24Z Live 记录在约 8.1 秒出现 command_timeout，随后 fixed_ns_stale 锁存，未见有效跟随。现准备短暂超时续接候选，详见 TELEOP.md；运行版本尚未应用本次修改。目标 TTL 仍不超过 100 ms，命令续接窗口和反馈持续过期门槛默认各 300 ms，可分别通过 teleop.continuation_timeout_ms / feedback_fault_timeout_ms 配置。

下一次部署须先明确停止并释放现有租约，不能重启清除锁存。两端更新后先核对 timing_policy 和 continuation_allowed/ready；旧 Driver 不提供这些字段时不会自动续接。采集工具除现有序号、关节和原始 last_failure 外，还需保留完整 driver.diagnostics、driver.watchdog_timing、adapter.diagnostics.last_send，才能分析后续瞬时反馈故障。使用当前部署实际镜像生成切换差异，以上历史命令及镜像标签不作为新候选发布命令。

### continuation 两服务更新：Agent Core 保持原状

2026-09-21 用户明确允许更新 ActuCore 和 Driver，完全不动 Agent Core。新镜像已在机上构建、核对源码、通过隔离 ROS2/MCP 续接测试并完成运行切换。当前入口为 `/data/hanzebei/pico-tianyi/updates/continuation/`，具体固定镜像 ID 见仓根计划。

1. 用户在 Canvas 停止项目；新脚本只查询 Core，不调用 stop-project，不修改用户的 Core 源码、配置或服务。
2. 操作者执行 `python3 /data/hanzebei/pico-tianyi/updates/continuation/live-trial.py --stop`，须返回 `STOP CONFIRMED`；旧租约/停止故障未解除不得切换。
3. `sudo python3 /data/hanzebei/pico-tianyi/updates/continuation/switch.py --apply`：再次检查空闲，仅替换 Compose 两项 image 和独立 ActuCore 配置的 Shadow 模式，`--no-deps` 依次更新 Driver、ActuCore。核验两端无执行权、300 ms 策略、反馈新鲜、Agent Core/Perception ID 与启动时间不变后返回 `SHADOW SWITCH PASS`。不 prepare/claim/start。
4. 读回实际部署及零输出标定，再由现场操作者显式执行同目录 `live-trial.py --start`。不带参数仅预检；`--start` 保留既有 60 秒首测窗口，证据写入 continuation 目录；默认记录原始输入、首个故障及分段时序，结束后 `--stop` 显式释放。
5. 需要回退时先停止释放并确认空闲，sudo 运行同一 `switch.py --rollback`；恢复固定旧两镜像且保持 Shadow。任何健康/占用/漂移检查失败均保留现场，不自动回退或重启清故障。

用户已完成停止及 sudo 认证，实际镜像、300 ms 策略、Shadow idle/无执行权和 Core 未重启均已回查。零输出标定通过，新入口无参数 PREFLIGHT PASS；此时 PICO 未连接，等待恢复 USB 与采集页，没有新的物理验收结果。此前旧入口自动停止 Canvas 的行为不适用于这次限定范围，新入口的 Core 写 API 已禁用。

### startup 首次接管补丁（已部署，后续恢复试验失败）

PICO随后已在线，USB仅用于调试，不作为遥操前提。13:40:49Z实体试验在ActuCore首次接管帧超时，Driver未收运动目标，控制权仍保留。新补丁只更新ActuCore，在零输出标定阶段准备本机DDS，并去掉接管帧无用IK；不重启Driver/Core/Perception。机上镜像tianyi-startup-20260921-r1已构建、校验并通过完整调度的隔离ROS测试。

先由操作者执行 `python3 /data/hanzebei/pico-tianyi/updates/startup/live-trial.py --stop`，确认释放后sudo运行同目录 `switch.py --apply`，预期 `SHADOW SWITCH PASS`。无参数仅预检；同脚本 `--rollback` 在空闲时恢复continuation的ActuCore镜像且保持Shadow。回查实际镜像、零输出标定及服务端PICO连接后，用户再执行同目录 `live-trial.py --start`。原始停止/故障状态不通过重启清除；最新物理跟随尚未通过。

用户已完成上述切换，21:59:27 ActuCore为3b7f7b85、Driver未重启，PICO在线。22:06 trial执行到序号3后，碰撞自动恢复的管理回复超时引发invalid_lease；最终Driver idle/stop_confirmed，无执行权，卡片锁存。正在准备updates/recovery的两端配套候选，尚未切换，不重复执行旧switch.py --apply。

恢复候选部署后，正常碰撞/IK恢复和丢失管理回复由卡片自动续接；已锁存故障由操作者调用现有teleop resume。该入口必须先确认Driver停止并释放、反馈新鲜，再重新建立卡片会话并标定，不重启容器，不改变Agent Core。PICO需经过新会话的松开/握持输入；若首次验收60秒窗口已到期，仍需现场显式重新prepare，不能由自动恢复延长。部署前后继续核对Core容器/镜像/启动时间和Compose非目标字段，不能用旧Core ID覆盖用户新版本。

recovery r2两镜像已构建、核对源码及通过隔离ROS恢复。当前部署入口 `/data/hanzebei/pico-tianyi/updates/recovery/switch.py`：无参数预检；`sudo python3 .../switch.py --apply`只切ActuCore/Driver并保持Shadow；空闲时--rollback恢复startup ActuCore和continuation Driver。Core写API仍禁用。2026-09-21 22:53用户已完成r2切换；实际镜像和Core保持不变已回查。当前为新Driver idle、无执行权/输出，Shadow零输出标定通过，PICO在线。不要重复执行--apply。

切换读回后，现场执行同目录 `live-trial.py --start`；默认仍只读。若卡片已锁存但Driver已确认释放，显式--start重新prepare首次验收窗口并使用新resume恢复；其余正常启动复用原入口。证据包含stop_confirmed及原始错误/分段时间；记录结束不自动重启或续开60秒窗口。

### 录制回放自动验收（2026-09-21）

用户随后明确授权双臂自动回放、失败修复后重测，无需逐轮操作PICO/确认；并选择一次性受限免密发布入口。新包为个人目录updates/replay，既有Core完全保留。一次性安装源码在Core仓deploy/install_tianyi_teleop_publish.py；用户用sudo安装，经哈希验证的固定发布程序位于/usr/local/sbin/tianyi-teleop-publish，对应sudo规则仅允许该程序；root备份/基线在/var/lib/phanthy-teleop-publish。安装不切服务、不启动动作。

后续`sudo -n /usr/local/sbin/tianyi-teleop-publish check|apply ACTUCORE_DIGEST DRIVER_DIGEST`检查/切换两镜像；`rollback`只在空闲时恢复备份镜像。任何非镜像Compose修改、Core占用、控制权/停止未确认都拒绝，不修改Core项目状态。安装和机上构建不等于真机验收。

ActuCore机内`python3 -m plugins.teleop.acceptance --config /work/config.yaml record`在首次有效双握把输入后录制10秒Shadow（之前最多等待120秒）；analyze/run/report/recover使用--recording ID。run包含三轮独立关节回放及一次完整生产运行时组合回放；保持每轮有限时长，失败后留存首因，由代理修改和回归后再试。异常退出的私有.lease.json必须先recover确认释放，不能靠重启消除所有权。报告记录真实q和厂商下发；只有物理反馈满足标准才算通过。
