# system_health 本地开发与提交准备

日期：2026-10-15（初版 2026-10-12，现增加 1 秒持续采样）。开发分支：`feat/u1-pro-system-health`。
本记录仅证明本地代码与模拟输入的行为，不证明 U1 Pro 真机健康检查有效。

## 规范核对

- 搜索了工作区、两个仓库及适用父目录，没有发现适用的 AGENTS.md。
- 已阅读驱动仓库和平台仓库 CONTRIBUTING.md，以及驱动 README_dev.md 的工具、
  生命周期、普通字典返回、配置、注册、构建、部署规范。
- 本地唯一实习生手册为工作区 `实习生入职 landing 文档 (1).pdf`；文档无明确版本号，
  文件日期为 2026-09-30。无法确认它是否等同公司线上最新版，按当前本地可用版本核对。
  重点读取第 3 节卡片开发/PR/镜像/部署测试流程及“任务提交流程”。未保存手册中的账号信息。
- 手册要求在 fork 的功能分支开发；现有工作区已创建本地功能分支；提交时核对用户 fork 与上游目标。PR 上评论 `/request_bot_review` 获得镜像，真机部署后
  在画布测试，联系 Master 和验收负责人，合并后提供验收/合并截图与提交信息。
- CONTRIBUTING 要求最小依赖、PEP 8、并发保护、声明敏感配置、普通 dict 返回，以及
  运行 Agent Core 的本地联调。新增模块只用标准库，没有敏感配置字段或执行类接口。
  相机快照复用帧条件锁，播放快照复用队列锁，健康卡片启停标记单独加锁；采样不持有生命周期锁。
- README_dev 的类型表仍描述 sensor 为流式卡片，但当前平台 CONTRIBUTING、
  `agent-core/src/mcp_client.py::present_to_llm/with_parallel_param` 和
  `agent-core/src/event/llm.py` 支持 sensor 查询并行、无动作 barrier。因此本卡声明 sensor，
  不沿用 Go1 健康卡的 actuator 类型。平台源码无修改。
- 参考 Go1 `SysHealthPlugin` 的一次查询汇总、逐项错误隔离与生命周期设计。
  未使用 MQTT、BMS、HighState、运动进程、vcgencmd 或 Go1 阈值。

## 本地验证结果

- U1 驱动测试：**102 tests，OK**（更新到最新上游后：72 个原有测试 + 30 个健康测试）。
- 镜像 COPY 检查：**2 passed**（只选择 U1 Pro）。
- 部署配置检查：**1 checked, 0 failed, 1 known gap**。
  U1 已登记为 CycloneDDS 例外；FastDDS fleet profile 对该驱动无效，应保留已有
  CYCLONEDDS_URI 策略。GAP 不是实际 DDS 联通成功。
- 源码编译、构建脚本语法与 `git diff --check`：通过。
- 开发电脑真实只读采样：磁盘可读；CPU、内存、cgroup、温度因 macOS 缺少 Linux 来源
  标记 unreadable，不输出“全部正常”。没有运行驱动主入口。
- 新增测试包含：正常观测汇总、指标缺失/损坏/权限失败、负载/内存/磁盘/温度策略、
  cgroup v1/v2 与上级使用率、隐藏层级未知、可见无限制、thermal/hwmon 标签、
  容器来源区分、相机未启动/有效首帧/重启会话/过期/异常及最近错误、共享头部动作
  播放任务过期和匹配失败结果、快照失败、启停幂等、并发采样期间停止、配置校验、
  注册启用/禁用、普通字典与 MCP 单次包装。
- MCP 初始化、tools/list、tools/call 使用现有 HTTP handler 的内存请求测试，
  **不是运行中的 Agent Core 或真实网络联调**。健康测试只构造现有对象、合成文件与消息；
  断言节点 mock 零调用、无相机资源创建、健康采样启动时只有一个线程、停止后无遗留线程。
  原有机器人接口回归测试同样使用 ROS/厂商替身，未连接机器人。

复现命令（测试 Python 需具备已有 PyYAML、NumPy、Pillow；Pillow 由镜像 apt 层提供）：

```bash
PYTHONPATH=ubtrobot/u1_pro python3 -m unittest discover -s ubtrobot/u1_pro -p 'test_*.py' -v
python3 -m pytest tests/test_dockerfile_copies.py -k u1_pro -q
python3 scripts/check_service_yml.py ubtrobot/u1_pro
PYTHONPYCACHEPREFIX=/tmp/u1-health-pycache python3 -m compileall -q ubtrobot/u1_pro
bash -n build.sh
git diff --check
```

本次测试使用临时目录 PyYAML 与 bundled Python，不新增运行依赖。
初次测试环境缺少 PyYAML/Pillow/NumPy，补齐可用本地依赖后上述测试全部通过。

## 1 秒持续监测更新

`start` 立即调度首轮采样，随后每 1 秒自动采样，查询只读缓存，不依赖查询维持监测。
重复启动保持同一个线程；`stop` 通过 Event 立即中断等待并 join，正常情况下回收线程。
若采样底层读取阻塞超过 2 秒，返回 stopping 而不是声称清理完成，保留引用防止重复线程；
旧线程释放后退出，不发布已取消的结果。重启清空旧缓存与计数。
缓存缺失、停止或超过两个间隔未更新时均不能表示全部正常。

本次额外验证真实 1 秒周期连续运行无需查询、查询不触发采样、缓存副本隔离、样本过期、
失败后下周期恢复、重复启动、等待期间快速停止、阻塞采样期间停止与拒绝重启、
阻塞超时如实报告、重启清空缓存、线程启动失败回滚。全部使用合成指标与现有驱动对象替身。
README、config.yaml 同步了采样策略；原 Dockerfile 已包含该模块，本轮不需要新增依赖或挂载。

## 构建准备及剩余验证

Dockerfile 已复制新模块，driver.yaml 声明 system_health sensor，config.yaml 添加独立启用项
及观察策略，deploy/service.yml 增加明确的容器指标来源标记，README 同步接口与边界。
主入口继续复用 build_plugins 和 vendor_runtime 的注册/心跳，无新增平台注册端点。
requirements.txt 无需变化，新增代码不引入第三方库。

Docker daemon 当前未运行，**ARM64 镜像尚未实际构建**。未启动 Docker 或任何服务。
统一 build.sh 会读取 registry 配置并可能推送镜像，未执行它。
Docker 可用后，可暂存驱动目录与 driver.yaml 的 `../../common` 到临时 context，
用 `docker buildx build --platform linux/arm64 --load` 构建本地标签；不得带 `--push`。

运行真实驱动会进行厂商授权检查并调整固件行为开关，也可能启动已有设备卡片；
这不符合本次只读验证限制，因此没有启动 main.py 或现有服务来做 Agent Core 联调。
缺少本地 ROS2/实际 U1 环境的情况下，真实 Agent Core 注册、画布发现与查询仍待验证。

真机验收需在后续授权后确认：

1. 实际 ARM64/ROS Humble 镜像运行与现有卡片兼容；注册后健康卡片独立可见。
2. 宿主机与容器的 `/proc`、cgroup namespace/层级和文件系统 mount 实际映射。
   v1 的上级计费策略和容器隐藏父级有效限制均保留未知，不将可见值称作最终限额。
3. sysfs 温度读权限、各 zone/hwmon 与硬件的实际对应，以及适用告警策略。
4. 利用已有相机运行/停止状态和已存在播放任务，核对首帧、持续帧、过期、错误与完成。
   验证健康查询本身没有 SDK 调用、播放、录音、动作、相机启动或开关变化。
   若需另行启动设备来制造测试状态，必须先取得对应机器人行为授权。
5. 把采样与计算板本地读数对照；确认单位、容量和可见限制。完成这些前不称为真机有效。

## PR 提交流程与验收边界

用户已授权 commit、push、创建 PR，并只通过 `/request_bot_review` 请求评审；
根据 issues 和 suggestions 修改后，继续推送同一分支、更新同一 PR 并重新请求评审。
远程 bot 构建结果在 PR 中核对；部署、重启服务、机器人控制、合并和联系验收人员仍未授权。

后续按手册：确认 fork/分支目标 → commit/push → PR → `/request_bot_review` →
镜像构建 → 获得部署及行为验证授权 → 真机画布验收 → Master/负责人验收 → 合并 →
保存有效截图并提交平台申请。当前无验收截图或提交序号，不编造申请记录。

可用的申请文案：卡片名称 system_health；所属机器人 UBTECH U1 Pro；
功能简述“只读检查计算板资源与可见温度，汇总已有相机和播放状态，保留未知及数据来源”。
