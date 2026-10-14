# 网页示教扩展

用户在 Phanthymotus 原有 `piper_teaching` 执行卡中选择「录制技能」，点击「执行」，在同一弹窗内完成 **设备检查 → 录制 → 试运行 → 保存**。不增加全局按钮；弹窗外壳复用 Core 现有样式，Driver 页面使用相同的暖白、陶土橙、细边框和控件圆角。当前交付的是**录制回放型技能**：不训练策略，不自动识别物体，不改变轨迹速度，不自动恢复起点。

首个生产适配器是 `agilex/piper/teaching.py`，基于 PR #395 的双臂控制器。`device.build_plugins` 仅在双臂配置下注册 `piper_teaching`；单臂卡片保持原样。

## 松灵测试的使用流程

1. 使用双臂 PiPER 配置启动 Driver，挂载持久化 `data_dir`，提供已真机核验的标定文件。完成原有 CAN、模式、电机、夹爪及运动许可流程。
2. 在画布连接 VR 输入卡到 `piper_teleop`，启动方案。再将 `piper_teaching` 卡接到决策核心的执行器端口。这条执行器连接只需设置一次；工作台会检查并提示缺失的连接。
3. 在 `piper_teaching` 卡中选择「录制技能」，点击原有的「执行」。先点击「检查设备」，逐项检查双臂连接、标定绑定、电机/夹爪/反馈、运动许可、控制占用和 VR 通道。未通过时显示处理建议并禁用下一步，原始原因折叠在检查详情。打开页面只读进度；主动检查可建立只读 SDK 连接，但不会使能、切模式或移动机器人。
4. 检查通过后进入录制；「进入遥操」会重新检查条件并显示接管确认。DDS 连线成功不代表头显正在跟踪。必须收到有效且新鲜的实时输入、松开双手握键建立映射，页面和 Driver 才允许开始录制。按住双手握键示范，结束后等待录制完整落盘和遥操终态。页面的录制计时来自服务端开始时间，录完后使用文件实际时长。
5. 试运行页显示录制完整性与起点状态，关节曲线和起点差异默认收进「动作详情」。恢复物体布置，通过原有控制卡手动调整到起点，再检查并确认本次试运行。沿用 #395 的起点阈值：关节 ±0.05°、夹爪 ±0.5 mm。执行时驱动仍重新校验，页面的绿灯不代替执行条件。
6. 等待实际回放完成，观察真机效果，勾选效果确认，只需填写名称和使用条件后保存。描述和调用说法默认采用名称，无需重复填写或登录资源中心。
7. 在对话中调用技能。每次均先预览，确认此次活动空间和接管，再执行；以 ACP 的相同 `action_id` 完成事件或 `info.last_run` 终态为准。

本地软件验证使用模拟 SDK 和模拟输入，不证明现场的头显跟踪、CAN 时序、标定或抓取效果。现场先做无物体的小幅动作，再验收固定物体位置的任务；检查停止、输入断流、起点不符和重启后的技能调用。

## 扩展边界

- `common/teaching/service.py`：草稿、发布条件、资产持久化、技能定义。
- `common/teaching/workbench.html`：自包含页面，通过 `motus.workbench/1` 与 Core 沙箱容器通信。
- 机器人适配器：设备检查、遥操会话、录制和回放、实际完成结果。高频控制继续属于原有机器人控制器。
- Phanthymotus：发现 `inputSchema.x-workbench`，装载页面、转发当前工具的允许操作、导入已有技能格式。Agent Loop、技能激活与 ACP 无需改动。

页面通过 MCP `ui` 获取，不需要额外的 Web 服务、端口、前端构建或 CDN。`common/` 已在 PiPER 镜像的复制范围内。

## 其他机器人需要实现的适配接口

目前共享页面面向双臂 VR 示教；天弈或 G1 的第一阶段可适配相同的双臂数据形状。若要录制底盘、躯干或更多关节，需要扩展页面的数据可视化，不能直接声称已有全身适配。

| 接口 | 责任与返回值 |
| --- | --- |
| `title` | 页面机器人名称 |
| `info()` | 无连接/运动副作用；含 `state`、`busy`、`session_id`、`recording`、完整录制列表、`input_fresh/mapped/can_record` 与尚未执行的检查项声明 |
| `check()` | 显式读取设备条件；返回 `ready` 与 `checks[{id,label,ok,reason?,help?}]`；逐项提供结果与处理建议，不能把通道连通当成实时跟踪；不得自行使能、切模式 |
| `begin(args)` | `execute=false` 预览；执行需短时、单次、绑定当前状态的 `preview_id` 和本次确认 |
| `record()` / `finish()` | Driver 也须检查实时输入和映射，不能只禁用前端按钮；录制实际下发目标和反馈，结束必须区分完整文件、仍在落盘、失败，并请求停止遥操 |
| `inspect(filename)` | 校验完整性、校验和与当前设备绑定；返回 `filename/sha256/binding/duration_seconds/frames/preview` |
| `start_difference(filename)` | 返回起点目标、当前差异和容差；仅报告，不自动回起点 |
| `play(filename,args)` | 复用原控制器的预览、互斥锁、限速、工作空间、跟踪/反馈检查；返回异步 `action_id` |
| `wait(action_id)` | 等到控制器的真实终态；不能用录制时长推算成功，取消和异常不能返回 completed |
| `stop()` | 随时可用，包括文件验证期间；请求停止后仍需要读取终态 |

`play` 的底层控制器负责发送 ACP，ID 必须与返回给调用方的 ID 相同。PiPER 继续由 `PairedMotion` 发送一次完成事件，扩展不重复发送。`TeachingPlugin` 仅观察该动作结果并记录试运行/运行状态。

发布必须同时满足：完整录制、当前绑定一致、同一校验值的试运行成功、用户确认效果、名称/使用条件等字段完整。发布确认不授权未来运动。页面的成功状态和技能文字都不能替代驱动校验。

## 持久化与恢复

原始文件保持在 `data_dir/recordings/`，资产定义原子保存到 `data_dir/skills/<uuid>.json`。资产绑定原始文件名、SHA-256、模型/标定/左右 CAN 绑定和成功试运行结果；每次运行重新校验。备份与迁移必须包含二者及对应的标定。换设备/标定应重新示教验证。

Core 中只保存标准技能定义，资产不复制进 Core 数据库，也不自动发布到资源中心。卸载本地技能不删除 Driver 资产。当前只有一个活动草稿；页面刷新恢复草稿，Driver 重启后须载入完整录制重新试运行。未完成文件不能导入。

保存直接写入现有 `skills.installed`，新技能 `active=true`。下一次对话构建提示词时即可发现，随后由既有 `activate_skill` 加载完整指令，调用已连接的 `piper_teaching.run`。无需重启、刷新页面或手工导入文件。执行卡连接只配置一次，后续技能沿用；未连接时页面会明确提示，保存本身不增加执行权限。

保存到 Driver 后若 Core 导入失败，可保持相同表单重试：两边均幂等。同一个已保存草稿不允许覆盖元数据，已安装同 slug 的不同定义返回冲突。

## 验证

```sh
uv run --no-project --with pytest --with numpy --with pyyaml python -m pytest -q \
  tests/test_piper_teaching.py tests/test_piper_dual.py tests/test_piper_trajectory.py tests/test_teleop_contract.py
```

跨仓浏览器测试：准备 Playwright Chromium，并设置 `TEACHING_CORE_ROOT` 指向带工作台宿主的 Phanthymotus checkout。测试会启动临时 Core API、MCP HTTP Server、SQLite 和模拟 SDK，不连接硬件。

```sh
TEACHING_CORE_ROOT=/path/to/phanthymotus uv run --no-project \
  --with pytest --with numpy --with pyyaml --with playwright --with uvicorn \
  --with fastapi --with python-multipart --with httpx --with aiohttp --with aiofiles \
  --with openai --with html2text --with jsonschema \
  python -m pytest -q tests/test_teaching_browser.py
```

可用 `PLAYWRIGHT_CHROMIUM_EXECUTABLE` 指定现有 Chromium，可用 `TEACHING_SCREENSHOT` 指定浏览器截图输出路径。

可用 `TEACHING_SCREENSHOT_DIR` 保存各步骤及阻断状态的真实页面截图。浏览器验收覆盖卡片原有入口、许可未通过、检查后撤销许可、起点不符、默认收起动作曲线、试运行效果确认、导入失败重试、现有提示词发现/激活及 MCP 回放调用。沙箱隔离、窄屏布局也纳入验证。
