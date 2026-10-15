# 通用网页示教模块

本模块承载草稿、试运行、效果确认、资产持久化、普通 Skill 定义及四步网页流程；运动和设备状态仍由各机器人原控制器负责。模块来源为 [PiPER 示教 PR #402](https://github.com/4paradigm/phanthymotus-driver/pull/402) 的公共部分。当前 G1 分支只复用公共模块，不包含 PiPER 适配器、驱动改动、测试或 #395 的提交历史。

共享路径与 #402 重叠；合并前需根据先后顺序重放公共模块改动并验证两边适配器，不应把公共模块当作两套独立实现。已通过交叉检查确认原 PiPER #402 缺少新接口 `presentation` / `replay_instruction`，不能直接套用该公共模块；需要在独立 PiPER 适配中补齐并重新回归。G1 的适配和验收属于独立 PR。网页宿主和本地 Skill 导入依赖 [Core #318](https://github.com/4paradigm/phanthymotus/pull/318)，Core 中的新增逻辑保持通用，含每动作必填参数、型号元数据及动作包传输。

## 宇树 G1 的使用流程

1. 部署包含 `g1_teaching` 的 G1 Driver 和配套 Core 工作台。保留原部署的持久化数据目录及 `/run/g1-machine-id` 挂载。
2. 画布连接 PICO 输入到 `teleop_control` 并启动项目，将 `g1_teaching` 执行卡连接到决策核心。在该卡选择「录制技能」并点击原有「执行」，页面操作固定转发到当前卡片的 MCP 服务。
3. 点击「检查设备」，查看本体反馈与模式、机型与设备身份、控制状态、VR 通道四项结果。检查不使能、不切模式、不申请运动权。
4. 人和机器人双臂下垂，松开握把，长按 X+A 至少一秒建立原遥操基准。页面复用原会话；实时输入和映射有效后才能录制。按住双握把示范，松握暂停，记录成功下发的关节参考、反馈及暂停时间。
5. 结束录制并等待完整文件和原控制器停止交权。页面显示 G1_23 的双臂 10 关节、固定假手，不显示夹爪。恢复物体布置、检查起点差异并确认本次路径和接管后试运行。原控制器以 0.1 rad/s 参考变化率先移到起点，再按录制时间轴回放，不重新运行 IK。
6. 等待相同 `action_id` 的真实完成回执，观察动作效果，填写名称和使用条件后保存。`completed` 表示轨迹发送和停止交权完成，不保证物体操作成功。
7. 普通 Skill 加入当前 Core 的 `skills.installed` 并启用；下一次对话可发现，经已有 `activate_skill` 加载后调用 `g1_teaching.run`。每次执行仍要新预览和确认。回放不依赖 PICO 跟踪，未连接 VR 时也可载入已有完整录制重新试运行。

实际网页截图见 [G1 界面验收](../../docs/teaching/g1.md)。

## 适配接口

| 接口 | 责任 |
| --- | --- |
| `title`、`presentation` | 名称、关节分组、夹爪能力、起点处理提示、会话由页面或原控制器建立 |
| `resources`、`completion_timeout` | 声明原控制资源和真实终态等待上限 |
| `info()` | 无运动副作用；返回 busy、session_id、实时输入、映射、can_record、录制及检查项 |
| `check()` | 逐项读取设备条件、原因和处理建议，不自行使能或切模式 |
| `begin(args)` | 页面建立会话时须预览与确认；控制器已建立会话时隐藏入口 |
| `record()`、`finish()` | 录制时重新检查原会话和有效输入，结束区分完整文件、落盘中、失败并请求原控制器停止 |
| `inspect(filename)` | 重新校验文件完整性、SHA 和设备绑定，提供时长、帧数及有限预览 |
| `start_difference(filename)` | 只读差异及机器人真实的起点处理方式，不能把不同机型的阈值通用化 |
| `play(filename,args)` | 短时单次预览/确认，复用原控制器的互斥、反馈、模式及回放 |
| `wait(action_id)` | 等待原控制器真实终态，不能按文件时长推算成功 |
| `stop()` | 文件验证期间仍可调用，不受示教服务锁阻塞 |

底层控制器发送原 action_id 的 ACP，公共服务只观察，不重复通知。高频控制、模型和设备协议不属于本模块。页面通过 MCP 返回 HTML，由 Core 通用沙箱承载，没有独立 Web 服务或新增端口。

## 持久化与运行限制

轨迹保留在 `data_dir/recordings/`；资产原子写入 `data_dir/skills/<uuid>.json`，绑定轨迹 SHA、设备身份、模型/几何 profile 和成功试运行结果。发布还需用户效果确认，失败或取消不得发布。资产和原始文件必须一起备份；换设备或改变轨迹需要重新验证。

G1 预览 60 秒内单次有效。回放入口核对预期 SHA 和停止代次，拒绝预览后文件变化、停止后旧请求再启动；原只传 filename 的接口保持兼容。

保存到 Driver 后由工作台宿主导入 Core 现有 Skill 格式，失败可幂等重试。不上传资源中心，不改变未来动作的确认要求。一个活动草稿；页面刷新恢复，Driver 重启后载入完整录制重新验证，已保存资产可直接请求运行。

G1 新定义记录来源 MCP/卡片，动作包导出和调用说明更新严格使用该来源；这不是 Agent 多设备自主路由的强制限制，多台同名 G1 的 Agent 选择仍待完善。网页没有新增聊天框，Agent 使用已有语音/消息渠道；网页「运行技能」仍是资产 ID 参数表单，按名称选择运行的便捷页面尚未实现。

## 浏览器复现

```sh
TEACHING_CORE_ROOT=/path/to/phanthymotus \
TEACHING_SCREENSHOT_DIR=/tmp/g1-teaching-screenshots \
uv run --python 3.12 --no-project --with pytest --with numpy --with scipy --with pyyaml \
  --with playwright --with uvicorn --with fastapi --with python-multipart --with httpx \
  --with aiohttp --with aiofiles --with openai --with html2text --with jsonschema \
  python -m pytest -q tests/test_g1_teaching_browser.py
```

预先安装 Playwright Chromium，或用 `PLAYWRIGHT_CHROMIUM_EXECUTABLE` 指定可执行文件。测试启动真实 Core API、MCP HTTP 和临时 SQLite，硬件输入/SDK 为模拟，不连接机器人，不调用真实 LLM。


## 可选的动作包能力

适配器实现 `compatibility()`、`export_recording(recording)` 和 `import_recording(manifest, data)` 后，工具声明 `inputSchema.x-skill-package.schema=motus.teaching.package/1`。未实现的机器人不暴露此能力。G1 当前支持 G1_23 双臂 10 关节固定手，兼容性与重新绑定由 G1 适配器执行。

`package_export` / `package_read` 下载完整包；`package_begin` / `package_write` / `package_finish` 上传并创建未批准草稿。它们不暴露给 LLM，不移动机器人。必须重新试运行和确认效果后通过原 `save` 流程安装新的本机 Skill。`package_definition` 为原已保存资产生成当前调用说明，供用户显式更新旧 Skill。

预览使用 `preview(asset_id)`，返回完整 `execution_arguments`，确认字段为 false。执行 `run` 的 per-action required 声明要求 asset_id、execute、preview_id、workspace_clear、take_control；用户明确确认后再填写 true。执行仍由适配器校验，不依赖 LLM schema 作为安全门。旧 `run(execute=false)` 线接口兼容保留。

资源中心的附件发布尚未接通；不要把仅含资产 ID 的文字 Skill 当作能在别的设备安装使用的动作包。文件格式、界面和资源中心调查边界见 [G1 文档](../../docs/teaching/g1.md)。
