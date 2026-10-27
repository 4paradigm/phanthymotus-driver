# 通用网页示教模块

提供检查、录制、试运行、保存普通 Skill 的网页流程。设备检查、运动、停止和完成回执由机器人原控制器负责。当前 G1 适配见 [使用说明](../../docs/teaching/g1.md)。

依赖已合并的 [Core #318](https://github.com/4paradigm/phanthymotus/pull/318)、[Core #334](https://github.com/4paradigm/phanthymotus/pull/334) 和 [Driver #417](https://github.com/4paradigm/phanthymotus-driver/pull/417)。本次只改 Driver，不增加 Core 专用接口。完整宿主契约见 Core 的 `docs/SKILL_WORKBENCH.md`。

共享模块与独立 [PiPER #402](https://github.com/4paradigm/phanthymotus-driver/pull/402) 存在重叠。G1 属于 [Driver #412](https://github.com/4paradigm/phanthymotus-driver/pull/412)；本轮不修改 PiPER 适配器。旧 PiPER 缺少 `presentation` / `replay_instruction`，合并公共路径前需要在 PiPER 自己的 PR 补齐和验证，不能直接宣称兼容。

## 用户入口

- G1 执行卡选择「录制技能」→ 原有「执行」→ 检查、录制、试运行、保存。保存后进入当前设备的普通技能库，通过既有 `activate_skill` 和工具调用使用。
- 导入：同一张卡选择「导入动作包」→ 选择 `.motus-skill` 或 `.zip` → 执行。导入只校验和创建草稿；切回「录制技能」继续检查、试运行和启用。
- 稍后验证：重新打开该卡工作台，展开「待验证动作」，选择草稿后点击「继续验证」。列表归 Driver 所有，Core 不再聚合草稿。
- 自动执行：示教保存时、导入启用时，或「我的 → 技能广场 → 已安装 → 技能 → 对话调用」设置。仅对此技能、此卡片、此录制生效，默认关闭；没有整卡开关和限时会话授权。

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
| `execution_binding()`（可选） | 返回稳定设备身份、模型/几何/标定摘要；声明持久自动执行能力。不能包含每次启动都会变化的会话代次 |

底层控制器发送原 action_id 的 ACP，公共服务只观察，不重复通知。高频控制、模型和设备协议不属于本模块。页面通过 MCP 返回 HTML，由 Core 通用沙箱承载，没有独立 Web 服务或新增端口。

## 文件和 Skill 如何保存

轨迹位于 `data_dir/recordings/`，资产位于 `data_dir/skills/<uuid>.json`。资产绑定轨迹 SHA、设备身份、模型/几何与成功试运行结果；保存还需要用户确认效果。资产与原始轨迹必须一起备份。

Driver 返回普通 Skill 定义，工作台宿主经既有 `/api/skills/import` 安装到当前 Core。定义用 `attachment: {schema, tool, asset_id}` 标明本机资产，不能靠 `taught-` 前缀反推身份。型号显示由 `compatibility.labels` 提供；兼容判断仍检查型号、关节顺序、模型与几何摘要等实际字段。

导入草稿位于 `skills/drafts/`。关闭页面不会删除它们；Driver 重启后保留文件，未保存的试运行回执失效，需重新验证。一个活动草稿可切换多个已落盘的导入草稿；不能用导入覆盖尚未保存的录制。

只有本机已有完全相同、试运行成功且已确认效果的资产才可以复用验证。若技能已经安装，直接使用现有技能；Core 不覆盖同标识下用户修改过的定义，刷新生成的调用说明请用技能详情的「更新调用说明」。

## 动作包与通用文件传输

适配器实现 `compatibility()`、`export_recording(recording)`、`import_recording(manifest, data)` 后，工具才声明 `x-skill-package` 并提供 `import` / `export`。

- `import(package)` 的参数声明 `format: file`、`accept: .motus-skill,.zip`、`uploadTo: mcp`。Core 通用文件转发把文件上传到 Driver `POST /file/upload`，再传入 Driver 容器内的绝对路径。不需要共享目录、base64 或另一套分块协议。
- `read_package` 只读有界普通文件，再执行 `unpack`。通用上传上限为 64 MiB；本协议仍限制压缩包 16 MiB、轨迹 64 MiB、manifest 16 KiB。
- ZIP 只允许 `manifest.json`、`trajectory.jsonl` 两个唯一条目，不按归档路径解压。检查大小、压缩方式、重复 JSON 字段及 SHA；新增 manifest 描述字段可以忽略，必需字段不能缺失。
- G1 导入去掉源设备的会话权限，检查兼容性后按目标设备身份重新绑定。导入声明不是试运行证明。
- `export(asset_id)` 将完整包原子写入 Driver 的 `skills/exports/<asset_id>.motus-skill`，返回路径、大小、SHA 和 manifest。该路径属于设备容器，**目前不会自动下载到浏览器，也不会上传技能广场**。本轮不新增专用下载链路。
- `package_definition(asset_id)` 留给 Core 的「更新调用说明」。旧分块 action、`Transfers` 和 `x-skill-package.pending` 已删除。

## 持久自动执行与完成判定

Core 保存每技能 × 每卡片 × 每份录制的选择。Driver 每次 `preview` / `run` 通过同机回环 `/api/skills/package/execution/read` 核验，使用 `enabled`、`source`、`binding_changed`；Core 不可用时不能使用自动执行权限。声明持久能力的 G1 不再读取每卡片配置或内存会话授权。

`preview(asset_id)` 返回 `confirmation_required` 和 `execution_arguments`。false 时直接执行返回的参数；true 时结合用户对同一次尚未执行请求的确认，补齐 `workspace_clear`、`take_control`。`run` 必填 `asset_id`、`execute`、`preview_id`。页面只有在用户明确勾选/取消后才写自动执行选择，重试保存不会因默认未勾选而清除原设置。

正常停止和重启不清除逐技能设置。设备、模型、标定、资产或条件变化会使绑定不匹配。自动执行仍保留新预览、60 秒单次凭据、实时反馈、起点检查、控制权互斥、停止代次及速度限制。G1 的 `execution_context()` 只用于拒绝停止期间的旧执行请求，不是一套授权。

卡片 `x-stop.pending_flags` 声明 `busy=true` / `authority_released=false`，配合 Core 通用停止判定。`running` 仅为提交成功，必须等待同一 `action_id` 的真实终态；`completed` 表示轨迹与交权完成，不代表物体操作效果成功。本轮不改交权等待时间和静止阈值。

## 验证与边界

2026-10-19 针对已合并 Core main `6df601f`，文件/协议/真实 HTTP 检查 **72 passed**：

```sh
TEACHING_CORE_ROOT=/path/to/phanthymotus PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3.11 -m pytest -q tests/test_teaching_package.py \
  tests/test_teaching_core_contract.py tests/test_file_upload_endpoint.py
```

依赖 pytest、PyYAML、FastAPI、uvicorn、httpx、aiohttp、jsonschema 及 Core 配置依赖。使用 Python 3.11/3.12；主分支通用上传仍使用 `cgi`。未配置 `TEACHING_CORE_ROOT` 时跨仓库用例跳过。

上述自动化检查未使用机器人 SDK 或运动模拟。已同步 SDK 夹具和浏览器回归定义，但本轮未运行模拟机器人流程。**实机验证记录：2026-10-19 的 `a335e68c` 接口修改版本已完成 G1 实机验证。** 当前使用与验证状态见 [G1 使用说明](../../docs/teaching/g1.md)；10 月 16 日的实机记录仍保留为对应旧版本的历史记录。

资源中心完整动作附件发布、审核、下载与安装闭环不在本次验证范围；不能把仅含本机资产 ID 的文字 Skill 当作跨设备动作包。多台同名设备的 Agent 选路仍沿用现有画布机制。
