# G1 录制技能（`g1_teaching`）

用 PICO 遥操示范一段双臂动作，试运行确认效果后保存成普通 Skill。Agent 通过既有的 `activate_skill` 调用它，回放由 G1 原有的 `teleop_control` 录制 / 回放 / 停止接口完成，不另建运动控制器。

**不需要单独启动服务。** 工作台网页由卡片的 `ui` 动作经 MCP 返回，在 Core 画布里渲染；没有新端口，也不加载外部资源。前提只有两个：

- 已部署 PICO Driver（`pico/4ultra`），头显已配对，画布上有 PICO → `teleop_control` 的连线。PICO 的配对与 APK 安装见它自己的说明。
- `g1_teaching` 卡连到决策核心。

## 录一个动作，保存成技能

1. 在 G1 示教卡选择「录制技能」，点「执行」打开工作台。
2. 点「检查设备」：本体反馈、机型与标定、控制状态、VR 通道。检查不会让机器人动。
3. 双方双臂下垂、松开握把，长按 X+A 一秒建立基准。页面提示可以录制后点「开始录制」，按住双握把示范，松握暂停，完成后点「结束录制」。
4. 恢复物体布置，执行试运行。机器人先低速移到录制起点，再按原时间轴回放。
5. 确认效果后填名称和使用条件，点「保存技能」。之后在「我的 → 技能广场 → 已安装」能找到，对话里说出技能名即可调用。

保存的是一段固定轨迹的回放，物体位置变了不会自己适应。

![设备检查](https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/resource-center/uploads/docs-g1-teaching-g1-01-check.png)

![遥操录制](https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/resource-center/uploads/docs-g1-teaching-g1-02-record.png)

![检查起点并试运行](https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/resource-center/uploads/docs-g1-teaching-g1-03-review.png)

![保存技能](https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/resource-center/uploads/docs-g1-teaching-g1-04-save.png)

## 免确认自动执行

默认每次调用都要用户确认。可以按技能单独开启，三个入口：保存时勾选「保存后允许自动执行」、导入启用时勾选「启用后允许自动执行」、或在「技能广场 → 已安装 → 技能 → 对话调用」里设置。

设置保存在 Core，重启不丢。它绑定的是技能 × 卡片 × 录制，以及模型、几何和标定摘要；任何一项变化都要重新设置。开启后每次执行仍会重新预览、检查实时反馈、起点、资源占用和停止代次。

## 动作包：导出与导入

- **导出**：卡片选「导出动作包」，填 `asset_id`。完整包写到容器里的 `skills/exports/<asset_id>.motus-skill`，返回路径和 SHA。目前没有浏览器下载入口，需要从设备上拷出来。
- **导入**：卡片选「导入动作包」，选择 `.motus-skill` / `.zip` 后执行。文件经 Core 通用上传送到 Driver；导入只校验并生成草稿，不会动。之后切回「录制技能」试运行、确认效果、启用。关闭页面后可在工作台的「待验证动作」里继续。

模型哈希、固定几何和关节顺序一致的 G1 之间可以互相导入。动作包不带源设备的任何权限，到新设备上一律要重新试运行；只有本机已有完全相同、且已确认效果的资产时才跳过。

## 实现

| 文件 | 内容 |
| --- | --- |
| `service.py` | `TeachingPlugin`：工作台动作、草稿、试运行回执、资产保存、Skill 定义、从本机 Core 读逐技能执行策略 |
| `package.py` | 动作包打包 / 解包：ZIP 只允许 `manifest.json`、`trajectory.jsonl` 两个条目，限制大小并校验 SHA |
| `workbench.html` | 工作台页面，经 `ui` 动作返回 |
| `../teaching.py` | G1 适配器：检查项、录制会话、回放、兼容性与可移植录制 |

数据位置：轨迹在 `teleop_control.data_dir/recordings/`，资产在 `skills/<uuid>.json`，导入草稿在 `skills/drafts/`，导出在 `skills/exports/`。资产和轨迹要一起备份。

完成判定：`run` 返回 `running` 只表示已提交，终态以同一 `action_id` 的 ACP 为准。`completed` 要求轨迹发完 **且** 同一次交权已物理确认；交权未确认时报 `error`，`failure_stage: release`。

## 测试

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  unitree/g1/tests/test_g1_teaching.py unitree/g1/tests/test_g1_teaching_execution.py \
  unitree/g1/tests/test_g1_teaching_package.py tests/test_teaching_package.py

# 跨仓库：真实 Core 的策略接口与文件转发、浏览器流程（需要 Core 检出）
TEACHING_CORE_ROOT=/path/to/phanthymotus PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tests/test_teaching_core_contract.py tests/test_g1_teaching_browser.py
```
