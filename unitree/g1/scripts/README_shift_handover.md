# G1 交接班助手 1.1.3

读取 G1 姿态和模式，语音询问人工交接内容，在画布展示草稿，修改后再次确认，生成可复制的交接记录。不使用相机或运动。

## 零基础操作

1. 画布启用 g1-shift-handover 技能；确认旧站岗任务与技能已停用。
2. remote_message 输入：请激活技能 g1-shift-handover，准备交接。点击执行。
3. 回答本班工作、发现问题、下班待办。缺项可以写未提供或跳过。
4. Activity 文字输出或会话历史中查看完整草稿，语音用于提问与简短提示。
5. 修改后重新检查，输入“确认交接”才生成已确认记录。自行复制保存。
6. 输入“取消交接”“结束本次交接”“不提交”立即进入取消分支，不反问。只说“换新一份”“准备新一份交接”属于新开一份，按流程重新读取并重新三问。

机器人无需起身；不自动保存文件、上传报告或联系其他人。报告区分设备返回与用户陈述，标注新鲜度未独立验证。不输出无关关节参数或未经核实的时间。

## 接线

数据：remote_message → decision_core。
执行：decision_core → posture、switch_mode__get_current_mode、G1 驱动的 tts。
模式卡片本体是 switch_mode，执行连接只绑定现有查询子工具，不绑定整个模式切换工具。

移除旧 camera_distance、loco、controlled_spatial 等站岗连接，关闭自动播报。旧项目取消后，后台任务与长期记忆仍可能在开机时恢复，需先备份再归档；不得清除无关任务。
画布绑定不是完整权限沙箱，平台通用工具仍存在，验收必须核对真实调用日志。

## 安装与验证

发布定义：g1_shift_handover.json。
离线安装器：install_g1_shift_handover.py，新安装默认禁用，备份 SQLite，保留其他配置。
确认数据库路径并停止 agent-core、关闭自动启动后运行：

    python3 install_g1_shift_handover.py --db /实际确认的路径/data.db

核对备份与结果，再启动 agent-core。在线更新 instruction 后，测试平台的提示词缓存可能仍是旧内容；需要重新加载并核对实际提示，必要时暂停控制后重启 agent-core。

本地回归测试：

    python -m unittest discover -s unitree/g1/scripts -p test_install_g1_shift_handover.py -v

真机验收场景与通过标准见 TESTING_shift_handover.md；完整证据报告在工作区根目录（不在本仓库）。

实际验收覆盖开始交接、用户报告未核实问题、修改不提前确认、确认最新草稿、取消不提交。工具不可用用隔离画布测试，不拔线或破坏设备服务。

已知开放项：组合句「结束本次交接，换新一份。」模型判定不一致（4 次测试 3 次判为新开一份）；「准备新一份交接，之前已确认的记录保持不变。」只追问变化项，未重新逐字三问。详见 TESTING_shift_handover.md。

本地测试、真机对话、现场听到语音、平台发布、Master 验收需分别记录。保存真实调用日志和截图，不将失败轮次计为通过。发布前核对最终版与测试证据一致。

## 语音修复依赖

本变更包含 G1 SDK TtsMaker 的一行修复：序号由错误的 += self.tts_index 改为 += 1。已有驱动镜像若仍带旧代码，需要由设备维护人员应用修复或部署包含修复的镜像。仅安装 Skill JSON 不会更新驱动。回归测试：python unitree/g1/scripts/test_g1_tts_index.py。

当前交互为画布 remote_message 的 TEXT 输入加执行按钮，机器人语音输出；没有接入麦克风/ASR，不要把它宣传成语音输入交互。首次启动建议输入完整技能名；确认后的完整文字在 Activity 或会话历史里复制。
