# 上海 G1 晨会主持助手

通过画布文字输入完成晨会开场、议程播报和收尾提醒。机器人只输出语音；可选挥手必须由操作员在同一轮明确确认机器人稳定站立且周围无遮挡。

## 安全范围

- 画布执行工具仅绑定 `tts` 和 `arm`；文字入口为 `remote_message`。
- 不调用麦克风/ASR、相机、OCR、传感器、姿态/模式读取、定位或导航。
- 不行走、不改变站立高度、不蹲起、不自动保存会议纪要。
- 未收到同轮安全确认时，不调用 `arm`。

## 画布与现场测试

1. 画布数据连接：`remote_message → decision_core`。
2. 画布执行连接：`decision_core → tts`、`decision_core → arm`。不要连接其他工具卡。
3. 启用 `g1-morning-meeting-host`，确认项目运行。
4. 在 `remote_message` 的 TEXT 输入框依次发送“开始晨会”“播报议程：……” “提醒收尾”“结束晨会”。每条都等播报结束后再发送下一条。
5. 只有现场确认站稳且周围无遮挡时，才测试可选挥手。不要用蹲起动作测试。

真机语音已通过画布入口现场确认。多条语音连续播报依赖 G1 TTS 序号修复（见驱动 PR #349）；旧镜像若未包含修复，应先验证驱动版本。SDK 修复不由本 Skill 安装器应用。

## 离线安装

确认数据库路径并停止 Agent Core 后执行：

```bash
python3 install_g1_morning_meeting_host.py --db /opt/phanthy-motus/data/data.db
```

安装器备份 SQLite，保留其他技能数据，并在新装时默认停用本 Skill。安装后启动 Agent Core，在技能页检查定义，再按需启用。不要直接修改上游 `main`。

本地回归测试：

```bash
python -m unittest discover -s unitree/g1/scripts -p test_install_g1_morning_meeting_host.py -v
```
