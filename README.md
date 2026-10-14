# G1 颁奖典礼主持 Skill

Unitree G1（上海，`10.110.12.110`）的**颁奖典礼主持 Skill**。机器人只负责念词、带气氛、
揭晓与庆祝，**不递送实物、不代领奖品**。

- Skill 唯一标识：`g1-award-ceremony`
- 名称：`G1 颁奖典礼主持`
- 依赖工具（已被 G1 driver 提供）：`tts`、`arm`、`led`
- 画布地址：`https://10.110.12.110:15678/`（HTTPS，自签证书，浏览器需选择「继续访问」）

## 一、颁奖仪式步骤（机器人执行流程）

整场典礼由主持人在画布 `remote_message` 用**文字指令**驱动。默认**只出语音**，
任何动作都必须先由操作员在本轮明确说「机器人已稳定站立、周围无遮挡，可以动作」。

| 步骤 | 指令（画布发送） | 机器人行为 |
|---|---|---|
| 1 开场 | `开始颁奖` | TTS 播报「颁奖典礼现在开始，请嘉宾就座，我们即将揭晓各个奖项。」 |
| 2 揭晓 | `揭晓：<奖项>，<获奖人>` | TTS 倒计时「三、二、一」→ 播报「<奖项> 的获得者是 <获奖人>」；仅在本轮安全确认后挥手一次 |
| 3 庆祝 | `庆祝：<奖项>` | TTS 播报「恭喜获奖！」；本轮已确认时可再挥手一次 |
| 4 灯光 | `主题色：金/蓝/红` | 仅在本轮安全确认后调用 `led` 设置一次主题色 |
| 5 收尾 | `结束颁奖` | TTS 播报「颁奖典礼到此结束，感谢大家，明年再见。」 |

安全边界：仅允许 `tts` + 单次 `arm` 挥手 + 单次 `led`；禁止 `loco / switch_mode /
posture / smart_motion / mic / speaker / ASR / 相机 / OCR`，禁止行走、转向、下蹲。

更完整的创意编排见 [`方案.md`](方案.md) 与 [`能力盘点与复合动作设计.md`](能力盘点与复合动作设计.md)。

## 二、在机器人上部署

```bash
# 1) 传入安装脚本（也可用 scp unitree@10.110.12.110:/tmp/）
base64 -i install_g1_award_ceremony.py | base64 -d   # 或直接 scp
# 2) 在真机上运行（会先自动备份 data.db）
python3 install_g1_award_ceremony.py
# 3) 重启核心容器以加载技能
cd /opt/phanthy-motus && docker compose restart agent-core
```

卸载：`python3 uninstall_g1_award_ceremony.py`（同样自动备份）。

## 三、验收 / 提交

1. 画布右上角「设计 ➡️ 技能」查看技能是否已安装，连接 `remote_message` + `agent_core` + `tts/arm/led` 卡片调试。
2. 测试通过后在 <https://motus.phanthy.com/zh/skills> 注册账号，在「技能 ➡️ 我的技能」创建并提交，找老师验收。
3. 验收时需带上对应机器人的 Master 一起。

## 文件说明

| 文件 | 说明 |
|---|---|
| `g1-award-ceremony.skill.json` | 技能定义（可直接粘贴/对照画布字段） |
| `install_g1_award_ceremony.py` | 写入 `data.db` 的安装脚本（幂等、自动备份） |
| `uninstall_g1_award_ceremony.py` | 卸载脚本 |
| `examples/揭晓复合动作.py` | 「登台揭晓」复合动作示例（默认 `--dry-run`） |
| `方案.md` / `能力盘点与复合动作设计.md` | 设计与实测能力盘点 |
