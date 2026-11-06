# 工位巡检拍照留档：操作与验收

本包是 PhanthyMotus 机器人的原生 Skill，不是 Codex 的开发技能。业务定义在 `skill.json` 中；运行时先展示 oneLiner，再由模型调用 activate_skill 加载完整 instruction。

适用场景：机器人已经停在待记录的工位前，操作者给出工位名称，采集一张现场照片并取得真实路径回执。工位名称是人工标签。第一版不包含导航、设备状态识别和图像质量检测。

## 文件

| 文件 | 用途 |
|---|---|
| `skill.json` | 可安装的原生 Skill 定义，含完整指令 |
| `instruction.md` | 便于阅读、修改和评审的指令原文，与 JSON 内容一致 |
| `deploy.py` | 在 Windows 上运行，经 SSH 安装到机器人 |
| `install_skill.py` | 原生 ConfigDB 安装器；自动定位 Agent Core 容器，或指定现有数据库 |
| `verify.py` | 直接调用已有 API 完成一次状态检查、拍照、照片下载和 JSON 留档 |
| `test_skill.py` | 安装器保护、异常处理和文件一致性检查 |
| `check_native.py` | 在隔离配置中加载原项目技能模块，核对可见性、按需激活和停用 |
| `records/` | API 实测生成的照片与回执；不包含认证信息 |

## 1. 保持 SSH 转发

在 Windows 终端运行，登录后保持窗口打开：

```powershell
ssh -p 2222 noetix@<机器人IP> -L 15678:127.0.0.1:15678
```

打开 https://127.0.0.1:15678/ ，在 Access Token 框输入你已有的 Dashboard Token。

## 2. 安装 Skill

按团队 landing 流程，优先在技能广场“我的技能”创建草稿，按 `skill.json` 填写名称、slug、简介、指令和依赖，保存并安装到目标机器人。
修改后先卸载旧版本再重新安装；完成 Master 与老师验收后提交审核。
以下脚本仅作为开发／网络异常时的应急入口，不是默认安装方式。

另开一个本机 PowerShell 窗口：

```powershell
cd D:\phanthymotus-driver\noetix\bumi\skills\station-photo-inspection
python deploy.py --host <机器人IP>
```

OpenSSH 会提示输入机器人登录密码。这一步由你在本机终端输入，脚本不收集或保存密码。部署时也不会把 Dashboard Token 写入机器人或本地文件。

安装器自动选择声明 15678 端口或名称匹配 Agent Core 的容器，在事务中只更新 ConfigDB 的 `skills` 一项，并保存该项的旧值备份；保留其他技能和其他配置。若有多个匹配容器，则停止并要求明确名称：

```powershell
python deploy.py --host <机器人IP> --container 实际的AgentCore容器名
```

同样的定义重复安装是幂等操作。只有确认需要覆盖同 slug 的旧定义时才使用 `--replace`。

看到包含 `"ok": true` 的安装结果后，刷新网页，打开 **技能 → 已安装**，应出现“工位巡检拍照留档”，状态为启用。安装状态以这一步为准。ConfigDB 每次读取技能列表时查询数据库，无需重启服务。

自动安装需要 noetix 对 Docker 有操作权限。若机器不是 Docker 部署，先将 `skill.json` 和 `install_skill.py` 复制到机器人，再在正确环境使用现有 ConfigDB 的实际路径：

```bash
python3 install_skill.py --db /实际路径/data.db
```

安装器不会新建数据库；路径不对、配置结构不符或权限不足时会停止。

如果需要恢复安装前的技能配置，安装输出中的 `backup` 指向包含旧 `skills` 值的 JSON 文件；它只备份这一项，不备份整库。日常停用或卸载可在网页“技能 → 已安装”中完成。

也可通过网页的 **我的 → 登录资源中心 → 技能 → 我的 → 创建**，按 `skill.json` 填写名称、slug、分类、版本、简介、描述、指令和依赖工具，保存为自己的草稿后安装。这条路径需要资源中心账号；Dashboard Token 与资源中心登录凭据不同。无需提交广场审核来完成设备上的验收。

## 3. 配置画布

如果当前画布已经有下述卡片，复用已有卡片。否则从工具列表添加：

1. AgentCore 的 **远程文本消息 / remote_message**。
2. AgentCore 的 **决策核心 / decision_core**。
3. Noetix Bumi 的 **vision_capture**。

完成两种不同的连接：

- **数据线**：remote_message 的数据输出 → decision_core 的数据输入，用于把你的文本交给决策核心。
- **执行绑定**：decision_core → vision_capture，使模型能够调用拍照能力。单独连相机数据线不能代替这个绑定。

确认决策核心的模型服务已配置，点击 **启动项目**。本 Skill 不要求添加新的感知模型；vision_capture 使用已有的 Bumi 相机 worker。

## 4. 执行场景

机器人放在待记录工位前。在 remote_message 卡片中选择 `send_message`，text 填写：

```text
执行“工位巡检拍照留档”，工位是装配工位 A。
```

点击该卡片的执行按钮。正常调用顺序为：

```text
用户消息 → 决策核心 → activate_skill
→ vision_capture.info → vision_capture.capture_photo → 结果回执
```

工具完整名称由运行时决定，Skill 不写死 MCP ID。正常回执应包含：

```text
工位：装配工位 A
结果：已保存
采集时间：本次相机返回的 captured_at（相机未声明时区）
照片路径：本次相机返回的绝对路径
文件核验：实际核验结果，或“尚未独立核验”
```

照片保留在相机服务返回的原始位置，Skill 不将工位标签传成不存在的拍照参数，也不重命名原图。如果没有可访问该位置的文件工具，模型只能报告相机返回已保存，不能报告已经独立打开文件。

## 5. 独立核验与留档

本机 PowerShell 运行：

```powershell
cd D:\phanthymotus-driver\noetix\bumi\skills\station-photo-inspection
python verify.py --station "装配工位 A" --insecure
```

按提示输入 Dashboard Token。`--insecure` 仅用于通过本地 SSH 隧道访问该自签名 HTTPS 服务，脚本将它限制在回环地址。

该命令会实际再拍一张照片，下载到 `records`，检查 JPEG 文件头、结束标记和非空内容，生成包含工位、北京时间记录时间、相机原始时间、真实远端路径、文件大小和 SHA-256 的 JSON 回执。返回 `status: success` 且退出码为 0 才算该项通过。打开生成的 `.jpg` 检查现场画面。

这项检查验证的是相机、API 和文件保存链路；第 4 步才是模型按 Skill 执行的验收，二者不能互相替代。

## 6. 验收用例

| 用例 | 输入或条件 | 通过标准 |
|---|---|---|
| 正常采集 | “给装配工位 A 拍照留档” | 先检查状态，再拍一张；回执路径真实，照片能打开 |
| 补齐名称 | “执行工位巡检拍照留档” | 先询问工位名称，未得到回答前不拍照 |
| 再次执行 | 工位改成“检测台 2” | 新照片对应本轮回执，不拿旧图代替 |
| 缺工具绑定 | 决策核心未绑定 vision_capture | 提示缺少工具绑定，不报拍摄成功 |
| 相机无帧或过期 | 无法确认 2 秒内新帧 | 不拍照，说明检查相机；不自动中断其他录像 |
| 拍摄失败或超时 | 失败、缺路径、调用结果不确定 | 明确失败或未确认，不编造照片路径，不自动重试 |

本地代码检查：

```powershell
python -m unittest -v test_skill.py
```

## 当前验证范围

真实 Bumi 的 info、capture_photo、照片下载链路可通过 `verify.py` 验证；历史验证范围见 `VALIDATION.md`。离线安装器可在隔离 SQLite 数据库验证，设备安装和模型驱动的场景验收需要按第 2–4 步完成。本次源码迁移没有连接机器人。
