# go1_bundle 卡片提交与验收指南

一张 go1_bundle 卡片从写完到被合入的**上传流程**与**验收标准**。
**写卡的格式契约见同目录 `CONTRIBUTING.md`;本文只管"怎么提交、怎么验收"这一段。**

> 适用于所有向本 bundle 提交卡片的同事。命令里凡是 `<...>` 的都替换成你自己的值。

---

## 0. 一图流程

```
写卡(.py + config/driver/Dockerfile)
   │   ← 格式契约见 CONTRIBUTING.md §1–§5
   ▼
离线 STUB 自测(开发机, 无硬件)                     ── 必过
   │   python3 main.py + curl tools/list / tools/call
   ▼
真机验证(狗上, 控制卡强制)                          ── 控制卡上架前必过(CONTRIBUTING §4)
   │   量程 / 自停 / 急停 / confirm
   ▼
提交:fork + PR → 目标分支                            ── 无直推权限时走 fork+PR
   │   git push <fork> <branch> ; gh pr create …
   ▼
验收:对照 §4 检查清单 + PR 如实标注 → 合入
```

---

## 1. 提交目标与仓库网络(先理清,否则会撞 403)

| 角色 | 仓库 | 说明 |
|---|---|---|
| **上游(网络根)** | `4paradigm/phanthymotus-driver` | 组织平台仓库 |
| **提交目标** | `<目标仓库>/phanthymotus-driver` | 分支 **`<目标分支>`**,路径 **`unitree/go1_bundle`**。框架/格式照它写 |
| **你的 fork** | `<你的账号>/phanthymotus-driver` | 从上游 fork 一份;与目标同 fork 网络 → 可跨 fork 提 PR |

> 当前 go1 卡片的提交目标 = `z007-jj/phanthymotus-driver`,分支 `feat/add_unitree_go1`(以维护者最新通知为准)。

**关键约束**:多数同事对目标仓库**没有直推权限**,直推会报 `403 Permission denied`。
→ 标准做法是 **fork + PR**:先 fork 目标(或从同一上游 fork),把分支推到**你自己的 fork**,再开 PR 进目标分支。

**交付物 = bundle 的 Docker 镜像**,用**标准 `robot_interface` SDK**(Dockerfile `cmake -DPYTHON_BUILD=ON` 编 unitree_legged_sdk go1 分支)。不要在 bundle 里换成其它私有后端。

---

## 2. 上传步骤(逐条,可复现)

> 在你本地的 bundle 检出目录里跑。占位符:`<fork-url>`=你 fork 的 git URL,`<branch>`=工作分支,`<目标仓库>`/`<你的账号>` 同上。

**① 确认改动范围(只动 `unitree/go1_bundle/` 内的文件)**

```
git status
```

**② 暂存 + 提交(逐个点名,别 `git add .`)**

```
git add unitree/go1_bundle/<卡名>.py unitree/go1_bundle/config.yaml unitree/go1_bundle/driver.yaml unitree/go1_bundle/Dockerfile
```
```
git commit -m "feat(go1_bundle): 新增<能力>卡 <卡名>"
```
> 若同时改了 `go1_sdk_client.py`(控制卡加下发原语时)一并 add。

**③ 加自己 fork 的 remote(第一次才需要)**

```
git remote add fork <fork-url>
```

**④ 推到自己 fork**

```
git push fork <branch>
```

**⑤ 开 PR 到目标**(base=目标分支,head=`你的账号:分支`)

```
gh pr create --repo <目标仓库>/phanthymotus-driver \
  --base <目标分支> \
  --head <你的账号>:<branch> \
  --title "feat(go1_bundle): 新增<能力>卡 <卡名>" \
  --body "<见 §3 PR 描述模板>"
```

**后续再提卡**:remote 已加,跳 ③;同分支追加 commit 后 `git push fork <branch>`,PR 自动更新。

---

## 3. PR 描述模板(必须如实标注验证状态)

```markdown
## 做了什么
<一句话 + 改动文件表>

## SDK / 后端
按 go1_bundle 设计,走标准 robot_interface SDK,交付即 bundle 的 Docker 镜像。

## 验证状态(如实)
- ✅ 离线 STUB 全绿:插件装配 / tools/list schema(含 x-action-params)/ dispatch / 越界拒绝
- ✅/⚠️ 真机:<写清哪部分真机验过、哪部分待复核,不含糊>
- ⚠️ 控制卡按 CONTRIBUTING §4 须真机验证量程+安全;建议合入前在狗上 build 复核实际动作
```

**红线**:验证状态**不许美化**。真机没验的部分写"待复核";哪条路径/后端验过就只写那条,不要把"机制验过"说成"整卡验过"。

---

## 4. 验收检查清单(reviewer / 自查)

**格式(照 CONTRIBUTING §7)**
- [ ] 自包含 `<卡名>.py`,导出 `make_plugin(...)`,遵守插件契约(get_tool/start/stop/dispatch)
- [ ] `卡名 == 模块名 == 文件名 == config.yaml key == NODE` 一致且唯一
- [ ] `config.yaml` 已 `enabled: true`;`driver.yaml` 描述已更新;`Dockerfile` 有 `COPY <卡名>.py`;新 pip 依赖进 `requirements.txt`
- [ ] `dispatch` 返回 plain dict(不自己包 `{content:[]}`);未知 action 返 `None`
- [ ] 状态卡数据带 `timestamp_ms`/`control_level`/`fresh`,无新包不伪造

**控制卡额外(强约束)**
- [ ] `inputSchema` 用 `action`(enum) + `x-action-params`
- [ ] 成功包 `{ok,card,action,control_level,applied,timestamp_ms}`;失败 `{ok:false,code,message}`(code∈INVALID_ARGUMENT/PRECONDITION_FAILED/SAFETY_LIMIT/RESOURCE_BUSY)
- [ ] 越界/缺 confirm 一律拒绝,不静默截断;危险动作(关电/特殊动作/低层)必须 confirm
- [ ] **已上真机验证量程+安全**(CONTRIBUTING §4)——PR 描述如实写明验到哪一步

**验证**
- [ ] 离线 STUB:`tools/list` 见新卡、`tools/call` 行为符合预期
- [ ] 真机:`fresh=true`、数值合理;控制卡悬空/低速实测
- [ ] Docker 能 build(`./build.sh go1_bundle`)

**提交**
- [ ] 只 add bundle 内文件,commit 信息含 `feat(go1_bundle):` 前缀
- [ ] PR base=目标分支、head=`你的账号:分支`
- [ ] PR 描述含"验证状态(如实)"段

---

## 5. 常见坑

- **`403 denied`**:对目标仓库无直推权限 → 走 fork+PR,不是配置问题,别改 remote 硬推。
- **公司网 github HTTPS 偶尔限流**:push/PR 超时**重试即可**,别动 git 配置。
- **别 `git add .`**:仓库里可能有其他人/其他机器人的改动,只 add 自己 bundle 内的文件。
- **交付用标准 SDK**:bundle 的下发路径就是 Dockerfile 编出来的标准 `robot_interface`;若你在别处用了其它后端做实机联调,验证结论要注明是哪条路径验的,别把两者混为一谈。
- **控制卡未真机验证不合入**:CONTRIBUTING §4 硬要求,PR 里如实写验证边界。

---

## 6. 相关
- 写卡格式契约:`CONTRIBUTING.md`(§1 心智模型 / §2 契约 / §3 状态卡 / §4 控制卡 / §7 检查清单)
- 平台驱动规范:仓库根 `README_dev.md`(MCP JSON-RPC、x-action-params、端口 15700–15799)
