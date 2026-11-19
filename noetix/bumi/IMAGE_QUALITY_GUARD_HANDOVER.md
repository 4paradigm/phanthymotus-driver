# 交接与人工审核

日期：2026-11-05。交付停在 PR 前，没有暂存、提交、推送、创建 PR、调用 review bot、上传镜像或部署机器人。阶段 A 两文件 SHA256 与开发前一致。

## 审核入口

1. Core `lifecycle_runs.py` / `api/lifecycle.py`：单 Owner、槽位锁、票据、epoch握手、未知状态恢复、有界终态和轮换。
2. Core `api/config.py` / `api/mcp_manage.py` / `mcp_client.py`：项目票据与冻结停止目标、回滚、共享清理任务、LLM执行边界。
3. Canvas `card-lifecycle.js` / canvas/dashboard/solutions/teardown：事件开始时捕获原票据，旧响应CAS，不把系统票据持久化进布局。
4. Driver Guard / Bundle：先校验票据后修改generation、owner保留、重复stop、shutdown与旧Card兼容。
5. Runtime：已有CompressedImage消息、QoS、最新帧、限频、无帧无发布、共享Executor不关闭、重试清理。
6. 测试与文档中的所有 PASS 应能对应实际命令；FakeRuntime、真实DDS、Bumi硬件明确分开。

完整改动列表以两仓库 `git status --short --branch` 为准；测试数量和环境见 TEST_REPORT。新增算法/插件文件原本未跟踪，不能因它们出现在状态中就判为本轮改写算法。

## 必须保留的接口语义

- run_id 只有一个含义：一次运行。Core生成 `epochUUID.runUUID`。
- 用户配置/模型输出不能赋予系统权限；run_id 不是认证密钥。
- 未知网络结果必须查询核对，不能当成功或已停止。
- 缺票 stop 不允许为了兼容而停 Guard。
- 清理失败仍归原运行；不得换票覆盖资源。
- 不以TTL/LRU删除可能被迟到请求引用的终态。
- 本地shutdown永久关闭准入，外部MCP没有shutdown动作。

## 当前剩余验收

Driver官方ARM64镜像已构建并消除LD_LIBRARY_PATH警告；上一轮145项及SIGTERM验证保留有效。Core官方Dockerfile本轮已修复APT_MIRROR未生效问题，使用临时Clash构建参数完成linux/arm64镜像。真实Core启动、11个关键模块/4个CLI工具、91包兼容性检查及Core→Driver三个场景54项断言均通过。正常SIGTERM/Core异常SIGKILL/显式stop-only恢复与Driver最终清理已实测；详见TEST_REPORT首节。本轮未重复跑无修改的全部测试。

镜像为Driver image-quality-guard:local（be6455114b2d）、Core image-quality-core:local（7afe1b07e8aa），仅本地保留；没有push或部署。测试容器、空resource卷和内部网络已清理，没有触碰其他运行中的容器。尚未做真实浏览器、跨容器DDS到Dashboard或Bumi Camera实验，阈值/处理负载未真机标定。单Owner之外的多机器部署不支持，不能只加worker。

旧Card兼容不等于完整的请求顺序保证：已经准入但尚未发送的旧Card start可能晚于项目stop，短project gate不足以消除此类竞态。严格Guard路径使用票据与墓碑隔离，但不得扩大该保证到未迁移Card。UNKNOWN项目遇Driver重启后的子Card恢复当前保守拒绝，需要人工核查和受控Core重启流程；没有实现自动项目恢复。这两项应由同事确认迁移/恢复策略，当前不能宣布所有最终验收条件已经满足。

恢复或轮换操作应由操作者在查询实际 Driver boot/epoch/active run 后显式执行；不要自动把 UNKNOWN 清成 idle。Core重启、Driver重启、启动响应丢失的组合需要同事重点复核。

进入PR前：核对差异、测试与打包范围；在内部批准后再按公司Fork/审核流程操作。本轮不自动执行任何Git写入、云端构建或机器人安装。

## 文件清单

主仓库分支 `feat/image-quality-guard-lifecycle`，当前累计22个变更文件；本轮仅增加官方Dockerfile的软件源修复，没有更改公共应用接口：

```text
agent-core/Dockerfile
agent-core/src/lifecycle_runs.py
agent-core/src/api/lifecycle.py
agent-core/src/api/canvas.py
agent-core/src/api/config.py
agent-core/src/api/mcp_manage.py
agent-core/src/api/solutions.py
agent-core/src/mcp_client.py
agent-core/src/start.py
agent-core/tests/test_image_quality_driver_e2e.py
agent-core/tests/test_lifecycle_mcp_boundary.py
agent-core/tests/test_lifecycle_owner_startup.py
agent-core/tests/test_lifecycle_runs.py
agent-core/web/js/card-lifecycle.js
agent-core/web/js/card-lifecycle.test.mjs
agent-core/web/js/canvas.js
agent-core/web/js/dashboard.js
agent-core/web/js/image-quality-view.test.mjs
agent-core/web/js/renderers/text.js
agent-core/web/js/solutions.js
agent-core/web/js/teardown-chip.js
agent-core/web/js/teardown-chip.test.mjs
```

Driver分支 `feat/bumi-image-quality`，当前累计26个开发变更文件；其中Guard及其测试原本已经存在但未跟踪。本轮新增两个Core/Driver集成helper，更新四份报告；未修改算法、Guard或Runtime：

```text
README_dev.md
noetix/bumi/Dockerfile
noetix/bumi/README.md
noetix/bumi/config.yaml
noetix/bumi/driver.yaml
noetix/bumi/main.py
noetix/bumi/image_quality_guard.py
noetix/bumi/image_quality_runtime.py
noetix/bumi/build-local.ps1
noetix/bumi/tools/image_quality_observe.py
noetix/bumi/tests/run_image_quality_ros_tests.sh
noetix/bumi/tests/run_container_validation.sh
noetix/bumi/tests/container_image_quality_smoke.py
noetix/bumi/tests/container_guard_mcp_server.py
noetix/bumi/tests/container_core_driver_integration.py
noetix/bumi/tests/test_bumi_bundle.py
noetix/bumi/tests/test_image_quality_guard.py
noetix/bumi/tests/test_image_quality_observer.py
noetix/bumi/tests/test_image_quality_ros.py
noetix/bumi/tests/test_image_quality_runtime.py
noetix/bumi/IMAGE_QUALITY_GUARD_README.md
noetix/bumi/IMAGE_QUALITY_GUARD_DEVELOPMENT_REPORT.md
noetix/bumi/IMAGE_QUALITY_GUARD_TEST_REPORT.md
noetix/bumi/IMAGE_QUALITY_GUARD_HARDWARE_TEST.md
noetix/bumi/IMAGE_QUALITY_GUARD_DEPLOYMENT_GUIDE.md
noetix/bumi/IMAGE_QUALITY_GUARD_HANDOVER.md
```

Driver状态还会显示两个既有未跟踪文件 `image_quality.py`、`tests/test_image_quality.py`，本轮未修改，不能误当作本轮算法改动。两仓库索引均无暂存变更、无未解决合并冲突，diff --check通过。
