# 来源与迁移边界

数值实现从主仓 `30280160` 后的四卡契约工作树所含既有 G1 实现按模块选择性迁移，来源文件及 SHA256 见 `source-manifest.json`。原始算法和 G1 模型来自 Driver PR #152（`0adb64cb63e358244f4e02c0a7823523824f17a8`），Unitree `xr_teleoperate` revision `845b25a32f7febedf220e830952a7134897adb9d`，Copyright 2025 HangZhou YuShu TECHNOLOGY CO.,LTD，Apache-2.0。完整许可证沿用仓库 LICENSE。

保留 G1_23 模型、锁定关节、手掌变换、PR152 五自由度加权目标和保守碰撞检查；删除 IK 每输入帧的执行限幅和数值进程内无用途的力矩生成，执行 arm 对最终 q 单独 RNEA。未迁移 PICO SDK、采集、相对映射、硬件发布或控制权逻辑。源码迁移不等于冻结基线复现或北京真机通过。

碰撞网格来自 Unitree `xr_teleoperate` 提交 `817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b` 的 `assets/g1/meshes`。`models/g1_collision/sha256.json` 与 LICENSE 保留；构建需固定源下载并核验全部哈希，运行时不下载。网格缺失时明确拒绝初始化，不跳过几何验证。

依赖仍要求带 CasADi 支持的 Pinocchio 3.1.0 / CasADi 3.6.7 / NumPy 1.26.4 ABI；普通 PyPI pin wheel 不提供该符号时不可冒充数值通过。目标 Linux ARM64 构建与真实求解需分别验证。

`fetch_assets.py` 选择性迁移主仓 `deploy/fetch_g1_collision.py`，只调整默认目录和宿主说明；构建通过此入口固定下载，未在数值控制循环引入网络。
