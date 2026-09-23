# 来源与许可证

设备通信模块和原生客户端迁移自 phanthymotus PR #259（源码提交 30280160，Apache-2.0）。capture/RTC/protocol 最初来自 phanthymotus-driver PR #152 / 0adb64cb63e358244f4e02c0a7823523824f17a8；原生客户端最初来自 PR #151 / a97bbfb56a97f1a4959120a470393a96e40fdbb8。保留仓库根 LICENSE 和客户端 THIRD_PARTY_NOTICES.md。

本次将宿主改为 Driver ext_vr，替换设备 runtime，移除设备采集与运动租约的耦合。未迁移 URDF、IK 或机器人执行代码。第三方锁定通信依赖保持原哈希及发行许可证。
