# 现场接口兼容

本次候选保留主实例的 arm_control、piper_photo、独立 piper_status 骨架卡及 model。第二单臂实例保持现有十卡片与 piper_monitor，不在本次主实例镜像切换范围内。

monitor.py、monitor_bundle.py、status_card.py 与 resource 来自 2026-10-13 现场镜像 agilex-piper:local-arm1-pose-24cb589e 的已有实现，按文件字节保留。镜像标签不是可验证的 Git 提交；上游未找到监控提交，不能把它写成已合并上游。未引入其他设备数据或配置。

- monitor.py SHA256: abe54f14ef6772945637032a7867858602a53b440c3bb608da082ca731942fa3
- monitor_bundle.py SHA256: fbedf54a057d70a2a7ee4acdd745bde301f75abbdca7f9caa2427bb1b2996e82
- status_card.py SHA256: e1106efdcfc9fde5afbdf8d15b2a41c0bb190799327cec10e9b01f3d0c9f9142

模型来源及许可证见 resource/SOURCE.md。显示模型不用于运动解算，也不证明物理标定通过。原源码中的测试描述属于来源说明，本次实际验证以执行记录为准。

单臂启停及 info 不建立 CAN 连接，手动状态 read 才使用原有共享连接。双臂配置保留原十卡路由；不会把单臂监控包装器用于双臂，也不默认切换 arms 配置。双臂监控界面、左右绑定与动作验收仍待完成。
