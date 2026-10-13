# 现场接口兼容

本次候选保留主实例的 arm_control、piper_photo、独立 piper_status 骨架卡及 model。第二单臂实例保持现有十卡片与 piper_monitor，不在本次主实例镜像切换范围内。

monitor.py、monitor_bundle.py、status_card.py 与 resource 来自 2026-10-13 现场镜像 agilex-piper:local-arm1-pose-24cb589e 的已有实现，按文件字节保留。镜像标签不是可验证的 Git 提交；上游未找到监控提交，不能把它写成已合并上游。未引入其他设备数据或配置。

- monitor.py SHA256: abe54f14ef6772945637032a7867858602a53b440c3bb608da082ca731942fa3
- monitor_bundle.py SHA256: fbedf54a057d70a2a7ee4acdd745bde301f75abbdca7f9caa2427bb1b2996e82
- status_card.py SHA256: e1106efdcfc9fde5afbdf8d15b2a41c0bb190799327cec10e9b01f3d0c9f9142

模型来源及许可证见 resource/SOURCE.md。显示模型不用于运动解算，也不证明物理标定通过。原源码中的测试描述属于来源说明，本次实际验证以执行记录为准。

单臂启停及 info 不建立 CAN 连接，手动状态 read 才使用原有共享连接。双臂配置保留原十卡路由；不会把单臂监控包装器用于双臂，也不默认切换 arms 配置。双臂监控界面、左右绑定与动作验收仍待完成。

## 2026-10-14 部署测试入口

本轮交付止于镜像构建与离线检查，不在夜间切换服务。候选标签为 `agilex-piper:dual-ready-20261013`。个人目录为 `/home/agilex/hanzebei/piper-dual-teleop`。

晚间现场主镜像为 `agilex-piper:local-arm1-live-camera-ts-a3a557ea`。候选复用其 camera_live.py、camera_owner.py、camera_worker.py，保留实时相机工具及拍照互斥。没有导入现场 direct_control.py 或自动 CAN 连接改动：本候选保留操作许可、一次性预览确认、手动状态 read，启动不连接 CAN。工具名保持五项，但控制 UI 会恢复许可与确认步骤；这不是现场镜像的无差异替换。

### 明天先做这些检查

1. 现场结束示教、停止 Canvas 项目，确认无人控制；实时核对项目、控制模式、使能及动作占用。不要沿用今晚的状态。
2. 核对实际运行镜像及 Compose。若现场版本又变更，先比较差异，不覆盖新功能。将当时 Compose 备份到个人目录，记录旧镜像 ID；旧镜像保留用于回退。
3. 检查候选镜像架构为 amd64，并对照个人目录 ready-receipt.json 的镜像 ID。
4. 首轮只替换 Compose 的 agilex-piper 服务镜像，保留端口、CAN、挂载和第二实例。保持运动许可关闭；相机测试须现场明确开始采集，首次不运动验收先关闭相机入口。不要启用双臂 arms 配置。
5. 在检查通过后，使用 `sudo docker compose -f /opt/phanthy-motus/docker-compose.yml up -d --no-deps agilex-piper` 仅重建主服务。此命令不是预检命令，今晚不执行。

### 验收顺序与通过条件

- 软件：health 返回 running；tools/list 包含 arm_control、piper_photo、piper_status、model、piper_camera_live。被动诊断：`sudo docker exec embodied-agilex-piper python3 /deploy/diagnose.py --output /tmp/piper-diagnosis.json`。PASS 仅表示软件接口通过，不表示 CAN 或实体通过。
- 状态：先读取 arm_control.info，确认没有动作且许可关闭；随后由现场操作者读取 piper_status.read。确认六轴/夹爪反馈新鲜、模式正确、无故障，并比较操作前后的模式及使能。
- 显示：在 Canvas 验证骨架和状态。现场开始相机测试后，检查实时画面时间与停止采集；照片、相机和实际 UI 均未被本次构建验收替代。
- 双臂：核对 USB/CAN 与实体左右标签，完成 TCP、安装方向和工作区标定，再停用重复占用 CAN 的旧单臂实例并切双臂配置。PICO MCP 15744、WSS 15743 均须重新查端口。PICO 配对、实体遥操、录制回放属于后续现场验收，不在首轮单臂镜像替换中自动开启。

### 回退

仅在设备静止、无任务且控制隔离时，将 agilex-piper 的镜像恢复为本次切换前记录的旧引用，再执行同一条定向 compose 命令。不要回退整个旧 Compose、数据库或第二服务。健康失败、接口缺失、许可异常或出现意外动作输出时停止验收；硬件状态未知时先由现场人员处理，不自动重复重启。

### 构建复现

本轮依赖与 dual-pose-20261013 相同，基镜像必须核验为 `sha256:11845baeec5e4d818a21b2cb0bd5e719b73eebdd9677d583acfa612318a9c231`。个人构建目录中的 Dockerfile.ready 只在该已验证镜像上复制当前 agilex/piper、common 和 deploy 源码，不重新安装依赖。完整 Dockerfile 仍保留官方固定 ROS 基础镜像的构建路径。构建日志与源码提交号保存在个人目录，增量构建不声称无缓存构建。
