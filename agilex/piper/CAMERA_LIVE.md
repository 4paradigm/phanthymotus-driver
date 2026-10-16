# PiPER 实时相机监控

本轮在 `codex/piper-live-camera` 本地分支开发，以 Bot 已审查的 `7d9e997` 为基线。
用户要求先现场测试，不推送分支、不更新 PR、不请求 Bot。

## 使用与配置

新增 `piper_camera_live`（实时相机画面），resource 类型，仅有框架 `start/stop/info`，
与机械臂控制和骨架动作检测组成三张主要画布卡。顶部「监控」显示实际彩色画面；
构造/get_tools/info 不打开相机，不调用 CAN、模式、使能或机械臂动作。
驱动生命周期 start 在配置启用且 `PIPER_CAMERA_ENABLED=1` 时自动采集，智能控制关闭时可观看。
框架 stop 会停止采集；重新启用需要框架 start 或重启驱动，不通过 info 隐式恢复。

```yaml
camera:
  device: /dev/piper-camera-1
  warmup_seconds: 3
  live:
    enabled: true
    width: 640
    height: 480
    fps: 10
    jpeg_quality: 80
```

源码默认 `enabled: false`。仅允许现有彩色接口别名 `/dev/piper-camera-1/2`，不自动回退到
任意视频节点或别人的相机。分辨率支持 320×240、640×480、1280×720，发布 FPS 为 1–30，
JPEG 质量 1–95。实际分辨率与采集/发布帧率以 info 反馈为准。
机械臂 1 独立话题 `/agilex/piper/camera/live`；第二台可使用 `/piper2/piper/camera/live`，本轮未部署第二台。
旧照片 `/agilex/piper/rgb` 与实时流隔离，文件导入保持原行为。

## 采集与生命周期

V4L2/OpenCV 采集使用 spawn 子进程，尽量排空相机采集缓冲；只在到达发布间隔时编码 JPEG。
共享内存容量固定为 4 MiB，一帧覆盖上一帧，主进程用非阻塞锁取最新帧，不引入积压队列。
复用现有 Core context/executor，以 CompressedImage、BEST_EFFORT、KEEP_LAST(1)、VOLATILE 发布。
不写照片或录像，不新增运行依赖。

首次有效帧之前为 starting；首次等待上限为预热时间加 5 秒。后续超过 2 秒无新帧时停止旧帧发布，
改发 OFFLINE 占位图，info 提供中文原因和最后有效时间。旧进程释放后每 2 秒重试相同绑定相机。
画面时间为上海时区，标准 JPEG APP15 中另携带 `PIPER_LIVE_V1 ` 加 8 字节大端浮点采集 Unix 时间，
供只读测试测量精确帧龄；普通 JPEG 渲染器忽略该元数据，图像仍兼容现有 Core。

停止事件按启动周期独立创建。停止后禁止迟到帧发布；子进程先请求退出，再限时 terminate/kill。
确认进程死亡后才释放占用文件锁、IPC 和 ROS 节点。旧监督线程未退出时报告 stopping 并拒绝 start。
相机失败/重连不持有机械臂 gate，不调用共享状态的控制副作用方法。

实时流和旧拍照用同一数据目录、设备别名对应的非阻塞文件锁。实时流运行时拍照明确返回占用提示，
不抢占设备；停止后原拍照授权和处理流程继续生效。首次失败也释放文件锁，其他采集不会永久被阻塞。

## 测试与部署记录

- 指定回归：228 passed、123 subtests passed，包含生产入口、控制、骨架、相机及打包检查。
- 相机新增 12 项模拟测试：新帧/时间推进、过期不重播、断流恢复、设备缺失/占用、重复启停、
  采集初始化期间停止、无 CAN/动作锁访问、旧拍照互斥、双设备隔离、最新帧邮箱、阻塞进程终止、
  JPEG 时间元数据和配置拒绝。
- 未修改 Core CameraRenderer 的本地浏览器模拟：15 个新帧、3 个场景、OFFLINE 与恢复通过，无脚本错误。
- 用户关闭智能控制后，仅重启机械臂 1 驱动；Core 镜像/启动时间及机械臂 2 镜像/启动时间保持不变。
  原画布两张卡保留，新增实时相机卡。相机采集开关设为 1，原彩色绑定和其他现场配置保留。
- 首次部署备份：`/opt/phanthy-motus/backups/piper-arm1-live-camera-20261013-144856/`。
  采集时间元数据补充备份：`/opt/phanthy-motus/backups/piper-camera-timestamp-20261013-145337/`。
  最终镜像：`agilex-piper:local-arm1-live-camera-ts-a3a557ea`。
- Edge 实际顶部监控显示新彩色画面和 10 FPS；用户确认镜头前改变场景时画面同步变化。
  被动快照确认原骨架反馈新鲜且时间推进。测试过程中代理未执行机械臂运动、模式或电机指令。
- 本机证据目录：`acceptance/20261013_piper_camera/`，包含模拟截图、只读快照、时钟校正及五分钟流验证。
  真实相机图像不上传 Git，流验收只记录计数/帧龄，不保存连续图像。
- Mac → Core WebSocket 连续 300.05 秒验收：2,988 个新帧，9.96 FPS，重复帧 0，断流帧 0。
  按 JPEG 采集时间校正时钟后的帧龄平均 0.143 秒、P95 0.189 秒、最大 0.485 秒；
  SSH 时钟校正不确定度约 ±0.115 秒。这是到 Mac 接收的传输测量，不包含浏览器绘制耗时。
- 仅终止机械臂 1 的单个相机 spawn 子进程进行故障注入：观察到 offline，5.43 秒后恢复新帧，
  主驱动没有重启，未发送机械臂控制指令。这验证进程故障恢复，不冒充真实 USB 拔插验收。
  记录 `camera-recovery-result.json`，实际恢复后截图 `camera-live-server.png`。

回归命令：在现有指定 PiPER 集合前增加 `tests/test_piper_camera_live.py`。
浏览器测试脚本 `tests/piper_camera_browser.mjs` 使用现有测试环境的 Playwright、Edge 和 OpenCV，
不作为运行依赖安装。环境变量与旧监控测试一致，另设 `PIPER_CAMERA_EVIDENCE` 指定输出目录。

本轮没有 PR 更新；相机版本尚未生成。已有 PR #382 的历史 Version
`release.261013.74ab514` 不包含这次实时相机功能。

## 2026-10-14 background discovery through Arm control

The live camera runs in the existing shared driver lifecycle and requires no camera canvas node. When camera.live.enabled is configured, Arm control declares the existing namespaced /camera/live JPEG topic alongside its skeleton and parameter outputs. Camera acquisition, ownership, retries and worker implementation are unchanged. The piper_camera_live resource retains its lifecycle/Info interface for diagnostics, like the model resource.

Arm 1 canvas migration removes only that device's piper_camera_live node and its incident connections, and updates control topicOut. The canvas now contains only Arm control. The latest layout and source/config/compose were backed up; unrelated cards and arm 2 are preserved.

Validation: 51 related tests and 44 subtests passed; the candidate image passed 63 offline unittest tests. The unchanged Core browser harness, with one control node and no camera node, discovers skeleton, parameters and camera, renders a simulated JPEG with its expected dimensions and loads PiPER URDF. Server read-only checks confirm fresh arm feedback and live camera publication around 10 fps. Browser access to the actual server tab was blocked by tool policy, so final visual refresh remains with the user. No physical arm controls were invoked.

Image: agilex-piper:local-arm1-background-camera-539a4529. Backup: /opt/phanthy-motus/backups/piper-arm1-background-camera-20261014-140525/. Motion, camera worker, configuration, Core and arm 2 checked unchanged. Evidence: workspace acceptance/20261014_piper_background_camera/. PR #382 pending user testing; current Bot Version 尚未生成.

Removing the last camera canvas node makes Core stop that resource. The migration resumes only piper_camera_live acquisition with its camera-only start action after removal; arm control receives no Start/Stop or mode/motor command. Post-migration live publication is verified separately. The first stream probe overlapped camera warm-up and failed its steady 10 fps check; the warmed-up repeat passed. Frame contents were not saved.


## Current two-card contract after the 2026-10-15 review

Arm control now declares only the optional live JPEG output. Joints owns the
skeleton output. The current bundle declares/publishes no Parameters stream.
The camera resource, device binding and acquisition lifecycle are unchanged;
no camera card is required. Earlier single-card validation above is historical.
