# PiPER Ubuntu 本版验收记录

日期：2026-10-11（北京时间）。PR #382，分支 `codex/piper-driver`。
实际运行源码：`1cc109335762a46f8c0f75abbefa6ff4f8bb3784`。
本次结果来自 Ubuntu 真实机器人及其 Agent Core 画布；Mac 与模拟测试仅作辅助证据。

## 真实相机拍照恢复（最新状态）

用户明确要求“这个模块改下，允许拍照，调用相机拍照”，取代此前不调用相机的偏好。
照片卡片现使用 `capture_confirmed=true` 确认本次拍摄当前画面，不要求相机朝前，也不调整机械臂姿态。
旧 `forward_view_confirmed` 只保留为可选元数据，不能代替本次拍照确认。代码默认相机许可仍为 0，本机按用户授权显式部署为 1。

- 指定测试集合：141 passed，57 subtests passed，覆盖部署许可、每次拍照确认、旧字段不能替代新确认、停止/超时清理、不连接/移动 CAN、照片与 JSON/话题元数据。
- 部署 `agilex-piper:local-amd64-1cc1093`，amd64，镜像 ID `sha256:78a6fd9e5e01563b5fae153caffd30b14ac7f7523eadfe8ba824867fce874af6`。容器 device.py SHA256 与本地一致，仍为已核对依赖的现场镜像离线叠加源码。
- 通过真实 Agent Core API 调用 D435 拍一张，action_id `piper_photo_696d4157d3d949a5bd65a5abfe23628c` 最终 completed。V4L2，640×480，预热获得 77 帧后保存一张 JPEG，57132 字节，对比度 72.55；对应 JSON 已保存，camera_accessed=true、capture_confirmed=true、arm_moved=false、dds_published=true。
- Edge 新照片卡片已显示 CAPTURE_CONFIRMED；监控页显示与保存照片一致的真实彩色画面。监控帧率来自重复发布最近一张照片，不代表持续开启相机视频流。实际拍摄经后台 API 完成，未声称前端执行按钮拍照已经验收。
- 拍照前后六关节反馈一致 `[26.037, 0, 0, 7.863, 23.518, 5.255]`，六轴与夹爪仍使能、CAN 控制、无故障、busy=false。运动许可关闭、相机许可开启、全局智能控制已停止，未执行失能或重新初始化。
- 原始照片、JSON、调用记录和真实画布截图仅保存用户本机，不上传 Git/PR。下方相机未启用等内容为此前阶段记录，当前状态以上述内容为准。

## JOINT 数值传递修复（最新状态）

用户手动初始化已完成，随后在单轴预览选择 JOINT=1 时收到整数校验错误。平台把枚举下拉框值按字符串发送，现将 JOINT 改为 1～6 的整数输入框，保留驱动严格校验，不接受字符串、布尔值、非整数或越界值。

- 指定测试集合：140 passed，57 subtests passed；覆盖六轴预览零控制命令、输入框 schema 和严格类型/范围校验。
- Ubuntu 部署 `agilex-piper:local-amd64-06055a5`，镜像 ID `sha256:1eb22c5d664e38d06edf7737c981f070a9bdf2322cb6b8c847edf882781be449`，沿用已核对依赖的离线叠加构建。
- Edge 已确认 JOINT 为数字输入框；卡片预填 joint=1、target_deg=36、speed_deg_s=1、speed_pct=5、execute=false，未代用户执行运动。
- 经 Agent Core 后台 API 验证同参数预览成功，delta_deg J1=+0.674°，executed=false；不声称前端执行按钮已验收。
- 更新前确认用户本次初始化 action_id `piper_arm_d7647862d67e4d64a439ba6559492c09` 已 completed，连续读取姿态静止。更新后反馈为 CAN 控制、六轴仍全部使能、J2/J3=0°、夹爪失能、无故障、busy=false、运动许可关闭，相机关闭，全局智能控制已停止。无需重新初始化；用户后续真实执行仍需重新开启许可并生成有效预览。
- 本次修复仅更新服务、只读检查与预览，没有发送使能、失能或运动命令。

## 人工运动许可开关补充

- 在已有机械臂卡片增加 `allow_motion` / `block_motion`，人工许可不保存，停止机械臂卡片、全局停止、重连或进程重启后关闭。部署环境保持运动与相机权限为 0。
- 开启只读检查反馈，不发送控制命令；关闭可取消正在进行的控制并尽力保持，不释放力矩。
- 两项操作不加入 `x-action-params`，按当前 Agent Core 转换逻辑不会生成模型工具；仍可通过原始 MCP 调用，不构成身份鉴权。
- 指定测试集合：139 passed，47 subtests passed。覆盖零控制命令、异常反馈、并发、旧预览作废、停止/重连/重启恢复关闭及许可检查竞态。
- 新镜像 `agilex-piper:local-amd64-adf188f`，ID `sha256:55e73ca1f9218b80c7499c025c29c3e94309131446e139952e09fe1f72715475`，仍为现场基础镜像离线叠加源码。容器 `device.py` SHA256 与本地一致。
- Ubuntu 只读验证五卡片、许可关闭、待机、六轴与夹爪均失能、无故障；Edge 已显示两个新选项并选中 `allow_motion`，未执行开启，留给用户手动操作。
- 下列真实运动记录来自此前 `462e1af` 版本；本次更新未再次执行初始化、运动、失能或拍照。为部署停止了全局智能控制，用户后续操作须重新生成预览。

## 此前基础验收收尾状态（当前状态见上方）

- Ubuntu 服务已部署并运行，五卡片保留在真实画布。
- 新鲜反馈：待机、六轴全部失能、夹爪失能、NORMAL、错误码 0、无电机故障。
- `PIPER_MOTION_ENABLED=0`、`PIPER_CAMERA_ENABLED=0`；已停止画布智能控制，开机自启动未勾选。
- 恢复默认权限并重启容器后，未自动使能或重放动作，反馈保持上述状态。
- 实际固件 `S-V1.9-0`；软件示教白名单保持为空。

## Ubuntu 真机已执行

现场人员本轮重新确认底座固定、空间无人无障碍、夹爪空载并可及时停止。
初始化前重新读取状态、生成有效预览，边界修正与确认内容一致。
下列动作经真实 Agent Core 的 `/api/mcp/{id}/call` 路径调用，未用 Mac 服务或模拟 CAN 代替。
状态、模式、夹爪读取、初始化预览、图片导入还通过 Edge 真实画布手动执行；
运动表中的执行记录来自上述画布后台 API，并非逐项通过前端按钮点击。

| 项目 | 实际结果 |
| --- | --- |
| 五卡片与生命周期 | status / mode / arm / gripper / photo 注册完整，项目启动成功；收尾 5/5 卡片停止 |
| 只读反馈 | 全关节、电机、夹爪、模式与固件可读；动作中仍可读，busy / active_card 符合动作状态 |
| 待机 ↔ CAN | 往返切换均 confirmed=true，六轴始终失能；之后再进入 CAN 初始化 |
| 初始化 | SDK 速度 5%；J2 +2.038°、J3 −0.899° 修正完成，六轴使能、无故障 |
| J1–J6 单轴往返 | 每轴目标变化 0.8°，J3 向负方向，其余向正方向；1°/秒、SDK 5%；全部完成后返回各自起点，反馈在 ±0.5° 内 |
| 双轴动作 | J1/J4 同时各 +0.6°，低速完成；其余轴保持 |
| 保存/返回停放位 | 保存六轴操作员停放位，双轴移动后 return_pose 完成；这是本次保存的姿态，非工厂零位 |
| 空载夹爪 | 0.5 N·m，目标 5 mm，反馈 4.6 mm；返回目标 0 mm，反馈 0.1 mm，均在 ±0.5 mm 内 |
| 动作取消 | J1 动作已 running 时 arm.stop，last_action=cancelled；六轴仍使能，无故障，随后保持并返回停放位；停止前反馈未见明显位移，不据此认定制动距离已验收 |
| 独立停止 | arm 动作中 photo.stop 的 operation_stopping=false，arm 仍 busy；随后 arm.stop 才取消该动作 |
| 保持 | arm.hold / gripper.hold 返回正常反馈，不释放力矩 |
| 支撑后失能 | 执行当时另获人员支撑确认；先夹爪后六轴，均读到 enabled=false；释放后角度有变化，已提醒持续支撑或放稳 |
| 收尾切回待机 | 全失能后切回 standby，confirmed=true；随后关闭运动权限、重启并再次读取反馈 |
| 示教门禁 | 预览返回 allowed=false / button_guidance=true；未发送软件示教切换命令 |

初始化完成反馈为 `[35.280, 0, 0, 6.178, 23.865, 3.986]°`。
保存停放位为 `[35.293, 0, 0, 6.199, 23.839, 4.046]°`。
支撑失能、切回待机并重启后的快照为 `[35.326, -2.198, 1.409, 7.863, 23.537, 5.255]°`；
失能后姿态不等于使能时停放位，不自动重新使能纠正。

## 文件图片、DDS 与 ACP

- 按用户要求，本轮没有调用相机、申请相机权限或恢复真实采集。
- 真实画布执行 `import_photo` 导入明确标记的 `TEST_ONLY_NO_CAMERA.png`，640×360。
- 保存 JPEG + JSON；元数据 `captured_at=null`、`camera_accessed=false`、`arm_moved=false`。
- 画布监控页通过 `/agilex/piper/rgb` 实际显示测试图，已保存截图。
  这证明文件导入/保存/显示链路，不算用户实拍照片或真实相机采集验收。
- 长动作返回 action_id，info 可读 completed/cancelled；Agent Core 日志保留 ACP HTTP 200
  及相同 action_id 的 action_complete 事件。未以本次手动调用声称 LLM pending barrier 已验收。
- 发现全局“智能控制”开启时，即使画布没有 decision_core 卡片，ACP 也会触发后台决策请求。
  本轮日志审计未发现 `[decision] tool_call` 记录；这不是总线级零额外命令证明。
  收尾已停智能控制并确认 running=false。后续纯手动验收应单独启动卡片，避免全局开启决策循环。

## 镜像与部署证据

- 实机 Ubuntu 22.04.5 LTS、x86_64，can0 为 1 Mbps、ERROR-ACTIVE；采集时总线错误计数 0。
- 镜像 `agilex-piper:local-amd64-462e1af`，linux/amd64。
- ID：`sha256:86b7d7ef5fe0efa1630185b655e64cbd9fc052d4b68c2fa0fd1a6d45b5fada68`。
- Docker inspect `Size=345244772` 字节；这是当前存储后端的报告值，不代表完整解压占用。
- 普通 Dockerfile 构建访问 Docker Hub 超时。本次以现场原 amd64 镜像离线叠加 Git 归档源码构建；
  不声称完成了从上游基础镜像开始的干净构建。
- 已核对容器内 device.py / arm.py / control.py SHA256 与本地源码一致。
- 依赖：piper-sdk 1.0.0、python-can 4.6.1、numpy 1.26.4、opencv-python-headless 4.11.0.86、PyYAML 6.0.2。
  SDK direct_url 对应官方固定提交 `8d4dc29bd6e753066dedee235c008e90ecb2ab4a`。
- Bot 曾报告 54c401c 构建成功，tag `release.261011.60f4891`，但 manifest 实测 linux/arm64，未部署。
  后续 Bot 构建须重新核对提交和架构，不能用 tag 后缀替代源码证据。
- compose 只更新 PiPER 服务，原配置均有备份；运动/相机门禁最终显式设为 0。

## 离线验证与剩余项目

当前代码指定 pytest 集合：133 passed、42 subtests passed，包含 PiPER、公共生命周期与 Docker COPY。
新增文件导入、相机默认关闭及每次确认都有测试；不满足门禁时断言 0 条模拟控制命令。

尚未完成，不能标为通过：

- 机身按键示教进入/停止/退出及软件示教固件能力验收。
- 真机明显位移后的停止/制动距离、断线、陈旧反馈、动作/切换超时、外部模式切换及重复点击压力场景；目前对应策略由模拟测试覆盖。
- 当前姿态的 Ubuntu D435 单张采集和真实画布显示已验证；不同姿态/照明下的多次采集、用户提供照片的验收仍待完成。
- 干净 amd64 构建、内部 COS 依赖归档和不可变 ROS 基础镜像的正式发布方案。
- Master/老师现场验收、合并和任务凭证。PR 保持草稿，不代发人员消息、不自行合并。

## 原始记录与 Mac 历史

原始记录保存在用户本机 `acceptance/20261011_ubuntu/`，不将登录口令或现场照片提交仓库。
包括 runtime.json、actions.jsonl、small-motion.jsonl、core-completions.log、core-audit.json、构建日志和画布截图。
Mac 历史保留在 `acceptance/20261011_mac/`：已验证首次 CAN 读取、固件与原相机路径；
关闭再打开 USB-CAN 的恢复稳定性尚未通过。历史照片采集发生在用户要求停止相机之前。
Mac 和旧版 J1 演示均不替代本次 Ubuntu 新功能验收。
