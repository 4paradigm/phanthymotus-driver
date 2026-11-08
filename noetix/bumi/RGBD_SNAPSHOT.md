# Bumi `vision_capture.capture_rgbd`

初始开发基线为官方 upstream/main `d97045825a685d4a81176a59b53f8d3a9803e744`。
PR #498 的合并基线为 `814b3ca953d33eba0aca9a61dcefa773f5da483b`。
独立 worktree：`D:/university/phanthymotus-driver-rgbd`，分支 `refactor/bumi-rgbd-snapshot`。
只修改本 Driver；ActiveSearch 原型、原工作区、主仓库审查代码及补丁均未修改。

## Card 输入输出契约

通过现有 MCP `vision_capture` 工具调用，不新增 Card，不修改端口或 Bundle 注册名称。

```json
{"name":"vision_capture","arguments":{"action":"capture_rgbd","timeout_s":5,
 "require_exposure_after_request":false}}
```

timeout_s 是有限数值，范围0.1–10秒，默认5。严格曝光参数必须是布尔值。
已受理的调用立即返回以下对象。`action_id` 和 `request_id` 使用同一个 UUID。

```json
{"ok":true,"state":"queued","action_id":"<32hex>","request_id":"<32hex>"}
```

后台 worker 将最终结果作为 `result`，通过 `POST /api/acp/complete` 发送一次终态。
成功使用 `completed`，采集或保存失败及超时使用 `error`，取消并完成清理使用
`cancelled`。`x-completion` 同时声明快照和录像，保留录像原有的
`max_duration_s + 15` 秒等待上限（默认 45 秒，最短 16 秒）。
HTTP 回调失败时 Driver 记录日志。`info.rgbd_snapshot.last_completion` 保留当前进程中
最近一个终态及完整结果，供调用者核对。后续快照完成会替换此记录。

采集等待和常规保存共用单调时钟预算，每次接收轮询最多 20 ms，控制发送超时 50 ms。
普通磁盘系统调用无法由 Python 安全中断。慢磁盘期间 `stop` 仍能设置取消，
取消获胜时禁止成功返回并清理该请求。当前请求退出前维持 `BUSY`。

成功示例（路径以实际配置为准）：

```json
{"ok":true,"request_id":"<32hex>","guarantee_level":"B",
 "exposure_after_request_verified":false,
 "directory":"/opt/phanthy-motus/data/vision_capture/rgbd/<32hex>",
 "files":{"rgb.ppm":".../rgb.ppm","depth.npy":".../depth.npy",
          "calibration.json":".../calibration.json","metadata.json":".../metadata.json"}}
```

终态失败结果包含 `ok=false, request_id, code`，通常附 `message`。
保存失败时附 `cleanup_errors`。主要运行时错误包括 CAMERA_UNAVAILABLE、
CAMERA_DISCONNECTED、CAMERA_CHANNEL_ERROR、CAMERA_RESTART、CLOCK_DOMAIN_CHANGED、
CLOCK_DISCONTINUITY、PAIR_DOMAIN_MISMATCH、PAIR_SKEW、INVALID_PAIR、INVALID_PIXELS、
INVALID_CALIBRATION、TIMEOUT 和 CANCELLED。BUSY、INVALID_TIMEOUT、INVALID_ARGUMENT
及 EXPOSURE_UNVERIFIABLE 在受理前同步返回，无 `action_id` 或 ACP 回调。
严格曝光请求直接拒绝，且不会发起采集或保存。

`stop` 取消当前快照并保留原有录像取消行为。`rgbd_cancel_requested` 仅表示
本次设置了取消标志，清理结果由终态报告。快照单飞，旧 RGB 照片和录像保持原有行为。
Agent Core 的普通工具调用可能等待 ACP 屏障。已注册的系统中断
`on_interrupt_all` 通过 `hooks.fire → call_tool_direct → vision_capture.stop`
绕开屏障；本地验证覆盖了从 `hooks.fire` 开始的路径。

### ACP 顺序限制

MCP handler 写出受理响应后解除 Driver 本地回调门，等待设有上限，无固定延时。
Agent Core 解析响应后才登记 Pending。目前正式接口没有 Pending 登记确认，
`/api/acp/complete` 即使没有匹配项也会返回 `ok: true`，且不会缓存该终态。
因此，先写出 MCP 响应仍可能遇到 Core 尚未解析响应的情况。这个场景下快照文件和
Driver 的最近终态仍可查询，Core 的 ACP 屏障则只能等自身超时或被中断。

上游可在 Core 按 `action_id` 有界缓存可信的早到完成事件，在登记 Pending 时消费，
并对已完成或已回收的 ID 去重；或者在登记后发出明确的 ACK，让 Driver 在收到 ACK
后才发送完成事件。仅延时或依据 HTTP 200 无法保证时序。

## 唯一 Pipeline 与请求流程

```text
tools/call -> 原 Bundle.dispatch -> VisionCapturePlugin -> SnapshotCapture
  -> CameraPlugin.capture_rgbd -> inherited socketpair -> 相机采集子进程
  -> 请求ID + producer-side设备帧基线 -> 后续frameset -> 原rs.align结果
  -> 校验 + 有界像素复制 -> 有界发送线程 -> 后台任务 -> 原子发布 -> ACP 终态
```

CameraPlugin 仍只启动一个相机子进程；子进程仍只有原来的 rs.pipeline。
本地 socketpair 不监听 TCP 端口，只把一个 FD 继承给相机子进程；生产目标为 Linux/POSIX。
此 FD 传递已在本地 WSL 的真实 Python 子进程中用 Fake frameset 验证，未在 ARM64/ROS 环境实测。

新 SnapshotProducer 与现有 RawCameraPublisher 分支并列，不访问 raw_enabled、RTAB-Map 或 VO 配置。
每帧只记录小型来源元数据；只有存在请求时才复制像素。单项请求结束后 pending 清空；
取消发送匹配 request_id 的撤销消息，若通道失效则由截止时间租约回收。
VO 停止不会关闭快照通路；快照结束/取消不会改变 VO 开关。无需启动 RTAB-Map。
原 JPEG、压缩深度和 VO 输出都沿用原路径。快照发送在单独线程，队列容量1，慢接收者不会无限缓存。
快照分支错误隔离，不关闭相机媒体；源相机退出时所有等待者失败，而非使用旧缓存成功。

## 身份、配对和实际保证

复用 visual_inputs.camera_metadata 获取 SDK 原始 source_ms、frame_number、domain、depth_scale。
从相机实际活动设备读取序列号作为 source；epoch 标识当前快照采集时间线。
不用 _CameraFrameNode.frame_sequence，不把 ROS Header.frame_id 当采集 ID。

请求在父进程建立 UUID 与单调时间边界，子进程处理后记录 admission 时间并复制最近设备帧基线。
若还没有基线，先观察一组只作基线，等待下一组；不会把首个缓存返回当新帧。
选中数据必须：

- 与基线的 source/epoch 一致；RGB和深度各自设备帧号、设备时间严格推进。
- 两流时间域一致且属于已知 SDK 时间域；设备时间偏差不超过20ms。
- 原始与对齐后各流帧号对应，像素来自此次同一 source/aligned frameset 调用。
- RGB为uint8 BGR，深度为uint16 Z16，尺寸与当前RGB内参一致，内参有限且焦距为正。
- 请求 ID、生产者接纳时间、选中帧的主机观察时间及响应接收时间相互关联。

RGB和Depth设备帧号不要求彼此数值相同；它们是各流计数器。
重复帧继续等待；计数/时间回退、时间域改变或采集 epoch 改变会拒绝当前请求。检测到设备计数、时钟回退或时钟域改变时，后续请求也会失败，直到重启相机进程。这样迟到的旧设备帧不会被标为新 epoch。
设备时间增量与主机单调经过时间相差超过1秒时拒绝为 CLOCK_DISCONTINUITY，并停用该进程的快照取样，直至相机进程重启。
这是保守连续性诊断阈值，不是时钟标定或曝光误差界。

成功提供 A：请求之后收到关联响应；以及 B：设备帧相对于生产者请求基线推进且配对经过检查。
**不提供 C：不能证明两传感器曝光发生在请求边界之后。**
设备/SDK缓冲仍可能包含请求前曝光的帧；对齐和20ms偏差也不代表两传感器同时曝光。
source_ms 保持“SDK时间戳”的原始语义；received_ns/received_monotonic_ns 是主机观察时刻，
saved_wall_time_ns 是保存元数据阶段时刻，均不冒充曝光时间。
没有引入 device-to-host 时间映射，也没有用到达偏移近似来宣称曝光资格。

## 保存格式与完整性

- rgb.ppm：标准P6二进制RGB，像素无损，BGR转换RGB；不增加JPEG/PNG编解码依赖。
- depth.npy：小端uint16原始Z16，`米 = 数值 × metadata.depth_unit_m`，值0是无效像素。
  没有用彩色深度可视化图代替真实距离数据，保留原非零测量值。
- calibration.json：实际活动RGB profile内参、畸变模型/系数及depth_alignment声明；不伪造机身外参。
- metadata.json：请求、基线、选中源帧、时钟语义、配对偏差、保证等级和文件字节数/SHA256。

文件先写入本次唯一的隐藏 `.UUID.partial` 目录，全部写完才在同一文件系统改名为UUID目录。
成功响应只指向完整目录。失败/取消只清理本次新建文件；已有同名目录不删除。
这是完成时可见性保证，不声称断电持久化：当前没有 fsync/目录恢复日志。
异常无法清理时返回 cleanup_errors，不把残留半成品报告为成功。
未增加文件历史/下载协议；返回文件路径沿用原 vision_capture 风格，容器文件访问仍需平台原机制。

检查保存结果（标准Python + 已有numpy）：

```python
import hashlib, json, pathlib, numpy as np
p = pathlib.Path('/path/to/rgbd/REQUEST_ID')
m = json.loads((p/'metadata.json').read_text())
for name, expected in m['files'].items():
    data = (p/name).read_bytes()
    assert len(data) == expected['bytes']
    assert hashlib.sha256(data).hexdigest() == expected['sha256']
z16 = np.load(p/'depth.npy', allow_pickle=False)
valid = z16 != m['invalid_depth_value']
metres = z16.astype(float) * m['depth_unit_m']
assert m['guarantee_level'] == 'B' and not m['exposure_after_request_verified']
```

## 回归与尚未验证事项

新增测试覆盖真实 Bundle/Card 路由、真实socket及Linux子进程FD继承、Fake source/aligned frameset、
真实NPY/PPM/JSON保存与校验；共享相机循环测试分别停止VO和快照，确认媒体及另一消费者继续。
旧拍照/录像调用与ACP契约有回归；视频编码器使用Fake，未验证真正ffmpeg编解码。
Windows全量首轮7项失败来自既有POSIX路径/shell测试，未删除断言；转本地Ubuntu后原项通过。
最终命令与结果见交付报告；本机WSL具备python3.10、numpy、pytest、PyYAML。

```powershell
wsl -d Ubuntu-22.04 --cd /mnt/d/university/phanthymotus-driver-rgbd -- env PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_bumi_rgbd_snapshot.py tests/test_bumi_bundle.py tests/test_bumi_camera_specs.py tests/test_bumi_loco_servo.py tests/test_bumi_odom.py tests/test_bumi_imu_distance.py tests/test_bumi_leg_odometry.py tests/bumi_visual_odometry noetix/bumi/tests -q -p no:cacheprovider
```

与 #471：仅复用main已有camera_metadata；未复制未合并的CaptureTimeMapper/metric-depth代码。
本功能不产生竞争的时钟映射。将来 #471 改动同一相机循环时可能有文本冲突，需要整合并回归，
不能以合并文件代替请求边界验证。#181健康/点云不作为依赖，也没有重新打开设备。
visual_inputs、visual_odometry_contract、camera_specs及现有VO生命周期无需修改。

真机前还需验证：RealSense对齐帧身份的实际保留、SDK/固件时间语义、设备重启与断流、
ARM64 FD继承和Socket吞吐、请求耗时、30fps预览及VO负载影响、慢盘/满盘清理、平台文件读取。
若未来要求C级，还需传感器曝光元数据、可信跨域映射及误差上界；本版严格请求始终拒绝。
没有连接真机、控制运动、修改dry_run、启动RTAB-Map或部署Docker；本地通过不能代替硬件验收。

### 初版回归记录

Ubuntu-22.04/WSL、Python3.10.12、pytest6.2.5、numpy1.21.5：上述完整命令 396 passed in 58.26s，0失败、0跳过。新增两个测试文件合计22项，原有相关回归374项。Python3.10 AST检查四个新增/修改Python文件通过，git diff --check通过。没有执行真实相机/ROS/Docker部署或真实ffmpeg编码。


取消包若发送失败，父进程关闭快照通道，避免残留半包污染下一次请求。快照通道故障后需重启相机进程恢复。MCP 并发取消由真实本地 HTTP handler 验证。额外 ACP 契约测试需要将
`BUMI_ACP_CORE_SOURCE` 指向官方 `agent-core/src`，并安装 `aiohttp` 和
`jsonschema`。测试基于官方提交
`990257f881af540a1949237678b71dceaab5ab1c`，运行实际 `mcp_client.py`、
`hooks.py` 以及 `start.py` 中的完成端点函数，HTTP 入口使用本地测试服务。
测试刻意暂停 MCP JSON 解析，复现 HTTP 回调成功但迟建的 Pending 未被释放的情况；
先登记的正常路径和中断钩子路径则验证了 Core 屏障确实解除。
完整 Agent Core 服务的启动、鉴权及真实设备部署仍待验证。

本轮 ACP 修订使用上述完整 Bumi 命令，额外设置 BUMI_ACP_CORE_SOURCE 指向固定提交源码，
并通过 PYTHONPATH 加载隔离的 HTTP 测试依赖。Ubuntu-22.04/WSL 上定向测试 29 项通过，
完整回归 405 项通过、0 失败、0 跳过。Python AST、Card JSON Schema、ACP 声明及 Docker COPY
静态检查均通过。
