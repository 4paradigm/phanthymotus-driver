# TRON2 原厂 VR 录制、回放与 VLA 导出

`tron2_teaching` 使用 G1 teach 的公共工作台和动作包服务。录制器只发送关节和夹爪查询，并读取三路 MJPEG。开始录制、停止录制和保存训练数据均不发送运动命令，不进入或退出原厂 VR。

实体回放当前被 `exclusive_control_unverified` 阻塞。已确认的厂家接口没有提供控制租约或可靠的 VR 退出证明。`tele_operation=OK` 不能作为独占证明。代码没有环境变量或页面勾选项绕过该门禁。生产独占适配仍需厂家接口证据；目前只验证了假厂家的回放执行器。

## 数据来源

2026-10-21 在指定 TRON2 上执行有界只读查询，每类 50 次、目标周期 20 ms；每路相机读取 20 帧。它们是独立短采样，不是原厂 VR 示教验收。

| 数据 | 已确认接口与顺序 | 单位和时间 | 短采样结果 |
| --- | --- | --- | --- |
| 双臂、头部 | `request_get_joint_state`；左 7、右 7、头 pitch/yaw | q 为 rad；dq/tau 原样保留；原始 timestamp 的时钟定义未验证 | 47.81 Hz，16 项；names 为空，按官方 SDK 索引绑定 |
| 双夹爪 | `request_get_limx_2fclaw_state`；left/right opening、speed、force | 开口为厂家 0–100 无量纲反馈；不使用未标定毫米值；timestamp 原样保留 | 47.96 Hz，两侧开口反馈可读 |
| 头部 RGB | `/camera/top/color/image_raw/compressed`，厂家端口 59001 | JPEG；只确认接收时间，未取得曝光时间 | 640×480，31.33 Hz |
| 左腕 RGB | `/camera/left/color/image_rect_raw/compressed` | 同头部 | 848×480，31.58 Hz |
| 右腕 RGB | `/camera/right/color/image_rect_raw/compressed` | 同头部 | 848×480，30.03 Hz |
| 原厂控制目标 | 已确认的状态、关节和夹爪查询未提供目标 | 不以实测反馈冒充原厂目标 | 未确认，保存为 null |
| 控制独占 | 只有模式、健康状态及本进程互斥 | 不足以证明其他进程停止 | 实体回放阻塞 |

机上当时运行另一个 `0.10.1-recording-local` 版本，提供 16 关节录制及显示骨架回放。本轮未覆盖、部署或重启该服务。旧自研 `limx/tron2_teleop` 代码及 PR #397 保留，不由新卡片启动。

## 保存内容

默认容器目录为 `/data/tron2-teaching`，可通过部署配置中的 `TRON2_TEACHING_DIR` 指定。部署片段新增独立可写数据挂载，原 home 配置仍只读。

每个 `episodes/<id>/` 保存：

- `manifest.json`：`motus.tron2.episode/1`、设备序列号、固件、URDF SHA、关节顺序、单位、任务、结果、备注、来源与质量。
- `trajectory.jsonl`：关节 q/dq/tau、双夹爪原始反馈、源时间、接收单调时间、接收墙钟时间、序号及原厂目标可用性。
- `head.mjpeg`、`left_wrist.mjpeg`、`right_wrist.mjpeg`：原始 JPEG 包按接收顺序连接，不重新编码。对应 `.jsonl` 保存逐帧序号、时间、字节位置和长度。
- `complete.json`：在所有文件 fsync、清单写入及目录同步之后写入；绑定清单 SHA。清单包含各数据文件的大小和 SHA256。

写盘线程使用有界队列，磁盘余量至少 256 MiB，最长 30 分钟，轨迹上限 64 MiB。溢出、时钟倒退、相机断流或关节断流结束录制并保留失败数据，不自动删除。重复源时间的关节样本不重复写入。三路图像或夹爪缺失时，不能标记为完整训练 episode。

源时间未验证，首版统一在同机单调接收时钟上对齐。不声称曝光同步或硬件同步。

## 工作台和训练导出

1. 在卡片打开工作台并检查设备。
2. 填写任务描述。开始只读录制，再使用原厂 VR 示范。
3. 结束录制，查看质量。可补充成功、失败或未标注结果和备注。
4. 直接导出 LeRobot v3；不要求机器人回放。

固定使用官方 `lerobot==0.4.4`（Apache-2.0）的 `LeRobotDataset.create/add_frame/save_episode/finalize` 和读取接口，输出 H.264 三路视频。导出完成后逐帧读取并解码三路图像，检查 state/action 形状和归一化统计。失败目录保留，已有目录不覆盖。

- `observation.state` / `action`：左 7、右 7、左夹爪、右夹爪；前 14 项 rad，后 2 项厂家无量纲开口。
- 当前真实采集使用 `measured_future_state`：10 Hz 训练帧的下一时刻实测状态作为绝对动作，派生间隔 0.1 秒。末尾没有未来状态时不生成动作。
- 图像选择当前时刻之前最近的帧，最大误差 50 ms；关节只在间隔不超过 100 ms 的相邻样本之间插值。夹爪使用最近过去值，不插值。
- 无效点排除，缺口两侧分成独立训练 episode；不压缩时间后拼成一条运动。
- `observation.head`、`source_time_ns`、`action_time_ns`、结果标签和任务保留在数据集；原始时间、来源和所有 episode 校验信息保存在导出报告及原 episode。
- 数据集目录对应一个来源 episode，只有一个动作来源；不混合派生动作和原厂目标。成功读取不证明训练效果。

也可离线运行：

```sh
python3 -m pip install -r limx/tron2/requirements.txt -r limx/tron2/requirements-vla.txt
python3 limx/tron2/vla_export.py /path/to/episodes EPISODE_ID /path/to/new-dataset
```

## 回放与动作包

轻量回放使用 `motus.tron2.trajectory/1`，不复用 G1 十关节 schema。外层动作包仍是原有两文件 ZIP，不含视频；训练导出直接读取完整 episode。

回放执行器校验文件、模型和设备绑定、限位、时间连续性、0.8 rad/s 速度上限、2° 起点差异及夹爪开口差异。超限直接拒绝，不减速、不自动回起点、不切模式。夹爪速度和力档位必须明确填写，不能从实测开口推断。

头部保持开始时实时位置。控制检查、反馈检查、厂家故障、50 ms 发送迟到和停止都会结束发送；不补发积压帧、不重连自动重放。终态要求连续推进的关节反馈、低速度和双夹爪目标反馈；失败或取消不能声称物理停止。回放中其他本机动作卡也受现有互斥机制约束。

生产独占门禁未通过前，不能完成真实试运行和 Skill 保存。导入动作包只创建待验证草稿；不能通过包内声明绕过本地试运行及效果确认。

## 验证入口与边界

```sh
python3 -m pytest -q limx/tron2/tests tests/test_teaching_package.py tests/test_file_upload_endpoint.py tests/test_dockerfile_copies.py
python3 tests/serve_tron2_teaching.py --root /private/tmp/tron2-browser-test
bash build.sh --mirror none limx/tron2
```

浏览器 fixture 只监听本机 15892，使用真实工作台、HTTP 和文件实现，厂家与相机为假服务。它不是 Core 部署或现场验收。

本轮尚未完成原厂 VR 实际示教采集、真实回放、物理停止、用户效果确认或 VLA 训练。依赖中的 torchvision 视频解码接口给出弃用提示；固定版本目前通过测试，升级官方依赖时须重新验证。
