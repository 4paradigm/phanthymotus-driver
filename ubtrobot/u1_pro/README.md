# 优必选 U1 Pro 驱动

本驱动为 Agent Core 提供 U1 Pro 的音频、相机、动作和系统行为开关卡片。机器人 SDK 控制服务使用 ROS domain `20`，设备音频/相机 topic 使用 domain `2`，Agent Core 使用 domain `42`。

## 卡片

- `mic`：调用 U1 SDK 的音频 `open_stream`/`stream_state`，优先从 SDK 音频共享内存读取；启动时给 SDK ring 1 秒首帧优先窗口，未收到 SDK 帧才切换到设备 domain `2` 的 `/sys/device/audio_in/raw` topic 备用源。驱动按消息采样率、声道和格式转换为统一的 `audio/pcm-16k`，启动时等待首个 PCM 帧；失败会关闭流并报告错误。
- `speaker`：订阅连接到卡片的 `audio/pcm-16k` 输入，优先通过设备 PulseAudio TCP sink 实时播放；`PULSE_SERVER` 环境变量可覆盖默认地址 `tcp:127.0.0.1:34567`。运行镜像中的 `pulseaudio-utils` 提供 `paplay`，设备侧必须开放对应 PulseAudio sink。队列短时满载时施加背压以保留 PCM；播放进程退出或 writer 持续无进展时停止接收并报告错误，避免无限阻塞 ROS 回调。未安装或无法启动 `paplay` 时才尝试已验证可用的 `EnableAudioOut`/`/sys/device/audio_out/raw` 兼容路径；两条路径都不可用时拒绝启动。音量和静音服务不属于本驱动能力。

部署服务使用 `restart: always`，用于主机或 Docker daemon 重启后自动恢复驱动；运维人员手动停止容器后，Docker 会在 daemon 重启时再次启动它。若需临时停用，应通过部署编排管理该服务，而不是依赖 `docker stop` 保持停机。
- `tts`：通过 U1 SDK `play_text` 进行文本播报，支持打断和 ACP 异步完成回报。真机音量服务不纳入本驱动能力。
- `expression`、`head`：通过 SDK `play_action` 播放声明的动作名称。动作提交后以 ACP 关联播放结果；`interrupt`/`stop` 会调用 SDK 中断服务。两张卡共享同一个动作执行队列。
- `system_controls`：统一控制 `wakeup`、`wakeup_followup` 和 `visual_behavior` 三个固件行为开关。
- `camera_left`、`camera_right`：打开 SDK video stream，并分别订阅 domain `2` 的 `/sensor/camera/left_eye/color/raw`、`/sensor/camera/right_eye/color/raw`，转换后输出 JPEG。SDK 共享内存视频流没有选择左右眼的参数；只有元数据明确标记了对应眼睛时，才会将环形缓冲帧作为该眼的备用输入，避免把一幅画面伪装成左右两路。

## 启动与行为控制

启动时先检查/完成 SDK 授权，然后依次关闭并读取确认 `wakeup`、`wakeup_followup` 和 `visual_behavior`。任一关闭请求失败或读回状态不符，驱动启动失败，不会注册卡片。需要使用时，可通过 `system_controls` 单独或批量重新开启。

这些开关不能停止固件内部 Agent 进程、ROS 服务、安全控制环或已经提交的动作。关闭 `visual_behavior` 用于停止固件自主视觉行为，不会屏蔽驱动显式提交的 expression/head 动作；显式动作可通过对应卡片的 `interrupt` 中断。

授权使用只读挂载的 `U1_PRO_AUTH_FILE` JSON 和同目录 license 文件，或受保护的环境变量。认证文件不得提交到仓库或写入镜像。

## 运行环境

部署需要 host network、domain `20`/`2`/`42` 所需的 ROS 访问，以及对 `/tmp/robo/ipc` 和 `/dev/shm` 的运行时挂载。`/tmp/robo/ipc` 是临时运行态，不应作为持久配置目录；服务端每次打开流都会重新创建共享内存文件。容器需要 ROS Humble 的 CycloneDDS RMW。Docker 构建阶段的 colcon/CMake/C++ 工具仅用于编译本地 ROS 接口；最终镜像还包含运行期 Pillow、NumPy、PyYAML 和 CycloneDDS 依赖。

## 本地检查

```bash
PYTHONPATH=ubtrobot/u1_pro python3 -m unittest discover -s ubtrobot/u1_pro -p 'test_*.py' -v
python3 scripts/check_service_yml.py ubtrobot/u1_pro
```

## system_health：只读系统健康

`system_health` 是单实例 `sensor` 卡片。`start` 启动一个后台采样线程，立即进行首轮采样，
之后按 `sample_interval_seconds: 1.0` 每 **1 秒**持续采样；`stop` 唤醒等待并回收线程。
`robot_info` / `diagnose` 只返回最近一次采样结果的副本，查询不会额外采样；`info` 返回
运行状态、采样间隔和累计采样次数。驱动按既有生命周期启动该卡片，无需反复查询来维持监测。

未启动或等待首个样本时返回未知；停止后可读取最后样本，但明确标记监测停止，不能表示当前健康。
结果增加 `sample_age_seconds`、`samples_collected`、`sample_interval_seconds`；
运行中样本年龄达到两个采样间隔（默认 2 秒）时，监测状态标为 `stale`，整体结果不再正常。
重新启动会清空上轮缓存与计数。单次读取超过间隔时不重叠采样，实际周期会延长；
采样异常记录为不可读取并在后续周期继续尝试。采样遵循单调时钟，系统时间仍用于返回时间戳。

每轮系统指标读取在独立 Python 进程中执行，只导入标准库，避免在已初始化 ROS 的进程中
直接 fork 后执行 Python 读取逻辑。全轮读取预算为 `probe_timeout_seconds: 0.8` 秒，
包含文件枚举、温度、`/proc`、cgroup 和文件系统容量读取；超时会终止辅助进程并回收。
全轮失败时系统指标标为不可读取，相机和播放快照仍独立返回，后续周期继续尝试。

重复启动不会创建第二个采样线程。停止会取消辅助读取、回收进程，并最多等待采样线程 2 秒。
若内核不可中断 I/O 使已发送终止信号的辅助进程仍未退出，或采样线程尚未退出，返回
`stopping`，保留引用并拒绝重启，避免堆积进程或重叠采样；停止请求后不发布在途结果。
卡片没有 ROS 节点或订阅，不启动相机、麦克风或播放任务，不调用 SDK 服务，
不改变 `system_controls` 的唤醒或视觉开关。配置 `plugins.system_health.enabled: false`
可从工具列表移除这张卡片，已有卡片保持原有装配方式。

| 检查 | 数据来源与边界 |
| --- | --- |
| CPU 负载 | `/proc/loadavg` 的 1/5/15 分钟负载和 `/proc/stat` 可见核数；这是 Linux 负载，不是 CPU 利用率，也未按容器 CPU quota 归一化 |
| 内存 | `/proc/meminfo` 的 MemTotal / MemAvailable；单独返回进程 cgroup 内存使用量和可见限制，不能用 `/proc` 总量代替容器限制 |
| 内存限制 | 解析 `/proc/self/cgroup`、`/proc/self/mountinfo`，支持 cgroup v1/v2，并检查可见上级的使用率；容器或隐藏上级时有效限制保留未知，`limit_bytes: null` 只表示可见层级未发现有限限制 |
| 磁盘 | `statvfs(disk_path)` 的当前挂载文件系统总量和当前用户可用空间；容器根目录可能是 overlay，不代表宿主机磁盘清单或配额 |
| 温度 | `/sys/class/thermal/thermal_zone*/temp`、`/sys/class/hwmon/hwmon*/temp*_input`；保留每个来源和标签，读取失败逐项报告，硬件归属未经确认，不能宣称是 CPU 或电机温度 |
| 左右眼相机 | 现有 EyeCameraPlugin 的运行标记、有效 JPEG 发布计数、本地接收时间、当前错误和最近错误；不打开相机或等待新帧，重启会话不复用旧帧的新鲜度 |
| 播放 | 现有 AudioPlugin 的共享 TTS/expression/head 队列、任务类型和关联 ID、排队年龄、最近播放事件和错误；只覆盖驱动管理的任务，不能判断固件自主播放或实际是否发声 |

部署片段通过 `U1_PRO_HEALTH_CONTAINER=1` 明确标记容器观察环境，不额外挂载宿主机
`/proc`、`/sys`，不扩大硬件访问权限。非容器标记环境也仅报告 `process_visible_host_unconfirmed`，
不自动认定这是机器人计算板。无 Linux 接口的开发电脑会返回不可读取，仍可完成其余检查。

结果含 `overall`、`complete`、`checks`、`problems`、`sampled_at` 和实际 `policy`。
每项含 `status`、`value`、`source`、`scope`、`sampled_at`（Unix 秒）和 `reasons`。
状态分别为 `ok`、`warning`、`error`（异常）、`stale`（数据过期）、
`not_started`（未启动）、`unreadable`（无法读取）、`unknown`（能读到值但含义/边界未知）。
有告警/异常/过期时整体为 `warning`；没有告警但存在未知、无法读取或未启动时为 `unknown`；
只有完整且所有项正常才为 `ok`。存在设备异常（`error`）、未知、无法读取、过期或未启动时，
`complete` 为 `false`，逐项原因仍保留。

配置阈值是可调整的观察策略，不是厂商安全限值：CPU 每可见核负载 ≥ 1.0、内存和磁盘
使用率 ≥ 90% 告警；相机有效帧年龄 ≥ 5 秒、共享播放任务未完成 ≥ 120 秒标为过期。
播放过期不代表确认播放失败，长任务可调整此窗口。温度默认不设阈值；设置
`temperature_warn_celsius` 前应核对真机传感器及适用温度策略，即使配置了阈值，
传感器硬件归属仍保留未知。最近错误保留最后一次及发生时间，不因健康查询清空；
相机恢复发布后当前错误清除、历史错误仍可见，播放最近错误保持异常提示直到驱动重建。

第一版不检查电池、IMU、关节、电机温度，也不确认 ROS 远端或厂商接口的健康。
Dockerfile 包含 `system_health.py`；新增实现仅使用 Python 标准库，无新增运行依赖。
可复现的本地检查和真机验证边界见 [SYSTEM_HEALTH_VALIDATION.md](SYSTEM_HEALTH_VALIDATION.md)。
