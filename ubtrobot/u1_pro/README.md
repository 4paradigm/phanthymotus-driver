# 优必选 U1 Pro 驱动

本驱动为 Agent Core 提供 U1 Pro 的音频、相机、动作和系统行为开关卡片。机器人 SDK 控制服务使用 ROS domain `20`，设备音频/相机 topic 使用 domain `2`，Agent Core 使用 domain `42`。

## 卡片

- `mic`：调用 U1 SDK 的音频 `open_stream`/`stream_state`，优先从 SDK 音频共享内存读取；启动时给 SDK ring 1 秒首帧优先窗口，未收到 SDK 帧才切换到设备 domain `2` 的 `/sys/device/audio_in/raw` topic 备用源。驱动按消息采样率、声道和格式转换为统一的 `audio/pcm-16k`，启动时等待首个 PCM 帧；失败会关闭流并报告错误。
- `speaker`：订阅连接到卡片的 `audio/pcm-16k` 输入，优先通过设备 PulseAudio TCP sink 实时播放；`PULSE_SERVER` 环境变量可覆盖默认地址 `tcp:127.0.0.1:34567`。运行镜像中的 `pulseaudio-utils` 提供 `paplay`，设备侧必须开放对应 PulseAudio sink。队列短时满载时施加背压以保留 PCM；播放进程退出或 writer 持续无进展时停止接收并报告错误，避免无限阻塞 ROS 回调。未安装或无法启动 `paplay` 时才尝试已验证可用的 `EnableAudioOut`/`/sys/device/audio_out/raw` 兼容路径；两条路径都不可用时拒绝启动。音量和静音服务不属于本驱动能力。

部署服务使用 `restart: always`，用于主机或 Docker daemon 重启后自动恢复驱动；运维人员手动停止容器后，Docker 会在 daemon 重启时再次启动它。若需临时停用，应通过部署编排管理该服务，而不是依赖 `docker stop` 保持停机。
- `tts`：通过 U1 SDK `play_text` 进行文本播报，支持打断和 ACP 异步完成回报。真机音量服务不纳入本驱动能力。
- `expression`、`head`：通过 SDK `play_action` 播放声明的动作名称。动作提交后以 ACP 关联播放结果；`interrupt`/`stop` 会调用 SDK 中断服务。两张卡共享同一个动作执行队列。
- `system_controls`：统一控制 `wakeup`、`wakeup_followup` 和 `visual_behavior` 三个固件行为开关。
- `idle_actions`：独立控制自动待机动作及自动眨眼，支持 `enable`、`disable`、`status`，设置由厂商配置服务持久保存。
- `camera_left`、`camera_right`：打开 SDK video stream，并分别订阅 domain `2` 的 `/sensor/camera/left_eye/color/raw`、`/sensor/camera/right_eye/color/raw`，转换后输出 JPEG。SDK 共享内存视频流没有选择左右眼的参数；只有元数据明确标记了对应眼睛时，才会将环形缓冲帧作为该眼的备用输入，避免把一幅画面伪装成左右两路。

## 启动与行为控制

启动时先检查/完成 SDK 授权，然后依次关闭并读取确认 `wakeup`、`wakeup_followup` 和 `visual_behavior`。任一关闭请求失败或读回状态不符，驱动启动失败，不会注册卡片。需要使用时，可通过 `system_controls` 单独或批量重新开启。

这些开关不能停止固件内部 Agent 进程、ROS 服务、安全控制环或已经提交的动作。关闭 `visual_behavior` 用于停止固件自主视觉行为，不会屏蔽驱动显式提交的 expression/head 动作；显式动作可通过对应卡片的 `interrupt` 中断。

授权使用只读挂载的 `U1_PRO_AUTH_FILE` JSON 和同目录 license 文件，或受保护的环境变量。认证文件不得提交到仓库或写入镜像。

### 自动待机动作卡片 `idle_actions`

此开关使用厂商原生 ROSA 配置服务，而 `system_controls` 的三个开关走公开 SDK，且在驱动启动时自动关闭。因此单独新增 `idle_actions`，避免把持久的待机偏好混入 SDK 开关和原有 `all` 操作。驱动、卡片的启动和停止均不改写待机偏好，只有显式调用 `enable`/`disable` 才会修改。

在 `idle_actions` 卡片上调用：

| 用途 | 输入 |
| --- | --- |
| 关闭自动待机动作 | `{"action":"disable"}` |
| 开启自动待机动作 | `{"action":"enable"}` |
| 查询当前设置 | `{"action":"status"}` |

`status` 返回 `{"ok":true,"code":"OK","data":{"enabled":true},"source":"vendor_user_config"}`。其中 `enabled` 是持久配置中的开关值，不是电机是否静止的实时反馈。修改成功返回 `ok`、`requested`、`enabled` 和读回的 `state`；服务拒绝、超时、读回失败或状态不一致均报错。超时后先查询状态，再决定是否重试。

关闭会让当前固件的循环待机分支和自动眨眼分支退出，覆盖 `S017/S018/S019/S007` 待机动作及 `S020/S032/S033` 交互眨眼。固件仍可能执行待处理的一次性 `Z001` 归位，`expression`/`head` 显式请求的动作也仍可执行。这张卡片不提供急停。

调用链为：驱动 → `nsenter` 进入宿主命名空间 → 宿主 Docker → 已运行的 `walker-behavior.behavior-1` → `/sys/config/user_manager/set_config`（`config_msgs/srv/SetConfig`），仅提交 `idle_action_enabled` 布尔值。ROSA 使用 `--no-daemon` 发现服务；收到业务成功响应后，从厂商 `/data/user_data/user_config_center.json` 读回核验。驱动不直接改写该文件，也不重启厂商程序。

该接口依赖当前固件的容器、服务和配置结构，公开 SDK domain `20` 没有对应接口。容器名可在 `config.yaml` 的 `idle_actions.native_container` 中调整。部署沿用现有的 `privileged: true`、`pid: host`、host network/IPC，并由 `util-linux` 提供 `nsenter`；宿主需有 `/usr/bin/docker`，厂商容器内需有 `python3`、`timeout`、`/opt/walker/setup.bash` 和 ROSA。缺少这些依赖时，该卡片操作会报错。

## 运行环境

部署需要 host network、domain `20`/`2`/`42` 所需的 ROS 访问，以及对 `/tmp/robo/ipc` 和 `/dev/shm` 的运行时挂载。`/tmp/robo/ipc` 是临时运行态，不应作为持久配置目录；服务端每次打开流都会重新创建共享内存文件。容器需要 ROS Humble 的 CycloneDDS RMW。Docker 构建阶段的 colcon/CMake/C++ 工具仅用于编译本地 ROS 接口；最终镜像还包含运行期 Pillow、NumPy、PyYAML 和 CycloneDDS 依赖。

## 本地检查

```bash
PYTHONPATH=ubtrobot/u1_pro python3 -m unittest discover -s ubtrobot/u1_pro -p 'test_*.py' -v
python3 scripts/check_service_yml.py ubtrobot/u1_pro
```
