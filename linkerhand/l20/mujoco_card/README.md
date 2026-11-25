# L20 轨迹仿真

本卡使用厂家的 G20 模型，对应本文板上验收的 LinkerHand L20-10。从同一个 LinkerHand L20 Driver 中选择 `hand_gesture_sim`、`hand_grasp_sim` 和 `l20_mujoco`。带 `_sim` 的动作卡仅驱动虚拟手；`hand_gesture` 和 `hand_grasp` 面向实手。每个实例在配置里选左手或右手；同侧的两个动作输出可以同时连入一个模拟实例。抓取使用当前驱动的空载收拢控制器，手势复用手势和数字的目标及避碰过渡。参数仅影响虚拟手。

启动已连好的实例，执行上游的手势、准备、空手收拢或张开。模拟手播放期间拒绝后来动作。点击模拟卡「查看数据流」在画布内显示实时 JPEG。执行 `view` 打开可旋转的三维工作台，切换实时与回放；支持 1×、0.5×、0.25× 和逐帧拖动。每个实例保留最近一个完整动作；中止动作不会覆盖它。实时画面和回放按同一配色标示五根手指末节掌侧的触觉区，工作台内附图例。着色边界按厂家网格的指腹外形估算，颜色用于识别手指。工作台显示运动学姿态，不提供物体、接触或力传感模拟。

## 接入其他轨迹来源

上游通过画布声明 `topic_out` 格式 `data/l20_joint_trajectory`，消息类型 `std_msgs/msg/String`，可靠传输。模拟卡的 `start` 接收画布传入的单个 `input_topic` 或多个 `input_topics`；只有已连接的话题参与。每个消息携带一次完整且有序的轨迹。以下为一帧的最小例子。

```json
{"schema":"motus.l20.trajectory/1","id":"example-1","side":"right","action":"open","frame_ms":50,"frames":[[255,255,255,255,255,70,200,132,55,0,255,255,255,255,255,255,255,255,255,255]]}
```

`id` 每次动作唯一；`side` 为 `left` 或 `right`，需与接收实例一致。每帧按 L20 的五个指根、五个侧摆、拇指旋转及四个保留位、五个指尖排列，共 20 个 0–255 整数。每次允许 1–600 帧，固定帧间隔为 20–1000 毫秒。第三方 DDS 发布者可以在模拟卡 `info` 的 `rejected` 查看最近一次拒绝原因。及时读取该字段；下一次接受的动作会清除它。

![右手实时画面中的五色指腹](right-hand-preview.jpg)

## 构建与部署

在仓库根目录运行 `bash build.sh --mirror none linkerhand/l20`。同一 ARM64 镜像、同一个 `embodied-linkerhand-l20` 容器提供所有卡片。由既有的 L20 Driver 在 `15734/mcp` 注册一次，Core 自动发现实手工具和三个虚拟工具。部署配置见[service.yml](../deploy/service.yml)。离线机器可以通过 `docker image save` 和 `docker image load` 传递此镜像。

虚拟动作由容器内的低权限进程执行。驱动将三个虚拟工具的调用转发到本机回环端口 15769。该进程的有效 Linux capabilities 为零，启用 no-new-privileges，并通过 seccomp 阻止 CAN 和原始链路套接字。仿真进程异常退出时驱动单独重启它。实例状态存储在已挂载的数据目录 `/opt/phanthy-motus/data/l20-mujoco`。部署沿用平台只读 DDS 配置、域 42 以及 10 MB × 3 的 Docker 日志轮转。

从早期独立仿真镜像迁移时，先结束虚拟播放、备份仿真状态，停止占用 15769 的旧仿真容器，再部署统一镜像。若旧版本使用命名卷，需把卷中的 `instances.json` 复制到上述状态目录；旧数据的原始实例编号需要匹配才可恢复。画布上从同一 L20 服务添加三个虚拟工具，检查左右手配置并重新连接轨迹。如果实手正在持物，应先接稳物品再切换镜像。

### 镜像依赖与大小

同一 ARM64 环境使用 `docker image inspect --format '{{.Size}}'` 测得 ROS 基础镜像约 261.6 MB，原实手镜像约 262.8 MB，统一镜像约 296.3 MB，较原实手镜像增加约 33.5 MB（十进制）。MuJoCo 3.2.7 用于运动学，Pillow 11.3.0 用于实时 JPEG；两个进程共享基础镜像中已有的 NumPy、PyYAML 和 ROS 2。

左右手网格、URDF 和标定表来自[厂家 G20 模型包](https://urdf-view.linkerbot.cn/api/v1/urdf-models/10/download)，压缩包约 16.3 MB，构建时和加载时均校验 SHA-256。来源和许可见[模型来源说明](../resource/SOURCE.md)。取得项目 COS 上经过校验的同一文件地址后，可将 `L20_MODEL_URL` 指向该地址。
