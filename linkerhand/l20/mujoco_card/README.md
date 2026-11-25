# L20 轨迹仿真

本卡使用厂家的 G20 模型，对应本文板上验收的 LinkerHand L20-10。仿真服务提供与实手隔离的 `hand_gesture`、`hand_grasp` 实例和 `l20_mujoco`。在「L20 仅仿真」服务里选择卡片。每个实例在配置里选左手或右手；同侧的两个动作输出可以同时连入一个模拟实例。抓取使用当前驱动的空载收拢控制器，手势复用手势和数字的目标及避碰过渡。参数仅影响虚拟手。

启动已连好的实例，执行上游的手势、准备、空手收拢或张开。模拟手播放期间拒绝后来动作。点击模拟卡「查看数据流」在画布内显示实时 JPEG。执行 `view` 打开可旋转的三维工作台，切换实时与回放；支持 1×、0.5×、0.25× 和逐帧拖动。每个实例保留最近一个完整动作；中止动作不会覆盖它。实时画面和回放按同一配色标示五根手指末节掌侧的触觉区，工作台内附图例。着色边界按厂家网格的指腹外形估算，颜色用于识别手指。工作台显示运动学姿态，不提供物体、接触或力传感模拟。

## 接入其他轨迹来源

上游通过画布声明 `topic_out` 格式 `data/l20_joint_trajectory`，消息类型 `std_msgs/msg/String`，可靠传输。模拟卡的 `start` 接收画布传入的单个 `input_topic` 或多个 `input_topics`；只有已连接的话题参与。每个消息携带一次完整且有序的轨迹。以下为一帧的最小例子。

```json
{"schema":"motus.l20.trajectory/1","id":"example-1","side":"right","action":"open","frame_ms":50,"frames":[[255,255,255,255,255,70,200,132,55,0,255,255,255,255,255,255,255,255,255,255]]}
```

`id` 每次动作唯一；`side` 为 `left` 或 `right`，需与接收实例一致。每帧按 L20 的五个指根、五个侧摆、拇指旋转及四个保留位、五个指尖排列，共 20 个 0–255 整数。每次允许 1–600 帧，固定帧间隔为 20–1000 毫秒。第三方 DDS 发布者可以在模拟卡 `info` 的 `rejected` 查看最近一次拒绝原因。及时读取该字段；下一次接受的动作会清除它。

![右手实时画面中的五色指腹](right-hand-preview.jpg)

## 独立服务

在仓库根目录运行 `bash build.sh --mirror none linkerhand/l20_mujoco` 即可通过统一构建入口生成 ARM64 仿真镜像。构建会从厂家下载模型并校验 SHA-256。Driver 清单和部署配置在 [linkerhand/l20_mujoco](../../l20_mujoco/driver.yaml)。物理手使用独立的 `linkerhand/l20` 驱动镜像。

### 已有实手驱动镜像时离线构建

构建目录需要本目录代码、厂家 [L20 模型包](https://urdf-view.linkerbot.cn/api/v1/urdf-models/10/download) 保存为 `vendor.zip`，以及 `wheels/`（Python 3.10 Linux ARM64 的 mujoco 3.2.7、numpy 1.26.4、Pillow 11.3.0 和依赖）。从当前 L20 驱动镜像复用关节与动作代码。运行时加载左右手各自的厂家 URDF 和标定表，按驱动的左手符号修正计算 21 个关节的姿态。

```sh
docker build --build-arg L20_DRIVER_IMAGE="$L20_DRIVER_IMAGE" -t local/l20-mujoco:trajectory .
docker run -d --name motus-l20-mujoco --restart unless-stopped --network host \
  --read-only --tmpfs /tmp --cap-drop ALL --security-opt no-new-privileges \
  -v l20-sim-state:/var/lib/l20-sim local/l20-mujoco:trajectory
```

服务在主机回环地址 `127.0.0.1:15769/mcp` 提供 MCP，通过 UDP 在 ROS 域 42 收发轨迹与预览。容器不挂载设备，仿真进程另外禁止创建 CAN 和原始链路套接字。专用状态卷保存实例绑定和最近完整轨迹。连接同一只手的多个发送源共用接收端的忙状态。
