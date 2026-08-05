# G1 FAST-LIVO2 shadow

本目录当前可运行 N2 的 LIO shadow：纯传感器 sidecar 从 G1 原始 DDS 发布标准化 MID360 点云/IMU，FAST-LIVO2 输出里程计与注册点云；不连接 EGO、不连接运控。N3 已增加显式、受控的 PCD 持久化入口，地图加载与全局重定位仍是下一道独立门禁。

对 Agent 只暴露既有 `controlled_spatial` 卡片，不新增传统导航专用工具。卡片名称、描述、14 个 action、字段和 `x-action-params` 统一由 `../controlled_spatial_contract.py` 生成；原生 SLAM 与传统导航后端必须遵守同一契约。`navigate_to_tag` / `navigate_to_pose` 保持非阻塞，调用方必须在同一轮继续调用 `wait_navigation_done`，其默认 `stall_timeout` 为 90 秒。

## 边界

- FAST-LIVO2 采用尚未合并的 ROS2/MID360 PR #318，固定为 `source-lock.env` 中的完整 commit。
- FAST-LIVO2（GPL-2.0）和 Vikit（GPL-3.0）在独立镜像内构建，不把上游源码复制进驱动镜像。
- Compose 同时启动 `sensor-bridge` 和 `fast-livo2`，二者都使用显式 `lio-shadow` profile、非特权、只读根文件系统、`cap_drop=ALL`、`restart: no`，没有 `/dev`、host PID、运动话题或设备写权限。
- `sensor-bridge` 只运行 `navigation_sensor_bridge_main.py`，不进入 G1 driver 主入口，不创建 MCP 服务或运动插件，也不替换现有 `embodied-unitree-g1`。
- G1 的 MID360 固定倒装。桥接层对点云和雷达内置 IMU 同时应用 `Rx(pi)`，不做随姿态变化的重力对齐；两路相对外参仍为单位阵。北京 G1 双 IMU 静态数据验证后的重力方向夹角为 `1.34°`，上海 G1 仍须复测。
- 锁定分支即使 `img_en=0` 也会构造 VIO，所以配置中有一组只为满足启动契约的 Pinhole 占位值。它不是 D435i 标定，不得用来开启视觉融合。
- 上游帧名仍为 `camera_init -> aft_mapped`；其 `/tf` 被隔离到 `/<ns>/navigation/tf_lio_raw`。N4 前由地面适配层转换为冻结的 `map -> odom -> base_link -> livox_frame`，N2 不向全局 `/tf` 注入未验帧。

验证顺序是“有合适 G1 就优先真机”。暂无合适机器人时，远端算力环境只做编译、单测和录包回放初验，不作为正式运行位置或真机验收替代。

## 本地构建检查

在本目录运行：

```bash
docker build \
  --platform linux/arm64 \
  -t phanthy-g1-driver:navigation-sensors-rx180 \
  -f ../Dockerfile \
  ..

docker compose \
  --env-file source-lock.env \
  -f compose.shadow.yml \
  --profile lio-shadow \
  config

docker compose \
  --env-file source-lock.env \
  -f compose.shadow.yml \
  --profile lio-shadow \
  build fast-livo2
```

构建必须显示 5 个源码仓均 checkout 到 `source-lock.env` 的完整 SHA；任一 revision 非 40 位 SHA 时 Dockerfile 会直接失败。

2026-08-04 已完成一次真实 `linux/arm64` 构建：镜像约 971 MB，锁定 revision `1fcd0d05cadaeb25ca59fd87cda95aaaee41e3ea`。ROS package/动态链接离线 smoke test、12 秒节点启动和真实 Compose healthcheck 均通过；这些仍不等于 G1 实时数据验收。

同日完成 G1 driver 目标镜像真实 `linux/arm64` 构建。当前坐标修正版为 `phanthy-g1-driver:navigation-sensors-rx180`，镜像 ID `sha256:e0ce5d75750e522fa10db304472feab322fbd3e0710cc38bf396c4fa2784f7f4`，逻辑大小 342,131,607 bytes。一次性容器成功导入桥接模块并把实测样例 `[-0.5205, 0.0213, -0.8561]` 校正为 `[-0.5205, -0.0213, 0.8561]`；真实 Compose 启动后两个容器均为 `healthy`，且仍为只读、非特权、`cap_drop=ALL`。基础镜像遗留的腾讯云内网 APT 源已通过 `UBUNTU_PORTS_MIRROR` 构建参数改为默认使用公开 Ubuntu ARM 源，内网构建时仍可显式覆盖。

真机复核后发现，上游 FAST-LIVO2 在单次前向 IMU 间隙超过 0.2 秒时会永久拒收后续样本，因为拒绝路径没有推进时间基线。`fast-livo2-runtime.patch` 现在接受间隙后的首个样本以重新同步。内容锁定的 hotfix 镜像 `phanthy-fast-livo2:g1-1fcd0d0-n2gap1` 已完成本地 arm64 真实构建，镜像 ID 为 `sha256:ba214f2512b57f1339cde7ed0d17f62d67b76e99fbeb45f74fc1c0426eb894b9`；确定性 `0.35 s` 间隙注入测试得到 `odom_before_gap=38`、`odom_after_gap=52`、`legacy_rejections=0`。`Dockerfile.fast-livo2-hotfix` 可在 G1 上复用已验证的 `n2qos1` 镜像，仅传输补丁和构建文件，不必重新传输约 973 MB 镜像。

上海首次长图实采在正确收到 `SIGINT` 后以 `133/SIGTRAP` 退出，13 分钟累计点云没有落盘；同镜像 10 秒合成数据可正常保存，只能排除通用参数和退出信号问题，不能覆盖 G1 的 bind mount DAC 权限。`fast-livo2-pcd-save.patch` 把纯 LIO 的 PCL 反射写盘替换为原子的 XYZ+intensity 二进制 PCD writer，并把长图改为运行中 checkpoint、退出保存尾段。新镜像 `phanthy-fast-livo2:g1-1fcd0d0-n3save1` 的本地 arm64 镜像 ID 为 `sha256:26bce71e5e0b056525f048114d873130203864cd356fc7339e6b99a721b512dc`；合成 ROS2 流以 20 帧间隔产生 4 个运行中 checkpoint 和 1 个尾段，SIGINT 退出码为 0，既有 IMU gap 回归仍得到 `odom_before_gap=38`、`odom_after_gap=52`。

上海第一次 `n3save1` 短测进一步定位到真实写入门禁：地图目录为宿主机 `unitree:unitree`、`0775`，映射容器虽然显示为 root，但 `cap_drop=ALL` 后没有 `CAP_DAC_OVERRIDE`，容器内 `test -w` 为 false，checkpoint 因而持续失败。第一次修正把 FAST-LIVO2 主进程改成目录 UID/GID，虽然能创建 `lidar_poses.txt`，但 host IPC 下始终没有注册 `/laserMapping`，健康检查失败。最终 mapping override 保留已验证的 root 运行身份，只把地图目录 GID 加入 supplementary groups，使用普通 group write 权限落盘；不增加 capability，也不把目录放宽为 world-writable。

北京 G1 复用锁定基础镜像完成远端 hotfix 构建，build context 为 23.55 kB；真机镜像 ID 是 `sha256:9151c5b22fddb3533045a92588a3ca10670b17aea83b7b448e8ed8c8d471218f`。重建后 IMU 为 `199.64 Hz`、里程计与注册点云均为 `9.96 Hz`，clock reset/rejected/stamp clamped 均为 0，两个 shadow 无重启/OOM且安全属性未变。短窗口没有自然触发大 IMU 间隙，所以真机只证明新镜像可持续正常运行；断流恢复结论仍以确定性注入测试为准。

## 现场启动前提

以下操作会修改机器人运行状态，只能由现场人员执行：

1. 把 `source-lock.env` 中的两个 arm64 镜像和运行配置部署到 G1；若机器人已有锁定的 `n2qos1` 基础镜像，可改用 `Dockerfile.fast-livo2-hotfix` 在本机构建新 FAST-LIVO2 镜像。
2. 确认 `driver.shadow.yaml` 只开启 `navigation_sensors`，且 `publish_raw_cloud: false`、`publish_fast_livo_cloud: true`。
3. 以实际主机名设置 `ROS_NAMESPACE`，以实际 DDS 网卡设置 `NETWORK_INTERFACE`，再显式启用 `lio-shadow` profile。
4. 短时功能验证可与现有 driver 并行；正式测 CPU/内存预算时必须把现有 driver 的重复点云处理计入结果，或另约受控窗口停用重复链路。

启动后仍是 shadow：现有 `embodied-unitree-g1` 不被替换，两个新容器没有 `/cmd_vel`、SmartMotion 或 action-service 连接。

仓内正式入口是 `deploy-g1-navigation-shadow.sh`。它从自身所在目录解析资产，不依赖本机绝对仓库路径；`preflight` 只读，其余模式均要求现场人员显式设置 `CONFIRM_G1_SHADOW_WRITE=YES`：

```bash
# 只读：连接、时钟、主 driver、本地镜像与磁盘前检
./deploy-g1-navigation-shadow.sh preflight g1-sh-wifi ubuntu eth0

# 现场人员执行：复用已加载镜像恢复普通 LIO shadow
CONFIRM_G1_SHADOW_WRITE=YES \
  ./deploy-g1-navigation-shadow.sh resume g1-sh-wifi ubuntu eth0

# 现场人员执行：开始/停止一张显式命名的地图
G1_MAP_NAME=sh_n3_smoke CONFIRM_G1_SHADOW_WRITE=YES \
  ./deploy-g1-navigation-shadow.sh start_mapping g1-sh-wifi ubuntu eth0
G1_MAP_NAME=sh_n3_smoke CONFIRM_G1_SHADOW_WRITE=YES \
  ./deploy-g1-navigation-shadow.sh stop_mapping g1-sh-wifi ubuntu eth0
```

`start_mapping` 默认每 600 个 LIO 帧保存一个 checkpoint；短测可显式设置 `G1_PCD_SAVE_INTERVAL=20`。启动时脚本读取专用地图目录 UID/GID，保持 FAST-LIVO2 的既有 root 身份，把目录 GID 作为 supplementary group，并把 group 注入、ROS health 与容器内目录可写性同时作为硬门禁。`stop_mapping` 在任何写操作前 fail-closed 校验当前容器确实处于 mapping 模式，且 map name 与 bind 目录同时匹配；随后单独停止 FAST-LIVO2 并报告退出码，不读取 G1 上的 Docker 日志，再清理 mapping、恢复普通只读 shadow，最后校验产物。即使产物报告失败，也不能把 sidecar 留在停止状态。`ubuntu` 和 `eth0` 必须分别替换为当台 G1 的实际 ROS namespace 与 DDS 网卡。

脚本回归入口为 `bash tests/test-navigation-shells.sh`。它使用 fake SSH/Docker 做真实 shell 调用，覆盖：跨工作目录解析、无确认拒写、preflight 无远端写命令、map name/interval 校验、mapping UID/GID 与可写门禁、stop 前保存退出码、恢复 shadow 后再统计 PCD，以及 awk 转义回归。`bash tests/run-pcd-save-smoke.sh` 则运行真实 ROS2 合成点云，验证运行中 checkpoint、退出尾段、PCD 头和退出码。

## N2 只读验收

北京代理数据的目标频率是：适配点云约 10 Hz、IMU约 200 Hz。现场至少检查：

```bash
ros2 topic hz /ubuntu/navigation/lidar_fast_livo
ros2 topic hz /ubuntu/navigation/imu
ros2 topic hz /ubuntu/navigation/odom
ros2 topic hz /ubuntu/navigation/cloud_registered
ros2 topic echo /ubuntu/navigation/sensor_diagnostics --once
```

阶段结论窗口按用户决定以约 20 分钟为上限，不再等待固定 30 分钟才推进：窗口内要求无 clock reset，点云/IMU无持续积压，`odom` 单调且静止漂移可记录，容器没有 OOM/重启，全程无运动输出。身份/外参未核验、频率不达标或时间回退时立即停止 shadow，不进入建图；更长 soak 作为后续发布质量项单独安排。

上海现场使用仓内只读采样脚本汇总相同证据：

```bash
./accept-g1-navigation-shadow.sh preflight g1-sh-wifi ubuntu eth0
./accept-g1-navigation-shadow.sh lio g1-sh-wifi ubuntu eth0
```

`lio` 依次采样四路 topic、读取 sensor diagnostics、容器资源和最近 10 分钟 gap/sync 日志，不修改机器人状态。输出按 `上海G1验收记录模板.md` 留档。

北京 G1 在 2026-08-04 14:48:48–15:08:18 完成约 19 分 30 秒阶段验收：两个容器始终 `healthy`，重启 0、OOM 0；clock reset/rejected/stamp clamped 均为 0；原始点云丢 3/11,829（约 0.025%），FAST-LIVO2 适配点云丢 0，IMU 丢 426/237,145（约 0.18%）；同步 warning 77 次且没有 fatal。按用户决定，该结果形成 N2 阶段结论并允许继续后续 shadow 设计；它不替代未来发布质量所需的更长稳定性测试。

## N3 前置边界

锁定的 FAST-LIVO2 只实现 PCD 落盘，没有 PCD 加载、`/initialpose` 或 scan-to-map 全局重定位。因此“保存一张地图后重启 LIO”不是重定位。

地图生成使用单独的 `compose.mapping.yml` override：普通 shadow 固定 `PCD_SAVE_EN=false`；mapping 必须显式提供一个专用 `G1_MAP_DIR`，只有 `/opt/fast_livo_ws/src/fast_livo/Log/pcd` 是持久可写目录。mapping 容器继续保持 `read_only`、非特权、`cap_drop=ALL` 和既有 root 主进程，只追加地图目录 GID，让无 DAC override 的进程通过 `0775` group write 落盘。默认 `interval=600`，运行中生成按 LIO 时间命名的 PCD，正常退出再生成 `tail_raw_points.pcd`；每个文件都先写 `.tmp`，完整关闭后才原子改名，避免半成品被验收。FAST-LIVO2 必须以安装后的 C++ 二进制直接作为 PID 1；使用 `ros2 run` 包装器时停止信号不会可靠转发。纯 LIO 文件固定为 `FIELDS x y z intensity`、`DATA binary`；多段地图的合并与体素降采样属于后续地图制品步骤，不能把任意单段误当成完整地图。

停止建图后必须按同一个 `G1_MAP_NAME` 做产物只读验收，不能仅凭接口返回成功：

```bash
G1_MAP_NAME=sh_n3_smoke \
  ./accept-g1-navigation-shadow.sh map g1-sh-wifi ubuntu eth0
```

该检查要求至少一份非空 PCD，读取 `POINTS` / `DATA` 头、文件大小和修改时间，并同步报告 `/tmp/nav_result.json` 是否存在及其当前内容。后续重定位传入的地图路径必须与这里实际验收的文件完全一致。

已审查的 G1/Humble 候选 `deepglint/FAST_LIO_LOCALIZATION_HUMANOID@df4772ec4797172430e7efe990711d09a529f4ad` 确认了 G1 MID360 倒装且点云/IMU要一起旋转，并提供离线点云配准节点；但其 `open3d_loc` 没有明确许可证，CMake 写死 Open3D 路径，ARM 预编译包标为未充分测试，默认话题/QoS/TF也不符合本接口。它只能作为算法参考，完成许可证和 arm64 构建审查前不进入运行镜像。

## RGB-LIVO 可视化增强（独立门禁）

RGB-LIVO 只用于生成带真实颜色的 PCD 和更直观的 RViz 预览，不接地图加载、重定位、规划或运动执行。普通 `compose.shadow.yml` 仍固定 `img_en=0`，现有 LIO/N3 建图入口不受影响。`compose.rgb-preview.yml` 只由显式的 `start_rgb_preview` 模式选中，并把云、里程计、路径和 TF 全部隔离到 `*_rgb_preview` 话题。

当前决策（2026-08-05）：导航与建图继续以纯 LiDAR/LIO 为准，RGB 不进入当前交付。上海完整 RGB 预览图虽成功恢复并以 `RGB8` 显示，但现场目视明显比纯 LIO 模糊；现有命令和恢复工具仅保留为实验资产。必须先对这台 G1 的 D435i 与 MID360 完成 FAST-Calib 联合外参标定，生成实测 `Rcl/Pcl`，并重新通过静态边缘对齐、慢速运动成图和清晰度对比，之后才允许恢复 RGB 集成。

G1 的 `/ubuntu/camera/rgb` 是 `sensor_msgs/msg/CompressedImage` 且发布 QoS 为 `BEST_EFFORT`。锁定的 FAST-LIVO2 原始图像订阅使用默认 `RELIABLE` 和 200000 深度队列，与该发布端不兼容。`fast-livo2-rgb-qos.patch` 把图像订阅改成深度 4 的 `SensorDataQoS`，并把解码后的 Image、LiDAR、IMU 应用层队列分别限制为 4、8、400；丢弃 LiDAR 帧时同步丢弃对应时间戳，避免相机断流后同步器不消费而让其他传感器 backlog 持续吃满 6 GiB。`Dockerfile.fast-livo2-rgb-hotfix` 从已验的 `n3save1` 派生，不改变普通 LIO 镜像。

2026-08-05 上海 G1 无 RViz/无 SSH bridge 对照确认：D435i 仍保留 publisher 端点但 8 秒内没有相机帧时，RGB 输出和 PCD 均不推进，旧镜像 `n3rgbqos1` 的 `fastlivo_mapping` RSS 在 76 秒内约从 306 MiB 增至 811 MiB。该斜率来自 LIVO 等待图像时无上限累积 LiDAR/IMU 应用队列，不是 RViz bridge。第一版隔离修复标签为 `n3rgbguard1`；RGB override 的 healthcheck 还必须在 5 秒窗口收到至少一个非空 `/navigation/cloud_registered_rgb`，不能再只凭 `/laserMapping` 节点存在判 healthy。

同日 Docker Desktop 真实 arm64 编译通过：镜像 `phanthy-fast-livo2:g1-1fcd0d0-n3rgbguard1`，ID `sha256:b342dc56ff75592d0baf65c17eb8eab43a16343f51ea8810d70d7e102fa9410a`，逻辑大小 979,408,355 bytes；镜像内 revision 为 `1fcd0d05cadaeb25ca59fd87cda95aaaee41e3ea`，RGB patch 标签与 `source-lock.rgb.env` 的 SHA256 一致。

相机恢复后又暴露出第二条独立增长链：`Feature::img_` 会让每个仍在视觉地图中的参考帧长期持有整张 1920×1080 灰度图。上海真机的 `n3rgbguard1` 在 RGB 约 6 Hz、PCD 每 20 帧成功清空的情况下，RSS 仍在 20 秒内从约 2.29 GiB 增到 2.61 GiB，约 `16 MiB/s`；这与每秒保留约 6 张 2.07 MB 灰度帧加特征对象的开销吻合。静态预览既没有实测外参/时偏，也不应让相机参与状态估计，因此当前 `source-lock.rgb.env` 已切到 `n3rgbpreview2`：它只保留已验证的 LIO 位姿和当前相机帧进行投影着色，跳过 `processFrame()` 的视觉特征建图。它是有色点云预览，不是已经验收的视觉里程计。

`n3rgbpreview2` 已完成 Docker Desktop 真实 arm64 编译，镜像 ID 为 `sha256:715cf510d2899a47071d66a20925a77b4b894ee9df1cec9b419e0850d284c94d`，逻辑大小 979,753,189 bytes；镜像内 revision、补丁 SHA256、`prepareFrameForColorization` 路径和安装后二进制 smoke 均通过。真机内存是否稳定仍须部署该新标签后单独验收，不能沿用旧镜像结论。

复现构建：

```bash
set -a
source source-lock.env
source source-lock.rgb.env
set +a

docker build \
  --platform linux/arm64 \
  --pull=false \
  --build-arg FAST_LIVO2_RGB_BASE_IMAGE="${FAST_LIVO2_RGB_BASE_IMAGE}" \
  --build-arg FAST_LIVO2_COMMIT="${FAST_LIVO2_COMMIT}" \
  --build-arg FAST_LIVO2_RGB_QOS_PATCH_SHA256="${FAST_LIVO2_RGB_QOS_PATCH_SHA256}" \
  -t "${FAST_LIVO2_RGB_IMAGE}" \
  -f Dockerfile.fast-livo2-rgb-hotfix \
  .
```

生产门禁仍保持 fail-closed：

```bash
./accept-g1-navigation-shadow.sh rgb g1-sh-wifi ubuntu eth0
```

该模式同时检查 D435i USB 枚举、RGB publisher/type/频率以及 `/home/unitree/.sensor-collector/calibration.json`。2026-08-04 恢复后的上海真机证据为：D435i serial `346522072810`，`1920x1080@15` 工厂内参可读，RealSense 三个模块的 global time 已开启；但 RGB 实测只有约 `6.6–6.9 Hz`，且当前 Driver 把 ROS header 盖为回调到达时间，所以仍不能认定为已验证的动态 RGB-LIVO。上海这台 D435i 已多次出现“USB 仍枚举、publisher 仍存在、实际无帧”的线缆故障；现场遇到该症状先重新插拔相机线，再做软件重启或参数判断。

`render-g1-livo-config.py` 复用 sensor-collector 的快照格式，只在下列条件全部成立时生成可运行 YAML：

- `calibration_id` 与规范化 JSON SHA256 一致，D435i serial 和精确运行 profile 一致；
- `ground_truth.transforms.lidar_to_camera` 状态为 `calibrated/measured/verified`，R 为合法旋转矩阵、T 为合理米制平移；
- 相机时间戳来自 RealSense hardware/global time，实测 `img_time_offset_s` 存在，RGB↔MID360 绝对 skew p95 不超过 20 ms；
- 内参与畸变模型可被 FAST-LIVO2 的 Pinhole 模型正确解释。

相机恢复并补齐联合标定后，在 Mac 上生成配置：

```bash
./render-g1-livo-config.py \
  <(ssh g1-sh-wifi 'cat /home/unitree/.sensor-collector/calibration.json') \
  /private/tmp/g1_livo.yaml
```

生成器默认对齐当前 Driver 的 `1920x1080@15`，任何缺失或占位值都会返回非零且不创建输出。只有生成器与 `accept ... rgb` 同时绿灯，才能升级为动态 RGB mapping shadow；当前强行复用 LIO 的单位阵/零平移占位值会产生错色重影，不能作为“照片纹理”验收。

为了在实测外参前先看方向和大致对齐，脚本另提供一个显式的静态预览通道。它先从 `rt/lowstate` 识别机型；上海真机已证实 `mode_machine=4`，对应宇树公开的 `g1_23dof_rev_1_0`。候选外参组合公开机身安装位、RealSense 当台工厂 depth→color 外参、optical frame 旋转，并抵消 sensor bridge 已经施加的 `Rx(pi)`。生成物固定标记为 `nominal_public_urdf` + `callback_arrival_preview` + `preview_only: true`，默认生产生成器仍会拒绝它。探针会在 native DDS/RealSense 析构前强制 flush stdout；部署端不再把 SSH 退出 0 当成成功，而是硬校验 JSON 并最多重试 3 次。3 次仍无效时在任何机器人写入前退出。

当前发布版 Driver 仍以 librealsense 回调到达时刻给 RGB header 盖章。不能把 `ros2 topic delay` 看到的约 0.2 秒直接写成时偏，因为它还包含 JPEG 编码和 DDS 传输；`probe-g1-rgb-time-offset.sh` 会在没有 mapping/RGB preview 活动时短暂停止并恢复 `embodied-unitree-g1`，用同一 Driver 镜像独占 D435I，连续比较 RealSense `GLOBAL_TIME` 帧时刻与主机交付时刻。输出的 `img_time_offset` 是回调延迟中位数的负值；回调延迟残差 p95 超过 20 ms、相机 serial/profile 不匹配、机器人重启导致 boot ID 改变时都 fail-closed。

现场人员先生成本次开机有效的时间探针：

```bash
G1_RGB_TIME_PROBE_OUTPUT=/private/tmp/g1-rgb-time-offset-shanghai.json \
CONFIRM_G1_SHADOW_WRITE=YES \
  ./probe-g1-rgb-time-offset.sh unitree@10.110.12.110 120
```

随后把该文件显式交给 RGB 入口：

```bash
G1_MAP_NAME=sh_rgb_full_nominal_20260805 \
G1_PCD_SAVE_INTERVAL=20 \
G1_RGB_TIME_PROBE=/private/tmp/g1-rgb-time-offset-shanghai.json \
CONFIRM_G1_SHADOW_WRITE=YES \
  ./deploy-g1-navigation-shadow.sh \
  start_rgb_preview unitree@10.110.12.110 ubuntu eth0
```

这一模式标记为 `nominal_public_urdf + measured_callback_latency + slow_manual_preview`：先静止 20 秒初始化，再慢速人工驾驶并避免快速转身。它比零时偏静态预览更适合生成一张粗真彩地图，但只校正 Driver 回调时间，不等于 D435I↔MID360 的硬件触发或端到端时偏精标，生产门禁继续保持关闭。

由现场人员执行静态预览：

```bash
G1_MAP_NAME=sh_rgb_static_20260804 \
G1_PCD_SAVE_INTERVAL=20 \
CONFIRM_G1_SHADOW_WRITE=YES \
  ./deploy-g1-navigation-shadow.sh \
  start_rgb_preview unitree@10.110.12.110 ubuntu eth0
```

开始后先让机器人静止 20 秒，RViz 添加 `PointCloud2` 话题 `/ubuntu/navigation/cloud_registered_rgb`，颜色变换选 `RGB8`。这一轮只看固定边缘是否大致对齐，不让机器人走动，也不将点云用于导航。预览镜像优先在 G1 上复用已有的 `n3save1` 基础层小上下文构建，不传输整镜像；已有同 checksum 镜像时直接复用。若相机或同步链路没有产生非空 RGB 云，Compose 启动门禁会把容器判为 unhealthy，不得继续打开 RViz 等待。

```bash
G1_MAP_NAME=sh_rgb_static_20260804 \
CONFIRM_G1_SHADOW_WRITE=YES \
  ./deploy-g1-navigation-shadow.sh \
  stop_rgb_preview unitree@10.110.12.110 ubuntu eth0
```

`stop_rgb_preview` 在写操作前同时校验 mode/map/bind 目录，优先恢复普通只读 LIO，再要求产物中至少一份 PCD 的头是 `FIELDS x y z rgb`，并保留本次候选标定 JSON。若静态边缘已明显错位，不调参“凑齐”；直接停止预览，转入标定板 + FAST-Calib 的实测外参流程。

若机器人在 RGB preview 期间整机重启，容器会以 `255` 退出，不能再调用正常停止入口并把这轮冒充为 clean shutdown。运行中的时间戳 PCD checkpoint 已经是统一世界坐标系，可由恢复脚本原子合并；它要求退出容器的 mode、map、bind 目录和 `255` 退出码全部匹配，保留所有原始分片，生成 `all_rgb_points.recovered.pcd` 与 `rgb-recovery-manifest.json`，随后恢复普通只读 LIO：

```bash
G1_MAP_NAME=sh_rgb_full_nominal_20260805 \
CONFIRM_G1_SHADOW_WRITE=YES \
  ./recover-g1-rgb-preview.sh unitree@10.110.12.110 ubuntu eth0
```

恢复清单固定标记 `clean_shutdown=false`、`unsaved_tail_recovered=false`：最后一个 checkpoint 之后、重启之前仍在内存中的尾段无法恢复。恢复只处理形如 `<秒>.<纳秒>.pcd` 的 RGB checkpoint，并逐个校验 PCD 头、payload 长度、有限坐标和最终 SHA256；不会递归吸收既有聚合文件。异常重启后若出现“文件长度已分配但内容全部为 `0x00`”的掉电写回残片，恢复入口只跳过这种可证实的全零文件，并把文件名、大小和原因写入 manifest；截断、错头或含非零未知数据的文件仍会 fail-closed。机器人重启后原时间探针因 boot ID 改变而失效，若要继续新的 RGB preview 必须重新测量。

## 回滚

现场人员对同一 Compose 执行 `down` 即可同时停止 sidecar 和 FAST-LIVO2。现有 G1 driver 从未被替换；N2 不写地图、不修改主运控，也不需要清理机器人数据。
