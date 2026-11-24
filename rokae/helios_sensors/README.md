# 珞石 Helios-2.1 相机传感器

使用机载 Orbbec SDK 2.9.3 和 GMSL2 Gemini 335Lg。已配置的传感器随驱动自动采集，Core 可独立管理两条流。驱动在 `127.0.0.1:15750` 注册 MCP，沿用官方 Core 的 ROS 2 domain 42 和本机 Fast DDS。可以和 Helios 的关节、电池、RGB 驱动并行部署。

| 卡片 | 格式 | 数据 |
|---|---|---|
| `imu` | `/rokae_helios/state/imu` · `data/json` | 同步采样的三轴加速度 m/s²、角速度 rad/s 和估算姿态四元数 XYZW。采集 50 Hz，发布约 10 Hz |
| `camera_depth` | `/rokae_helios/camera/depth` · `image/depth-zlib` | 原生 640×480、5 fps 深度图。用 SDK 每帧比例换算成毫米，保留无效像素的零值 |

IMU 加速度和角速度使用传感器坐标系。姿态是从传感器坐标系旋转到重力对齐的局部坐标系的单位四元数。陀螺仪按设备时间积分，加速度幅值接近 1g 时参与倾角修正。初始化需连续约 0.3 秒满足低角速率和加速度幅值条件。初始化航向作为偏航零点，偏航随时间可能漂移，持续线加速度也可能影响倾角。数据间隔超过 0.25 秒后重新初始化，准备期间四元数留空。`orientation_source` 和 `orientation_reference` 标明数据来源及参考系。

`device_stamp_us` 为 SDK 设备时间，`timestamp_ms` 为驱动接收后生成状态的本机时间。停流或样本过期时清除 IMU 读数。深度帧用 `sensor_msgs/msg/CompressedImage` 发布小端 16UC1 毫米值，zlib 压缩。深度内参来自当前原生分辨率的 SDK profile，读取失败时保持空值。

## 构建与部署

- 按仓库流程构建 `./build.sh rokae/helios_sensors`。审核镜像的服务模板为 [service.yml](deploy/service.yml)。
- 将 [compose.orbbec-gmsl.yml](deploy/compose.orbbec-gmsl.yml) 合并在部署 Compose 之后。填写机器上已安装的 `HELIOS_ORBBEC_SDK_HOST_DIR` 和两个已核对的 GMSL 相机设备节点。SDK、USB 枚举目录和 udev 信息以只读方式挂载，设备访问权限限于配置里的 V4L2 节点。设备节点与序列号属于现场配置。
- 设置 `HELIOS_ORBBEC_IMU_SERIAL` 和 `HELIOS_ORBBEC_DEPTH_SERIAL`，分别选择不同的 Gemini 335Lg。一张卡的序列号留空时，该卡显示 `unconfigured`。可用 `HELIOS_ORBBEC_IMU_FRAME_ID` 和 `HELIOS_ORBBEC_DEPTH_FRAME_ID` 设置已核对的传感器坐标系。
- 机载 Core 使用 host 网络时，直接用标准服务模板。Core 使用 bridge 网络时再合并 [compose.core-network.yml](deploy/compose.core-network.yml)，配置 `HELIOS_CORE_CONTAINER` 共享 Core 的网络命名空间。

部分 Jetson GMSL 现场环境中，SDK 在容器里能枚举相机但收不到深度帧。这时可在机载主机启动原生深度源，再合并 [compose.depth-host.yml](deploy/compose.depth-host.yml)。数据经本机 Unix socket 传给 Driver，Core 的 ROS 2 流与卡片接口一致。

在仓库根目录以有相机权限的用户运行：

```bash
export LD_LIBRARY_PATH="$HELIOS_ORBBEC_SDK_HOST_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
python3 -B -m rokae.helios_sensors.depth_socket \
  --sdk-dir "$HELIOS_ORBBEC_SDK_HOST_DIR" --serial "$HELIOS_ORBBEC_DEPTH_SERIAL" \
  --socket "$HELIOS_DEPTH_SOCKET_DIR/depth.sock" --group video
```

预先创建由运行用户和 `video` 组访问的 socket 目录，并将持有该组 GID 的 `HELIOS_DEPTH_SOCKET_GID` 传给 Compose。主机深度源只在 Driver 连接后采集，Driver 停流时释放相机。

## 验证

在仓库根目录执行 `python -m pytest -q tests/test_helios_camera_sensors.py` 和 `python scripts/check_service_yml.py rokae/helios_sensors`。

在机载 Core 的侧栏找到「珞石 Helios-2.1 相机传感器」，手动把两张卡拖入画布。直接打开「查看数据流」核对 IMU 数值和深度色图。移除后重新拖入并查看数据流时，可以恢复预览。正式验收使用 PR 触发的 bot 镜像，并记录镜像号和真机截图。

2026-11-24 使用真机 ARM64 调试镜像验证了机载 Core 注册、两路自动采集、深度内参与浏览器数据流解码。现场 IMU 由容器直接采集，深度使用机载主机 socket。该记录对应开发调试镜像。

参考 [Motus 深度格式](https://github.com/4paradigm/phanthymotus-driver/blob/main/README_dev.md#depth-rendering--imagedepth-zlib) 和 [Orbbec SDK 2.9.3 深度与 IMU 接口](https://github.com/orbbec/OrbbecSDK_v2/blob/v2.9.3/include/libobsensor/h/Frame.h#L450-L506)。
