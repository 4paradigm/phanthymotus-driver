# Go1 板载相机 Adapter（camera_rgb 卡的板卡侧组件）

Pi 侧驱动的 `camera_rgb` 卡（`../camera.py`）只负责**探测/控制/收流转发**；真正打开相机、跑
UnitreecameraSDK 采集的是本目录的 **板载 Adapter**，运行在 Go1 的 Nano 板卡上。这样满足能力
卡片 §8.6.1「相机设备由一个板载 Adapter 独占访问，三个视觉卡共享同一采集会话」。

```
Nano 板卡 (.13/.14/.15)                         Pi 驱动容器 (.161)
┌───────────────────────────┐   JSON/TCP 控制   ┌──────────────────────────┐
│ camera_adapter            │◀─────────────────│ camera.py (camera_rgb)    │
│  UnitreecameraSDK 取帧     │                  │  启动探测→只建可达位置实例 │
│  gst 编码 H.264/RTP        │──── UDP 图像流 ──▶│  gst 解码→发布 JPEG topic  │
└───────────────────────────┘                  └──────────────────────────┘
```

## 组件构成

| 文件 | 作用 |
|---|---|
| `camera_adapter.cpp` | 板载 Adapter：UnitreecameraSDK 取帧 + JSON-TCP 控制服务 + gst 编码 RTP/H.264 定向发流 |
| `build_adapter.sh` | 在板卡上编译（依赖板载 UnitreecameraSDK + OpenCV + GStreamer） |

## 位置 → 板卡 / 设备 / 端口（能力卡片 §8.2）

| position | board_ip | device_id | image_port | control_port(约定=image+100) |
|---|---|---:|---:|---:|
| front | 192.168.123.13 | 1 | 9201 | 9301 |
| chin  | 192.168.123.13 | 0 | 9202 | 9302 |
| left  | 192.168.123.14 | 0 | 9203 | 9303 |
| right | 192.168.123.14 | 1 | 9204 | 9304 |
| belly | 192.168.123.15 | 0 | 9205 | 9305 |

> 这些是缺省约定值；真实值以 Pi 侧 `config.yaml` 的 `camera_rgb.positions` 为准，两侧必须一致。

## 编译（在板卡上）

```bash
# 1) 板卡上先准备 UnitreecameraSDK（clone + make 出 lib）
git clone https://github.com/unitreerobotics/UnitreecameraSDK
cd UnitreecameraSDK && mkdir build && cd build && cmake .. && make -j2   # 产出 lib/

# 2) 编译 Adapter
export UNITREE_CAMERA_SDK=/home/unitree/UnitreecameraSDK
cd <本目录>
bash build_adapter.sh          # 产物 ./camera_adapter
```

### ⚠️ `[SDK-API]` 触点（真机首编大概率要对齐）
`camera_adapter.cpp` 里标了 `[SDK-API]` 的调用（构造、`isOpened`、`getFrameSize`、`getFrameRate`、
`getSerialNumber`、`getRectStereoFrame`、`getRawFrame`、`startCapture`、`stopCapture`）按官方 example
常见签名书写。不同 SDK 版本方法名/参数可能不同——**编译期报错时，对照板载
`UnitreecameraSDK/include/UnitreeCameraSDK.hpp` 与 `examples/example_getRawFrame.cc`、
`example_getRectStereoFrame` 修正即可**。这与 `../robot_interface_v32.cpp` 的思路一致：wrapper 随硬件编译、以编译期报错兜底。

## 运行（每个在用相机一个进程）

```bash
# front（device 1）+ chin（device 0）在 .13 上各起一个
./camera_adapter --device-id 1 --device-node /dev/video1 --control-port 9301 &
./camera_adapter --device-id 0 --device-node /dev/video0 --control-port 9302 &
```

设备节点（`/dev/videoX`）以板卡实际枚举为准，不写死跨机器默认值（能力卡片 §8.3）。
建议用 systemd / 板卡 autostart 托管；不要在已被官方图像服务占用时启动——Adapter 打不开设备时
会对 `probe`/`start` 返回 `RESOURCE_BUSY`，**不会去杀占用进程**（§8.6.2）。

## 控制协议（逐行 JSON，一问一答；与 `camera.py::_AdapterClient` 对齐）

| 请求 | 响应 |
|---|---|
| `{"cmd":"probe","device_id":N}` | `{"ok":true,"online":true,"busy":false,"serial":..,"width":..,"height":..,"fps":..,"calibration":{..}}` |
| `{"cmd":"start","device_id":N,"config":{mode,frame_size,fps,rectified_size,hfov_deg,target_ip,image_port}}` | `{"ok":true,"applied":{..},"calibration":{..},"streams":[{"eye":"left","port":9203},{"eye":"right","port":9204}]}` |
| `{"cmd":"stop","device_id":N}` | `{"ok":true}` |
| `{"cmd":"snapshot","device_id":N,"eye":"left"}` | `{"ok":true,"seq":S,"timestamp_us":T}` |

失败统一 `{"ok":false,"code":..,"message":..}`，`code` 取能力卡片 §3.1 错误码
（`RESOURCE_BUSY`/`DEVICE_NOT_FOUND`/`INVALID_ARGUMENT`/`COMMUNICATION_ERROR`/`PRECONDITION_FAILED`）。

## 图像传输

- 编码：Adapter 用 `gst-launch-1.0 fdsrc → rawvideoparse → videoconvert → x264enc → rtph264pay → udpsink`
  把 SDK 取到的 `cv::Mat` 编成**标准 RTP/H.264**，定向 UDP 发到 `target_ip:image_port`
  （`target_ip` 由 Pi 在 `start` 时下发，**不硬编码**，能力卡片 §8.6.3）。
- 解码：Pi 侧 `camera.py` 用对称的 `udpsrc → rtph264depay → avdec_h264 → jpegenc` 收成 JPEG，
  发布到 `/{ns}/vision/{position}/{mono|left|right}`。编解码两端由本仓库同一套约定构造，天然对齐。
- stereo 模式：`left` 走 `image_port`、`right` 走 `image_port+1`；mono 走 `image_port`。

## 尚未闭环 / 待真机校准

1. **标定 `calibration`**：`read_calibration()` 目前是占位（`status:"unverified"/"file"`）。真机接通后
   按 SDK 实际可取字段填充内参/畸变/Xi/旋转/平移/校正内参（能力卡片 §8.2 info 要求）。
2. **左右目命名**：`raw_stereo` 按帧宽对半切 left/right，**目别未经标定确认**，Pi 侧 `eye` 会标
   `unverified_*`（§8.3「不能未经验证直接命名左右目」）；用 SDK 的 `getRectStereoFrame` 校正输出后
   再据标定确认目别。
3. **depth / pointcloud**：本 Adapter 只做 rgb 取帧 + 发流，已留同一 `CameraSession` 采集会话，
   后续 §8.4/§8.5 两卡在此基础上加 `startStereoCompute + getDepthFrame / getPointCloud`。
