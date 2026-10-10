# ArUco 区域规划与画布卡片（仅视觉）

## 平台画布

RealMan Driver 已注册 `aruco_zones` 处理卡片。部署更新后的 Driver 后，在画布上添加
`ext_camera` 和 `aruco_zones`，将相机实例的 `channel` 设为 `rgb`，把它的 JPEG 输出连接到
`aruco_zones` 输入。卡片会持续检测所有可见 ArUco 码，输出带区域框的 JPEG 预览和
`data/json` 区域结果；`info` 可查看最新识别状态。相机断流超过 3 秒时，`info.latest.ready`
变为 `false`。

在卡片配置中填写 `waiting_rect`、`sorting_1_rect`、`sorting_2_rect`，每项格式为
`x1,y1,x2,y2`，都是 0 到 1 的归一化图像坐标。例如横向三列可填：

```text
waiting_rect   = 0,0,0.3,1
sorting_1_rect = 0.35,0,0.65,1
sorting_2_rect = 0.7,0,1,1
```

还可配置 ArUco 字典和两个分拣区的颜色标签。区域必须互不重叠；每区恰好一个可见码、
三个码 ID 互不相同时，输出 `ready: true`。画布当前通过配置表单填写矩形坐标；下方的
独立 GUI 支持在画面上拖拽框选，保存的 JSON 中 `rect_normalized` 数值可以填入卡片。
这些区域只用于视觉规划，不涉及机械臂抓取或坐标标定。

## 独立框选 GUI

`aruco_zone_gui.py` 参考 `interactive_gui.py` 的画面框选方式，实现第一阶段的区域规划。
它只读取 RealSense 彩色画面或已有照片，不连接 RealMan API2，不控制机械臂或夹爪。

## 使用

在有图形桌面的 Python 环境安装 `numpy` 和带 ArUco 模块的 `opencv-contrib-python`。
直接读取 RealSense 时还需要 `pyrealsense2`，且应先停止占用该相机的 `ext_camera`。
在仓库根目录执行：

```bash
# 先使用上一阶段相机测试保存的照片，便于不接真机调布局。
python3 realman/rm75_6f_v/aruco_zone_gui.py \
  --image camera_test_output/color.jpg \
  --output aruco_zones.json

# 在连接 RealSense 且有图形桌面的上位机实时查看。
python3 realman/rm75_6f_v/aruco_zone_gui.py \
  --camera --serial YOUR_CAMERA_SERIAL \
  --output aruco_zones.json
```

在窗口顶部选择“待分拣区”“分拣区 1”“分拣区 2”，在画面上分别拖拽矩形。
程序检测画面中的 ArUco 码（默认 `DICT_4X4_50`），把**中心落在区域内**的码绑定到该区域；
无需预先指定码 ID 或摆放顺序。分拣区 1/2 的颜色下拉框仅保存目标颜色标签，当前不识别物体颜色。
保存时要求三个区域互不重叠、每区恰好一个可见码且 ID 各不相同。

输出 `aruco_zones.json` 保存三个区域在图像中的归一化矩形、当前像素矩形、绑定码 ID、
目标颜色和图像尺寸；`aruco_zones_preview.jpg` 保存带码与区域框的画面。
这些都是**图像坐标**，尚未进行相机到机械臂的坐标标定，也不用于抓取。

这个脚本是独立的区域规划窗口，不会自动修改已部署画布卡片的配置。
