# G1 传感器卡片整合

## 范围

仅修改 G1 Driver。新增 `lidar_imu`（MID360），保留机身 `imu`；标准导航点云合入
`lidar_cloud`，RGB/Depth PSE1 分别合入 `camera_rgb` / `camera_depth`。
删除四个独立入口 navigation_lidar、navigation_imu、camera_rgb_frame、camera_depth_frame。
不修改消费者、Go2。初始实现阶段不发布；用户随后授权提交部署，进入范围化提交、
远端同步及目标设备部署检查阶段。部署设备须明确，切换前须核实空闲及回滚镜像。

## 契约

原 topic、ROS 类型、schema、QoS、时间/坐标转换保持不变。原输出在列表首位，新增
输出使用独立 port。旧 Canvas 卡片由消费端同事迁移，不提供隐藏别名。

## 生命周期

按卡片输出组启停，不串停共享 worker 的其他输出。使用有界等待的跨进程发布门控，
停止确认后不能再发布；重新启动丢弃上一周期的在途/排队数据。
LiDAR 安全流与相机拍照缓存使用内部通道，与公开发布解耦。共享采集保留有效需求，
最后一个需求释放后停止相应采集；全局 shutdown 统一回收进程和监控节点。
状态按对应输出统计，区分主动停止、未收到数据、worker 故障及过期。

## 验证

覆盖工具注册及 topic 兼容、交叉启停、并发/重复启停、停止后无发布、旧帧抑制、
安全流和照片缓存独立性、worker 重启及全局清理。运行定向测试、G1 全量测试、
service 检查和差异检查。真实 ROS/设备行为留待获授权后的现场验收。

## 文档

同步 README 中的卡片名、生命周期及迁移对照；历史验收记录不改写成新版本已验收。

## 本地实施与验证记录

- 2026-09-20：完成卡片整合、共享输出门控、内部安全/照片通道、按输出健康状态及
  worker 重启/最终资源回收。发布门控等待上限为 2 秒，超时返回错误；DDS 回调取
  发布周期令牌不等待锁，避免因启停阻塞共享采集。
- Legacy 点云安全采集在 Driver 运行期间持续保留；标准 LiDAR/IMU 输出均关闭时
  释放独立导航 worker。相机没有公开输出和照片取帧需求时暂停硬件采集，保留监督
  进程以支持重新启动。RGB 开启时同时预热内部照片缓存，保持原拍照 info 语义。
- `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=unitree/g1 python3 -m unittest discover -s unitree/g1/tests -p 'test_sensor_output_lifecycle.py'`：18 项通过。
- `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=unitree/g1 python3 -m unittest discover -s unitree/g1/tests -p 'test_navigation_sensor_card.py'`：14 项通过。
- `PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=unitree/g1 python3 -m unittest discover -s unitree/g1/tests -p 'test_*.py'`：268 项通过。
- `python3 scripts/check_service_yml.py unitree/g1`：1 checked，0 failed。
- `git diff --check`：通过。README 中英双语、driver.yaml 与配置注释已同步核对。
- 验证等级：本地 unit/contract，包含真实 spawn 进程门控及真实发布函数的模拟
  ROS/设备端点测试；没有运行实际 ROS 网络、RealSense、MID360 或 ARM64 镜像。
  上述为实现阶段验证记录，不代表已部署或真机验收，消费端改动由其他同事负责。
