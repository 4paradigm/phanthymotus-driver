# 无 SDK 手套遥操部署

镜像包含驱动代码和左右手 URDF。手套使用串口直读，不需要上位机、DLL、Mono、宿主机源码或厂商 SDK 目录。
设备运行参数需要按每台机器填写，不使用开发机的 PCI/USB 路径作为默认值。

## 镜像构建

在仓库根目录使用官方构建入口：

```bash
./build.sh linkerhand/l20_lite
```

也可以用纯 Python 脚本生成独立构建上下文，再在支持 ARM64 构建的 Docker 环境构建：

```bash
python3 linkerhand/l20_lite/build_context.py --context /tmp/l20-lite-sdkfree-build
docker buildx build --platform linux/arm64 --load \
  -t local/phanthy-l20-lite:sdkfree /tmp/l20-lite-sdkfree-build
```

上下文目录必须不存在或为空；脚本不会覆盖已有文件。构建脚本只复制运行文件、公共模块和资源，排除本机 .env 与缓存。

## 官方镜像/审核机器人部署

原有 service.yml 保持 hand/hand_gesture 的部署方式，手套参数采用可选默认值。没有 SDK 挂载或强制双手设备要求。
使用手套时，必须在创建容器前填写接收器和至少一个灵巧手端口。先退出上位机并停止其他读取器。

```bash
ls -l /dev/serial/by-path/
export GLOVE_SERIAL_PORT=/dev/serial/by-path/REPLACE_WITH_GLOVE_RECEIVER
export L20_SERIAL_PORT=/dev/serial/by-path/REPLACE_WITH_HAND_RS485
```

然后在同一终端运行审核机器人给出的完整部署命令。环境变量必须传给部署脚本使用的 bash，不要只加在管道左侧 curl 前面。
官方网页部署时应将上述变量放入 Motus 主机使用的环境文件（通常为 /opt/phanthy-motus/.env），再重新创建驱动容器。
只使用原有灵巧手卡片的用户无需配置 SDK 或接收器；没有接收器时不要启动 glove_teleop 卡片。

## 本地 overlay 部署

在本目录复制 .env.example 为 .env，填写本机端口和镜像。假定 Motus 的主配置已包含 linkerhand-l20-lite 服务：

```bash
docker compose -p phanthy-motus \
  --env-file ./linkerhand/l20_lite/deploy/.env \
  -f /opt/phanthy-motus/docker-compose.yml \
  -f ./linkerhand/l20_lite/deploy/glove-overlay.yml \
  up -d --no-deps --force-recreate linkerhand-l20-lite
```

如果主配置尚未包含服务，先从网页或审核机器人命令部署完整镜像，不要只运行缺少网络/设备权限的 overlay。
镜像使用 ARM64，原生 ARM64 主机直接运行；x86_64 主机需要另行配置 ARM64 模拟。

## 使用与验证范围

将 glove_teleop 卡片加入画布，启动项目并查看数据流。左右手分别记录张开、握拳姿态。
确认可运动后，操作员显式 arm(confirm_motion=true)；只启用已标定且找到匹配灵巧手的侧。
disarm 和项目停止解除输出；断流后不自动 arm。重启项目/容器需要重新标定。
手动 hand/hand_gesture 动作会先结束遥操、等待在途控制清理，然后接管同一硬件。

模型为目标姿态预览，不是实体反馈或碰撞模型；pitch/yaw 是相对旋转向量的诊断投影，不等于厂商过滤后的欧拉角。
实际标定和控制使用 root/top 两路四元数。当前仅支持已验证的 24 字节六四元数协议分支，不宣称支持全部厂商设备模式。

已验证的部署版本在 x86_64 主机的 ARM64 容器上完成左右手读取、骨架和实机遥操。原始录制 9657 帧与 SDK 四元数逐帧对照一致。
约 128 秒读取抽查没有超过 800ms 的断流。原生 ARM64、长时间稳定性及 USB 热插拔仍待验证。
提交整理阶段的显式手动接管改动另有离线测试；最终应以机器人构建的镜像再做实机验收。

## Core 前端依赖

Core 需正确传递卡片 instance_id、向骨架渲染器传递 model 服务 ID，并按 URDF 固定轴 RPY 使用 ZYX 顺序。
这些修改属于 phanthymotus 仓库的独立 PR，不应混入 driver PR。没有它们可能出现“手卡未绑定”、默认机器人模型或拇指姿态错误。
