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
也可以使用以下完整命令，将两个 REPLACE 路径替换为本机设备，将 IMAGE 替换为机器人给出的完整镜像地址：

```bash
curl -fsSL https://motus.phanthy.com/run-pr-image.sh | env \
  GLOVE_SERIAL_PORT=/dev/serial/by-path/REPLACE_WITH_GLOVE_RECEIVER \
  L20_SERIAL_PORT=/dev/serial/by-path/REPLACE_WITH_HAND_RS485 \
  GLOVE_PREVIEW_ONLY=1 \
  bash -s -- IMAGE
```

这条命令拉取并启动已构建的镜像，不在本机编译。新版镜像默认只允许手套预览与标定，arm 不会驱动实体。
`release.261029.1be62fd` 是此次默认预览和时序日志修改之前的镜像；更新 PR 后使用机器人新生成的标签。
接收器需使用 921600 波特率，灵巧手使用 115200；不要互换端口，也不要同时运行上位机或旧读取器。
默认 `/dev/glove` 是可配置的设备别名，普通 USB 插入不会自动创建它。不同机器应填写各自路径，不能照抄开发机的 PCI/USB 路径。

若提示容器名称冲突，先确认旧容器属于本驱动，并在画布停止项目，然后保留旧容器作为回退：

```bash
docker stop embodied-linkerhand-l20-lite
docker update --restart=no embodied-linkerhand-l20-lite
docker rename embodied-linkerhand-l20-lite embodied-linkerhand-l20-lite-backup
```

备份名必须未被占用；停止旧 Compose 对该服务的自动管理，不要同时启动两个项目争用设备。
若只是更新已有的 PR 容器参数，重新运行带参数的部署命令即可，无需再次改名。
检查实际生效的参数和日志：

```bash
docker inspect --format '{{range .Config.Env}}{{println .}}{{end}}' embodied-linkerhand-l20-lite \
  | grep -E '^(GLOVE_SERIAL_PORT|L20_SERIAL_PORT|GLOVE_PREVIEW_ONLY)='
docker logs --tail 80 embodied-linkerhand-l20-lite
```

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
先完成下节的受控时序验收，再允许日常遥操。受控实机验收也需显式选择运动：将部署命令中的 `GLOVE_PREVIEW_ONLY=1` 改为 `0` 并重建容器。
此设置只解除手套遥操的预览限制，仍需重新标定、操作员显式 arm(confirm_motion=true)；只启用已标定且找到匹配灵巧手的侧。
该开关不禁止独立 hand/hand_gesture 卡片的显式动作。
disarm 和项目停止解除输出；断流后不自动 arm。重启项目/容器需要重新标定。
手动 hand/hand_gesture 动作会先结束遥操、等待在途控制清理，然后接管同一硬件。

模型为目标姿态预览，不是实体反馈或碰撞模型；pitch/yaw 是相对旋转向量的诊断投影，不等于厂商过滤后的欧拉角。
实际标定和控制使用 root/top 两路四元数。当前仅支持已验证的 24 字节六四元数协议分支，不宣称支持全部厂商设备模式。

已验证的部署版本在 x86_64 主机的 ARM64 容器上完成左右手读取、骨架和实机遥操。原始录制 9657 帧与 SDK 四元数逐帧对照一致。
约 128 秒读取抽查的四路最大接收间隔约 38ms；这是串口读取层观察，不是端到端回调、Modbus 或卸力延迟保证。
原生 ARM64、长时间稳定性及 USB 热插拔仍待验证。
提交整理阶段的显式手动接管改动另有离线测试；最终应以机器人构建的镜像再做实机验收。

## 断流阈值与时序验收（尚未完成）

原配置 800ms 缺少足够依据，已改成 300ms 候选值，并默认禁止手套实体运动。
300ms 约为 15Hz 输出的 4.5 个周期；15Hz 是发送指令上限，不是输入频率。
该候选值高于短时读取观察的 38ms，但尚未证明适用于完整系统的最坏抖动，不能称为已测安全值。

当前实现及已知限制：

| 环节 | 证据或配置 | 限制 |
| --- | --- | --- |
| 接收器四路更新 | 127.905 秒只读观察，最大间隔约 38ms | 没有证明 CPU 压力及长时条件下的上界 |
| 驱动回调/排队 | info.timing 的 max_callback_gap_ms、max_source_gap_ms、source_lag_ms | 最终镜像待实机采样 |
| 超时检测 | 300ms 候选值，独立线程每 20ms 检查 | 通常约 300ms 加检查/调度延迟后禁发新目标；不是实时系统保证 |
| Modbus 在途写入 | response_timeout_s=0.5；写超时 0.5s，随后分段读取回复 | 每段读取有独立超时，flush 及总线锁也可能等待，不能把 0.5s 当整个事务上界 |
| 卸力 | 当前事务返回后，逐手发送零扭矩 | 双手串行，前一只手异常会延迟后一只；通信故障时无法保证卸力成功 |

历史故障日志曾出现 apply_ms=1021ms，因此即使将 watchdog 改为 300ms，也不能声称 300ms 内完成实体卸力。
旧日志并非当前镜像的最坏时序验收结果。软件 watchdog 不替代实体断电/急停。

受控验收时保持实体运动空间空闲、可随时切断电源。使用最终镜像，分别检查单手、双手及正常主机负载下的数据流，
记录 info.timing；再通过关闭手套输入触发断流，检查以下日志：

```bash
docker logs --since 10m embodied-linkerhand-l20-lite 2>&1 \
  | grep -E '\[glove-(watchdog|release)\]'
```

watchdog 日志记录判停时每侧样本年龄、在途 apply 耗时和历史最大回调/控制耗时；
release 日志记录每侧零扭矩应答结果、卸力事务耗时以及最后接收样本到卸力事务完成的时间。
`zero_torque_acknowledged=false` 不能计作成功卸力；即使为 true，也只是协议应答，仍需观察实体停止/卸力。
这些测试应记录镜像标签、主机架构、单/双手、负载、样本数、观测最大值和选定余量，不能只记录“没有断流”。

只有完整链路的观测值及余量满足本装置允许的停机时间，才可接受该 watchdog 值用于日常运动。
若在途事务/零扭矩写入已超过允许时间，须先缩短并验证总线事务等待或使用实体保护；增大 watchdog 不能解决。
目前 Modbus 最坏事务及实体卸力时间未完成测量，所以本提交不默认开放手套运动，也不宣称机器人的时序验收项已通过。

## Core 前端依赖

Core 需正确传递卡片 instance_id、向骨架渲染器传递 model 服务 ID，并按 URDF 固定轴 RPY 使用 ZYX 顺序。
这些修改属于 phanthymotus 仓库的独立 PR，不应混入 driver PR。没有它们可能出现“手卡未绑定”、默认机器人模型或拇指姿态错误。
