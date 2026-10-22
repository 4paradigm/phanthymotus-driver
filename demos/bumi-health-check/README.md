# Bumi 独立健康检查服务

此服务提供一张 health_check MCP 卡片，使用现有 Bumi 驱动发布的 ROS 状态。它没有机器人 SDK、ROS 发布器或运动控制客户端。无需替换、重启 Bumi 驱动镜像；卡片可手动执行，也可绑定到画布 Agent 调用。

## 数据与判定

必须显式指定一个 Bumi ROS 命名空间，不自动选择发现的第一台机器人。

| 数据源 | ROS 话题（std_msgs/msg/String，JSON） | 默认发布间隔 | 过期门槛 |
|---|---|---|---|
| 电池 | /命名空间/state/battery | 1 秒 | 3 秒 |
| IMU | /命名空间/state/imu | 0.05 秒 | 0.15 秒 |
| 21 个关节 | /命名空间/state/joints | 0.1 秒 | 0.3 秒 |
| 运动状态 | /命名空间/motion/state | 0.5 秒 | 1.5 秒 |

总体优先级：异常 > 数据不足 > 正常。电量低于 20%、非零电池报警、保护模式或已识别电机故障报告异常；缺失、过期、错误 JSON、无效字段，以及未收录的非零电机错误码报告数据不足，列出原始关节和错误码供核实，不擅自解释其含义。没有 motion/state 的旧驱动也能部署此服务，但该项始终数据不足，不假装检查完成。独立主板状态不支持。

config.json 可调整实际发布间隔和认可后的电池/关节温度上限；默认温度只显示、不判定。修改后重启本服务生效。检查的是消息到达本服务的时间，不是 SDK 底层采样时间：驱动持续重发旧 SDK 数据无法仅靠这些话题识别。“正常”仅表示已启用规则未发现异常，不是运动安全许可。

## 部署（Bumi SSH 终端）

先确定命名空间和 ROS domain：

```bash
docker exec embodied-noetix-bumi bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic list | grep -E "/state/(battery|imu|joints)$|/motion/state$"'
docker exec embodied-noetix-bumi printenv ROS_DOMAIN_ID
```

例如实际话题为 /robot_abc/state/battery，则 BUMI_NAMESPACE=robot_abc。不要直接照抄这个例子。

```bash
cd ~/phanthymotus-driver-meeting
git pull --ff-only
cd demos/bumi-health-check
cp -n .env.example .env
nano .env
test -f /opt/phanthy-motus/dds-local.xml
docker compose config --quiet
docker compose up -d --build
docker compose logs --tail 60
```

.env 填实际 BUMI_NAMESPACE 和 ROS_DOMAIN_ID（当前部署通常为 42）。DDS 文件检查失败时先修复现有平台的 DDS 配置，不创建同名空目录。镜像复用本机 ros-base，不安装 apt/pip 依赖，不需要设备挂载、特权模式或 SDK。当前电脑无法验证 Docker/ROS，以上命令需在 Bumi 验证。

在 Agent Core 的 MCP 服务管理中添加：
- 名称：Bumi Health Check
- URL：http://localhost:15741/mcp（Agent Core 与健康服务同机）
- 如果 Agent Core 在另一台机器，URL 使用 http://192.168.55.101:15741/mcp

刷新工具列表，选择这个独立服务下的 health_check 卡片，ACTION=check，可手动执行。若要让画布 Agent 调用，需将 decision_core 底部的执行器端口连接到这张卡片；无需连接音频卡片。卡片在消息到达前会显示数据不足。

直接检查接口：

```bash
curl -sS http://localhost:15741/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"health_check","arguments":{"action":"check"}}}'
```

## 验证、停用和兼容

- 对照电池、IMU、关节和运动状态原始话题，检查报告 topics、received_at、age_seconds 和 reasons。确认 ROS 命名空间及 domain 与 Bumi 驱动一致。
- 启动尚无消息时必须数据不足；部署时缺少话题必须明确指出；配置温度界限前不得声称温度安全。
- 缺失/断流和低电量等场景可通过离线测试验证，不要为测试把真机放电或改变机器人状态。
- /healthz 仅表示服务状态，不代表机器人健康。transport_error 表示 ROS 初始化或接收异常。
- 在本目录运行 docker compose stop 即可停用独立服务，原 Bumi 驱动不受影响。删除画布上的独立卡片即可撤销使用，数据只缓存在内存。
- 仓库中的 Bumi 驱动已移除旧健康插件、打包及专用缓存；真机现有镜像不会因此自动改变。如果旧镜像已暴露同名卡片，请使用独立服务分组下的版本，暂不为了删除旧卡片替换整机驱动。
- 历史会议服务的健康检查入口仍默认关闭；若将来明确启用，应指向 15741 的独立服务。

本地测试（需要 pytest）：

```bash
python3 -m pytest -p no:cacheprovider demos/bumi-health-check/tests -q
```
