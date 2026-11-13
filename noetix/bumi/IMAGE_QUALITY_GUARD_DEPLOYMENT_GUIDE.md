# 部署前指南

当前仅本地开发。没有部署Bumi、没有上传镜像。Driver与Agent Core两个官方ARM64镜像均已本地构建通过；真实Core应用和仅Guard的Driver在内部Docker网络完成MCP控制面验证。真实摄像头、跨容器DDS到Dashboard、浏览器及Bumi实验仍未验收，不能据本地集成通过宣布最终验收完成。

## 配套版本

Driver 使用 `feat/bumi-image-quality`，Core 使用独立本地分支 `feat/image-quality-guard-lifecycle`。不要把这两个分支的改动直接写入 main。Core/Canvas/Driver 的生命周期协议需要配套，旧版 Core 调 Guard 会缺票失败；旧 Card 仍保持原协议。

Core 当前采用一个 Uvicorn worker、无 reload，生命周期 Owner 在 auto_start 前初始化，资源目录中的文件锁拒绝第二个共享目录 Owner。跨机器多 Owner 不受支持，不要设置多个副本绕过这个约束。Core 内存票据不进共享布局/方案数据库。

## 本地构建

官方 Bumi Dockerfile 用 `driver.yaml.build_context_extras: ../../common`。直接在 Bumi 目录 `docker build .` 会缺 common，新增脚本在临时目录准备相同构建上下文且从不 push：

```powershell
cd D:\Internship_Projects\card_1\phanthymotus-driver\noetix\bumi
powershell -NoProfile -ExecutionPolicy Bypass -File .\build-local.ps1
# 仅当前子进程脚本策略，不修改机器策略；需要时显式选择已核实的镜像源：
powershell -NoProfile -ExecutionPolicy Bypass -File .\build-local.ps1 -UbuntuMirror https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports
```

临时目录保留供检查，不覆盖仓库文件。官方基础镜像 digest 和 SDK 构建流程沿用原文件。不要用不带 SDK 的简化镜像冒充完整 Bumi 镜像。脚本不会运行 Bundle，也不会连接机器人。

Core 构建命令：

```powershell
cd D:\Internship_Projects\card_1\phanthymotus\agent-core
# 在当前终端先配置IQG_BUILD_PROXY为已验证可达的宿主Clash代理URL；不要写进项目。
# Docker内宿主使用host.docker.internal，不能直接使用宿主的127.0.0.1地址。
docker build --platform linux/arm64 --progress plain `
  --build-arg APT_MIRROR=ports.ubuntu.com `
  --build-arg PYPI_MIRROR=https://pypi.org/simple/ `
  --build-arg "HTTP_PROXY=$env:IQG_BUILD_PROXY" `
  --build-arg "HTTPS_PROXY=$env:IQG_BUILD_PROXY" `
  --build-arg IMAGE_TAG=local-image-quality-validation `
  -t image-quality-core:local -f Dockerfile .
```

软件源根因是继承base的腾讯VPC域名，不是Docker整体DNS故障。Core Dockerfile现在使用官方build_common已有的APT_MIRROR参数，默认/空值为官方HTTPS ports源，显式Tencent选择保留原VPC源；apt恢复签名校验。使用原base digest和Ubuntu22.04/Jammy，未删除ffmpeg或更换基础镜像。Desktop代理设置没有修改；临时build args已在RUN中核实，最终image ENV没有代理。

不要直接执行deploy/build_core.sh作为本地验证替代：它会加载deploy/.env，在凭据齐全时可能login/push/注册镜像。以上直接官方Dockerfile命令只构建本地镜像。

当前镜像：Driver `image-quality-guard:local` / `be6455114b2d`；Core `image-quality-core:local` / `7afe1b07e8aa`，均linux/arm64。Driver上一轮LD路径修复及145项验证仍有效，本轮未重复运行无修改的全部测试。

## 本轮隔离运行验证的复现边界

本轮使用单独的 `docker network create --internal <本轮网络名>` 和新建resource卷，不使用官方生产compose的host网络/PID/IPC及Docker/DBus挂载，不发布宿主端口。Core卷必须用 `--mount type=volume,source=<本轮卷>,target=/work/resource,volume-nocopy`，防止首次挂载复制镜像中的原data.db。保持默认auto_start=False、无模型凭据与其他机器人服务；运行环境设置UV_OFFLINE=1、UV_NO_SYNC=1、ROS_LOCALHOST_ONLY=1、ROS_DOMAIN_ID=219、RMW_IMPLEMENTATION=rmw_fastrtps_cpp、FASTRTPS_DEFAULT_PROFILES_FILE=/deploy/dds-local.xml。

Core使用原官方CMD，网络别名phanthy-motus匹配应用生成证书的CN。只读挂载Driver的tests到/validation用于验证客户端。Driver使用同一内部网络、别名iqg-driver，source原ROS工作区后执行 `/validation/container_guard_mcp_server.py`，不能调用完整main.main()。bootstrap只配置Guard，但实际Bundle/MCP Handler/Runtime均来自Driver镜像，SDK和Camera不加载。

客户端在Core容器执行 `/work/.venv/bin/python -B /validation/container_core_driver_integration.py --phase initial`，使用本次生成证书作为CA，保持TLS与主机名校验。leave-active保存原票据供仅本轮Core SIGKILL实验；同卷重启后recover核对新Owner、拒绝旧票直接调用，再显式授stop-only权限清理。普通独立Card的该恢复已经实测；UNKNOWN项目恢复不是本轮完成项。

正常Core SIGTERM退出143并完成应用/ROS清理，不能要求它伪报退出0；SIGKILL退出137不能声称运行了finally。Driver SIGTERM退出0且节点/线程归零。只清理本轮创建的容器、卷和网络；不要prune或删除其他项目。本轮临时资源均已清理，两个镜像保留。

## 部署前检查

1. 同事复核跨层票据、项目冻结集合、LLM 参数隔离、恢复/轮换流程。
2. 复核本轮两个ARM64镜像的ID、依赖和构建记录；目标Jetson安装前仍需内部审核，不把仿真CPU结果当硬件性能结果。
3. 获内部批准后才进行安装；此次不提供自动部署或 push 脚本。
4. 检查已有 Camera JPEG Topic、namespace、ROS_DOMAIN_ID=实际部署域、端口 Core15678/Driver15704；不要打开额外服务。
5. tools/list 中存在 image_quality_guard 与顶层 x-lifecycle；initialize 不重置 epoch。
6. 确认能力从发现/heartbeat到Canvas保留，run_id 不出现在用户配置或LLM业务Schema。
7. 新 Core 遇到旧 Driver 活动运行时先查询；显式恢复只授予原票据 info/stop 权限，不允许自动接管后重启。清理确认空闲后才轮换会话。
8. 按 HARDWARE_TEST 单人验收，记录实际 Camera 结果和资源释放。

不要在本机直接运行完整 main.py 进行“烟测”：现有 Bundle 可能初始化厂商 SDK、Camera 与其他机器人功能。本轮使用隔离的真实 HTTP Handler/Bundle 方法测试，以及独立 ROS2 DDS 测试，没有运行厂商控制初始化。
