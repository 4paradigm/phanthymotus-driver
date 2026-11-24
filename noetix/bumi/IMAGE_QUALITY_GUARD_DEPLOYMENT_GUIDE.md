# 部署前指南

PR #495 的 `release.261119.73ee237` 构建对应 `e5568120`，早于此处的本地发现门控修复。配套 Core 镜像仍待确认。[PR 构建记录](https://github.com/4paradigm/phanthymotus-driver/pull/495#issuecomment-7010165340)

## 配套版本与部署条件

| 组件 | 当前已知信息 | 部署前必须记录 |
|---|---|---|
| Driver 源码 | PR #495，已知提交 `8e0468b2`、`e5568120` | 包含本轮修复的完整提交 SHA |
| Driver 镜像 | PR 已构建 `bj-warehouse.tencentcloudcr.com/phanthy-motus/drivers/noetix/bumi:release.261119.73ee237` | 含门控修复的新 ARM64 镜像完整引用、digest、对应源码 SHA |
| Core 源码 | 本地分支 `feat/image-quality-guard-lifecycle` 存在未提交配套修改；HEAD 为 `fc64fbc5541ef3966d81c2f28f9637475a673282` | 实际已发布的配套 PR/提交；该本地 HEAD 本身不能代表配套修改 |
| Core 镜像 | 是否已有包含配套接口的发布镜像尚未核实 | 完整引用、digest、对应源码 SHA |
| 协议验证 | 历史本地 MCP 验证见测试报告 | 对实际部署版本确认下列生命周期能力 |

Card 需要 `motus.lifecycle/1`：工具发现保留顶层 `x-lifecycle`，Driver 支持 status/bind，Core 在启动前签发票据，Canvas 的 start/stop 保留同一原始 `run_id`，旧票停止不能影响新运行。票据不出现在用户配置或 LLM 业务表单中。

`config.yaml` 默认设为 `plugins.image_quality_guard.enabled: false`，`driver.yaml` 的 `cards` 不列出 Guard。生产 `main()` 使用 Bundle 的默认门控，沿用旧配置中的 `enabled: true` 时，MCP `tools/list` 也不会暴露 Guard。生产重新开放需要单独提交，配合已验证的 Core 版本更新运行时注册、默认配置和市场清单。

**联调需使用已核实的配套 Core。** Core 签票和原票停止的约束持续有效，UNKNOWN 需要核查实际运行归属。Core 当前方案只支持单 Owner。

## 隔离联调入口

`tests/container_image_quality_smoke.py` 和 `tests/container_guard_mcp_server.py` 使用只启用 Guard 的测试配置，并在构造 Bundle 时显式传入 `allow_image_quality_guard_for_test=True`。两者使用真实 ROS Runtime，在本地容器中运行且不需要接入 Bumi、厂商 SDK 或 Camera Card。配套 Core 的 HTTP 验证客户端位于 `tests/container_core_driver_integration.py`。Core 和测试 Driver 使用独立本地网络及测试数据卷。操作步骤及历史结果见[测试报告](IMAGE_QUALITY_GUARD_TEST_REPORT.md)。

## 历史本地构建与隔离验证（2026-11-05）

官方 Bumi Dockerfile 用 `driver.yaml.build_context_extras: ../../common`。直接在 Bumi 目录 `docker build .` 会缺 common，以下历史命令使用当时的本地脚本准备构建上下文。`build-local.ps1` 未随仓库发布。

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

历史镜像：Driver `image-quality-guard:local` / `be6455114b2d`；Core `image-quality-core:local` / `7afe1b07e8aa`，均linux/arm64。Driver上一轮LD路径修复及145项验证仍有效，本轮未重复运行无修改的全部测试。

## 本轮隔离运行验证的复现边界

本轮使用单独的 `docker network create --internal <本轮网络名>` 和新建resource卷，不使用官方生产compose的host网络/PID/IPC及Docker/DBus挂载，不发布宿主端口。Core卷必须用 `--mount type=volume,source=<本轮卷>,target=/work/resource,volume-nocopy`，防止首次挂载复制镜像中的原data.db。保持默认auto_start=False、无模型凭据与其他机器人服务；运行环境设置UV_OFFLINE=1、UV_NO_SYNC=1、ROS_LOCALHOST_ONLY=1、ROS_DOMAIN_ID=219、RMW_IMPLEMENTATION=rmw_fastrtps_cpp、FASTRTPS_DEFAULT_PROFILES_FILE=/deploy/dds-local.xml。

Core使用原官方CMD，网络别名phanthy-motus匹配应用生成证书的CN。只读挂载Driver的tests到/validation用于验证客户端。Driver使用同一内部网络、别名iqg-driver，source原ROS工作区后执行 `/validation/container_guard_mcp_server.py`，不能调用完整main.main()。bootstrap只配置Guard，但实际Bundle/MCP Handler/Runtime均来自Driver镜像，SDK和Camera不加载。

客户端在Core容器执行 `/work/.venv/bin/python -B /validation/container_core_driver_integration.py --phase initial`，使用本次生成证书作为CA，保持TLS与主机名校验。leave-active保存原票据供仅本轮Core SIGKILL实验；同卷重启后recover核对新Owner、拒绝旧票直接调用，再显式授stop-only权限清理。普通独立Card的该恢复已经实测；UNKNOWN项目恢复不是本轮完成项。

正常Core SIGTERM退出143并完成应用/ROS清理，不能要求它伪报退出0；SIGKILL退出137不能声称运行了finally。Driver SIGTERM退出0且节点/线程归零。只清理本轮创建的容器、卷和网络；不要prune或删除其他项目。本轮临时资源均已清理，两个镜像保留。

## 生产启用后的验收与回滚

以下步骤供操作者在确认配套 Core 已发布、Driver 的生产入口已开放且获得真机测试授权后执行。

1. 同事复核跨层票据、项目冻结集合、LLM 参数隔离、恢复/轮换流程。
2. 记录实际待部署的 Core 和 Driver 镜像 digest、源码提交及回滚版本；目标Jetson安装前仍需内部审核，不把仿真CPU结果当硬件性能结果。
3. 获内部批准后才进行安装；此次不提供自动部署或 push 脚本。
4. 检查已有 Camera JPEG Topic、namespace、ROS_DOMAIN_ID=实际部署域、端口 Core15678/Driver15704；不要打开额外服务。
5. tools/list 中存在 image_quality_guard 与顶层 x-lifecycle；initialize 不重置 epoch。
6. 确认能力从发现/heartbeat到Canvas保留，run_id 不出现在用户配置或LLM业务Schema。
7. 新 Core 遇到旧 Driver 活动运行时先查询；显式恢复只授予原票据 info/stop 权限，不允许自动接管后重启。清理确认空闲后才轮换会话。
8. 按 HARDWARE_TEST 单人验收，记录实际 Camera 结果和资源释放。

不要在本机直接运行完整 main.py 进行“烟测”：现有 Bundle 可能初始化厂商 SDK、Camera 与其他机器人功能。本轮使用隔离的真实 HTTP Handler/Bundle 方法测试，以及独立 ROS2 DDS 测试，没有运行厂商控制初始化。
