# 实际测试与验收报告

日期：2026-11-05。以下结果来自实际执行，不以静态阅读代替 PASS。

## 最新：Core 网络修复、构建与跨容器集成（2026-11-05）

本节覆盖最新状态；后面的Driver复验及初始回归保留为历史证据。本轮没有重复执行未修改的Driver145项或主仓库全套测试。

### 网络故障定位（先只读）

官方入口为主仓库 `deploy/build_core.sh` 和 `deploy/build_common.sh`，实际Dockerfile是 `agent-core/Dockerfile`。入口会读取deploy/.env，并可能login/push及注册镜像，因此本轮没有直接运行发布脚本，而是等价执行官方Dockerfile的本地ARM64构建命令。

来源已证实：`deploy/ros-base/Dockerfile` 默认把Ubuntu源换成腾讯VPC域名；Core继承的同一base digest是 `82d45949e7c3fd85e6baf4a2b24b384a3ec020a5e237c5f801bc2f2269ca649f`，系统仍为Ubuntu22.04.5/Jammy、arm64。`/etc/apt/sources.list` 中所有Ubuntu源是 `http://mirrors.tencentyun.com/ubuntu-ports/`。官方build_common虽传入APT_MIRROR，原Core Dockerfile没有声明或使用它。

| 实测层次 | 内网软件源 | 官方Ubuntu HTTPS源 |
|---|---|---|
| Windows直接连接 | 无A地址，curl退出6，无法解析 | HTTP200、TLS校验0 |
| Windows经现有Clash | HTTP502 | HTTP200、TLS校验0 |
| ARM64基础镜像容器直接连接 | gaierror，无法解析 | HTTP200，PGP签名InRelease内容 |
| 容器经Clash宿主代理 | HTTP502 | HTTP200，TLS校验开启 |
| BuildKit RUN直接连接 | gaierror，无法解析 | HTTP200 |
| BuildKit RUN经显式临时代理参数 | HTTP502 | HTTP200 |

Clash进程和混合代理监听已只读核查；Docker中 `host.docker.internal` 正常解析，实际可连接Clash，不使用容器内127.0.0.1指代宿主。Desktop daemon报告内部代理 `http.docker.internal:3128`，settings-store无显式代理覆盖；但RUN探针的HTTP(S)_PROXY默认未设置，不能据daemon配置推断构建命令自动继承代理。显式传入预定义build args后，RUN中大小写HTTP(S)_PROXY均正确出现。Docker DNS可解析官方站点与宿主别名，不能把本例归结为Docker DNS整体失效。Clash的502仅表示该源的代理请求失败，不能单靠502断定TLS或DNS细节；明确的DNS失败来自直接探测。当前官方源未复现TLS失败。

### 最小修复与实际构建

主仓库本轮唯一生产文件改动为 `agent-core/Dockerfile`：使用官方脚本既有的APT_MIRROR参数，默认/空值选择官方ports源；显式Tencent选择保留原VPC地址；恢复apt默认签名验证，保留ffmpeg等原依赖、原基础镜像和Ubuntu版本。没有修改公共API、Docker Desktop配置或公司服务器，没有关闭TLS校验。个人代理仅通过临时build args传递，未写入Dockerfile、项目配置或最终image ENV。

实际成功命令（工作目录agent-core；IQG_BUILD_PROXY仅在本次终端设置为已验证的Clash宿主代理URL，不保存到项目）：

```powershell
docker build --platform linux/arm64 --progress plain `
  --build-arg APT_MIRROR=ports.ubuntu.com `
  --build-arg PYPI_MIRROR=https://pypi.org/simple/ `
  --build-arg "HTTP_PROXY=$env:IQG_BUILD_PROXY" `
  --build-arg "HTTPS_PROXY=$env:IQG_BUILD_PROXY" `
  --build-arg IMAGE_TAG=local-image-quality-validation `
  -t image-quality-core:local -f Dockerfile .
```

本轮实际调用为等价的临时代理build args，代理地址未持久化。apt签名索引与ffmpeg安装成功；uv lock解析133个跨环境锁记录，实际Python3.10环境安装91个包。`uv pip check --python /work/.venv/bin/python` 返回 `All installed packages are compatible`。真实导入rclpy、sensor_msgs、std_msgs、audio_msgs、FastAPI、aiohttp、Uvicorn、NumPy、coordinator、lifecycle API和mcp_client成功；ffmpeg4.4.2、Docker CLI27.5.1、Compose2.37.0、file5.41均能执行。最终镜像未持久化代理环境变量。

| 镜像 | 本轮检查的image ID | 架构 | 构建/状态 |
|---|---|---|---|
| image-quality-core:local | sha256:7afe1b07e8aaf760fd8aa486806da5f9ea5fca3e5810f1e3fce1bf820c13ce7d | linux/arm64 | 本轮官方Dockerfile完整构建成功 |
| image-quality-guard:local | sha256:be6455114b2dfd81efd0d73af2d41f8376f7969b4a213bd225addc87a70ab76a | linux/arm64 | 保留上一轮已验证镜像，本轮未重建 |

以上都是本机Docker Desktop/ARM64仿真构建或运行，不是云端构建、Jetson性能或Bumi验收。

### 真实应用与MCP集成结果

Core使用官方CMD，在新建 `--internal` Docker网络和专用空resource卷启动；卷用volume-nocopy，避免继承镜像中原有data.db。未挂设备、Docker/DBus socket、主机目录或主机网络，没有发布宿主端口。ROS限制loopback及测试域219，禁止外部联网。默认auto_start=False、无模型密钥、无其他机器人MCP。Core日志实际出现Application startup complete、HTTPS15678监听和ros2_bridge started；`/api/config/status`及`/api/lifecycle/state`返回正常，Owner启用。镜像没有配置Docker HEALTHCHECK，本轮健康结论依据真实应用startup/API，不把docker ps运行状态冒充healthy检查。隔离实例无.env测试令牌，认证遵循原应用默认状态；没有修改认证代码。

Driver验证bootstrap导入镜像内真实BumiDeviceBundle/make_handler，仅启用Guard，使用真实ROS Runtime，不调用main.main()、Camera或厂商SDK。Core通过Docker内部HTTP访问Driver15704；HTTPS客户端信任本次Core生成的证书并校验CN别名phanthy-motus，未使用-k或跳过证书校验。

| 本轮真实集成阶段 | 断言数 | 通过 | 失败 |
|---|---:|---:|---:|
| initial：健康、注册、发现、metadata、绑定、签票、重复start、缺票/伪造拒绝、同Card/Topic重启旧stop隔离、重复stop和释放 | 24 | 24 | 0 |
| leave-active：准备实际活动运行并保存原票据供异常退出验证 | 12 | 12 | 0 |
| recover：重启Owner变化、旧票拒绝、原Driver运行保留、显式stop-only恢复、拒绝重启、原票释放 | 18 | 18 | 0 |
| 合计 | 54 | 54 | 0 |

这是3个场景中的54项断言，含每阶段重复的健康/发现检查，不能宣称新增54个独立单元测试。两个helper是 `tests/container_guard_mcp_server.py` 与 `tests/container_core_driver_integration.py`；已执行静态编译及实际容器调用。客户端实际运行命令为：

```text
docker exec <本轮Core容器> /work/.venv/bin/python -B /validation/container_core_driver_integration.py --phase initial
docker exec <本轮Core容器> /work/.venv/bin/python -B /validation/container_core_driver_integration.py --phase leave-active
# 对本轮Core SIGKILL，随后使用原卷重启同一Core容器，再执行：
docker exec <本轮Core容器> /work/.venv/bin/python -B /validation/container_core_driver_integration.py --phase recover
```

运行票据仅存于专用容器卷中的0600文件，未将令牌打印/写入票据文件。全程没有假Runtime或假MCP服务器替代生产协议实现；Driver采用测试bootstrap配置，但实际Bundle、HTTP Handler、Guard和ROS Runtime都来自镜像。没有摄像头输入，质量状态UNKNOWN；本轮验证MCP控制面，未宣称跨容器DDS到Dashboard的数据面通过。上一轮7项真实DDS测试的合成JPEG证据单独保留。

### 正常和异常退出

Core正常SIGTERM两次均记录Application shutdown complete、ROS bridge stopped，容器退出143（128+SIGTERM，官方uv run的信号退出状态），非OOM、State.Error为空；不能改写为退出0。实际Owner文件锁在运行期间拒绝第二Owner，正常停止后可重新获取。

Core SIGKILL实测退出137、非OOM；该场景不执行应用finally，不能称“优雅清理”。OS释放文件锁已验证，Driver仍持原run；新Core不自动接管，原票被拒，只有显式stop-only恢复后才以原票停止，真实Driver资源最终释放。

Driver最后SIGTERM退出0；清理日志nodes=0、resources_pending=false、executor_stopped=true、HTTP/ROS worker均false、errors=[]。只清除了本轮命名的两个容器、空测试卷和内部网络，保留两个镜像与源码；没有prune其他资源。

本轮未放宽任何run_id校验。未迁移旧Card的已准入但延迟发送start仍无完整顺序保证；UNKNOWN项目拥有子Card时恢复仍返回PROJECT_BUSY，需要内部确认恢复策略。本轮通过的是独立Card的Core重启恢复，不是UNKNOWN项目自动恢复。Bumi摄像头、跨进程DDS展示、真实浏览器及真机性能仍待相应环境验收。

## 上一轮 Driver ARM64 镜像与容器复验（2026-11-05）

实际本机标签是 `image-quality-guard:local`，不是无标签的 `image-quality-guard:latest`。本轮修复并重新本地构建后：

```text
image ID: sha256:be6455114b2dfd81efd0d73af2d41f8376f7969b4a213bd225addc87a70ab76a
platform: linux/arm64
image LD_LIBRARY_PATH: /work/noetix_sdk_bumi/dds/aarch64/lib
```

固定基础镜像的 Config.Env 没有 LD_LIBRARY_PATH，原来的 `${LD_LIBRARY_PATH}` 展开为空，结果带尾部冒号。空路径项会让动态加载器搜索当前工作目录。最终最小修复为显式设置厂商库目录；ROS 路径继续由启动时三个 setup.bash 加入。最终构建成功且没有 UndefinedVar 警告。此前的软件源失败保留在历史记录中，不能再代表本轮 Driver 构建结果。

| 本轮验证 | 数量 | 通过 | 失败 | 跳过 | 实际边界 |
|---|---:|---:|---:|---:|---|
| 镜像模块/实际Bundle/ROS资源检查 | 4 | 4 | 0 | 0 | 镜像内真实模块、真实rclpy；只启用Guard |
| 图像质量完整 unittest 集合 | 141 | 141 | 0 | 0 | 含56算法、56Guard、18假ROS transport、4观察器、7真实ROS |
| 上述容器用例合计 | 145 | 145 | 0 | 0 | 容器进程退出码0 |
| SIGTERM容器退出探针 | 1 | 1 | 0 | 0 | 实际分配Node后docker stop，退出码0 |
| Driver官方Dockerfile重建 | 1 | 1 | 0 | 0 | 使用现有缓存，没有push |
| Agent Core官方Dockerfile历史重试 | 1 | 0 | 1 | 0 | 上轮apt源DNS失败；最新构建成功见首节 |
| Bumi真机 | — | — | — | 未执行 | 未接摄像头、设备或机器人网络 |

4项镜像检查是本轮新增用例，141项是既有套件重跑，不重复累加成145项新测试。先执行过4+7项聚焦检查也全部通过；随后完整运行4+141项。完整验证第一次只挂载tests，观察器测试因缺少可选工具源文件失败，补齐只读tools挂载后141项全部通过；没有为此修改生产算法或跳过测试。

实际环境：Docker Desktop的ARM64仿真容器，ROS2 Humble / Python3.10 / OpenCV5.0.0 / NumPy2.2.6。三个生产模块从镜像 `/work` 加载，其SHA256与本地待审文件一致；测试和可选观察工具只读挂载，未覆盖生产模块。初始化后LD路径包含ROS和audio_msgs库目录，导入OpenCV还加入其lib64目录，整个路径没有空项。真实DDS消息收发及Node创建成功，未观察到动态库加载错误；这不是Jetson性能或Bumi摄像头验收。

实际命令（Driver仓库根）：

```powershell
cd noetix/bumi
powershell -NoProfile -ExecutionPolicy Bypass -File ./build-local.ps1 -Image image-quality-guard:local
cd ../..
docker image inspect image-quality-guard:local --format '{{.Id}} {{.Os}}/{{.Architecture}}'
$iqgValidation = (Resolve-Path -LiteralPath noetix/bumi/tests).Path
$iqgValidationTools = (Resolve-Path -LiteralPath noetix/bumi/tools).Path
docker run --rm --network none --platform linux/arm64 `
  --mount "type=bind,source=$iqgValidation,target=/work/tests,readonly" `
  --mount "type=bind,source=$iqgValidationTools,target=/work/tools,readonly" `
  --entrypoint /bin/bash image-quality-guard:local `
  /work/tests/run_container_validation.sh 'test_image_quality*.py'
```

本轮最终结果：镜像检查 `Ran 4 tests ... OK`，完整质量套件 `Ran 141 tests in 12.434s ... OK`，没有跳过。真实ROS7项使用合成JPEG发布源，验证真实CompressedImage/String、QoS、2Hz上限、新帧覆盖、无新帧不发布、旧run_id隔离和资源释放；并非FakeRuntime冒充DDS。

退出验证另用 `run_container_validation.sh --serve` 启动隔离探针，确认READY时一个真实Node，再执行 `docker stop --time 20 <本轮探针容器>`（此CLI提示--time已弃用，复现可用--timeout）。清理结果：

```json
{"probe":"CLEANUP","nodes":0,"resources_pending":false,"executor_stopped":true,"worker_alive":false}
```

容器 `ExitCode=0`、`OOMKilled=false`、State.Error为空；重复Bundle.stop_all安全。测试容器已移除。探针自行管理独立context/executor，Guard Runtime仍复用传入的Executor。未运行默认完整main()，只导入真实Bundle类并配置Guard，避免SDK、Camera及其他机器人功能初始化。

Driver验证通过后，实际重试主仓库agent-core目录下：

```powershell
docker build --platform linux/arm64 --progress plain -t image-quality-core:local -f Dockerfile .
```

仍失败于官方Dockerfile的apt步骤：`Could not resolve 'mirrors.tencentyun.com'`，继而 `Unable to locate package ffmpeg`，RUN退出100，docker build退出1。本轮未修改Core Dockerfile或Docker网络设置，没有关闭TLS验证、上传镜像或反复重试。Core镜像构建/容器运行不能标记PASS。

本轮只改Dockerfile的LD路径、两个新增容器验证文件及已有报告/状态说明；算法、Guard、Runtime与主仓库代码保持原样。镜像构建成功、容器运行成功、隔离ROS2集成通过、Bumi真机未验证是四个独立结论。

## 基础回归记录（上一轮，保留为基线）

| 独立测试集合 | 数量 | 通过 | 失败 | 未执行/跳过 | 说明 |
|---|---:|---:|---:|---:|---|
| 阶段 A 算法 | 56 | 56 | 0 | 0 | 真实 OpenCV 合成图像，原文件未改 |
| Guard 插件与并发 | 56 | 56 | 0 | 0 | 原41+新增15；FakeRuntime资源，真实analyzer |
| Runtime 单元 | 18 | 18 | 0 | 0 | 假ROS资源、确定性时钟；真实JPEG解码与算法 |
| 观察器解析 | 4 | 4 | 0 | 0 | 不加载ROS，不写图片 |
| Bundle/MCP | 9 | 9 | 0 | 0 | 包含真实本地HTTP；隔离SDK与ROS资源 |
| Bumi其余既有回归 | 142 | 142 | 0 | 0 | 全Bumi pytest的一部分 |
| 真实 ROS2 | 7 | 7 | 0 | 0 | WSL/Humble，真实DDS，合成JPEG发布源 |
| Core相关套件（不含下行） | 180 | 180 | 0 | 0 | 140既有+38生命周期+2Owner启动释放 |
| Core→Driver真实HTTP跨仓库 | 3 | 3 | 0 | 0 | 真实coordinator/RPC/Handler/Bundle；FakeRuntime |
| 前端Node | 235 | 235 | 0 | 0 | 202既有+33新增，纯逻辑/renderer测试 |
| **上述独立用例总数** | **710** | **710** | **0** | **0** | 不重复累计不同环境重跑；193 subtests另列 |

本轮新增用例129项：Driver53，Core43，前端33。此总数包含为验证兼容性执行的既有测试，不能与历史仅算法/插件的97项直接比较。Windows全Bumi收集292项：285通过、7项ROS跳过；这7项随后在WSL中实际通过，不能说Windows执行了ROS。

| 其他验收 | 结果 |
|---|---|
| Driver官方ARM64 Docker构建 | **本轮已通过**，结果见上节；此前apt网络失败已不代表当前Driver状态 |
| Core官方ARM64 Docker构建 | **最新已通过**：本轮修复继承软件源并经临时Clash参数构建，见首节 |
| 真实浏览器人工/E2E操作 | 未执行；Node测试不能代替 |
| Bumi摄像头/负载/阈值/单人实验 | 未执行，待硬件；没有真实摄像头验收结论 |
| 主仓库全部测试 | 未执行全仓库全套；已执行本次相关16个套件 |

没有把构建失败、未执行浏览器或真机实验算成PASS。因此目前不满足任务中的全部最终完成标准。

## 实际执行命令

开发前97项基线与当前聚焦回归：

```powershell
py -B -m unittest discover -s noetix/bumi/tests -p "test_image_quality*.py" -v
```

基线97全部通过；当前Windows141项：134通过、7跳过。最终WSL执行相同141项全部通过，9.752秒。算法两文件SHA256保持：

```text
image_quality.py
B64C13D75D0612C279A62CFC17F85E68E17051F3966AF8601813238A31C3FB0A
tests/test_image_quality.py
F726610842DC693C3301C75EC594B635D4AEC9019EA80CD198EA9A5F2E09C216
```

Driver全Bumi回归（仓库根）：

```powershell
$env:PYTHONPATH = Join-Path $env:TEMP 'image-quality-guard-test-deps'
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD = '1'
$env:PYTHONUTF8 = '1'
py -B -m pytest noetix/bumi/tests -q -p no:cacheprovider
```

最终结果：`285 passed, 7 skipped, 193 subtests passed in 51.68s`。pytest及少量测试依赖放在TEMP target；没有修改全局Python环境或生产依赖清单。最初既有测试使用GBK读取UTF-8源码出现3项失败，通过进程级PYTHONUTF8修正测试环境后全过，未改旧Card算法。

Core相关组合（工作目录主仓库agent-core）：

```powershell
$env:PYTHONPATH = Join-Path $env:TEMP 'image-quality-guard-test-deps'
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD = '1'
$env:PYTHONUTF8 = '1'
py -B -m pytest tests/test_lifecycle_runs.py tests/test_lifecycle_mcp_boundary.py tests/test_lifecycle_owner_startup.py tests/test_image_quality_driver_e2e.py tests/test_start_project_order.py tests/test_start_project_resolution.py tests/test_stop_project_stages.py tests/test_start_project_control_plane.py tests/test_start_project_camera_info.py tests/test_mcp_call_errors.py tests/test_mcp_registry_merge.py tests/test_mcp_file_args.py tests/test_mcp_file_proxy.py tests/test_sse_subscription_lifecycle.py tests/test_solution_test_block.py tests/test_canvas_editor_idle.py -q -p no:cacheprovider
```

最终组合：`183 passed in 6.09s`。覆盖LLM alias/split/direct伪造、首次工具发现/heartbeat能力保留、同槽位并发、共享stop与取消隔离、epoch超时核对/轮换、终态容量、Owner互斥、项目原票与冻结目标、recover/status旧响应CAS。

前端（主仓库根）：

```powershell
node --test "agent-core/web/js/*.test.mjs"
```

最终235/235通过，0失败/跳过。另对生产JS执行node --check通过。测试包括保存布局拒绝、原票据捕获、pending启动取消、重复启动共享、即时重连、旧响应、失败启动清理后新票、参数不泄漏、旧Card和质量状态渲染。没有自动化真实浏览器。

真实ROS2（Driver根，先保证Humble及cv2/numpy可用）：

```powershell
wsl.exe -d Ubuntu-22.04 --exec bash /mnt/d/Internship_Projects/card_1/phanthymotus-driver/noetix/bumi/tests/run_image_quality_ros_tests.sh
wsl.exe -d Ubuntu-22.04 --exec bash /mnt/d/Internship_Projects/card_1/phanthymotus-driver/noetix/bumi/tests/run_image_quality_ros_tests.sh 'test_image_quality*.py'
```

7项真实ROS单测/发布订阅全部通过；随后141项完整质量套件全部通过。脚本强制ROS_LOCALHOST_ONLY=1、隔离域219。真实消息为CompressedImage与String，验证QoS、新帧替换、无帧不重复、2Hz最大频率、JPEG异常/多原因、Guard重启旧票隔离、Node释放与共享context仍有效。测试进程自己初始化独立ROS context，生产Runtime不调用init。

测试环境：Windows Python3.13、OpenCV5/NumPy2；WSL Ubuntu22.04、ROS2 Humble、Python3.10、OpenCV4.10（单独/tmp测试target）、系统NumPy。没有声称ARM64/Jetson性能已验证。

## 此前构建历史（本轮复验结果见首节）

Driver使用 `powershell -NoProfile -ExecutionPolicy Bypass -File ./build-local.ps1`，ARM64官方Dockerfile与common额外上下文。先前直接docker build缺common的上下文问题已由脚本修正。正确上下文后，官方Ubuntu源TLS连接中断，apt无法取得索引/定位包；指定Tsinghua HTTPS镜像仍失败。容器curl诊断官方HTTP为502、HTTPS为SSL unexpected EOF。没有关闭TLS校验或无限重试。

Core执行 `docker build --platform linux/arm64 --progress plain -t image-quality-core:local -f Dockerfile .`，基础镜像的mirrors.tencentyun.com无法解析，apt无法定位ffmpeg。

本机诊断日志：`%TEMP%/iqg-driver-build.log`、`%TEMP%/iqg-driver-build-mirror.log`、`%TEMP%/iqg-core-build.log`；临时目录不是长期证据存储，不应依赖其部署。无镜像push或云端操作。

## 仍需人工审查的安全边界

严格票据保护覆盖Guard及显式opt-in Card。旧Card保持原接口，不获得新的Driver级生命周期身份。Core短项目gate可拒绝项目STOPPING/UNKNOWN期间新准入，但**不能证明此前已准入而网络延迟的旧Card start一定先于stop到达Driver**；若要求这一顺序保证，需要额外的旧Card调度/迁移设计，不能把兼容行为写成完整串行保证。

UNKNOWN项目下的子Card恢复被保守拒绝，避免重写冻结项目归属；Driver重启导致的该组合目前需要操作者核查及受控Core重启后恢复，不是自动项目恢复。复核前不要启用跨机器多Owner或把未知状态强制清为idle。

独立复核已发现并修复：recover跨await覆盖新owner、旧status覆盖新session、失败启动已释放资源却无法解除Core归属、前端终态ACK误读；均有回归。以上未覆盖边界仍是交付限制，需要同事确认后再进入PR准备。
