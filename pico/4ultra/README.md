# PICO Driver

独立设备卡 `teleop_device`，通过标准DDS输入连接机器人 `teleop_control`。Agent Core与ActuCore无需改代码、增加专用代理或启用旧teleop插件；机器人URDF、IK和执行均由机器人Driver承担，消息格式见 [输入契约](#输入契约)。

## 用户流程

1. 用普通部署入口部署PICO与机器人Driver，添加两张卡并连接 `teleop_device → teleop_control`。
2. 在 `teleop_device` 齿轮中首次自行设置四位数字管理 PIN，保存后复制 `https://<主机>:15741/onboarding` 到浏览器。网址由 Driver 生成，不能配置修改；不提供已安装勾选、设备名或滤波参数。后续留空保存保留已有 PIN，填写新 PIN 则更新。
3. 在PICO浏览器进入下载页、下载APK并确认系统安装；返回网页，输入管理 PIN 解锁，点击“连接这台机器人”，然后点击“打开 App 并连接”，地址自动填入。其他连接方式折叠在设备管理中：允许手动配对后，在App发现设备、核对指纹，网页会自动显示待批准请求。
4. 开启Canvas项目只启动采集与DDS。设备配对和项目运行都不会自动启动机器人；控制卡在有效的松握输入上准备基准，双握把使能；PICO不再发操作请求，执行状态在控制卡监控查看。
5. 已配对设备冷启动自动重连。停止Canvas只停止采集，控制卡按其停止语义保持，不追加收臂。

齿轮复制网址是用户已确认的首版设计。普通Core的实际齿轮保存、项目启动/停止和输入文字监控已在隔离浏览器验证；PICO浏览器安装与证书首次信任仍待设备验收。研发APK安装不能代替这条普通用户流程。

## 配置与生命周期

- 硬件目录与镜像为 `pico/4ultra`，显示名为 `PICO 4 Ultra`；Driver ID `pico-driver`，MCP只监听loopback 15742；HTTPS/WSS在15741，与原App端点兼容。
- `PICO_PUBLIC_HOST` 或机器配置 `pico.public_host` 指定头显可达地址；未指定时通过本机路由选择地址，不发送探测数据。TLS与配对保存在 `/var/lib/motus/pico`。普通重建不换证书，IP改变也不静默覆盖信任身份。
- 配置默认空闲，无安装确认门槛。首次齿轮保存或项目启动建立监听；`info`可无ROS/TLS实例地解析端口。
- 管理 PIN 由用户首次在齿轮配置中设置，没有默认 PIN；只接受四位 ASCII 数字（可含前导零）。Driver 只保存加盐哈希，文件权限 0600，不通过 `info/config` 返回 PIN。配置声明 `format=password` 和 `x-sensitive`，由普通 Core 的既有敏感字段机制处理导出脱敏。普通 Core 的本地配置数据库仍保存该配置值，密码类型仅提供界面掩码与分享脱敏，不代表 Core 数据库加密。齿轮字段沿用 `scope=instance` 以适配现有表单，Driver 只保存一份 PIN。旧 `pairing_admin_password` 不迁移为 PIN。
- 下载页面和 APK 公开；`/manage/login` 使用 JSON POST 验证 PIN，生成 Secure/HttpOnly/SameSite=Strict Cookie。管理状态、生成邀请、打开配对窗口、批准/拒绝与撤销均需该会话。会话固定 15 分钟；连续五次错误后暂停登录五分钟。限次全服务共享，会话与限次仅存内存，重启后会话失效、限次重新开始；不是公网账户系统。修改 PIN 撤销管理会话、未使用邀请和待批准请求，不删除已配对头显。已配对 App 重连无需 PIN。
- MCP 配置入口仅监听 loopback，依赖现有主机/Core 的访问控制；PIN 保护的是局域网 HTTPS 配对管理，不能替代主机权限。不要把设备凭据和私钥加入 Git。
- 普通Core通过既有注册与MCP调用工作；无需 `X-Ext-VR-Management`、`X-Teleop-Management` 或 `management_binding`。
- 容器无机器人设备映射、无privileged/host PID/host IPC。使用host network与现有本机DDS profile，域42；启动校验profile存在、禁用内建传输且仅允许loopback UDP，缺失或不合规直接失败。模板只持久化设备状态并挂载已有DDS配置。

## 输入契约

`teleop_device` 是设备输入源；机器人 `teleop_control` 负责映射、IK 和执行。设备 Driver 不加载机器人模型、不申请运动控制权，也不依赖 Agent Core 或 ActuCore 的专用改动。

### 接线与传输

Canvas 连接 `teleop_device` 输出到机器人 `teleop_control` 输入。设备只有一个输出：

| 项目 | 值 |
|---|---|
| Topic | `/teleop/command` |
| Format | `data/teleop-cmd` |
| Schema | `motus.teleop.command/1` |
| 消息 | `std_msgs/String` 中的 JSON |
| DDS | 同机 domain 42，部署加载本机隔离配置 |
| 当前 QoS | RELIABLE、KEEP_LAST(16)、VOLATILE |

卡片声明 `multiInstance=false`，当前同一 ROS domain 支持一个设备输入源。升级时复用唯一已有实例配置或配对文件（兼容旧版直接启动后配对、未保存配置的情况），改变画布卡 ID 不再新建第二个设备源；发现多份旧配置时报明确错误，不猜测采用哪份。设备和实例身份放在消息中，topic 不拼接品牌、日期或实例 ID。应用层仅保留最新待发姿态；DDS 历史深度不代表允许排队执行历史动作。

### 输入消息

`kind` 固定为 `input`。主要字段：

- `instance_id`、`device_id`：设备卡实例和设备身份。
- `connection_epoch`、`space_epoch`、`sequence`：连接、跟踪空间代次及输入序号。
- `clock_id`、`received_monotonic_ns`：接收主机的时钟身份及接收时间。
- `source_monotonic_ns`：头显采样时间，只用于追踪来源，不能直接与机器人时钟比较。
- `tracking_frame`：`tracking_x_forward_y_left_z_up`，右手坐标系，X 前、Y 左、Z 上。
- `head_reference`、`left`、`right`：`tracked`、位置 `position`（米）、单位四元数 `orientation_xyzw`；控制器额外携带 `[0,1]` 的 `grip` 和 `trigger`。

- 每侧可选 `controls`：`buttons` 按名称提供 `trigger/grip/thumbstick`、左 `x/y`、右 `a/b`；各项 `available` 为布尔，`value`（0–1）、`pressed`、`touched` 按设备实际支持情况提供。
- `controls.axes.thumbstick`：`available` 和二维 `value: [x,y]`（各分量 −1–1）。不可用项仅包含 `available=false`，不填零值或按键状态。
- 根级可选 `extensions` 为 JSON 对象。消费者不拒绝未知可选扩展；核心字段的有限值、类型和时效校验不变。缺少 `controls` 的既有消息仍有效。

输入 Driver 不做滤波，设备采集到的有效帧直接转换坐标并输出。动作平滑与执行属于机器人控制端。

未跟踪对象的位姿为 null。消费者校验格式、有限值、身份、代次、序号和时效；默认输入年龄上限为 300 ms。转发、心跳和滤波不得把旧采样时间刷新为当前时间。完整字段验证见 `common/teleop_contract.py` 的 `validate_input`。

### 生命周期与显示

设备配对只建立输入连接，不启动机器人运动。Canvas 启动和停止设备采集及机器人控制卡；握把是否使能由机器人控制卡处理。

PICO 显示连接状态、透视画面及握把提示，不发送 begin/finish/stop 操作请求，不订阅机器人执行反馈。机器人卡的 `/teleop/state` 用于 Canvas 监控，不是设备卡的输入或输出。共享模块中保留的历史操作/反馈辅助接口，不属于当前设备接线契约。

## 构建与验证

从Driver仓执行普通 `./build.sh pico/4ultra`；构建上下文自动纳入common；镜像只复制共享日志/输入契约及本bundle运行源码，不携带原生App源码、测试和构建缓存。无新增构建开关。Dockerfile安装锁定的通信依赖并核验固定正式APK。清单与源码版本不符会失败，不静默装旧APK。

### 镜像依赖与体积

PICO 是独立进程，复用 `ros-base` 提供的 ROS/Python，不依赖另一个服务容器的 Python 包。普通 `pip install` 不使用 `--upgrade` 或 `--force-reinstall`：已满足锁定版本的包会复用；基础镜像中版本不满足锁定版本的包才补充。基础镜像并未声明提供本卡所需的完整 WebRTC 依赖组合。

| 依赖 | 本卡用途 |
|---|---|
| PyYAML | 读取 Driver 配置；固定 6.0.2，避免随基础镜像中的系统版本变化 |
| aiohttp | HTTPS 管理、下载与 WSS 信令 |
| aiortc、aioice、pylibsrtp、av 等传递依赖 | WebRTC 数据通道、ICE/DTLS/SCTP；即使不用视频，aiortc 的标准安装/导入仍需要其传递依赖，不裁剪未验证的私有分支 |
| cryptography、pyOpenSSL/cffi 等 | TLS 证书、配对公钥验证以及 WebRTC 加密依赖 |
| zeroconf | 局域网 DNS-SD/mDNS 发现 |

2026-09-24 从 registry manifest 核对 `release.260923.46aee15`：压缩层合计 **304.76 MiB**，其完全相同的基础层合计 **249.46 MiB**，新增 **55.30 MiB**，其中通信依赖层 **52.15 MiB**、PyYAML 层 **0.74 MiB**、APK 层 **2.31 MiB**，其余为源码/配置。此为压缩传输层统计，不是解压磁盘占用；基础 config digest 为 `sha256:cf6b24578bf4e9d75812923f11d0386d3d13c03df92ff0bbf4f0672be2e58238`，不是永久固定的 `latest` 大小。

本卡明确接受这部分局部增长，以提供原生 WebRTC 接入和离线可用的安装包；不扩大其他 Driver 或基础镜像。APK 在构建时按清单下载入镜像，使部署完成后的局域网下载不再依赖 COS 在线，且 APK 与该 Driver 版本固定绑定。移动到启动时下载只会转移到容器可写层，并增加现场网络依赖。镜像不包含 SDK、JDK、Android 源码或构建缓存，不增加 BOT 开关。基础镜像 config 未设置 `PYTHONUNBUFFERED` / `RCUTILS_COLORIZED_OUTPUT`，本卡显式设置用于日志及时性和禁用 ROS ANSI 颜色。

### APK 校验边界

发布 APK 前，`stage_apk.py` 用 Android `apksigner verify` 验证真实签名，提取证书指纹，并用 aapt2/zipalign 核验包信息与对齐。清单中的证书指纹是**发布元数据**，不是容器运行时验签结果。

普通 Docker 构建的 `fetch_apk.py` 核对下载字节（含 gzip 源与解压后的 APK）的大小和 SHA256；下载接口 `package_metadata()` 再核对镜像内 APK 的大小和 SHA256，只报告 `verification=sha256`、`signature_verified=false`，不返回未经运行时提取的签名证书指纹。`available=true` 只表示与受版本控制的清单字节一致，不是独立签名认证；清单与发布流程是信任边界。容器不安装 Android 验签工具。

本地软件测试：

```sh
PYTHONPATH=.:tests python -m pytest -q tests/test_pico_device.py tests/test_pico_lifecycle.py tests/test_pico_management_pin.py tests/test_pico_onboarding.py tests/test_pico_transport.py tests/test_teleop_contract.py
```

安装页的浏览器回归覆盖 PIN 解锁、错误 PIN、会话过期/退出、邀请生成、过期重试、指纹批准及已配对重连，可用现有 Playwright 与 Chrome 运行：

```sh
PLAYWRIGHT_MODULE=/path/to/playwright/index.mjs CHROME_BINARY=/path/to/chrome node tests/pico_onboarding_ui.mjs
```

原生App构建、签名、来源许可证与安装边界见 [客户端手册](ext_vr/openxr_capture_native/README.md)。

普通Core兼容性可用 `tests/verify_pico_core_upstream.py <只读源码导出目录> --core-sha <SHA> --evidence <JSON>` 重跑，需要Core的Python依赖、`PLAYWRIGHT_MODULE`和`CHROME_BINARY`指向已有浏览器测试环境。该脚本只创建临时SQLite与loopback服务，使用真实Core模块和Driver MCP；DDS使用sink，不能替代真实通信测试。

当前输出固定为 `/teleop/command`，JSON 仍携带设备、实例、空间/连接代次和序号。当前单机器人单设备源；没有反向反馈订阅。`/teleop/state` 是机器人控制卡的监控输出，不接回本卡。PICO只显示配对连接与握把提示，不显示机器人动作结果，不提供开始/结束/停止按钮。


## 局域网发现

App 使用 Android NsdManager 发现 `_motus-teleop._tcp` DNS-SD/mDNS 服务，读取机器人广播的名称、地址和端口，不扫描 IP。头显与机器人应在允许组播的同一局域网；跨网段或访客 Wi-Fi 可能无法发现。此时使用下载页的连接入口，或在 App 输入地址。配对凭据保存在头显本地，后续打开 App 自动重连。

App 图标使用 PhanthyMotus [官方 Logo](https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/embodied_logo.svg)，按 Android 图标密度导出。
