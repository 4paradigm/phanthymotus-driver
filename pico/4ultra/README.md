# PICO Driver

独立设备卡 `teleop_device`，通过标准DDS输入连接机器人 `teleop_control`。Agent Core与ActuCore无需改代码、增加专用代理或启用旧teleop插件；机器人URDF、IK和执行均由机器人Driver承担，消息格式见 [输入契约](#输入契约)。

## 用户流程

1. 用普通部署入口部署PICO与机器人Driver，添加两张卡并连接 `teleop_device → teleop_control`。
2. 在 `teleop_device` 查看安装说明，复制 `https://<主机>:15741/onboarding` 到浏览器。网址由 Driver 生成，不能配置修改；无需密码、已安装勾选、设备名或滤波参数。齿轮保留固定“无需配置”说明项，以兼容普通 Core 的按钮显示规则。
3. 在PICO浏览器进入下载页、下载APK并确认系统安装；返回网页，点击“连接这台机器人”，然后点击“打开 App 并连接”，地址自动填入。其他连接方式折叠在设备管理中：允许手动配对后，在App发现设备、核对指纹，网页会自动显示待批准请求。
4. 开启Canvas项目只启动采集与DDS。设备配对和项目运行都不会自动启动机器人；控制卡在有效的松握输入上准备基准，双握把使能；PICO不再发操作请求，执行状态在控制卡监控查看。
5. 已配对设备冷启动自动重连。停止Canvas只停止采集，控制卡按其停止语义保持，不追加收臂。

齿轮复制网址是用户已确认的首版设计。普通Core的实际齿轮保存、项目启动/停止和输入文字监控已在隔离浏览器验证；PICO浏览器安装与证书首次信任仍待设备验收。研发APK安装不能代替这条普通用户流程。

## 配置与生命周期

- 硬件目录与镜像为 `pico/4ultra`，显示名为 `PICO 4 Ultra`；Driver ID `pico-driver`，MCP只监听loopback 15742；HTTPS/WSS在15741，与原App端点兼容。
- `PICO_PUBLIC_HOST` 或机器配置 `pico.public_host` 指定头显可达地址；未指定时通过本机路由选择地址，不发送探测数据。TLS与配对保存在 `/var/lib/motus/pico`。普通重建不换证书，IP改变也不静默覆盖信任身份。
- 配置默认空闲，无安装确认门槛。首次齿轮保存或项目启动建立监听；`info`可无ROS/TLS实例地解析端口。
- 当前配对管理页不要求密码或登录；用户明确选择此局域网流程。旧密码配置不再使用或写入。窗口、指纹批准与已配对设备凭据仍保留；配对不授予机器人运动权限。不要把设备凭据和私钥加入 Git。
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

当前同一 ROS domain 支持一个设备输入源。设备和实例身份放在消息中，topic 不拼接品牌、日期或实例 ID。应用层仅保留最新待发姿态；DDS 历史深度不代表允许排队执行历史动作。

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

本地软件测试：

```sh
PYTHONPATH=.:tests python -m pytest -q tests/test_pico_device.py tests/test_pico_lifecycle.py tests/test_pico_onboarding.py tests/test_pico_transport.py tests/test_teleop_contract.py
```

安装页的浏览器回归覆盖邀请生成、过期重试、指纹批准及已配对重连，可用现有 Playwright 与 Chrome 运行：

```sh
PLAYWRIGHT_MODULE=/path/to/playwright/index.mjs CHROME_BINARY=/path/to/chrome node tests/pico_onboarding_ui.mjs
```

原生App构建、签名、来源许可证与安装边界见 [客户端手册](ext_vr/openxr_capture_native/README.md)。

普通Core兼容性可用 `tests/verify_pico_core_upstream.py <只读源码导出目录> --core-sha <SHA> --evidence <JSON>` 重跑，需要Core的Python依赖、`PLAYWRIGHT_MODULE`和`CHROME_BINARY`指向已有浏览器测试环境。该脚本只创建临时SQLite与loopback服务，使用真实Core模块和Driver MCP；DDS使用sink，不能替代真实通信测试。

当前输出固定为 `/teleop/command`，JSON 仍携带设备、实例、空间/连接代次和序号。当前单机器人单设备源；没有反向反馈订阅。`/teleop/state` 是机器人控制卡的监控输出，不接回本卡。PICO只显示配对连接与握把提示，不显示机器人动作结果，不提供开始/结束/停止按钮。


## 局域网发现

App 使用 Android NsdManager 发现 `_motus-teleop._tcp` DNS-SD/mDNS 服务，读取机器人广播的名称、地址和端口，不扫描 IP。头显与机器人应在允许组播的同一局域网；跨网段或访客 Wi-Fi 可能无法发现。此时使用下载页的连接入口，或在 App 输入地址。配对凭据保存在头显本地，后续打开 App 自动重连。

App 图标使用 PhanthyMotus [官方 Logo](https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/embodied_logo.svg)，按 Android 图标密度导出。
