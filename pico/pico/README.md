# PICO Driver

独立设备卡 `teleop_device`，通过标准DDS输入连接天轶 `teleop_control`。Agent Core与ActuCore无需改代码、增加专用代理或启用旧teleop插件；机器人URDF、IK和执行均由天轶Driver承担。完整目标与验收顺序见 [实施契约](../../docs/plans/pico-teleop-device.md)。

## 用户流程

1. 用普通部署入口部署PICO与天轶Driver，添加两张卡并连接 `teleop_device → teleop_control`。
2. 在PICO卡齿轮页阅读安装步骤，设置配对管理密码（至少12字符）。复制Driver显示的安装地址 `https://<主机>:15741/onboarding`，在浏览器打开。普通Core显示为文本输入框，首版不提供可点击链接；地址由Driver生成，不要编辑。默认“是否已经安装遥操驱动”为否，完成安装后手动选择是。
3. 在PICO浏览器进入下载页、下载APK并确认系统安装；返回网页，登录后生成一次性邀请，点击“打开并连接”。也支持打开配对窗口后，在App发现设备、核对指纹、网页批准。
4. 开启Canvas项目只启动采集与DDS。设备配对和项目运行都不会自动启动机器人；实际开始/结束/停止由PICO按钮请求天轶卡处理。
5. 已配对设备冷启动自动重连。停止Canvas只停止采集，天轶控制卡按其停止语义保持，不追加收臂。

齿轮复制网址是用户已确认的首版设计。普通Core的实际齿轮保存、项目启动/停止和输入文字监控已在隔离浏览器验证；PICO浏览器安装与证书首次信任仍待设备验收。研发APK安装不能代替这条普通用户流程。

## 配置与生命周期

- Driver ID `pico-driver`，MCP只监听loopback 15742；HTTPS/WSS在15741，与原App端点兼容。
- `PICO_PUBLIC_HOST` 或机器配置 `pico.public_host` 指定头显可达地址；未指定时通过本机路由选择地址，不发送探测数据。TLS与配对保存在 `/var/lib/motus/pico`。普通重建不换证书，IP改变也不静默覆盖信任身份。
- 配置默认空闲、未确认安装。首次齿轮配置建立监听；`info`可无ROS/TLS实例地解析端口。
- 配对页密码使用标准 `format=password` / `x-sensitive` 配置；Driver只存哈希，info不回读密码。不要把状态目录、私钥和Canvas私有配置加入Git。
- 普通Core通过既有注册与MCP调用工作；无需 `X-Ext-VR-Management`、`X-Teleop-Management` 或 `management_binding`。
- 容器无机器人设备映射、无privileged/host PID/host IPC。使用host network与现有本机DDS profile，域42；启动校验profile存在、禁用内建传输且仅允许loopback UDP，缺失或不合规直接失败。模板只持久化设备状态并挂载已有DDS配置。

## 构建与验证

从Driver仓执行普通 `./build.sh pico/pico`；构建上下文自动纳入common，无新增开关。Dockerfile安装锁定的通信依赖并核验固定正式APK。清单与源码版本不符会失败，不静默装旧APK。

本地软件测试：

```sh
PYTHONPATH=.:tests python -m pytest -q tests/test_pico_device.py tests/test_pico_lifecycle.py tests/test_pico_onboarding.py tests/test_pico_transport.py tests/test_teleop_contract.py
```

原生App构建、签名、来源许可证与安装边界见 [客户端手册](../../common/ext_vr/openxr_capture_native/README.md)。阶段严格为离线及普通Core集成、开发设备/真机、精确提交BOT review、最终实机。没有BOT或部署自动动作。

普通Core兼容性可用 `tests/verify_pico_core_upstream.py <只读源码导出目录> --core-sha <SHA> --evidence <JSON>` 重跑，需要Core的Python依赖、`PLAYWRIGHT_MODULE`和`CHROME_BINARY`指向已有浏览器测试环境。该脚本只创建临时SQLite与loopback服务，使用真实Core模块和Driver MCP；DDS使用sink，不能替代真实通信测试。结果边界见 [离线记录](../../docs/validation/pico-two-driver-offline-20260923.md)。
