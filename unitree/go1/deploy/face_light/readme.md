# face_light 的 SDK 部署与升级

`face_light` 是控制卡，默认启用并统一使用官方 SDK；旧 MQTT 后端不再提供。
镜像不包含私有 SDK，需先准备 SDK 并停止其他灯光写入源，否则卡片不可用或灯光会被覆盖。

1. 从现场头部 Nano 核实并取得 `faceLightSDK_Nano`。已审计来源为
   `/home/unitree/Unitree/sdk/faceLightSDK_Nano`，版本 `v1.0.1: first UDP version`；
   必需文件为 `include/FaceLightClient.h`、`include/LEDPixel.h`、`version.txt`
   和与运行主控架构匹配的 `lib/libfaceLight_SDK_arm64.so` 或 `lib/libfaceLight_SDK_amd64.so`。
   校验值及接入依据见 [SDK_AUDIT.md](SDK_AUDIT.md)。其他版本需重新审计，不能跳过校验。
2. 将 SDK 放在主控宿主机 `/opt/phanthy-motus/data/go1/faceLightSDK_Nano`，
   确保驱动容器内同一路径可读。若部署已经挂载整个 data 目录，沿用该挂载；
   否则在驱动服务的 Compose `volumes` 中添加以下只读挂载，再重建该驱动容器：

   ```yaml
   volumes:
     - /opt/phanthy-motus/data/go1/faceLightSDK_Nano:/opt/phanthy-motus/data/go1/faceLightSDK_Nano:ro
   ```

   不要将厂商 SDK 提交到 Git 或打进镜像。以下命令在主控执行，容器名按实际部署替换：

   ```bash
   docker exec embodied-unitree-go1 /deploy/face_light/run_sdk.sh --check
   ```

   必须返回 `VERIFIED official faceLight SDK v1.0.1; no hardware command sent`。
   此检查只验证文件和校验值，不加载 SDK，也不发送灯光指令。
3. 获得现场负责人允许后，在头部 Nano 核实 `faceLightMqtt` 和 `faceLightServer` 进程。
   若仍使用已审计的自启布局，先备份
   `/home/unitree/Unitree/autostart/faceLightMqtt/faceLightMqtt.sh`，
   仅注释其中启动 `./bin/faceLightMqtt &` 的行；若由其他服务管理，应停止并禁用对应 MQTT 灯光服务。
   然后在 Nano 上结束已有 MQTT 灯光写入进程，并复查：

   ```bash
   pgrep -a -x faceLightMqtt
   pkill -TERM -x faceLightMqtt
   pgrep -a -x faceLightMqtt     # 应无输出；若重新出现，先处理其自启来源
   pgrep -a -x faceLightServer  # 必须仍在运行
   ```

   使用进程所属用户或经授权的管理员身份操作；保持 `faceLightServer` 运行，
   同时停止其他 SDK 灯光客户端。停止桥接后，依赖旧 `face_light/color` MQTT 主题的程序将无法控制灯带。
   回退前先停止 SDK 卡片，再恢复备份的自启配置和 MQTT 灯光服务，避免两个写入源同时运行。
4. 保持 `plugins.face_light` 的 `enabled: true`、`backend: sdk`、`sdk_exclusive: true`。
   卡片不再提供配置齿轮，SDK 独占默认开启（Yes）。刷新驱动工具信息后，
   平台不再自动应用旧的画布 MQTT/No 配置；宿主机驱动配置仍需满足以上取值。
   默认 Yes 是部署前提声明，卡片不会代为停止现场进程。
   启动卡片后先调用 `info`，检查 `config_valid`、`available` 和 `connected`；失败时查看 `unavailable_reason`。
   首次启动会编译 SDK 适配器；若首次编译超时，确认编译完成后重试。
   `--check` 成功和 `info` 可用都不代表灯带已实际显示，颜色和灯效仍需现场观察验收。

`set_color`、`preset`、`off` 保留原调用方式。`set_led` 编号范围 0–11；
`set_leds` 可选 `color_format: hex`（12 个以空格或逗号分隔的六位 RGB 颜色）或
`rgb_array`（原 12 组 RGB 数组，画布可粘贴 JSON）。RGB 与编号留空按 0 执行，
`colors` 留空将全部灯设为黑色。RGB 输入框底字为 `0`，周期、时长底字分别为 `2`、`5`；
底字只是提示，不会覆盖手动输入，留空分别使用对应默认值（周期、时长单位为秒）。
`fade` 在 `duration_s` 内单向切换起始 RGB 到目标 `to_r/to_g/to_b`，随后保持目标颜色；
闪烁、呼吸、流水灯到期关闭。新灯光指令会抢占旧灯效。
