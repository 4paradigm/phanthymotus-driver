# PICO 两 Driver 离线验证记录（2026-09-23）

本记录只覆盖 #329 独立 PICO Driver 候选，工作树 `pico-two-driver-20260923`。本次代码与文档一起交付，不以旧四卡记录代替本轮结果。共享协议及组合验证由主任务统一；没有修改机器人bundle、Core或ActuCore，没有部署、申请BOT或操作设备。

## 本轮通过

| 验证 | 实际结果 |
|---|---|
| Python设备/协议回归 | `PYTHONPATH=.:tests python -m pytest -q tests/test_pico_device.py tests/test_pico_lifecycle.py tests/test_pico_onboarding.py tests/test_pico_transport.py tests/test_teleop_contract.py`：85 passed；包括47项共享契约测试，不能叠加重复计数 |
| localhost MCP | 实际ThreadingHTTPServer、标准tools/call，无Core专属Header；未确认安装拒绝start，配置读回、start/info/stop正常，`error:null`不误报失败 |
| localhost HTTPS | 真实TLS监听，下载页、管理登录、Secure/HttpOnly cookie、CSRF、Origin验证、邀请生成及撤销；匿名配对管理拒绝 |
| localhost RTC | 真实WSS、ICE仅loopback、DataChannel；采集分配、规范化坐标和断连旧回调拒绝通过，不是实体PICO |
| 连续输入/停止 | 松握移动再握首帧为新位置，空间/连接代次不变；实际空间失效另处理；阻塞DDS替身最多1个pose+16个operation，stop优先；Bloom全置位/回执满/RTC tracking丢失仍可受理stop |
| 原生宿主 | `tests/run_host_tests.sh` 通过；生产采集assignment被C++解析器读取，帧/配对/会话/按钮与脚本契约通过 |
| 正式APK | code25，`0.4.1-pico1-operator1-ikview2`；实际Android ARM64 release构建、包/权限/ELF/对齐/许可/签名检查通过 |
| APK发布/下载 | 主任务完成不可变COS发布；匿名公网回读gzip/APK双SHA吻合，实际`fetch_apk.py`通过，正式manifest已锁定code25 |
| 普通Core隔离集成 | upstream/main `9802eae43f158af75a9848909d7366d15c985e63`原样导出；Chrome实际执行Core齿轮JS→普通Core API/SQLite→Driver MCP，注册发现、字段默认、密码类型、保存、分享脱敏、项目启动、输入文字显示、项目停止均通过 |
| 普通ARM64镜像 | 天轶Orin执行原样`build.sh --mirror tuna pico/pico`完成；无BOT修改、无额外开关、无镜像传输。最终镜像`sha256:94768a952f6dd0f5c041e0ca67e7d72c40405635fa78b76ca7824604a1c5a806`，未切换业务服务 |
| 真实DDS组合 | domain42 / loopback，无网络与硬件挂载的容器，实际RosTransport/BoundedWriter、OperatorCommands、控制卡、独立IK与有限速plant通过；首次47个输入、92次模拟执行写入，begin/finish/RTC断流后的stop均返回completed |
| 静态检查 | 本次Python文件编译、Black及`git diff --check`通过 |

DDS堵塞测试使用受控publisher替身，不能称为真实DDS集成。Core浏览器测试使用最小DOM容器、未修改的实际JS/API及SQLite；它不是完整生产页面或真实ROS监控链路。普通Core的`data/teleop-cmd`选择ActivityRenderer并显示安全数字`text`摘要。骨架默认关闭的Android代码已编译，透视/按钮仍需头显验证。

实际集成曾复现保存与启动auto-config并发写密码临时文件的失败；已串行化设备生命周期修改，复测通过，4个并发配置回归通过。RTC丢失后停止回执按原操作三项身份匹配，即使顶层仍是最后姿态的旧代次也能完成；错误设备/空间与重复server序号拒绝。

浏览器证据：`/private/tmp/pico-core-upstream-integration.json`及同名前缀gear/monitor截图、`.log`；JSON记录被执行的8个Core关键源文件SHA256。脚本为`tests/verify_pico_core_upstream.py`与`tests/pico_core_ui.mjs`。Core所有源码保持不变。

## APK身份

- 包名：`com.phanthymotus.picocapture`。
- APK：2,431,298 bytes，SHA256 `1cc279426ca8e94928e77b59a074c5bc3937b1a739d1fc61d52ae358b31b7938`。
- 签名证书SHA256：`12b025ef35194425aa994c025e87f3733ff02d30637fdcd9e78e89a488c38b5f`，与既有正式包身份一致。
- 确定性gzip：2,381,623 bytes，SHA256 `ebf5d162197c6aab5068014f6f79a8778e4865d144a09d4b65d08c2e2fe8c902`。
- 本地制品目录：`/private/tmp/pico-two-driver-release-20260923`；构建日志 `pico-two-driver-release-build-isolated.log`、检查 `pico-two-driver-apk-verify.json`、原生测试 `pico-two-driver-native-host.log` 均在 `/private/tmp`。
- COS回读凭据：同制品目录`publish-receipt.json`，`public_readback=verified`。

第一次构建受到用户全局Gradle init脚本的仓库注入影响而失败；改用已有任务GRADLE_USER_HOME后3分45秒完成。没有修改用户全局Gradle配置、没有替换签名。SDK曾提示platform-tools许可未接受，构建和检查仍实际成功；没有自动安装额外SDK。

## 尚未通过

- 完整生产Canvas页面及其真实ROS监控桥接；本轮已验证实际模块与API，但未启动完整Core应用。
- 完整冻结执行链路及现场跟随对照；本轮已完成真实DDS组合及719帧数值对照，具体集成证据由两个PR共同记录。
- PICO系统浏览器独立TLS首次信任、真实下载/安装/配对与透视按钮。
- 开发真机、BOT review、最终设备验收。

用户已接受齿轮复制网址的首版设计，Core保持零修改；实际字段为可选择文本input，没有可点击链接且未执行readOnly。Driver拒绝伪造安装地址。实体PICO浏览器流程仍不标通过。输入滤波默认关闭保留采集对照；30ms是可选实验参数。基线镜像/源码/标定完整对应由主任务核对，本记录不宣称冻结实机行为已复现。
