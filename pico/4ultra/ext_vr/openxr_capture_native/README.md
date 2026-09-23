# PICO 原生遥操 App

此模块由独立PICO Driver维护，复用原生Android OpenXR、现场透视、WSS配对与WebRTC采集。当前 `versionCode=28`，`0.4.4-pico-input`。默认不显示机器人骨架/IK线，仅保留透视、配对连接状态及握把提示。App不求解IK、不发送机器人关节命令；末端映射与执行由机器人 `teleop_control` 承担。

## 安装、连接与使用

从Canvas PICO卡齿轮复制网址，在 PICO 浏览器进入Driver托管的安装配对页。先下载安装App，再回到网页依次点击“连接这台机器人”和“打开 App 并连接”，预填当前设备地址和证书身份；不能假设系统安装器首次打开会继承网页URL。短网址为兜底；扫码能力需要PICO实机确认。安装、权限及MR安全边界仍需头显确认。

配对后冷启动自动连接保存的设备，普通断网按退避重连。改变配对证书不会静默重新信任；同签名覆盖升级保留本地配对，卸载会丢失配对。配对服务WSS15741路径保留 `/ws/teleop-capture`。首次浏览器自签TLS信任和网页唤起必须在实机验证，不以ADB安装替代。

透视页不再提供开始、结束或停止按钮，也不接收机器人执行反馈。先在 Canvas 启动控制卡，保持双握把松开让其就绪，然后按住两侧握把遥操；执行状态从控制卡监控查看。Canvas 停止停止遥操，G1 收臂复用 arm release。PICO 的“已配对连接”仅代表设备通信，不代表机器人执行成功。

松握只表示暂停运动，App仍采集当前位置。再次双握是下一有效帧，**不重新建基准、不重置空间或会话**。真正OpenXR空间重置退出当前采集，旧空间不能继续用。失焦、休眠、透视失效或跟踪丢失不会伪造有效输入。

## 构建

固定组件：JDK17、Android35、build-tools35.0.1、NDK27.0.12077973、CMake3.22.1、Gradle8.9；OpenXR loader1.1.60、libdatachannel0.24.3、MbedTLS3.6.7。源码保留Meta flavor，共享编译通过不意味着Meta设备已验收。

```sh
export JAVA_HOME=/path/to/jdk-17
export ANDROID_SDK_ROOT=/path/to/sdk
./scripts/build_android.sh --platform pico --build-type release
python3 tests/verify_android_apks.py --platform pico --build-type release
```

Release需要现有稳定签名变量 `MOTUS_APK_KEYSTORE/MOTUS_APK_STORE_PASSWORD/MOTUS_APK_KEY_ALIAS/MOTUS_APK_KEY_PASSWORD`；密钥和密码不写Git、Docker context或日志。不得回退debug假装可覆盖升级。若用户全局Gradle init脚本改写仓库规则，使用独立 `GRADLE_USER_HOME`，不修改用户全局配置。

构建后 `scripts/stage_apk.py` 校验真实包信息、签名与对齐，生成正式制品。维护者发布不可变文件后更新 `package-manifest.json` 并核实下载字节；普通Driver镜像自动下载和验包，不要求用户传构建开关。当前远端清单是否已更新以实际验证记录为准，APK构建不等于普通镜像已包含它。

包名固定 `com.phanthymotus.picocapture`，只构建arm64。PICO4 Ultra为首轮设备。系统/loader具体兼容性仍按实机记录，不扩大支持声明。

## 离线检查与调试

```sh
./tests/launch_capture_test.sh
python3 tests/launcher_manifest_test.py
PYTHON=/path/to/transport-venv/bin/python NLOHMANN_JSON_INCLUDE=/path/to/include ./tests/run_host_tests.sh
```

可使用原 `launch_capture.sh --platform pico --resume` 和ADB进行研发；日常用户不需要USB。`record_input_only=10` 的NativeActivity调试入口只录制OpenXR输入，不连接机器人。宿主测试、编译、签名验证与真实安装/透视/控制器操作分别记录。

## 来源与许可证

客户端选择性迁移自Driver PR151，经过ActuCore采集实现采用至Driver，保持来源说明：[NOTICE](../NOTICE.md)、[ADOPTED_SOURCE](../ADOPTED_SOURCE.json)、[THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES.md)。第三方与Android NDK完整许可证保留随包分发，不删除或裁剪。

图标沿用 [PhanthyMotus 官方 SVG](https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/embodied_logo.svg)，源文件 SHA256 为 `bd650bbddc71eeef89840b5945e33d5507e2f30cccde9ccfee203a66b154f8f4`。使用 `rsvg-convert -w <尺寸> -h <尺寸>` 导出 48、72、96、144、192 像素的 Android launcher PNG，未重绘或改变品牌图案。

## 手柄输入

采集使用 Khronos [PICO controller interaction](https://registry.khronos.org/OpenXR/specs/1.1/html/xrspec.html#XR_BD_controller_interaction) 与 [Ultra interaction](https://registry.khronos.org/OpenXR/specs/1.1/html/xrspec.html#XR_BD_ultra_controller_interaction) 定义的 component paths：左 X/Y、右 A/B 的 click/touch，trigger value/click/touch，squeeze value/click，thumbstick 二维轴/click/touch。系统菜单键不参与绑定。

旧的扳机/握把字段保持兼容；命名输入位于每侧 `controls.buttons` 和 `controls.axes`。运行时无有效输入时报告 `available=false` 并省略测量字段，不伪造松开或零值。不对设备输入进行平滑或机器人运动学处理。
