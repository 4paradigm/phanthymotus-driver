# PICO 原生遥操 App

此模块由独立PICO Driver维护，复用原生Android OpenXR、现场透视、WSS配对与WebRTC采集。当前 `versionCode=36`，`0.4.12-pico-handback`。G1 透视中显示橙色映射目标、目标点、方向参照及左右末端误差，不绘制绿色实测骨架。App不求解IK、不发送机器人关节命令；末端映射与执行由机器人 `teleop_control` 承担。

## 安装、连接与使用

从Canvas PICO卡齿轮复制网址，在 PICO 浏览器进入Driver托管的安装配对页。先下载安装App，再回到网页输入齿轮中设置的四位管理 PIN，依次点击“连接这台机器人”和“打开 App 并连接”，预填当前设备地址和证书身份；不能假设系统安装器首次打开会继承网页URL。短网址为兜底；扫码能力需要PICO实机确认。安装、权限及MR安全边界仍需头显确认。

配对后冷启动自动连接保存的设备，普通断网按退避重连。改变配对证书不会静默重新信任；同签名覆盖升级保留本地配对，卸载会丢失配对。配对服务WSS15741路径保留 `/ws/teleop-capture`。首次浏览器自签TLS信任和网页唤起必须在实机验证，不以ADB安装替代。

启动 Canvas 项目后，确认机器人和操作者双臂自然下垂，松开双 Grip，长按 X+A 一秒确认基准。首次模型准备可能需要数秒，保持按钮直到出现“请松开组合键”；松键后重新双握接通。结束本轮时，松开双 Grip，长按 Y+B 一秒。G1 停止遥操目标，并复用 Canvas 收尾的控制权渐降与交还确认。头显收到确认后显示“控制权已交还”。设备和配对连接保持运行，旧基准清除；下次遥操长按 X+A 重新确认，松键后重新双握接通。Reset 不主动移动到 reset_pose.json，也不发送动作 99。交还失败仍显示错误，不自动接通。

G1 映射目标为橙色实线，目标肘部来自机器人 IK，不是人体实测值。毫米和角度只供参考，不作为接通条件。头显显示中文控制状态；没有 IK 骨架时仍显示有效状态和错误原因。连接但没有 assignment 时提示在 Canvas 启动智能控制；已分配但没有反馈时等待机器人反馈。反馈过期撤销旧接通提示，不推断机器人已停止或复位。App 只透传已有按键，不计算接通或复位判定。App、PICO Driver 和配套 G1 Driver 一起升级，不增加协议兼容协商。

失焦、跟踪丢失后按机器人状态重新确认或松握再接通；设备重连或 OpenXR 空间重置后重新确认下垂基准。反馈或目标过期后保留橙色历史骨架，并显示过期提示。新反馈恢复后更新骨架；跟踪丢失、断线或代次改变时清除旧显示。历史骨架不作为当前姿态或接通依据。PICO 的“已配对连接”仅代表设备通信，不代表机器人已接通或已停止。Reset 的控制权交还仍需现场验收。

尚无骨架时显示“暂无手臂预览”，不表示输入断流。首次基准确认前可以没有骨架；按顶部机器人控制提示操作。当前连接状态以新鲜反馈为准。

网页与 Android 连接页使用 Canvas 暖色主题。Android 保留原生控件、横屏和滚动；连接、备用配对与连接管理分区显示，输入键盘使用窗口缩放。导入另一机器人邀请时要求“忘记并导入”；取消保留原凭据、地址及邀请。只有确认后才清理旧凭据并导入新邀请；导入不发连接请求，仍需点击连接。证书校验及同机器人冷启动重连保持。

XR 保留透明背景。文字使用开发期生成的 24 像素高字形和深色描边，覆盖显示文案所需中文、可打印 ASCII、数字及标点。UTF-8 按字符查找，异常字节显示占位。固定控制提示最多三行，原始原因最多两行；超出时显示 `...`。布局扫描与绘制量受可见行宽限制。非 G1 分支保留原有模型语义；坐标、双目、头部跟随及过期判断不变。字体数据由 `scripts/generate_operator_labels.swift` 生成，不增加运行时字体依赖。构建与宿主检查不能替代 Android 确认/取消回调及 PICO 明暗背景、双目和帧率验收。

旧头显开始／结束／停止按钮及射线交互已移除。App 只采集按键并显示机器人反馈，X+A／Y+B 的判定仍由 G1 执行。

## 构建

固定组件：JDK17、Android35、build-tools35.0.1、NDK27.0.12077973、CMake3.22.1、Gradle8.9；OpenXR loader1.1.60、libdatachannel0.24.3、MbedTLS3.6.7。源码保留Meta flavor，共享编译通过不意味着Meta设备已验收。

```sh
export JAVA_HOME=/path/to/jdk-17
export ANDROID_SDK_ROOT=/path/to/sdk
./scripts/build_android.sh --platform pico --build-type release
python3 tests/verify_android_apks.py --platform pico --build-type release
```

Release需要现有稳定签名变量 `MOTUS_APK_KEYSTORE/MOTUS_APK_STORE_PASSWORD/MOTUS_APK_KEY_ALIAS/MOTUS_APK_KEY_PASSWORD`；密钥和密码不写Git、Docker context或日志。不得回退debug假装可覆盖升级。若用户全局Gradle init脚本改写仓库规则，使用独立 `GRADLE_USER_HOME`，不修改用户全局配置。

构建后 `scripts/stage_apk.py` 校验真实包信息、签名与对齐，生成正式制品。维护者发布不可变文件后更新 `package-manifest.json` 并核实下载字节；普通Driver镜像自动下载并核对大小/SHA256，不再次执行 APK 签名验证，不要求用户传构建开关。清单证书指纹仅为发布元数据，运行时下载接口明确报告仅哈希校验、不报告签名已验证。当前远端清单是否已更新以实际验证记录为准，APK构建不等于普通镜像已包含它。

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
