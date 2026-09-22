# 来源与许可证

`kinematics.py`、`workspace.py` 和 `tianyi_visualization.py` 从 `4paradigm/phanthymotus` 提交 `4edefe81e0abba7aed56f16368c8af4f561915d0` 的 `actucore/plugins/teleop/` 迁入，适用仓库 Apache-2.0。只保留天轶运动学；PICO 相对映射留在 ActuCore。IK 目标函数参考 Driver PR #152（Apache-2.0），未采用 G1 模型或执行模式。

`models/tianyi2-official.urdf` 未修改复制自官方 `Open-X-Humanoid/TienKung_URDF` 提交 `5c221783fb92fcc4af891ef1dc0502963caf2266` 的 `tianyi2_urdf/urdf/tianyi2.0_urdf_with_hands.urdf`，SHA256 `a7e742ad600c7f1e9eeecdd04046dea4cd5f81cccb6dde73f290968b345e4b20`。该模型独立适用 OpenAtom Open Hardware License v1.0，完整文本保存在 `models/LICENSE`。模型来源不代表现场安装、碰撞或停止验收通过。

数值依赖沿用原 bundle 的 15 个固定版本并保留包哈希。安装包 metadata 核对：NumPy/SciPy 为 BSD；pin、coal、cmeel-urdfdom 为 BSD-3-Clause；cmeel、eigenpy 为 BSD-2-Clause；cmeel-tinyxml2 为 Zlib。依赖的完整许可证仍随各自 wheel 的 dist-info/许可证资源分发，不将它们重标为本仓许可证。此目录不包含 PICO SDK、G1 模型或厂商二进制。

The copied model license preserves its text; trailing whitespace is normalized.
