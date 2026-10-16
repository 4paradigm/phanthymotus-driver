# 来源与许可证

`kinematics.py`、`workspace.py` 和 `tianyi_visualization.py` 从 `4paradigm/phanthymotus` 提交 `4edefe81e0abba7aed56f16368c8af4f561915d0` 的 `actucore/plugins/teleop/` 迁入，适用仓库 Apache-2.0。只保留天轶运动学；本轮相对映射迁到同 Driver 的 `teleop_control.py`，输入为设备 Driver 已规范化的跟踪位姿。IK 目标函数参考 Driver PR #152（Apache-2.0），未采用 G1 模型或执行模式。

`models/tianyi2-official.urdf` 未修改复制自官方 `Open-X-Humanoid/TienKung_URDF` 提交 `5c221783fb92fcc4af891ef1dc0502963caf2266` 的 `tianyi2_urdf/urdf/tianyi2.0_urdf_with_hands.urdf`，SHA256 `a7e742ad600c7f1e9eeecdd04046dea4cd5f81cccb6dde73f290968b345e4b20`。该模型独立适用 OpenAtom Open Hardware License v1.0，完整文本保存在 `models/LICENSE`。模型来源不代表现场安装、碰撞或停止验收通过。

数值依赖沿用原 bundle 的 15 个固定版本并保留包哈希。安装包 metadata 核对：NumPy/SciPy 为 BSD；pin、coal、cmeel-urdfdom 为 BSD-3-Clause；cmeel、eigenpy 为 BSD-2-Clause；cmeel-tinyxml2 为 Zlib。依赖的完整许可证仍随各自 wheel 的 dist-info/许可证资源分发，不将它们重标为本仓许可证。此目录不包含 PICO SDK、G1 模型或厂商二进制。

The copied model license preserves its text; trailing whitespace is normalized.

`worker.py` 与 `motion_control.py` 的数值进程、取消及收臂回执改动选择性移植自 2026-09-23 本地 r9g 候选；仅移植已逐段审阅的两个文件，未采用旧工作树其他变更。迁移源文件 SHA256 分别为 `1223ce400b5e9f1a0fdbd42ed5c29b0f4355f543f97037db8b3307b33ae3c20c`、`c610da0592e440eb15ea0ad21bae941f72dd638faa0a8bb169df97924312755c`。适用 Apache-2.0；本轮继续改为完整关节参考与经证明的独立关节运动包络，不能把该候选或文档视为新真机验收。

`teleop_control.py` 的相对映射复用原 ActuCore `DualArmMapping` 与冻结 r4 `RelativeMapping` 数学语义（Apache-2.0），保留 `controller_to_palm` 的平移和旋转标定。因设备输入已从 OpenXR 转成 X 前/Y 左/Z 上，标定变换同步换基。重握不再重标定是本轮明确变更，不将冻结版旧 clutch 行为当成必须保留的功能。
