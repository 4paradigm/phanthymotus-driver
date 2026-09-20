"""末端位姿 → 关节角。阻尼最小二乘（CLIK），不是 NLP。

一个 `eef_pose` 的指令流到了驱动，剩下的问题是「这台机器人要把关节摆成什么样」。
那是**机器人的知识**，所以它在边端，不在云端 —— 云端只负责把模型的私有布局抹平成
标准的绝对末端位姿（`phanthymotus-cloud/runtimes/common/normalize.py`）。

## 为什么在边端解，而不是云端

动作块是 **833 ms 的开环**。`actucore/plugins/vla/plugin.py` 的 `next_command()`
只有在**块用完了**才去取新的一块，`chunk_size=25` @ 30 Hz 就是机器人执行完 25 步
才重新看一眼世界。

这对 IK 放哪是决定性的：7 自由度手臂对同一个手部位姿有**无穷多组解**（肘部可上可
下），求解器靠「离种子最近」挑一个。云端解的话，种子是 833 ms 前的关节角，而一个
过时的种子可能让求解器**跳到另一个解支** —— 手的位姿是对的，整条手臂甩过去。边端
在每一步下发之前才解，种子永远是此刻的真实构型，把陈旧度从 833 ms 压到 33 ms。

## 为什么是 CLIK，不是 NLP

宇树的 `g1_arm_ik.py` 第一行是 `from pinocchio import casadi as cpin`，而 PyPI 的
`pin` wheel **不带 casadi 绑定**（要 `BUILD_WITH_CASADI_SUPPORT=ON` 编译）。这跟
平台无关，任何机器上都一样。

但就算能跑，**NLP 对一个 30 Hz 的控制回路本来就是错的工具**：宇树那套是给遥操作
重定向用的，容忍几十毫秒、把关节限位当硬约束。控制回路只需要在上一解附近迭代几次
雅可比，快两个数量级而且确定性。限位在这里是每步 clamp，不是硬约束 —— 一个「解不
出来就整步失败」的求解器，在 30 Hz 上等于周期性丢指令。

Orin 6（JetPack 6.1，aarch64，Python 3.10）实测，纯 pinocchio，锁掉腿只解 7 个臂
关节，warm start：中位 0.239 ms、p95 0.259 ms、最大 0.313 ms，迭代中位 3 次，
280 次全部收敛。30 Hz 的预算是 33.3 ms，占 0.8%。

## 解不出来必须是响亮的

`solve()` 返回残差，调用方**必须检查**。这是唯一能让「解不出来」变成一次可观察的
失败的办法：迭代法不收敛时返回的是**某个**关节角，不是异常 —— 一个离目标半米远、
但每个分量都在限位内、能通过下游每一道检查的关节角。

## 腰不是被锁住的，是被**给定**的

腰在 URDF 里就在 pelvis 和两条手臂之间（`pelvis → waist_yaw → waist_roll →
waist_pitch → torso → shoulder …`），绕不过去。而标准动作向量里的腰是模型**指令**
的值，不是实测值。所以这里不用 pinocchio 的 `buildReducedModel` 把它烘死 —— 那会把
一个每步都可能变的量固化进模型 —— 而是保留完整模型，每次把腰写进构型向量，只在
**臂关节那几列**上求解。代价是雅可比多算几列，收益是腰变化时什么都不用重建。

## 依赖

`pinocchio` 与 `numpy` 都是**函数内 import**。`common/control` 被这个仓库里每一个
bundle import，而其中绝大多数既不解 IK 也装不起 200 MB 的 pinocchio；把它抬到模块
顶层会让所有驱动一起背上这个依赖。缺了它是**拒绝启动**并写出包名，不是降级成不解
IK —— 和 `unitree/g1/servo.py` 处理 Dex1 idl 同一条规矩。
"""

from __future__ import annotations

import math

# 收敛判据。位置 1 mm、姿态 1 mrad —— 比 G1 手臂的重复定位精度小一个量级，
# 所以它约束的是求解器而不是硬件。
POSITION_TOLERANCE = 1e-3          # m
ROTATION_TOLERANCE = 1e-3          # rad
MAX_ITERATIONS = 50
# 阻尼。大了收敛慢，小了在奇异位形附近 dq 爆掉 —— 而爆掉的那一步恰好会被下游的
# 步长钳位削成一个「看起来正常」的指令。1e-2 是 CLIK 的常用量级。
DAMPING = 1e-2
# 单步位移上限，防止一次迭代跨过整个工作空间。
MAX_STEP = 0.2                     # rad / 每个关节每次迭代


class KinematicsError(RuntimeError):
    """模型建不起来，或者链路描述和 URDF 对不上。"""


class ArmChain:
    """一条 base → tip 的链，外加「哪些关节允许动」。

    一个实例只解一条手臂。双臂就是两个实例共用同一份 URDF 文本 —— 模型各建一份，
    因为求解过程会写 `data`，而共享 `data` 的两条手臂会互相踩，表现是偶发的、
    和负载相关的错误解。
    """

    def __init__(self, urdf_path: str, *, tip_link: str, joint_names,
                 damping: float = DAMPING,
                 max_iterations: int = MAX_ITERATIONS,
                 position_tolerance: float = POSITION_TOLERANCE,
                 rotation_tolerance: float = ROTATION_TOLERANCE):
        try:
            import numpy as np
            import pinocchio as pin
        except ImportError as error:      # pragma: no cover - 环境问题，不是逻辑
            raise KinematicsError(
                f"解 IK 需要 pinocchio 和 numpy，而它们不在这个环境里（{error}）。"
                "装 `pin` 与 `numpy`。**不要**把这张卡片降级成不解 IK —— "
                "那等于让一张声明了 eef_pose 的卡片收下位姿然后什么都不做"
            ) from error
        self._np = np
        self._pin = pin

        import os

        if not os.path.isfile(urdf_path):
            # pinocchio 对不存在的文件报的是 "does not contain a valid URDF
            # model" —— 一句指向文件内容的话，而问题是路径。容器里的默认路径
            # （/work/resource/...）在别处都不成立，测试和本地跑最常撞的就是这个。
            raise KinematicsError(f"URDF 不存在：{urdf_path}")
        try:
            self._model = pin.buildModelFromUrdf(urdf_path)
        except Exception as error:
            raise KinematicsError(f"{urdf_path} 解析失败：{error}") from error
        self._data = self._model.createData()

        if not self._model.existFrame(tip_link):
            raise KinematicsError(
                f"URDF 里没有 {tip_link!r} 这个 frame。已有的末端候选："
                + ", ".join(sorted(
                    f.name for f in self._model.frames if "hand" in f.name))
            )
        self._tip = self._model.getFrameId(tip_link)

        # 允许动的那些关节在**速度**向量里的下标。用 idx_v 而不是 idx_q：对 G1 的
        # 全转动关节两者相等，但一个带浮动基或球关节的 URDF 会让它们错开，而错开
        # 之后雅可比的列和 dq 的分量就对不上了 —— 结果是一个收敛到别处的解。
        self._joint_names = tuple(joint_names)
        self._v_index = []
        self._q_index = []
        for name in self._joint_names:
            if not self._model.existJointName(name):
                raise KinematicsError(
                    f"URDF 里没有关节 {name!r} —— joint_names 的顺序和名字是这条链"
                    "的全部含义，拼错一个不会报错，只会少解一个自由度"
                )
            joint = self._model.joints[self._model.getJointId(name)]
            self._v_index.append(joint.idx_v)
            self._q_index.append(joint.idx_q)

        self._damping = float(damping)
        self._max_iterations = int(max_iterations)
        self._position_tolerance = float(position_tolerance)
        self._rotation_tolerance = float(rotation_tolerance)

        self.lower = tuple(float(self._model.lowerPositionLimit[i]) for i in self._q_index)
        self.upper = tuple(float(self._model.upperPositionLimit[i]) for i in self._q_index)

    # ── 正解 ─────────────────────────────────────────────────────────────────

    def forward(self, joint_values, extra=None):
        """给定关节角，返回末端在 base 系下的 `[x, y, z, qx, qy, qz, qw]`。

        `extra` 是这条链上**不由 `joint_names` 控制**的关节，`{名字: 弧度}` ——
        对 G1 就是腰那三个。它们照样进构型向量：手臂挂在腰上，腰动了末端就动了，
        把它们当零处理会让正解报出一个机器人从未处于过的位姿。
        """
        configuration = self._configuration(joint_values, extra)
        self._pin.forwardKinematics(self._model, self._data, configuration)
        self._pin.updateFramePlacement(self._model, self._data, self._tip)
        return self._pose_of(self._data.oMf[self._tip])

    # ── 逆解 ─────────────────────────────────────────────────────────────────

    def solve(self, target_pose, seed, extra=None):
        """`(关节角, 位置残差 m, 姿态残差 rad, 迭代次数)`。

        **残差是返回值而不是异常，调用方必须看。** 不收敛时返回的是最后一次迭代的
        构型 —— 一个每个分量都在限位内、能通过下游每一道检查、却离目标很远的解。
        把它当异常抛掉会更省事，但调用方在超时那一拍需要的是「保持」，而不是一个
        需要 try 的控制流。

        `seed` 是上一拍的真实关节角。它不是优化的初值那么简单：7 自由度手臂对同一
        个手部位姿有无穷多组解，而「离种子最近」就是挑中哪一支的全部依据。
        """
        np = self._np
        pin = self._pin

        if len(target_pose) < 7:
            raise KinematicsError(
                f"目标位姿要 7 个数 [x, y, z, qx, qy, qz, qw]，收到 {len(target_pose)}"
            )
        configuration = self._configuration(seed, extra)
        target = self._placement_of(target_pose)

        iterations = 0
        for iterations in range(1, self._max_iterations + 1):
            pin.forwardKinematics(self._model, self._data, configuration)
            pin.updateFramePlacement(self._model, self._data, self._tip)
            current = self._data.oMf[self._tip]

            # base 系下的误差。位置是差值，姿态是 log3(R_target · R_currentᵀ)。
            position_error = target.translation - current.translation
            rotation_error = pin.log3(target.rotation @ current.rotation.T)
            error = np.concatenate([position_error, rotation_error])

            if (np.linalg.norm(position_error) < self._position_tolerance
                    and np.linalg.norm(rotation_error) < self._rotation_tolerance):
                break

            # LOCAL_WORLD_ALIGNED：雅可比表达在「原点跟着末端、朝向跟着 base」的
            # 系里，和上面那个误差同一个约定。用 LOCAL 会让平移和旋转都表达在末端
            # 自身系里 —— 迭代仍然会收敛（误差是标量判据），但每一步走的方向不同，
            # 在奇异位形附近表现完全不一样。
            jacobian = pin.computeFrameJacobian(
                self._model, self._data, configuration, self._tip,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )[:, self._v_index]

            # 阻尼最小二乘：dq = Jᵀ(JJᵀ + λ²I)⁻¹ e。伪逆在奇异位形处会让 dq 爆掉，
            # 而爆掉的那一步会被下游的步长钳位削成一个看起来正常的指令。
            square = jacobian @ jacobian.T
            square += self._damping ** 2 * np.eye(6)
            step = jacobian.T @ np.linalg.solve(square, error)
            step = np.clip(step, -MAX_STEP, MAX_STEP)

            velocity = np.zeros(self._model.nv)
            velocity[self._v_index] = step
            configuration = pin.integrate(self._model, configuration, velocity)
            # 每步 clamp 到限位，而不是把限位当硬约束 —— 见模块文档：一个解不出来
            # 就整步失败的求解器，在 30 Hz 上等于周期性丢指令。
            for slot, q_index in enumerate(self._q_index):
                configuration[q_index] = min(
                    max(configuration[q_index], self.lower[slot]), self.upper[slot])

        pin.forwardKinematics(self._model, self._data, configuration)
        pin.updateFramePlacement(self._model, self._data, self._tip)
        final = self._data.oMf[self._tip]
        position_residual = float(np.linalg.norm(target.translation - final.translation))
        rotation_residual = float(np.linalg.norm(
            pin.log3(target.rotation @ final.rotation.T)))

        values = tuple(float(configuration[i]) for i in self._q_index)
        return values, position_residual, rotation_residual, iterations

    # ── 内部 ─────────────────────────────────────────────────────────────────

    def _configuration(self, joint_values, extra):
        np = self._np
        if len(joint_values) != len(self._q_index):
            raise KinematicsError(
                f"这条链有 {len(self._q_index)} 个关节，收到 {len(joint_values)} 个值"
            )
        configuration = self._pin.neutral(self._model)
        for slot, q_index in enumerate(self._q_index):
            configuration[q_index] = float(joint_values[slot])
        for name, value in (extra or {}).items():
            if not self._model.existJointName(name):
                raise KinematicsError(f"URDF 里没有关节 {name!r}")
            joint = self._model.joints[self._model.getJointId(name)]
            configuration[joint.idx_q] = float(value)
        return np.asarray(configuration, dtype=float)

    def _pose_of(self, placement):
        quaternion = self._pin.Quaternion(placement.rotation)
        quaternion.normalize()
        translation = placement.translation
        return (float(translation[0]), float(translation[1]), float(translation[2]),
                float(quaternion.x), float(quaternion.y),
                float(quaternion.z), float(quaternion.w))

    def _placement_of(self, pose):
        pin = self._pin
        x, y, z, qx, qy, qz, qw = (float(v) for v in pose[:7])
        length = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if length < 1e-9:
            raise KinematicsError(
                "目标姿态是零四元数 —— 上游没有填这四个数，或者填的是别的表示"
            )
        # pinocchio 的 Quaternion 构造是 **wxyz**，而线上和这个仓库里到处都是 xyzw。
        # 这一行是两种约定的唯一交界处，写在这里而不是散在调用方。
        rotation = pin.Quaternion(qw / length, qx / length, qy / length, qz / length)
        return pin.SE3(rotation.matrix(), self._np.array([x, y, z]))
