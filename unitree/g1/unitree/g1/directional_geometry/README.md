# 方向示意的几何来源与复现

运行模块 [`directional_kinematics.py`](../directional_kinematics.py) 使用随驱动提供的 G1_23 URDF、固定手 profile、原始关节限位及碰撞覆盖。左右臂各自解析 URDF 的轴、关节原点、旋转和限位；左臂网格覆盖独立生成，不通过镜像右臂数据代替。

## 左臂网格证据

来源为 [Unitree `unitree_ros` 固定提交](https://github.com/unitreerobotics/unitree_ros/tree/157ff950351e5f535e03c5d85a16b7d594b3dbcc)。本目录不复制源 STL，也不在驱动启动时下载网格。源码和生成器中固定的 SHA-256 为：

| 文件 | SHA-256 |
|---|---|
| `left_shoulder_yaw_link.STL` | `1aa97e9748e924336567992181f78c7cd0652fd52a4afcca3df6b2ef6f9e712e` |
| `left_elbow_link.STL` | `fa752198accd104d5c4c3a01382e45165b944fbbc5acce085059223324e5bed3` |
| `left_wrist_roll_rubber_hand.STL` | `e81030abd023bd9e4a308ef376d814a2c12d684d8a7670c335bbd5cd7809c909` |

[`point-left-collision-cover.json`](point-left-collision-cover.json) 记录各网格三角形数、坐标极值、切片边界、每片保守包围盒及贡献三角形数。对每个三角形先求 AABB，再与所有相交的闭切片求交；每片并集向外舍入至整毫米。闭切片共同覆盖网格范围，跨片三角形会计入各片，因此贡献数之和可以大于三角形总数。

左肩 pitch/roll 使用 URDF 自带圆柱的覆盖盒；其原点和旋转来自 URDF。新增的右肩 pitch 覆盖保留挂载躯干的豁免，但参与左右臂碰撞检查。其余躯干覆盖和 3 mm 模型间距没有因跨身动作而放宽。

## 本地复现

从上述固定提交取得三个文件，放入同一目录。在仓库根目录执行，将 `PATH_TO_MESHES` 替换为该目录：

```sh
python unitree/g1/directional_geometry/build_left_collision_boxes.py --mesh-dir PATH_TO_MESHES --check
```

生成器只读取本地文件，首先核对固定哈希，再重新检查所有三角形切片覆盖，并与提交的 JSON 比较。成功输出 `LEFT_COLLISION_COVER_OK meshes=3 mode=check`；任何源文件改变或证据不一致均失败。去掉 `--check` 才会写报告，`--output` 可指定输出路径；报告生成不会修改运行模块内的常量，更新常量需要代码评审和回归测试。

## 证明的范围

名义右臂路径是当前起点到 IK 终点的关节直线插值，左臂固定为本次采样姿态。模型用每个碰撞盒的角点 Jacobian 与全局二阶余项包围整个关节区间，逐区间检查与躯干和左臂的 3 mm 间距；预算耗尽时拒绝，不通过稀疏采样推断整段通过。可选右臂误差区间也进入同一分盒证明。

执行期间另以同一 LowState 样本中的实际左右臂关节角检查几何，缓存键包含全部十个关节。当前 point 不再以细小跟踪误差阈值中止动作，因此名义路径证书不保证实际手臂始终落在该路径上；实际姿态检查也不证明反馈样本之间、左臂漂移过程或停止距离的间距。外部障碍、柔性手形变、模型误差以及变化腿姿不由这份证据覆盖。验收要求见[现场验收说明](../directional_pointing_validation.md)。
