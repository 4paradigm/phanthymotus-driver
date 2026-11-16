# Kuavo 5-W 骨架资源

`kuavo5w_model.urdf` 供 Agent Core 的 `sensor/skeleton` 渲染器使用，由驱动的
`model` resource 返回。目录约定与 `unitree/g1/resource/g1_model.urdf`、
`x-humanoid/tianyi2.0/resource/tianyi2_model.urdf` 一致。

## 来源与许可证

- 官方仓库：[LejuRobotics/kuavo-ros-opensource](https://github.com/LejuRobotics/kuavo-ros-opensource)
- 固定提交：`d9c036e19fce5689fcfa8c596ee89b2045bebb32`
- 原始文件：`src/kuavo_assets/models/biped_s400062/urdf/biped_s400062.urdf`
- [固定版本源文件](https://github.com/LejuRobotics/kuavo-ros-opensource/blob/d9c036e19fce5689fcfa8c596ee89b2045bebb32/src/kuavo_assets/models/biped_s400062/urdf/biped_s400062.urdf)
- 原始文件 SHA256：`77e3d15491b968f164739d8d13e86851c369a98aa6c3ef0ec8b26f1dd24b0ed6`
- 本地衍生文件 SHA256：`3bcf7f9b96ef842e1726eef09e6c8a4bbacf1070ee65ecbb65cb08c14f8ea384`
- 同一提交的根目录许可证原文保留于 `LICENSE.leju`。

## 精简方式与用途

保留原始 robot 名称、全部 69 个 link 的名称及全部 68 个 joint 的完整定义，
包括 parent、child、origin、axis、limit 等。删除 link 内的 visual、collision、
inertial 及顶层其他扩展节点；添加来源提示注释，并重新缩进 XML。
模型以 `base_link` 为根，保持一个连通树。

当前前端根据连杆和关节关系绘制骨架，不加载 mesh，因此无需附带 STL 或纹理。
本文件仅用于显示，不能用于动力学、碰撞检测、运动控制或标定。

该文件是 **400062 参考版本**，尚未确认与现场机器人具体硬件修订、尺寸及标定完全一致。
本地模型可在机器人断线或全身控制器未启动时读取；此时显示的是 URDF 参考姿态，
不是测得的零关节角。`joints` 的实时姿态必须来自新鲜的实测 q，并通过真机
`/humanoid_description` 核实 20 个受测关节的名称与本地模型一致。
名称校验不代表几何或标定一致；现场核实具体型号后，应更新本文件及来源记录。
轮子和手指虽保留在模型中，原始 20 关节数据不包含其测量值，驱动不会补造它们。
