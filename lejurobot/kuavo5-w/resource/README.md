# Kuavo 5-W 骨架资源

`kuavo5w_model.urdf` 供 Agent Core 的 `sensor/skeleton` 渲染器使用，由驱动的
`model` resource 返回。目录约定与 `unitree/g1/resource/g1_model.urdf`、
`x-humanoid/tianyi2.0/resource/tianyi2_model.urdf` 一致。

## 来源与许可证

- 官方仓库：[LejuRobotics/kuavo-ros-opensource](https://github.com/LejuRobotics/kuavo-ros-opensource)
- 真机控制器工作区提交：`69274e610eca8640abfa42821ecbac0721d16602`
- 原始文件：`src/kuavo_assets/models/biped_s63/urdf/biped_s63.urdf`（该文件无工作区修改）
- 原始文件 SHA256：`fecef6f980806af7d2c4b1abe56d3fe1d0dbfbd69b3475303eb158bcc06c85dc`
- 本地衍生文件 SHA256：`05b9bb6b886385bd32fdbace9a7f4b6fb696dfad0d483b034b33c9f8d7ac7d39`
- 同一提交的根目录许可证原文保留于 `LICENSE.leju`。

## 精简方式与用途

保留原始 robot 名称、全部 36 个 link 的名称及全部 35 个 joint 的完整定义，
包括 parent、child、origin、axis、limit 等。删除 link 内的 visual、collision、
inertial 及顶层其他扩展节点；添加来源提示注释，并重新缩进 XML。
模型以 `base_link` 为根，保持一个连通树。

当前前端根据连杆和关节关系绘制骨架，不加载 mesh，因此无需附带 STL 或纹理。
本文件仅用于显示，不能用于动力学、碰撞检测、运动控制或标定。

该文件与现场版本 **63** 控制器加载的 biped_s63 模型一致；相对旧400062参考骨架，
修正底盘及肩部尺寸、头部相机安装位，并移除未建模手指。
文件几何对齐不代表已独立测量实物尺寸及标定。
本地模型可在机器人断线或全身控制器未启动时读取；此时显示的是 URDF 参考姿态，
不是测得的零关节角。`joints` 的实时姿态必须来自新鲜的实测 q，并通过真机
`/humanoid_description` 核实 20 个受测关节的名称与本地模型一致。
名称校验不代表几何或标定一致；现场核实具体型号后，应更新本文件及来源记录。
轮子虽保留在模型中，原始 20 关节数据不包含其测量值，驱动不会补造它们。


## 头部显示说明（2026-11-19现场复核）

当天从运行中`/humanoid_description`读取的原始URDF指纹仍为上述`fecef6f9…`，
全部35个joint的属性、parent/child、origin、axis、limit等与本地精简模型逐项相同；
正式MCP `model`返回的文件指纹仍为上述`05b9bb6b…`。
这证明与当前控制器模型一致，不是独立的实物尺寸/零位标定认证。

- `zhead_1_joint`对应原始q[18]、yaw绕Z轴；`zhead_2_joint`对应q[19]、pitch绕Y轴，单位rad。
- 两个头部关节原点重合：第二个关节origin为0；骨架中不会画出两轴之间不存在的连杆。
- 相机固定安装位的pitch为0.3406rad（约19.515°），不能把相机倾斜当成头部关节反馈的零位错误。
- 官方URDF保留一个名为`radar`的固定分支，位于yaw关节下方0.078453m；该连杆是模型几何，
  不能证明现场安装了雷达，也不能因此启用不存在的三维点云卡。
- 当前Agent Core只根据关节原点绘制骨架，不加载头部mesh外壳；外观不等同于完整头部形状。
  不应为了让骨架“像实物”就修改官方运动轴、添加角度偏移或用控制目标替代测量。
