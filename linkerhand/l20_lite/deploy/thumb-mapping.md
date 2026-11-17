# 拇指三通道适配依据

对用户已有的 Hex Ubuntu 3.4.6 上位机 Assembly-CSharp.dll 做静态方法检查，未启动上位机、串口或电机。
所核对文件 SHA-256：`1af47f9fc2bea04b0f3684e38fe7952a7b6fdf30c40a15f23d548f712cb7ef2c`。
仅记录接口语义，不将厂商 DLL、反编译输出或 Unity 实现加入镜像/仓库。

核对到的厂商默认分支：

| L20 Lite 电机索引 | 厂商数据来源 | 本驱动的新实现 |
| --- | --- | --- |
| 0 | HandDataProcessor_L20Lite.ApplyDefaultMapping 读取 thumb.pipFlexionText | 指尖 top[0] 相对拇指根部 root[1]，张开/握拳四元数标定 |
| 1 | 同方法读取 thumb.spreadText，经 MapThumbYawToStroke | 根部相对手掌，用独立侧摆姿态标定 |
| 9 | ReadThumbRollSourceAngle 默认读取 thumb.mcpFlexionText；可切到扭转角来源 | 根部相对手掌，用独立根部屈曲姿态标定 |

CalLingXin_L20Lite.OnProcessorLeft/Right 将 FingerPositions 的 0/1/9 原顺序复制到电机位置数组。
HandDataProcessor_L20Lite 的嵌套 DynamicAngleRange.MapToPosition 默认由 255 向 0 映射；MapThumbYawToStroke 可配置反向。
ReadThumbRollSourceAngle 的 mapThumbRollFromThumbMcp 构造默认值为 true，Unity 场景序列化设置仍可能覆盖它。
因此不能简单把第三路称为传感器 roll，也不能由同一个握拳比例同步三个电机。

此前映射只使用 root[1]，tip[0] 未参与拇指控制；侧摆/旋转停在张开值，弯曲沿 hand 的 254→93 端点。
新版不改四指路径，改由三个独立标定量计算 0/1/9，并单独配置它们的端点。
额外两项标定避免硬猜左右手 IMU 轴方向；若根部两种姿态近似共线，拒绝 arm 前的完整标定。

这不是对厂商 Unity 几何、角度、动态范围和对指学习的完全复刻。
离线检查仅能验证通道独立性、四元数符号/整体转腕不变性、限幅、缺样保护和打包，
不能证明默认全行程在具体实体上无碰撞，也不能证明混合动作与厂商逐帧一致。最终镜像仍需操作者实机验收。
