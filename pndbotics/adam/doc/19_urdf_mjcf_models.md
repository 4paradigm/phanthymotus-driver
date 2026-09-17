# 模型文件（URDF/MJCF）

> Source: https://wiki.pndbotics.com/robot/pnd_models/

本仓库包含了PNDbotics Adam机器人的URDF和MJCF模型文件，可以用作仿真和控制中。其中包括了详细的描述，mesh模型等。

- 详情请见 [PND Models GitHub](https://github.com/pndbotics/pnd_models)

---

### Link 方向注意事项（脚趾连杆翻转问题）

如果你使用了强化学习例程, 请再次确定你已经按照这里要求修改了对应参数，否则仿真将会受到影响。

#### URDF 配置

在 `toe_left` 和 `toe_right` 的link definitions, 请确保参数修改如下：

```xml
<collision name="toe_*">
  <origin rpy="1.57 0 0" xyz="0 0 0"/>
</collision>
```

这样方向设置可确保仿真中脚趾连杆的正确对齐。

#### Isaac Gym 配置

在Isaac Gym 训练前，对应的 `*_config.py` 要进行如下修改:

```python
flip_visual_attachments = True
```

> 默认值是 `False`, 这可能会导致某些模型中脚趾部件的视觉对齐问题。

---

## Models

| Model Name | 说明 |
|-----------|------|
| adam_inspire | Adam + Inspire灵巧手 |
| adam_lite | Adam Lite 基础版 |
| adam_lite_agx | Adam Lite + AGX |
| adam_sp | Adam SP 版 |
| adam_sp_agx_ir | Adam SP + AGX + IR |
| adam_standard | Adam Standard 标准版 |
| adam_u | Adam U 版 |

## Driver model selection

The Adam driver returns an URDF through its `model` resource.  For the `pro`
configuration it uses the official `adam_inspire` kinematic tree rather than
the older `adam_standard` fallback, because the latter does not describe wrist
roll or either hand. The robot's 12 hand feedback channels remain available
from the separate `hand_state` sensor and are added to the `joints` skeleton
stream. Since Adam uses `1000=opened` and `0=closed`, the driver inversely maps
each channel into the public URDF finger limits so the visual opening direction
matches the real hand. This is visualization only; it is not a vendor-provided
actuator-space calibration. PNDbotics has not published an Adam Pro head/neck kinematic tree,
so the driver adds a clearly marked visual-only neck/head mount using the
published ±60-degree head limits. `head_control` uses the actual DDS joint
names and limits; it does not rely on this visual approximation.

The official mesh archives are intentionally not copied into the driver image.
They are large, while the dashboard resource contract only requires the URDF
for skeleton rendering.  The `model` response states this explicitly.
