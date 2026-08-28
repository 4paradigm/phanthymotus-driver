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
