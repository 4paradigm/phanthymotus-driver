# Kp / Kd 参数说明

> Source: https://wiki.pndbotics.com/robot/kp_kd/

## 在仿真 (Isaac Gym)

PD 控制器：

```
τ = Kp * ep + Kd * ev
```

- `Kp`：刚度
- `Kd`：阻尼
- `ep`：位置误差
- `ev`：速度误差

---

## 在真实机器人执行器

PID 控制器：

```
I = Pp * Pv * ep + Pv * ev
τ' = g * I * Kt = g * (Pp * Pv * Kt * ep + Pv * Kt * ev)
```

- `Pp`：位置环比例参数
- `Pv`：速度环比例参数
- `g`：齿轮比
- `Kt`：扭矩常数

---

## Sim-to-Real 参数对齐关系

当满足 `τ = τ'` 时，仿真与实机参数的对应关系如下：

```
Kp = g * Pp * Pv * Kt
Kd = g * Pv * Kt
```

**由此推导出实机配置参数：**

```
Pv = Kd / (Kt * g)
Pp = Kp / Kd
```

---

## 关节逻辑代码片段

```cpp
void RealRobot::computeJointPdParams() {
    joint_Kp_s = joint_Kp_;
    joint_Kd_s = joint_Kd_;

    for (int i = 0; i < robot_dof_nums_; i++) {
        if (bAdam_u_) {
            // Adam-U 逻辑
            joint_Kp_(i) = joint_Kp_(i) / joint_Kd_(i);
            joint_Kd_(i) = joint_Kd_(i) / PConfig::getInstance().kdScale()(i);
        } else {
            // Adam 逻辑：踝关节特殊处理
            if (i == kAnkleJoint1 || i == kAnkleJoint2) {
                joint_Kp_(i) = joint_Kp_(i - 1);
                joint_Kd_(i) = joint_Kd_(i - 1);
            } else {
                joint_Kp_(i) = joint_Kp_(i) / joint_Kd_(i);
                joint_Kd_(i) = joint_Kd_(i) / PConfig::getInstance().kdScale()(i);
            }
        }
    }
    std::cout << "joint_Kp: " << joint_Kp_.transpose() << std::endl;
    std::cout << "joint_Kd: " << joint_Kd_.transpose() << std::endl;
}
```

### 说明

- 对于 Adam（非 Adam-U）版本，踝关节有特殊处理逻辑
- `Pp = Kp / Kd`（位置环比例 = 仿真刚度/仿真阻尼）
- `Pv = Kd / kdScale`（速度环比例 = 仿真阻尼/缩放因子）
- `kdScale` 对应 `g * Kt`（齿轮比 x 扭矩常数）
