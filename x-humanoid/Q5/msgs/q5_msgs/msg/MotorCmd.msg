# Q5 单个电机控制命令
uint8 id                 # 电机ID
float64 target_position  # 目标位置(rad)
float64 target_velocity  # 目标速度(rad/s)
float64 target_torque    # 目标扭矩(N·m)
float64 kp               # 位置环增益
float64 kd               # 速度环增益