# Q5 差速底盘控制命令
std_msgs/Header header
float32 linear_x        # 前后速度(m/s)，最大±1.5m/s
float32 angular_z       # 旋转角速度(rad/s)
float32 linear_y        # 横向速度（预留，Q5为差速底盘默认0）
bool emergency_stop     # 底盘紧急停止标志