# Q5 头部控制命令
std_msgs/Header header
float32 yaw             # 左右偏航目标角度(deg)，范围±60°
float32 pitch           # 上下俯仰目标角度(deg)，范围±30°
float32 speed           # 运动速度(0-1)
bool is_relative        # 是否为相对角度偏移，false为绝对位置