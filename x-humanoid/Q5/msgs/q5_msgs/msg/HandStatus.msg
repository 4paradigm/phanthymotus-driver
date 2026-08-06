# Q5 灵巧手状态消息
std_msgs/Header header
uint8 hand_id           # 手ID: 0=左手, 1=右手
float32[] finger_positions # 各手指当前位置(rad)
float32[] finger_velocities # 各手指当前速度(rad/s)
float32 temperature     # 手部电机平均温度(℃)
bool is_moving          # 是否正在执行动作
uint8 current_gesture   # 当前手势: 0=自定义,1=张开,2=握拳,3=点赞,4=比心,5=OK,6=捏取,7=挥手