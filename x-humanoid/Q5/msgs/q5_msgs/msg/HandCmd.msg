# Q5 XHAND灵巧手控制命令
std_msgs/Header header
uint8 hand_id           # 手ID: 0=左手, 1=右手
uint8 gesture           # 预设手势: 0=自定义,1=张开,2=握拳,3=点赞,4=比心,5=OK,6=捏取,7=挥手
float32[] finger_positions # 各手指目标位置(rad)，自定义手势时使用，五指灵巧手共10个自由度
float32 speed           # 动作速度(0-1)
float32 force           # 握力控制(0-1)