# Q5 单个电机状态消息
std_msgs/Header header
uint8 id                 # 电机ID
float64 position         # 当前位置(rad)
float64 velocity         # 当前速度(rad/s)
float64 effort           # 当前扭矩(N·m)
float64 temperature      # 电机温度(℃)
uint8 error_code         # 错误码
bool enabled             # 电机是否使能