# Q5 机器人整体状态消息
std_msgs/Header header
uint8 system_state      # 系统状态: 0=待机, 1=运行中, 2=错误, 3=急停
uint8 control_mode      # 控制模式: 0=手动, 1=自动, 2=导航
float32 cpu_usage       # CPU使用率(0-100)
float32 mem_usage       # 内存使用率(0-100)
uint32 uptime           # 系统运行时间(s)
string error_message    # 错误信息（无错误时为空）
bool estop_pressed      # 急停是否被按下
bool is_charging        # 是否在充电