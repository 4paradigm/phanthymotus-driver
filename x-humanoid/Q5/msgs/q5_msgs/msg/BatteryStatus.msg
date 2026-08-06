# Q5 电池状态消息
std_msgs/Header header
float32 voltage          # 电池电压(V)
float32 current          # 电池电流(A，正为充电，负为放电)
float32 percentage       # 剩余电量百分比(0-100)
uint8 charge_state       # 充电状态: 0=未充电, 1=充电中, 2=充电完成
uint32 remaining_time    # 预估剩余使用时间(s)
float32 temperature      # 电池温度(℃)
uint8 led_color          # 状态灯颜色: 0=绿色(满电), 1=蓝色(>33%), 2=黄色(21-32%), 3=橙色(9-20%), 4=红色(<8%)