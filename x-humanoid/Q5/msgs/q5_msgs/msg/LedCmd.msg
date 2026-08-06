# Q5 LED状态灯控制命令
std_msgs/Header header
uint8 red               # 红色分量(0-255)
uint8 green             # 绿色分量(0-255)
uint8 blue              # 蓝色分量(0-255)
uint8 brightness        # 亮度(0-255)
uint8 effect            # 灯光效果: 0=常亮,1=呼吸,2=闪烁,3=彩虹循环
float32 effect_speed    # 效果速度(0-1)，仅在非普通模式下生效