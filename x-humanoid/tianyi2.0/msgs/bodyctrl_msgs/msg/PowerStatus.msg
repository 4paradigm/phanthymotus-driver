# @brief 电源板电压、电流、电量、温度等信息

std_msgs/Header header

# temperature information
float32 waist_temp # 腰部MOS温度
float32 arm_a_temp # 臂A MOS温度
float32 arm_b_temp # 臂B MOS温度
float32 leg_a_temp # 腿A MOS温度
float32 leg_b_temp # 腿B MOS温度

float32 waist_temp_max # 腰部MOS温度最大值
float32 arm_a_temp_max # 臂A MOS温度最大值
float32 arm_b_temp_max # 臂B MOS温度最大值
float32 leg_a_temp_max # 腿A MOS温度最大值
float32 leg_b_temp_max # 腿B MOS温度最大值

float32 waist_temp_min # 腰部MOS温度最小值
float32 arm_a_temp_min # 臂A MOS温度最小值
float32 arm_b_temp_min # 臂B MOS温度最小值
float32 leg_a_temp_min # 腿A MOS温度最小值
float32 leg_b_temp_min # 腿B MOS温度最小值

# current information
float32 arm_a_curr # 臂A电流
float32 arm_b_curr # 臂B电流
float32 leg_a_curr # 腿A电流
float32 leg_b_curr # 腿B电流
float32 waist_curr # 腰部电流
float32 head_curr  # 头部电流

float32 arm_a_curr_max # 臂A电流最大值
float32 arm_b_curr_max # 臂B电流最大值
float32 leg_a_curr_max # 腿A电流最大值
float32 leg_b_curr_max # 腿B电流最大值
float32 waist_curr_max # 腰部电流最大值
float32 head_curr_max  # 头部电流最大值

float32 arm_a_curr_min # 臂A电流最小值
float32 arm_b_curr_min # 臂B电流最小值
float32 leg_a_curr_min # 腿A电流最小值
float32 leg_b_curr_min # 腿B电流最小值
float32 waist_curr_min # 腰部电流最小值
float32 head_curr_min  # 头部电流最小值

float32 arm_a_volt #臂A电压
float32 arm_b_volt #臂B电压
float32 leg_a_volt #腿A电压
float32 leg_b_volt #腿B电压
float32 waist_volt #腰部电压
float32 bus_volt   #母线电压

float32 arm_a_volt_max #臂A电压最大值
float32 arm_b_volt_max #臂B电压最大值
float32 leg_a_volt_max #腿A电压最大值
float32 leg_b_volt_max #腿B电压最大值
float32 waist_volt_max #腰部电压最大值
float32 bus_volt_max #母线电压最大值

float32 arm_a_volt_min #臂A电压最小值
float32 arm_b_volt_min #臂B电压最小值
float32 leg_a_volt_min #腿A电压最小值
float32 leg_b_volt_min #腿B电压最小值
float32 waist_volt_min #腰部电压最小值
float32 bus_volt_min #母线电压最小值

# version information
string software_version # 软件版本
string hardware_version # 硬件版本

# battery status information
float32 battery_voltage # 电池电压信息
float32 battery_current # 电池电流信息
float32 battery_power   # 电池电量
