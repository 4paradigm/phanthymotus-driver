# 故障状态消息
# 对应 YAML 配置文件中的所有字段

# 消息头（包含时间戳和frame_id）
std_msgs/Header header  # 故障发生/更新的时间戳

# 基本信息
uint32 fault_code              # 故障码编号（唯一标识）

byte FATAL=0
byte CRITICAL=1
byte ERROR=2
byte WARN=3
byte NOTICE=4
byte INFO=5

uint8 level                   # 故障等级 (0-致命错误, 1-严重错误, 2-错误, 3-警告, 4-提示, 5-信息)
string name                   # 故障名称
string message                # 故障摘要消息（用于 summary/mergeSummary 接口）
int32 fault_origin_code       # 原始故障码（用于关节模组等，-1表示无原始故障码）
string hardware_id            # 

# 关节信息
string joint_name             # 所属关节

# 分类信息
string fault_type             # 故障类型 (battery, joint, encoder, imu, realsense, microphone, nuc, orin, algorithm, software等)
string fault_module           # 所属模块

# 描述信息
string fault_cause            # 故障原因
string[] fault_strategy       # 故障策略（数组，支持多个策略）
string fault_description_zh   # 故障描述（中文）
string fault_description_en   # 故障描述（英文）
string fault_voice_broadcast  # 故障语音
string fault_solution         # 故障解决方案

# 状态信息
bool is_active                # 是否激活（当前是否发生故障）
bool is_reported              # 是否上报

# 额外信息（键值对形式，用于扩展）
diagnostic_msgs/KeyValue[] extra_info  # 额外信息的键值对

