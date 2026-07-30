string TOPIC_NAME = /xsys/light/ctrl

# 系统
int32 CMD_SYSTEM_SHUTTING = 0   # 关机中
int32 CMD_SYSTEM_ERROR_OCCUR = 10  # 故障
int32 CMD_SYSTEM_ERROR_CLEAR = 11  # 故障消除
int32 CMD_SYSTEM_WARN_OCCUR = 12   # 告警
int32 CMD_SYSTEM_WARN_CLEAR = 13   # 告警消除
int32 CMD_SYSTEM_SERVICE_WAIT = 20    # 服务等待中
int32 CMD_SYSTEM_SERVICE_START = 21   # 服务启动中
int32 CMD_SYSTEM_SERVICE_READY = 22   # 服务就绪
int32 CMD_SYSTEM_SERVICE_FAILED = 23  # 服务启动失败
int32 CMD_SYSTEM_STANDBY = 99   # 系统待机

# 基础组件
int32 CMD_OTA_QUIT = 100   # 升级退出
int32 CMD_OTA_START = 101  # 升级开始

# 电源
int32 CMD_POWER_QUIT = 200                  # 电量状态退出
int32 CMD_POWER_BATTERY_NORMAL = 201        # 放电中，电量充足
int32 CMD_POWER_BATTERY_LOW = 202           # 放电中，低电量
int32 CMD_POWER_BATTERY_CRITICAL = 203      # 放电中，电量危急
int32 CMD_POWER_CHARGING = 210              # 充电中
int32 CMD_POWER_CHARGING_FULL = 211         # 充电中，已充满
int32 CMD_POWER_BACKUP_NORMAL = 220         # 备份电池放电中

# 交互
int32 CMD_CHAT_QUIT = 300                   # 对话结束，已休眠
int32 CMD_CHAT_WAKEUP = 301                 # 对话开始，已唤醒
int32 CMD_CHAT_ASR = 310                    # 拾音中
int32 CMD_CHAT_LLM = 311                    # 推理中
int32 CMD_CHAT_TTS = 312                    # 合成中
int32 CMD_CHAT_PLAY = 313                   # 应答播报中
int32 CMD_CHAT_NET_OFFLINE = 320            # 断网
int32 CMD_CHAT_NET_ONLINE = 321             # 网络恢复

# 运动
int32 CMD_MOTION_QUIT = 400                 # 运动退出
int32 CMD_MOTION_RUNNING = 401              # 奔跑中

std_msgs/Header header
int32  cmd              # 灯效定义
int8[] data             # 可选的载荷数据
string caller_id        # 调用方身份
string caller_msg       # 调用方提供的描述信息 —— 基于什么原因更新了灯效，用作日志审计目的
