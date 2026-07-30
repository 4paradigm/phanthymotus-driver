# @brief 异常码消息定义

int8 ALARM_INFO=0
int8 ALARM_WARN=1
int8 ALARM_ERROR=2
int8 ALARM_FATAL=3

builtin_interfaces/Time stamp  # 时间戳
string node_name
int32 error_code  # 异常码
int8 level
string description # 异常描述
