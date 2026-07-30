# lyre_msgs/msg/AsrEvent.msg
# ASR 设备报出的事件。

string TOPIC_NAME = /audio_asr/event

int8 EVENT_ERROR   = 2  # 出错事件。arg1 是错误码。
int8 EVENT_STATE   = 3  # 服务状态事件。
int8 EVENT_WAKEUP  = 4  # 唤醒事件。
int8 EVENT_SLEEP   = 5  # 休眠事件。
int8 EVENT_VAD     = 6  # VAD 事件。

int8 EVENT_PRE_SLEEP            = 10  # 准备休眠事件。
int8 EVENT_CONNECTED_TO_SERVER  = 13  # 与服务端建立连接。
int8 EVENT_SERVER_DISCONNECTED  = 14  # 与服务端断开连接。

int8 event
int8 arg1
int8 arg2
