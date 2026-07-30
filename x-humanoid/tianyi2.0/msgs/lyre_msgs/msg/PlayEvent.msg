# lyre_msgs/msg/PlayEvent.msg
# Play 音频播放的事件。

string TOPIC_NAME = /audio_play/event

int8 EVENT_STARTED   = 0
int8 EVENT_COMPLETED = 1
int8 EVENT_STOPPED   = 2
int8 EVENT_CANCELLED = 3
int8 EVENT_FAILED    = 4

string sid
uint32 seq
int8   event
string message
