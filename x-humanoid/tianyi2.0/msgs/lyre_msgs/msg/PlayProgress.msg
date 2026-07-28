# lyre_msgs/msg/PlayProgress.msg
# Play 音频播放的进度。

string TOPIC_NAME = /audio_play/progress

string sid
uint32 seq
float64 position
float64 duration
