# Q5 语音合成控制命令
std_msgs/Header header
string text             # 要合成的文本内容
uint8 volume            # 音量(0-100)
float32 speed           # 语速(0.5-2.0, 1.0为正常语速)
string voice            # 发音人音色，支持"xiaoyan"、"xiaoyu"等
bool interrupt          # 是否打断当前正在播放的语音
bool block              # 是否阻塞等待播放完成再返回