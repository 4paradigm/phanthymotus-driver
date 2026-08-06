# Q5 语音识别结果消息
std_msgs/Header header
string text             # 识别到的文本内容
float32 confidence      # 识别置信度(0-1)
bool is_final           # 是否为最终结果（false为中间结果）
uint32 duration_ms      # 语音片段时长(ms)
uint8 result_type       # 结果类型: 0=唤醒词,1=命令词,2=自由对话,3=降噪模式结果