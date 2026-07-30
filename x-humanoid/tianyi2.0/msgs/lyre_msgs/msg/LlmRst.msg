# lyre_msgs/msg/LlmRst.msg
# LLM 大模型语音对话返回的内容。

string TOPIC_NAME = /audio_llm/rst

string sid
uint32 seq
bool   last
string text
