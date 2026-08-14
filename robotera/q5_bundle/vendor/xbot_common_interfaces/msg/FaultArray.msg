# 故障数组消息
# 用于发布所有当前故障状态，便于在 Foxglove Studio 中进行分析

std_msgs/Header header        # 标准消息头，包含时间戳和坐标系信息
xbot_common_interfaces/FaultStatus[] faults  # 故障状态列表

