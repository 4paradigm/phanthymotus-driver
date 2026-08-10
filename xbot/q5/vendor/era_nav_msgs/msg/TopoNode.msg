
geometry_msgs/Point pos  # 节点的位置
int64 id  # 节点的ID
bool has_orientation  # 节点是否有朝向
float64 orientation  # 节点的朝向

# 节点的业务属性，由开发者根据业务需求自己定义和管理
# 使用两个数组表示 map<string, string>
string[] business_attr_keys
string[] business_attr_values
