
int64 from_node  # 开始节点的ID
int64 to_node  # 结束节点的ID
float64 cost  # 边的代价

# 边的业务属性，由开发者根据业务需求自己定义和管理
# 使用两个数组表示 map<string, string>
string[] business_attr_keys
string[] business_attr_values
