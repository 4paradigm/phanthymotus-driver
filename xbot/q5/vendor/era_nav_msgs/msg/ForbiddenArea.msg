
int64 id  # 禁行区域的 ID
era_nav_msgs/Shape2d shape  # 禁行区域的形状
float64 z_min  # [z_min, z_max] 定义了禁行区域在 z 轴上的范围。2D 导航中会忽略此值。
float64 z_max

# 禁行区域的业务属性，由开发者根据业务需求自己定义和管理
# 使用两个数组表示 map<string, string>
string[] business_attr_keys
string[] business_attr_values
