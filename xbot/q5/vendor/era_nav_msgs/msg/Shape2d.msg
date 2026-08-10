
string shape_type  # 形状类型，支持的类型包括： "Circle", "Rectangle", "Polygon".
float64 radius  # 半径，仅用于圆形。
float64 width   # 宽度，仅用于矩形。(宽度是沿着y轴定义的)
float64 length  # 长度，仅用于矩形。(长度是沿着x轴定义的)
float64 orientation  # 方向，仅用于矩形。矩形 X 轴的方向，以弧度表示。
geometry_msgs/Point center  # 几何中心，仅用于圆形和矩形。
geometry_msgs/Point[] vertices  # 顶点，仅用于不规则多边形。最后一个顶点自动与第一个顶点连接闭合。
