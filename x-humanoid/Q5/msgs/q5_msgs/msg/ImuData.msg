# Q5 IMU数据消息
std_msgs/Header header
geometry_msgs/Quaternion orientation  # 姿态四元数
float64[9] orientation_covariance     # 姿态协方差
geometry_msgs/Vector3 angular_velocity # 角速度(rad/s)
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration # 线加速度(m/s²)
float64[9] linear_acceleration_covariance
float32 temperature                   # IMU温度(℃)