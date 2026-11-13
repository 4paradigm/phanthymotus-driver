# Same fields as `ros2 interface show unix_msgs/msg/GripperCommand` on the Panther controller
# (as reported in PR #520, which captured it there). The package name and message name must
# match the robot's, or the gripper controller will not accept what is published.
string mode
sensor_msgs/JointState command
