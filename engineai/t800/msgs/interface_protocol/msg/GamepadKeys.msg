uint8 LB = 0
uint8 RB = 1
uint8 A = 2
uint8 B = 3
uint8 X = 4
uint8 Y = 5
uint8 BACK = 6
uint8 START = 7
uint8 CROSS_X_UP = 8
uint8 CROSS_X_DOWN = 9
uint8 CROSS_Y_LEFT = 10
uint8 CROSS_Y_RIGHT = 11
uint8 LT = 0
uint8 RT = 1
uint8 LEFT_STICK_X = 2
uint8 LEFT_STICK_Y = 3
uint8 RIGHT_STICK_X = 4
uint8 RIGHT_STICK_Y = 5
std_msgs/Header header
bool hardware_connected
int32[12] digital_states
float64[6] analog_states
