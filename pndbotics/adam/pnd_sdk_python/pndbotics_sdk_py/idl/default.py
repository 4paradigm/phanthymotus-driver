from .pnd_adam.msg.dds_ import *

"""
" std_msgs.msg.dds_ dafault
"""
# IDL for pnd_adam

"""
" std_msgs.msg.dds_ dafault
"""
def pnd_adam_msg_dds__IMUState_():
    return IMUState_([0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0)

def pnd_adam_msg_dds__MotorCmd_():
    return MotorCmd_(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)

def pnd_adam_msg_dds__MotorState_():
    return MotorState_(0, 0.0, 0.0, 0.0, 0.0, 0, 0)

def pnd_adam_msg_dds__LowCmd_(num):
    # 使用空的 sequence 初始化 motor_cmd
    return LowCmd_(0, [pnd_adam_msg_dds__MotorCmd_() for i in range(num)], 0)


def pnd_adam_msg_dds__LowState_(num):
    # 使用空的 sequence 初始化 motor_cmd
    return LowState_(0, 0, pnd_adam_msg_dds__IMUState_(), [pnd_adam_msg_dds__MotorState_() for i in range(num)], 
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                    pnd_adam_msg_dds__BatteryData_(),
                     0)

def pnd_adam_msg_dds__HandCmd_():
    return HandCmd_([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0)

def pnd_adam_msg_dds__HandState_():
    return HandState_([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 0)

def pnd_adam_msg_dds__BatteryData_():
    return BatteryData_(0, 0.0, 0.0, 0.0, 0.0, "")
