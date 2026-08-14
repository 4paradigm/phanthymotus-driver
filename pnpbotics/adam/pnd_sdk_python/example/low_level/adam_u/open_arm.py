import time
import sys
import numpy as np

from pndbotics_sdk_py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from pndbotics_sdk_py.idl.default import pnd_adam_msg_dds__LowCmd_
from pndbotics_sdk_py.idl.default import pnd_adam_msg_dds__LowState_
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import LowCmd_
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import LowState_
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import HandCmd_
from pndbotics_sdk_py.idl.default import pnd_adam_msg_dds__HandCmd_
ADAM_U_NUM_MOTOR = 19
KP_CONFIG = [
    405.0,  # waistRoll (0)
    405.0,  # waistPitch (1)
    205.0,  # waistYaw (2)
    9.0,   # neckYaw (3)
    9.0,   # neckPitch (4)
    18.0,  # shoulderPitch_Left (5)
    9.0,   # shoulderRoll_Left (6)
    9.0,   # shoulderYaw_Left (7)
    9.0,   # elbow_Left (8)
    9.0,   # wristYaw_Left (9)
    9.0,   # wristPitch_Left (10)
    9.0,   # wristRoll_Left (11)
    18.0,  # shoulderPitch_Right (12)
    9.0,   # shoulderRoll_Right (13)
    9.0,   # shoulderYaw_Right (14)
    9.0,   # elbow_Right (15)
    9.0,   # wristYaw_Right (16)
    9.0,   # wristPitch_Right (17)
    9.0    # wristRoll_Right (18)
]

# Kd 配置数组（对应19个关节）
KD_CONFIG = [
    6.1,   # waistRoll (0)
    6.1,   # waistPitch (1)
    4.1,   # waistYaw (2)
    0.9,   # neckYaw (3)
    0.9,   # neckPitch (4)
    0.9,   # shoulderPitch_Left (5)
    0.9,   # shoulderRoll_Left (6)
    0.9,   # shoulderYaw_Left (7)
    0.9,   # elbow_Left (8)
    0.9,   # wristYaw_Left (9)
    0.9,   # wristPitch_Left (10)
    0.9,   # wristRoll_Left (11)
    0.9,   # shoulderPitch_Right (12)
    0.9,   # shoulderRoll_Right (13)
    0.9,   # shoulderYaw_Right (14)
    0.9,   # elbow_Right (15)
    0.9,   # wristYaw_Right (16)
    0.9,   # wristPitch_Right (17)
    0.9    # wristRoll_Right (18)
]


open_arm_pos = np.array([0, 0, 0,
                       0.7, -0.5,
        -1.6, 2.06, -1.65, -1.77,
                      0.32, 0, 0,
       - 1.6, -2.06, 1.65, -1.77,
                      0.32, 0, 0
],
                              dtype=float)

close_arm_pos = np.array([0, 0, 0,
                             0, 0,
                       0, 0, 0, 0,
                          0, 0, 0,
                       0, 0, 0, 0,
                          0, 0, 0
],
                                dtype=float)

open_hand = np.array([1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000], dtype=int)
close_hand = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=int)

dt = 0.0025
runing_time = 0.0


low_state = None
getstate_flag = False  # flag to indicate if data is received

def low_state_handler(msg):
    global low_state, getstate_flag
    low_state = msg
    getstate_flag = True  # activate the flag when data is received

input("Press enter to start")

if __name__ == '__main__':

    if len(sys.argv) <2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(1, sys.argv[1])

    # Create a publisher to publish the data defined in UserData class
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    cmd = pnd_adam_msg_dds__LowCmd_(19)

    hand_pub = ChannelPublisher("rt/handcmd", HandCmd_)
    hand_pub.Init()
    hand_cmd = pnd_adam_msg_dds__HandCmd_()

    lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
    lowstate_sub.Init(low_state_handler, 1)

    print("waiting for low_state data...")
    wait_interval = 0.001  # 1ms wait interval
    timeout = 10.0  # maximum wait time (seconds) to prevent infinite waiting
    start_time = time.time()
    
    while not getstate_flag:
        # check for timeout
        if time.time() - start_time > timeout:
            print(f"Time out! Failed to get low_state data in {timeout}seconds")
            sys.exit(0)
        time.sleep(wait_interval)

    # [Stage 1]: set robot to zero posture
    duration = 2.0
    start_time = time.time()
    while time.time() - start_time < duration :
        for i in range(ADAM_U_NUM_MOTOR):
            ratio = np.clip((time.time() - start_time) / duration, 0.0, 1.0)
            cmd.motor_cmd[i].mode =  1 # 1:Enable, 0:Disable
            cmd.motor_cmd[i].tau = 0. 
            cmd.motor_cmd[i].q = (1.0 - ratio) * low_state.motor_state[i].q 
            cmd.motor_cmd[i].dq = 0.
            cmd.motor_cmd[i].kp = KP_CONFIG[i]
            cmd.motor_cmd[i].kd = KD_CONFIG[i]
        pub.Write(cmd)
        time.sleep(wait_interval)
    print("Back to zero posture completed. Starting open/close arm demo...")

    while True:
        step_start = time.perf_counter()

        runing_time += dt

        if (runing_time < 3.0):
            # Stand up in first 3 second
            
            # Total time for standing up or standing down is about 1.2s
            phase = np.tanh(runing_time / 1.2)
            for i in range(ADAM_U_NUM_MOTOR):
                cmd.motor_cmd[i].q = phase * open_arm_pos[i] + (
                    1 - phase) * close_arm_pos[i]
                # 使用配置的 Kp 和 Kd 值
                cmd.motor_cmd[i].kp = KP_CONFIG[i]
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = KD_CONFIG[i]
                cmd.motor_cmd[i].tau = 0.0

            for i in range(12):
                hand_cmd.position[i] = close_hand[i]
        else:
            # Then stand down
            phase = np.tanh((runing_time - 3.0) / 1.2)
            for i in range(ADAM_U_NUM_MOTOR):
                cmd.motor_cmd[i].q = phase * close_arm_pos[i] + (
                    1 - phase) * open_arm_pos[i]
                # 使用配置的 Kp 和 Kd 值
                cmd.motor_cmd[i].kp = KP_CONFIG[i]
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = KD_CONFIG[i]
                cmd.motor_cmd[i].tau = 0.0

            for i in range(12):
                hand_cmd.position[i] = open_hand[i]

        #print(cmd.motor_cmd[6].q)
        pub.Write(cmd)
        hand_pub.Write(hand_cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
