import time
import sys
import os

from pndbotics_sdk_py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from pndbotics_sdk_py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from pndbotics_sdk_py.idl.default import pnd_adam_msg_dds__LowCmd_
from pndbotics_sdk_py.idl.default import pnd_adam_msg_dds__LowState_
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import LowCmd_
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import LowState_
from pndbotics_sdk_py.idl.pnd_adam.msg.dds_ import HandCmd_
from pndbotics_sdk_py.utils.thread import RecurrentThread
from pndbotics_sdk_py.idl.default import pnd_adam_msg_dds__HandCmd_

import numpy as np

ADAM_PRO_NUM_MOTOR = 31

Kp = [
    305.0, 700.0, 405.0, 305.0, 30.0, 0.0,      # Left leg: hipPitch, hipRoll, hipYaw, kneePitch, anklePitch, ankleRoll
    305.0, 700.0, 405.0, 305.0, 30.0, 0.0,      # Right leg: hipPitch, hipRoll, hipYaw, kneePitch, anklePitch, ankleRoll
    205.0, 405.0, 405.0,                        # Waist: waistYaw, waistRoll, waistPitch
    18.0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0,         # Left arm: shoulderPitch, shoulderRoll, shoulderYaw, elbow, wristYaw, wristPitch, wristRoll
    18.0, 9.0, 9.0, 9.0, 9.0, 9.0, 9.0,         # Right arm: shoulderPitch, shoulderRoll, shoulderYaw, elbow, wristYaw, wristPitch, wristRoll
    40,40                                       # head: head_pitch, head_yaw
]

Kd = [
    6.1, 30.0, 6.1, 6.1, 2.25, 0.25,     # Left leg: hipPitch, hipRoll, hipYaw, kneePitch, anklePitch, ankleRoll
    6.1, 30.0, 6.1, 6.1, 2.25, 0.25,     # Right leg: hipPitch, hipRoll, hipYaw, kneePitch, anklePitch, ankleRoll
    4.1, 6.1, 6.1,                       # Waist: waistYaw, waistRoll, waistPitch
    0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9,   # Left arm: shoulderPitch, shoulderRoll, shoulderYaw, elbow, wristYaw, wristPitch, wristRoll
    0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9,
    1, 1                                              # head: head_yaw, head_pitch
]

class ADAMJointIndex:
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11
    WaistYaw = 12
    WaistRoll = 13        
    WaistPitch = 14       
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   
    LeftWristYaw = 21     
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  
    RightWristYaw = 28    
    HeadYaw = 29   
    HeadPitch = 30   


class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints

class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.0025  # [2ms]
        self.duration_ = 3.0    # [3 s]
        self.counter_ = 0
        self.low_cmd = pnd_adam_msg_dds__LowCmd_(ADAM_PRO_NUM_MOTOR)  
        self.low_state = pnd_adam_msg_dds__LowState_(ADAM_PRO_NUM_MOTOR)

        self.hand_cmd = pnd_adam_msg_dds__HandCmd_()
        self.close_hand = np.array([500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500], dtype=int)
        self.open_hand = np.array([1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,  1000, 1000, 1000, 1000], dtype=int)
        self.getstate_flag = False
    def Init(self):
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        self.hand_pub = ChannelPublisher("rt/handcmd", HandCmd_)
        self.hand_pub.Init()
        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.control_dt_, target=self.LowCmdWrite, name="control"
        )
        self.lowCmdWriteThreadPtr.Start()

    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        self.counter_ +=1
        if (self.counter_ % 500 == 0) :
            self.counter_ = 0
            print(self.low_state.imu_state.ypr)


    def LowCmdWrite(self):
        self.time_ += self.control_dt_

        if self.time_ < self.duration_ :
            for i in range(ADAM_PRO_NUM_MOTOR):
                ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                self.low_cmd.motor_cmd[i].mode =  1 # 1:Enable, 0:Disable
                self.low_cmd.motor_cmd[i].tau = 0. 
                self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q 
                self.low_cmd.motor_cmd[i].dq = 0. 
                self.low_cmd.motor_cmd[i].kp = Kp[i] 
                self.low_cmd.motor_cmd[i].kd = Kd[i] 
            for i in range(12):
                self.hand_cmd.position[i] = self.close_hand[i]

        elif self.time_ < self.duration_ * 2 :
            # [Stage 2]: swing ankle using PR mode
            max_P = np.pi * 30.0 / 180.0
            max_R = np.pi * 10.0 / 180.0
            t = self.time_ - self.duration_
            L_P_des = max_P * np.sin(2.0 * np.pi * t)
            L_R_des = max_R * np.sin(2.0 * np.pi * t)
            R_P_des = max_P * np.sin(2.0 * np.pi * t)
            R_R_des = -max_R * np.sin(2.0 * np.pi * t)

            self.low_cmd.mode_pr = Mode.PR
            self.low_cmd.motor_cmd[ADAMJointIndex.LeftAnklePitch].q = L_P_des
            self.low_cmd.motor_cmd[ADAMJointIndex.LeftAnkleRoll].q = L_R_des
            self.low_cmd.motor_cmd[ADAMJointIndex.RightAnklePitch].q = R_P_des
            self.low_cmd.motor_cmd[ADAMJointIndex.RightAnkleRoll].q = R_R_des
        elif self.time_ < self.duration_ * 3 :
            # [Stage 3]: swing ankle using AB mode
            max_A = np.pi * 30.0 / 180.0
            max_B = np.pi * 10.0 / 180.0
            t = self.time_ - self.duration_ * 2
            L_A_des = max_A * np.sin(2.0 * np.pi * t)
            L_B_des = max_B * np.sin(2.0 * np.pi * t + np.pi)
            R_A_des = -max_A * np.sin(2.0 * np.pi * t)
            R_B_des = -max_B * np.sin(2.0 * np.pi * t + np.pi)

            self.low_cmd.mode_pr = Mode.AB
            self.low_cmd.motor_cmd[ADAMJointIndex.LeftAnkleA].q = L_A_des
            self.low_cmd.motor_cmd[ADAMJointIndex.LeftAnkleB].q = L_B_des
            self.low_cmd.motor_cmd[ADAMJointIndex.RightAnkleA].q = R_A_des
            self.low_cmd.motor_cmd[ADAMJointIndex.RightAnkleB].q = R_B_des
            
            max_WristYaw = np.pi * 30.0 / 180.0
            L_WristYaw_des = max_WristYaw * np.sin(2.0 * np.pi * t)
            R_WristYaw_des = max_WristYaw * np.sin(2.0 * np.pi * t)
            self.low_cmd.motor_cmd[ADAMJointIndex.LeftWristRoll].q = L_WristYaw_des
            self.low_cmd.motor_cmd[ADAMJointIndex.RightWristRoll].q = R_WristYaw_des
            for i in range(ADAM_PRO_NUM_MOTOR):
                self.low_cmd.motor_cmd[i].mode =  1 # 1:Enable, 0:Disable
                self.low_cmd.motor_cmd[i].tau = 0. 
                self.low_cmd.motor_cmd[i].q = 0
                self.low_cmd.motor_cmd[i].dq = 0. 
                self.low_cmd.motor_cmd[i].kp = Kp[i] 
                self.low_cmd.motor_cmd[i].kd = Kd[i] 
            for i in range(12):
                self.hand_cmd.position[i] = self.open_hand[i]

        self.hand_pub.Write(self.hand_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)



if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv)>1:
        ChannelFactoryInitialize(1, sys.argv[1])  # Use domain ID 1 instead of 0
    else:
        ChannelFactoryInitialize(1, "lo")  # Use domain ID 1 instead of 0

    custom = Custom()
    custom.Init()
    print("Initialized")
    custom.Start()

    while True:        
        time.sleep(1)