Pose lh0
Pose lh1
Pose lh2
Pose lh3
Pose lh4

Pose rh0
Pose rh1
Pose rh2
Pose rh3
Pose rh4
bool is_use_arms_ee_pose
bool is_hands_valid
bool is_use_arms_jnt
float32[7] left_arm_ee_pose #x,y,z,quatern(qx,qy,qz,qw)
float32[7] right_arm_ee_pose #x,y,z,quatern(qx,qy,qz,qw)

float32[3] left_elbow_position_to_baselink
float32[3] right_elbow_position_to_baselink

float32[6] left_arm_ee_v
float32[6] right_arm_ee_v

float32[3] lefthumb
float32[3] leftindex
float32[2] leftmid
float32[2] leftring
float32[2] leftpinky

float32[3] righthumb
float32[3] rightindex
float32[2] rightmid
float32[2] rightring
float32[2] rightpinky

float32[7] left_arm
float32[7] right_arm

float32[2] head
float32[2] waist
float32[6] body
float32[24] ori_hand

####vel####
float32[7] left_arm_dq
float32[7] right_arm_dq

float32[2] head_dq
float32[2] waist_dq
float32[6] body_dq
