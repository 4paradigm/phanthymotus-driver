bool         is_upper_limb_ctrl
float32[4]   quat_est_qw_last#qx,qy,qz,qw
float32[3]   angular_vel_local # angular velocity local
float32[6]   base_pose_act_rpy_fake  #x,y,z,roll,pitch,yaw,fake
float32[6]   base_vel_act_fake #linear vel and angular vel act

float32[]    jnt_q_act
float32[]    jnt_v_act
float32[]    jnt_tau_act

float32[] l_hand_jnt_q_act
float32[] r_hand_jnt_q_act

float32[] l_hand_jnt_v_act
float32[] r_hand_jnt_v_act

float32[] l_hand_jnt_tau_act
float32[] r_hand_jnt_tau_act

