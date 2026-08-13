#ifndef GIMBAL_MGR_H
#define GIMBAL_MGR_H

int gimbal_mgr_init(void);
int gimbal_mgr_rotate(float pitch, float yaw, float roll, const char *mode, float duration);
int gimbal_mgr_reset(void);
int gimbal_mgr_set_mode(const char *mode);
int gimbal_mgr_get_angles(float *pitch, float *yaw, float *roll);
void gimbal_mgr_cleanup(void);

#endif
