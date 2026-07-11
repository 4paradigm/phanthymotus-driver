#include "gimbal_mgr.h"
#include <stdio.h>
#include <string.h>

/*
 * PSDK Gimbal Manager for Mavic 3E.
 * Gimbal range: pitch -90~+35, yaw -40~+40 (narrower than M300/M350).
 */

#ifdef PSDK_ENABLED
#include "dji_gimbal_manager.h"

#define MOUNT_POS DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1

int gimbal_mgr_init(void) {
    T_DjiReturnCode rc = DjiGimbalManager_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[gimbal] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }
    printf("[gimbal] initialized\n");
    return 0;
}

int gimbal_mgr_rotate(float pitch, float yaw, float roll, const char *mode, float duration) {
    T_DjiGimbalManagerRotation rotation = {0};
    rotation.pitch = pitch;
    rotation.yaw = yaw;
    rotation.roll = roll;
    rotation.time = duration;

    if (strcmp(mode, "relative") == 0)
        rotation.rotationMode = DJI_GIMBAL_MANAGER_ROTATION_MODE_RELATIVE_ANGLE;
    else if (strcmp(mode, "speed") == 0)
        rotation.rotationMode = DJI_GIMBAL_MANAGER_ROTATION_MODE_SPEED;
    else
        rotation.rotationMode = DJI_GIMBAL_MANAGER_ROTATION_MODE_ABSOLUTE_ANGLE;

    return (DjiGimbalManager_Rotate(MOUNT_POS, rotation) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int gimbal_mgr_reset(void) {
    return (DjiGimbalManager_Reset(MOUNT_POS) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int gimbal_mgr_set_mode(const char *mode) {
    E_DjiGimbalMode gm = DJI_GIMBAL_MODE_YAW_FOLLOW;
    if (strcmp(mode, "free") == 0) gm = DJI_GIMBAL_MODE_FREE;
    else if (strcmp(mode, "fpv") == 0) gm = DJI_GIMBAL_MODE_FPV;
    return (DjiGimbalManager_SetMode(MOUNT_POS, gm) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int gimbal_mgr_get_angles(float *pitch, float *yaw, float *roll) {
    /* Angles come from FC subscription (gimbal topic), not gimbal manager directly */
    *pitch = 0; *yaw = 0; *roll = 0;
    return 0;
}

void gimbal_mgr_cleanup(void) {
    DjiGimbalManager_DeInit();
}

#else /* stub */

static float s_pitch = 0, s_yaw = 0, s_roll = 0;

int gimbal_mgr_init(void) { printf("[gimbal] stub mode\n"); return 0; }
int gimbal_mgr_rotate(float pitch, float yaw, float roll, const char *mode, float duration) {
    s_pitch = pitch; s_yaw = yaw; s_roll = roll;
    return 0;
}
int gimbal_mgr_reset(void) { s_pitch = s_yaw = s_roll = 0; return 0; }
int gimbal_mgr_set_mode(const char *mode) { return 0; }
int gimbal_mgr_get_angles(float *pitch, float *yaw, float *roll) {
    *pitch = s_pitch; *yaw = s_yaw; *roll = s_roll;
    return 0;
}
void gimbal_mgr_cleanup(void) {}

#endif
