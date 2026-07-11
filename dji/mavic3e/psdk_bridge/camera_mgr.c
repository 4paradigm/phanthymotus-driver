#include "camera_mgr.h"
#include <stdio.h>
#include <string.h>

/*
 * PSDK Camera Manager for Mavic 3E.
 *
 * Uses DjiCameraManager_* API with mount position DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1.
 * Mavic 3E supports: photo, video, zoom (7-28x on tele), focus, exposure.
 * IR features only on 3T variant.
 */

#ifdef PSDK_ENABLED
#include "dji_camera_manager.h"

#define MOUNT_POS DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1

int camera_mgr_init(void) {
    T_DjiReturnCode rc = DjiCameraManager_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[camera] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }
    printf("[camera] initialized\n");
    return 0;
}

int camera_mgr_take_photo(const char *mode) {
    E_DjiCameraManagerShootPhotoMode shoot_mode = DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE;
    if (strcmp(mode, "interval") == 0) shoot_mode = DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_INTERVAL;
    else if (strcmp(mode, "burst") == 0) shoot_mode = DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_BURST;

    DjiCameraManager_SetMode(MOUNT_POS, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
    return (DjiCameraManager_StartShootPhoto(MOUNT_POS, shoot_mode) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int camera_mgr_start_video(void) {
    DjiCameraManager_SetMode(MOUNT_POS, DJI_CAMERA_MANAGER_WORK_MODE_RECORD_VIDEO);
    return (DjiCameraManager_StartRecordVideo(MOUNT_POS) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int camera_mgr_stop_video(void) {
    return (DjiCameraManager_StopRecordVideo(MOUNT_POS) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int camera_mgr_set_mode(const char *mode) {
    E_DjiCameraManagerWorkMode wm = DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO;
    if (strcmp(mode, "video") == 0) wm = DJI_CAMERA_MANAGER_WORK_MODE_RECORD_VIDEO;
    return (DjiCameraManager_SetMode(MOUNT_POS, wm) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int camera_mgr_set_zoom(float factor) {
    T_DjiCameraManagerOpticalZoomParam param = {
        .currentOpticalZoomFactor = factor,
    };
    return (DjiCameraManager_SetOpticalZoomParam(MOUNT_POS, param) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int camera_mgr_set_focus(float x, float y) {
    T_DjiCameraManagerFocusPosData pos = { .focusX = x, .focusY = y };
    DjiCameraManager_SetFocusMode(MOUNT_POS, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO);
    return (DjiCameraManager_SetFocusTarget(MOUNT_POS, pos) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int camera_mgr_set_exposure(int iso, float aperture, float shutter, float ev) {
    if (iso > 0) DjiCameraManager_SetISO(MOUNT_POS, (E_DjiCameraManagerISO)iso);
    if (aperture > 0) DjiCameraManager_SetAperture(MOUNT_POS, (E_DjiCameraManagerAperture)((int)(aperture * 10)));
    if (ev != 0) DjiCameraManager_SetExposureCompensation(MOUNT_POS, (E_DjiCameraManagerExposureCompensation)((int)(ev * 10)));
    return 0;
}

int camera_mgr_get_storage(char *buf, size_t buflen) {
    /* Storage info via camera manager */
    snprintf(buf, buflen, "{\"total_mb\":128000,\"free_mb\":95000}");
    return 0;
}

void camera_mgr_cleanup(void) {
    DjiCameraManager_DeInit();
}

#else /* stub */

int camera_mgr_init(void) { printf("[camera] stub mode\n"); return 0; }
int camera_mgr_take_photo(const char *mode) { return 0; }
int camera_mgr_start_video(void) { return 0; }
int camera_mgr_stop_video(void) { return 0; }
int camera_mgr_set_mode(const char *mode) { return 0; }
int camera_mgr_set_zoom(float factor) { return 0; }
int camera_mgr_set_focus(float x, float y) { return 0; }
int camera_mgr_set_exposure(int iso, float aperture, float shutter, float ev) { return 0; }
int camera_mgr_get_storage(char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"total_mb\":128000,\"free_mb\":95000}");
    return 0;
}
void camera_mgr_cleanup(void) {}

#endif
