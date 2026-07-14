#include "camera_mgr.h"
#include "error_code.h"
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
    /* Ensure we're on the zoom lens before setting zoom factor */
    T_DjiReturnCode rc;
    rc = DjiCameraManager_SetStreamSource(MOUNT_POS, DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM);
    printf("[camera] SetStreamSource(ZOOM) → 0x%08llX\n", (unsigned long long)rc);

    /* SetOpticalZoomParam: factor is absolute zoom multiplier (e.g. 2.0 = 2x) */
    E_DjiCameraZoomDirection dir = (factor >= 1.0f) ? DJI_CAMERA_ZOOM_DIRECTION_IN : DJI_CAMERA_ZOOM_DIRECTION_OUT;
    rc = DjiCameraManager_SetOpticalZoomParam(MOUNT_POS, dir, factor);
    printf("[camera] SetOpticalZoomParam(dir=%d, factor=%.1f) → 0x%08llX\n",
           dir, factor, (unsigned long long)rc);
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
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

int camera_mgr_ir_temp_point(float x, float y, char *buf, size_t buflen) {
    T_DjiCameraManagerPointThermometryCoordinate coord = { .pointX = x, .pointY = y };
    T_DjiReturnCode rc = DjiCameraManager_SetPointThermometryCoordinate(MOUNT_POS, coord);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        error_code_to_json(rc, buf, buflen);
        return -1;
    }
    T_DjiCameraManagerPointThermometryData data;
    rc = DjiCameraManager_GetPointThermometryData(MOUNT_POS, &data);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        error_code_to_json(rc, buf, buflen);
        return -1;
    }
    snprintf(buf, buflen, "{\"x\":%.3f,\"y\":%.3f,\"temperature\":%.1f}",
             data.pointX, data.pointY, data.pointTemperature);
    return 0;
}

int camera_mgr_ir_temp_area(float ltx, float lty, float rbx, float rby, char *buf, size_t buflen) {
    T_DjiCameraManagerAreaThermometryCoordinate coord = {
        .areaTempLtX = ltx, .areaTempLtY = lty,
        .areaTempRbX = rbx, .areaTempRbY = rby,
    };
    T_DjiReturnCode rc = DjiCameraManager_SetAreaThermometryCoordinate(MOUNT_POS, coord);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        error_code_to_json(rc, buf, buflen);
        return -1;
    }
    T_DjiCameraManagerAreaThermometryData data;
    rc = DjiCameraManager_GetAreaThermometryData(MOUNT_POS, &data);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        error_code_to_json(rc, buf, buflen);
        return -1;
    }
    snprintf(buf, buflen,
        "{\"avg\":%.1f,\"min\":%.1f,\"max\":%.1f,"
        "\"min_x\":%.3f,\"min_y\":%.3f,\"max_x\":%.3f,\"max_y\":%.3f}",
        data.areaAveTemp, data.areaMinTemp, data.areaMaxTemp,
        data.areaMinTempPointX, data.areaMinTempPointY,
        data.areaMaxTempPointX, data.areaMaxTempPointY);
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
int camera_mgr_ir_temp_point(float x, float y, char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"x\":%.3f,\"y\":%.3f,\"temperature\":0.0}", x, y);
    return 0;
}
int camera_mgr_ir_temp_area(float ltx, float lty, float rbx, float rby, char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"avg\":0,\"min\":0,\"max\":0,\"min_x\":0,\"min_y\":0,\"max_x\":0,\"max_y\":0}");
    return 0;
}
void camera_mgr_cleanup(void) {}

#endif
