#include "camera_mgr.h"
#include "error_code.h"
#include <stdio.h>
#include <string.h>



#ifdef PSDK_ENABLED
#include "dji_camera_manager.h"

static int s_camera_manager_initialized;

static E_DjiMountPosition _mount_position(const char *camera) {
    if (camera && (strcmp(camera, "payload2") == 0 || strcmp(camera, "port2") == 0))
        return DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2;
    if (camera && (strcmp(camera, "payload3") == 0 || strcmp(camera, "port3") == 0))
        return DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3;
    return DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
}

int camera_mgr_init(void) {
    if (s_camera_manager_initialized)
        return 0;
    T_DjiReturnCode rc = DjiCameraManager_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[camera] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }
    s_camera_manager_initialized = 1;
    printf("[camera] initialized\n");
    return 0;
}

static int _ensure_camera_manager_initialized(void) {
    /* This M300 exposes FPV by default.  Do not subscribe to external payload
     * camera state unless a caller explicitly performs a payload operation. */
    return camera_mgr_init();
}

int camera_mgr_take_photo(const char *camera, const char *mode) {
    if (_ensure_camera_manager_initialized() != 0) return -1;
    E_DjiMountPosition pos = _mount_position(camera);
    E_DjiCameraManagerShootPhotoMode shoot_mode = DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE;
    if (strcmp(mode, "interval") == 0) shoot_mode = DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_INTERVAL;
    else if (strcmp(mode, "burst") == 0) shoot_mode = DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_BURST;

    DjiCameraManager_SetMode(pos, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
    DjiCameraManager_SetShootPhotoMode(pos, shoot_mode);
    return (DjiCameraManager_StartShootPhoto(pos, shoot_mode) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int camera_mgr_start_video(const char *camera) {
    if (_ensure_camera_manager_initialized() != 0) return -1;
    E_DjiMountPosition pos = _mount_position(camera);
    DjiCameraManager_SetMode(pos, DJI_CAMERA_MANAGER_WORK_MODE_RECORD_VIDEO);
    return (DjiCameraManager_StartRecordVideo(pos) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int camera_mgr_stop_video(const char *camera) {
    if (_ensure_camera_manager_initialized() != 0) return -1;
    return (DjiCameraManager_StopRecordVideo(_mount_position(camera)) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int camera_mgr_set_mode(const char *camera, const char *mode) {
    if (_ensure_camera_manager_initialized() != 0) return -1;
    E_DjiCameraManagerWorkMode wm = DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO;
    if (strcmp(mode, "video") == 0) wm = DJI_CAMERA_MANAGER_WORK_MODE_RECORD_VIDEO;
    return (DjiCameraManager_SetMode(_mount_position(camera), wm) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int camera_mgr_set_zoom(const char *camera, float factor) {
    if (_ensure_camera_manager_initialized() != 0) return -1;
    E_DjiMountPosition pos = _mount_position(camera);
    /* Ensure we're on the zoom lens before setting zoom factor */
    T_DjiReturnCode rc;
    rc = DjiCameraManager_SetStreamSource(pos, DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM);
    printf("[camera] SetStreamSource(ZOOM) → 0x%08llX\n", (unsigned long long)rc);

    /* SetOpticalZoomParam: factor is absolute zoom multiplier (e.g. 2.0 = 2x) */
    E_DjiCameraZoomDirection dir = (factor >= 1.0f) ? DJI_CAMERA_ZOOM_DIRECTION_IN : DJI_CAMERA_ZOOM_DIRECTION_OUT;
    rc = DjiCameraManager_SetOpticalZoomParam(pos, dir, factor);
    printf("[camera] SetOpticalZoomParam(dir=%d, factor=%.1f) → 0x%08llX\n",
           dir, factor, (unsigned long long)rc);
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int camera_mgr_set_focus(const char *camera, float x, float y) {
    if (_ensure_camera_manager_initialized() != 0) return -1;
    E_DjiMountPosition mount_pos = _mount_position(camera);
    T_DjiCameraManagerFocusPosData focus_pos = { .focusX = x, .focusY = y };
    DjiCameraManager_SetFocusMode(mount_pos, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO);
    return (DjiCameraManager_SetFocusTarget(mount_pos, focus_pos) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int camera_mgr_set_exposure(const char *camera, int iso, float aperture, float shutter, float ev) {
    if (_ensure_camera_manager_initialized() != 0) return -1;
    E_DjiMountPosition pos = _mount_position(camera);
    if (iso > 0) DjiCameraManager_SetISO(pos, (E_DjiCameraManagerISO)iso);
    if (aperture > 0) DjiCameraManager_SetAperture(pos, (E_DjiCameraManagerAperture)((int)(aperture * 10)));
    if (ev != 0) DjiCameraManager_SetExposureCompensation(pos, (E_DjiCameraManagerExposureCompensation)((int)(ev * 10)));
    return 0;
}

int camera_mgr_get_storage(const char *camera, char *buf, size_t buflen) {
    (void)camera;
    snprintf(buf, buflen, "{\"error\":\"storage query is unavailable for this payload\"}");
    return -1;
}

int camera_mgr_ir_temp_point(const char *camera, float x, float y, char *buf, size_t buflen) {
    if (_ensure_camera_manager_initialized() != 0) {
        snprintf(buf, buflen, "{\"error\":\"camera manager initialization failed\"}");
        return -1;
    }
    E_DjiMountPosition pos = _mount_position(camera);
    T_DjiCameraManagerPointThermometryCoordinate coord = { .pointX = x, .pointY = y };
    T_DjiReturnCode rc = DjiCameraManager_SetPointThermometryCoordinate(pos, coord);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        error_code_to_json(rc, buf, buflen);
        return -1;
    }
    T_DjiCameraManagerPointThermometryData data;
    rc = DjiCameraManager_GetPointThermometryData(pos, &data);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        error_code_to_json(rc, buf, buflen);
        return -1;
    }
    snprintf(buf, buflen, "{\"x\":%.3f,\"y\":%.3f,\"temperature\":%.1f}",
             data.pointX, data.pointY, data.pointTemperature);
    return 0;
}

int camera_mgr_ir_temp_area(const char *camera, float ltx, float lty, float rbx, float rby, char *buf, size_t buflen) {
    if (_ensure_camera_manager_initialized() != 0) {
        snprintf(buf, buflen, "{\"error\":\"camera manager initialization failed\"}");
        return -1;
    }
    E_DjiMountPosition pos = _mount_position(camera);
    T_DjiCameraManagerAreaThermometryCoordinate coord = {
        .areaTempLtX = ltx, .areaTempLtY = lty,
        .areaTempRbX = rbx, .areaTempRbY = rby,
    };
    T_DjiReturnCode rc = DjiCameraManager_SetAreaThermometryCoordinate(pos, coord);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        error_code_to_json(rc, buf, buflen);
        return -1;
    }
    T_DjiCameraManagerAreaThermometryData data;
    rc = DjiCameraManager_GetAreaThermometryData(pos, &data);
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
    if (s_camera_manager_initialized) {
        DjiCameraManager_DeInit();
        s_camera_manager_initialized = 0;
    }
}

#else /* stub */

int camera_mgr_init(void) { printf("[camera] stub mode\n"); return 0; }
int camera_mgr_take_photo(const char *camera, const char *mode) { return 0; }
int camera_mgr_start_video(const char *camera) { return 0; }
int camera_mgr_stop_video(const char *camera) { return 0; }
int camera_mgr_set_mode(const char *camera, const char *mode) { return 0; }
int camera_mgr_set_zoom(const char *camera, float factor) { return 0; }
int camera_mgr_set_focus(const char *camera, float x, float y) { return 0; }
int camera_mgr_set_exposure(const char *camera, int iso, float aperture, float shutter, float ev) { return 0; }
int camera_mgr_get_storage(const char *camera, char *buf, size_t buflen) {
    (void)camera;
    snprintf(buf, buflen, "{\"error\":\"camera manager unavailable in stub mode\"}");
    return -1;
}
int camera_mgr_ir_temp_point(const char *camera, float x, float y, char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"x\":%.3f,\"y\":%.3f,\"temperature\":0.0}", x, y);
    return 0;
}
int camera_mgr_ir_temp_area(const char *camera, float ltx, float lty, float rbx, float rby, char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"avg\":0,\"min\":0,\"max\":0,\"min_x\":0,\"min_y\":0,\"max_x\":0,\"max_y\":0}");
    return 0;
}
void camera_mgr_cleanup(void) {}

#endif
