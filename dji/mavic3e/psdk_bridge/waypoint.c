#include "waypoint.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/*
 * PSDK Waypoint V3 for Mavic 3E.
 *
 * Mavic 3E uses Waypoint V3 (NOT V2). Mission defined via KMZ file.
 *
 * Key APIs:
 *   DjiWaypointV3_Init()
 *   DjiWaypointV3_UploadKmzFile(filePath, fileLen)
 *   DjiWaypointV3_Action(START/PAUSE/RESUME/STOP)
 *   DjiWaypointV3_RegMissionStateCallback()
 */

#ifdef PSDK_ENABLED
#include "dji_waypoint_v3.h"
#include <stdlib.h>

static const char *s_state_str = "idle";
static int s_uploaded = 0;

static T_DjiReturnCode _mission_state_cb(T_DjiWaypointV3MissionState state) {
    switch (state.state) {
        case DJI_WAYPOINT_V3_MISSION_STATE_IDLE:
            s_state_str = "idle";
            break;
        case DJI_WAYPOINT_V3_MISSION_STATE_PREPARE:
            s_state_str = "preparing";
            break;
        case DJI_WAYPOINT_V3_MISSION_STATE_TRANS_MISSION:
        case DJI_WAYPOINT_V3_MISSION_STATE_MISSION:
        case DJI_WAYPOINT_V3_MISSION_STATE_RESUME:
        case DJI_WAYPOINT_V3_MISSION_STATE_RETURN_FIRSTPOINT:
            s_state_str = "executing";
            break;
        case DJI_WAYPOINT_V3_MISSION_STATE_BREAK:
            s_state_str = "paused";
            break;
        default: s_state_str = "unknown"; break;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

int waypoint_init(void) {
    T_DjiReturnCode rc = DjiWaypointV3_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[waypoint] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }
    rc = DjiWaypointV3_RegMissionStateCallback(_mission_state_cb);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[waypoint] state callback registration failed: 0x%08llX\n", (unsigned long long)rc);
        DjiWaypointV3_DeInit();
        return -1;
    }
    printf("[waypoint] V3 initialized\n");
    return 0;
}

int waypoint_upload(const char *kmz_path) {
    FILE *f = fopen(kmz_path, "rb");
    if (!f) {
        printf("[waypoint] cannot open: %s\n", kmz_path);
        return -1;
    }
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return -1; }
    long fsize = ftell(f);
    if (fsize <= 0 || fsize > UINT32_MAX || fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        return -1;
    }
    uint8_t *data = (uint8_t *)malloc((size_t)fsize);
    if (!data) { fclose(f); return -1; }
    size_t bytes_read = fread(data, 1, (size_t)fsize, f);
    fclose(f);
    if (bytes_read != (size_t)fsize) { free(data); return -1; }

    T_DjiReturnCode rc = DjiWaypointV3_UploadKmzFile(data, (uint32_t)fsize);
    free(data);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[waypoint] upload failed: 0x%08llX (%s)\n", (unsigned long long)rc, kmz_path);
        s_uploaded = 0;
        return -1;
    }
    s_uploaded = 1;
    s_state_str = "ready";
    return 0;
}

int waypoint_start(uint64_t *raw_rc) {
    if (!s_uploaded) {
        printf("[waypoint] start rejected: upload a KMZ first\n");
        if (raw_rc) *raw_rc = UINT64_MAX;
        return -1;
    }
    T_DjiReturnCode rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_START);
    if (raw_rc) *raw_rc = (uint64_t)rc;
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_state_str = "executing";
    } else {
        printf("[waypoint] start failed: 0x%08llX\n", (unsigned long long)rc);
    }
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int waypoint_pause(void) {
    T_DjiReturnCode rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_PAUSE);
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) s_state_str = "paused";
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int waypoint_resume(void) {
    T_DjiReturnCode rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_RESUME);
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) s_state_str = "executing";
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int waypoint_stop(void) {
    T_DjiReturnCode rc = DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_STOP);
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) s_state_str = "idle";
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int waypoint_get_status(char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"state\":\"%s\",\"uploaded\":%s}",
             s_state_str, s_uploaded ? "true" : "false");
    return 0;
}

void waypoint_cleanup(void) {
    DjiWaypointV3_DeInit();
}

#else /* stub */

int waypoint_init(void) { printf("[waypoint] stub mode\n"); return 0; }
int waypoint_upload(const char *kmz_path) { return 0; }
int waypoint_start(uint64_t *raw_rc) {
    if (raw_rc) *raw_rc = 0;
    return 0;
}
int waypoint_pause(void) { return 0; }
int waypoint_resume(void) { return 0; }
int waypoint_stop(void) { return 0; }
int waypoint_get_status(char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"state\":\"idle\"}");
    return 0;
}
void waypoint_cleanup(void) {}

#endif
