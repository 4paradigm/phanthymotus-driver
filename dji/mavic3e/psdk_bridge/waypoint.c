#include "waypoint.h"
#include <stdio.h>
#include <string.h>

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

static T_DjiReturnCode _mission_state_cb(T_DjiWaypointV3MissionState state) {
    switch (state.state) {
        case 0: s_state_str = "idle"; break;
        case 1: s_state_str = "executing"; break;
        case 2: s_state_str = "paused"; break;
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
    DjiWaypointV3_RegMissionStateCallback(_mission_state_cb);
    printf("[waypoint] V3 initialized\n");
    return 0;
}

int waypoint_upload(const char *kmz_path) {
    FILE *f = fopen(kmz_path, "rb");
    if (!f) {
        printf("[waypoint] cannot open: %s\n", kmz_path);
        return -1;
    }
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *data = (uint8_t *)malloc(fsize);
    if (!data) { fclose(f); return -1; }
    fread(data, 1, fsize, f);
    fclose(f);

    T_DjiReturnCode rc = DjiWaypointV3_UploadKmzFile(data, fsize);
    free(data);
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int waypoint_start(void) {
    return (DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_START) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int waypoint_pause(void) {
    return (DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_PAUSE) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int waypoint_resume(void) {
    return (DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_RESUME) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int waypoint_stop(void) {
    return (DjiWaypointV3_Action(DJI_WAYPOINT_V3_ACTION_STOP) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int waypoint_get_status(char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"state\":\"%s\"}", s_state_str);
    return 0;
}

void waypoint_cleanup(void) {
    DjiWaypointV3_DeInit();
}

#else /* stub */

int waypoint_init(void) { printf("[waypoint] stub mode\n"); return 0; }
int waypoint_upload(const char *kmz_path) { return 0; }
int waypoint_start(void) { return 0; }
int waypoint_pause(void) { return 0; }
int waypoint_resume(void) { return 0; }
int waypoint_stop(void) { return 0; }
int waypoint_get_status(char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"state\":\"idle\"}");
    return 0;
}
void waypoint_cleanup(void) {}

#endif
