#include "hms.h"
#include <stdio.h>
#include <string.h>

/*
 * PSDK HMS (Health Management System) for Mavic 3E.
 *
 * Key APIs:
 *   DjiHmsManager_Init()
 *   DjiHmsManager_RegHmsInfoCallback()
 *
 * Callback receives error codes + severity level.
 * Cross-reference with hms.json database for human-readable messages.
 */

#ifdef PSDK_ENABLED
#include "dji_hms.h"
#include "dji_hms_info_table.h"

#define MAX_ALERTS 32

typedef struct {
    uint32_t error_code;
    uint8_t  component_index;
    uint8_t  error_level;
} hms_alert_t;

static hms_alert_t s_alerts[MAX_ALERTS];
static int s_alert_count = 0;

static T_DjiReturnCode _hms_cb(T_DjiHmsInfoTable info) {
    s_alert_count = 0;
    for (uint32_t i = 0; i < info.hmsInfoNum && s_alert_count < MAX_ALERTS; i++) {
        s_alerts[s_alert_count].error_code = info.hmsInfo[i].errorCode;
        s_alerts[s_alert_count].component_index = info.hmsInfo[i].componentIndex;
        s_alerts[s_alert_count].error_level = info.hmsInfo[i].errorLevel;
        s_alert_count++;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Lookup error code in built-in hmsErrCodeInfoTbl (727 entries) */
static const char *_lookup_msg(uint32_t code, int is_flying) {
    extern const T_DjiHmsErrCodeInfo hmsErrCodeInfoTbl[];
    extern const uint32_t hmsErrCodeInfoTblSize;
    for (uint32_t i = 0; i < hmsErrCodeInfoTblSize; i++) {
        if (hmsErrCodeInfoTbl[i].alarmId == code) {
            if (is_flying && hmsErrCodeInfoTbl[i].flyAlarmInfo && hmsErrCodeInfoTbl[i].flyAlarmInfo[0])
                return hmsErrCodeInfoTbl[i].flyAlarmInfo;
            if (hmsErrCodeInfoTbl[i].groundAlarmInfo && hmsErrCodeInfoTbl[i].groundAlarmInfo[0])
                return hmsErrCodeInfoTbl[i].groundAlarmInfo;
            return hmsErrCodeInfoTbl[i].flyAlarmInfo ? hmsErrCodeInfoTbl[i].flyAlarmInfo : "";
        }
    }
    return "";
}

int hms_init(void) {
    T_DjiReturnCode rc = DjiHmsManager_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[hms] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }
    DjiHmsManager_RegHmsInfoCallback(_hms_cb);
    printf("[hms] initialized\n");
    return 0;
}

int hms_get_info(char *buf, size_t buflen) {
    int offset = 0;
    offset += snprintf(buf + offset, buflen - offset, "{\"alerts\":[");
    for (int i = 0; i < s_alert_count; i++) {
        if (i > 0) offset += snprintf(buf + offset, buflen - offset, ",");
        const char *ground_msg = _lookup_msg(s_alerts[i].error_code, 0);
        const char *fly_msg = _lookup_msg(s_alerts[i].error_code, 1);
        offset += snprintf(buf + offset, buflen - offset,
            "{\"code\":\"0x%08X\",\"component\":%d,\"level\":%d,"
            "\"ground_msg\":\"%s\",\"fly_msg\":\"%s\"}",
            s_alerts[i].error_code, s_alerts[i].component_index, s_alerts[i].error_level,
            ground_msg, fly_msg);
    }
    offset += snprintf(buf + offset, buflen - offset, "]}");
    return 0;
}

void hms_cleanup(void) {
    DjiHmsManager_DeInit();
}

#else /* stub */

int hms_init(void) { printf("[hms] stub mode\n"); return 0; }
int hms_get_info(char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"alerts\":[]}");
    return 0;
}
void hms_cleanup(void) {}

#endif
