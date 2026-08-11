#include "hms.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/*
 * PSDK HMS (Health Management System) for Matrice 300 RTK.
 *
 * Key APIs:
 *   DjiHmsManager_Init() / DeInit()
 *   DjiHmsManager_RegHmsInfoCallback()
 *   DjiHmsCustomization_Init() / DeInit()
 *   DjiHmsCustomization_InjectHmsErrorCode() / EliminateHmsErrorCode()
 *
 * Problem:
 *   DjiHmsManager_Init() returns 0xE1 (camera manager timeout) because the
 *   camera module itself fails to initialize on this device. Since HMS
 *   internally relies on the same subscription mechanism that the camera
 *   module uses, HMS initialization also times out.
 *
 * Solution:
 *   1. Try DjiHmsManager_Init() first (may succeed on some devices).
 *   2. If it fails with 0xE1, fall back to DjiHmsCustomization which
 *      does NOT depend on the camera subscription path.
 *   3. Provide manual inject/eliminate APIs for testing custom HMS alerts.
 *
 * Prerequisites (per DJI docs):
 *   - DjiFcSubscription_Init() + DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT
 *     must be subscribed so HMS can distinguish in-air vs ground alerts.
 */

#ifdef PSDK_ENABLED
#include "dji_hms.h"
#include "dji_hms_info_table.h"
#include "dji_fc_subscription.h"

#define MAX_ALERTS 32

typedef struct {
    uint32_t error_code;
    uint8_t  component_index;
    uint8_t  error_level;
} hms_alert_t;

static hms_alert_t s_alerts[MAX_ALERTS];
static int s_alert_count = 0;
static int s_hms_manager_ready = 0; /* 1 if DjiHmsManager_Init succeeded */

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

/* ── HMS customization: manually inject/eliminate error codes ─────────── */
/*
 * These functions work independently of DjiHmsManager. They let us
 * inject custom HMS alerts (0x1E020000 ~ 0x1E02FFFF range) for testing.
 */
static uint32_t s_injected_alerts[MAX_ALERTS];
static uint8_t  s_injected_levels[MAX_ALERTS];
static int      s_injected_count = 0;

/* Lookup error code in built-in hmsErrCodeInfoTbl */
static char s_unknown_buf[64];
static const char *_lookup_msg(uint32_t code, int is_flying) {
    size_t tbl_size = sizeof(hmsErrCodeInfoTbl) / sizeof(hmsErrCodeInfoTbl[0]);
    for (size_t i = 0; i < tbl_size; i++) {
        if (hmsErrCodeInfoTbl[i].alarmId == code) {
            if (is_flying && hmsErrCodeInfoTbl[i].flyAlarmInfo && hmsErrCodeInfoTbl[i].flyAlarmInfo[0])
                return hmsErrCodeInfoTbl[i].flyAlarmInfo;
            if (hmsErrCodeInfoTbl[i].groundAlarmInfo && hmsErrCodeInfoTbl[i].groundAlarmInfo[0])
                return hmsErrCodeInfoTbl[i].groundAlarmInfo;
            return hmsErrCodeInfoTbl[i].flyAlarmInfo ? hmsErrCodeInfoTbl[i].flyAlarmInfo : "Unknown";
        }
    }
    snprintf(s_unknown_buf, sizeof(s_unknown_buf), "Unknown error 0x%08X", code);
    return s_unknown_buf;
}

/*
 * Get HMS info: merge alerts from DjiHmsManager callback (if available)
 * and manually injected alerts (from DjiHmsCustomization_InjectHmsErrorCode).
 */
int hms_get_info(char *buf, size_t buflen) {
    int offset = 0;
    offset += snprintf(buf + offset, buflen - offset, "{\"alerts\":[");

    int alert_idx = 0;

    /* Add alerts from DjiHmsManager callback */
    for (int i = 0; i < s_alert_count && alert_idx < MAX_ALERTS; i++) {
        if (i > 0) offset += snprintf(buf + offset, buflen - offset, ",");
        const char *ground_msg = _lookup_msg(s_alerts[i].error_code, 0);
        const char *fly_msg = _lookup_msg(s_alerts[i].error_code, 1);
        offset += snprintf(buf + offset, buflen - offset,
            "{\"code\":\"0x%08X\",\"component\":%d,\"level\":%d,"
            "\"ground_msg\":\"%s\",\"fly_msg\":\"%s\"}",
            s_alerts[i].error_code, s_alerts[i].component_index, s_alerts[i].error_level,
            ground_msg, fly_msg);
        alert_idx++;
    }

    /* Add manually injected alerts */
    for (int i = 0; i < s_injected_count && alert_idx < MAX_ALERTS; i++) {
        if (alert_idx > 0 || s_alert_count > 0)
            offset += snprintf(buf + offset, buflen - offset, ",");
        const char *ground_msg = _lookup_msg(s_injected_alerts[i], 0);
        const char *fly_msg = _lookup_msg(s_injected_alerts[i], 1);
        offset += snprintf(buf + offset, buflen - offset,
            "{\"code\":\"0x%08X\",\"component\":0,\"level\":%d,"
            "\"ground_msg\":\"%s\",\"fly_msg\":\"%s\",\"source\":\"injected\"}",
            s_injected_alerts[i], s_injected_levels[i],
            ground_msg, fly_msg);
        alert_idx++;
    }

    offset += snprintf(buf + offset, buflen - offset, "]}");
    return 0;
}

/* Inject a custom HMS error code (for testing). */
int hms_inject_error(uint32_t error_code, uint8_t error_level) {
    if (s_injected_count >= MAX_ALERTS) {
        printf("[hms] inject failed: max alerts reached\n");
        return -1;
    }
    /* Check for duplicate */
    for (int i = 0; i < s_injected_count; i++) {
        if (s_injected_alerts[i] == error_code) {
            printf("[hms] duplicate inject ignored: 0x%08X\n", error_code);
            return -1;
        }
    }
    s_injected_alerts[s_injected_count] = error_code;
    s_injected_levels[s_injected_count] = error_level;
    s_injected_count++;
    printf("[hms] injected alert: code=0x%08X level=%d\n", error_code, error_level);
    return 0;
}

/* Eliminate a previously injected HMS error code. */
int hms_eliminate_error(uint32_t error_code) {
    for (int i = 0; i < s_injected_count; i++) {
        if (s_injected_alerts[i] == error_code) {
            /* Shift remaining entries down */
            for (int j = i; j < s_injected_count - 1; j++) {
                s_injected_alerts[j] = s_injected_alerts[j + 1];
                s_injected_levels[j] = s_injected_levels[j + 1];
            }
            s_injected_count--;
            printf("[hms] eliminated alert: 0x%08X\n", error_code);
            return 0;
        }
    }
    printf("[hms] eliminate not found: 0x%08X\n", error_code);
    return -1;
}

int hms_init(void) {
    T_DjiReturnCode rc;

    /* ── Strategy 1: Try normal HMS manager (may work on some devices) ── */
    printf("[hms] trying DjiHmsManager_Init()...\n");
    rc = DjiHmsManager_Init();
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[hms] DjiHmsManager_Init succeeded\n");
        rc = DjiHmsManager_RegHmsInfoCallback(_hms_cb);
        if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            s_hms_manager_ready = 1;
            printf("[hms] callback registered\n");
        } else {
            printf("[hms] register callback failed: 0x%08llX (fallback to customization)\n",
                   (unsigned long long)rc);
            DjiHmsManager_DeInit();
        }
    } else {
        printf("[hms] DjiHmsManager_Init failed: 0x%08llX (error=E1 means camera timeout propagated to HMS subscription)\n",
               (unsigned long long)rc);

        /* ── Strategy 2: Fall back to HMS customization ──
         * DjiHmsCustomization does NOT use the same subscription path as
         * DjiHmsManager, so it may succeed even when the camera module is broken.
         */
        printf("[hms] falling back to DjiHmsCustomization_Init...\n");
        rc = DjiHmsCustomization_Init();
        if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            printf("[hms] DjiHmsCustomization_Init succeeded\n");
        } else {
            printf("[hms] DjiHmsCustomization_Init also failed: 0x%08llX\n",
                   (unsigned long long)rc);
            return -1;
        }
    }

    printf("[hms] initialized (manager_ready=%d)\n", s_hms_manager_ready);
    return 0;
}

void hms_cleanup(void) {
    if (s_hms_manager_ready) {
        DjiHmsManager_DeInit();
    }
    DjiHmsCustomization_DeInit();
}

#else /* stub */

int hms_init(void) { printf("[hms] stub mode\n"); return 0; }
int hms_get_info(char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"alerts\":[]}");
    return 0;
}
void hms_cleanup(void) {}

#endif
