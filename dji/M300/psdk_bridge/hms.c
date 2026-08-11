#include "hms.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "cJSON.h"

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
 * Error code lookup (two-tier):
 *   1. Primary: hms_2023_08_22.json (DJI's latest error code database, ~1.7MB).
 *      Contains 1000+ error codes vs ~150 in the compiled-in hmsErrCodeInfoTbl.
 *   2. Fallback: hmsErrCodeInfoTbl (compiled into PSDK library).
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
static uint32_t s_injected_alerts[MAX_ALERTS];
static uint8_t  s_injected_levels[MAX_ALERTS];
static int      s_injected_count = 0;

/* ── JSON-based error code lookup (hzhy demo approach) ────────────────── */
/*
 * The compiled-in hmsErrCodeInfoTbl only contains ~150 error codes.
 * DJI publishes a much more comprehensive JSON database (~1000+ codes)
 * that is updated regularly.  We load it at startup and search it first,
 * falling back to hmsErrCodeInfoTbl only when the JSON is unavailable
 * or the code is not found.
 *
 * JSON key format:
 *   "fpv_tip_0x%08X"             — ground message
 *   "fpv_tip_0x%08X_in_the_sky"  — in-air message
 * Each key maps to an object with language keys: "en", "zh", etc.
 */
static cJSON *s_hms_json_root = NULL;

/* Search paths for the HMS JSON database, tried in order */
static const char *HMS_JSON_SEARCH_PATHS[] = {
    "data/hms_2023_08_22.json",                         /* relative to CWD */
    "/opt/dji/M300/psdk_bridge/data/hms_2023_08_22.json",  /* deployed path */
    NULL
};

static int _load_hms_json(void) {
    FILE *fp = NULL;
    long file_size;
    char *file_buf = NULL;

    for (int i = 0; HMS_JSON_SEARCH_PATHS[i] != NULL; i++) {
        fp = fopen(HMS_JSON_SEARCH_PATHS[i], "rb");
        if (fp) {
            printf("[hms] loading JSON database: %s\n", HMS_JSON_SEARCH_PATHS[i]);
            break;
        }
    }
    if (!fp) {
        printf("[hms] JSON database not found, falling back to hmsErrCodeInfoTbl only\n");
        return -1;
    }

    fseek(fp, 0, SEEK_END);
    file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    file_buf = (char *)malloc(file_size + 1);
    if (!file_buf) {
        printf("[hms] failed to allocate %ld bytes for JSON\n", file_size);
        fclose(fp);
        return -1;
    }
    fread(file_buf, 1, file_size, fp);
    file_buf[file_size] = '\0';
    fclose(fp);

    s_hms_json_root = cJSON_Parse(file_buf);
    free(file_buf);

    if (!s_hms_json_root) {
        printf("[hms] JSON parse failed: %s\n",
               cJSON_GetErrorPtr() ? cJSON_GetErrorPtr() : "unknown error");
        return -1;
    }

    printf("[hms] JSON database loaded (%ld bytes)\n", file_size);
    return 0;
}

/* Lookup error code in the JSON database.
 * Returns the message string (English), or NULL if not found.
 * The returned pointer is valid as long as s_hms_json_root is alive. */
static const char *_lookup_msg_json(uint32_t code, int is_flying) {
    if (!s_hms_json_root) return NULL;

    char key[64];
    if (is_flying) {
        snprintf(key, sizeof(key), "fpv_tip_0x%08X_in_the_sky", code);
    } else {
        snprintf(key, sizeof(key), "fpv_tip_0x%08X", code);
    }

    cJSON *error_obj = cJSON_GetObjectItem(s_hms_json_root, key);
    if (!error_obj) {
        /* If in_the_sky key not found, try the ground key as fallback */
        if (is_flying) {
            snprintf(key, sizeof(key), "fpv_tip_0x%08X", code);
            error_obj = cJSON_GetObjectItem(s_hms_json_root, key);
        }
        if (!error_obj) return NULL;
    }

    cJSON *en_msg = cJSON_GetObjectItem(error_obj, "en");
    if (!en_msg || !en_msg->valuestring) return NULL;

    return en_msg->valuestring;
}

/* ── Error code → human-readable message ─────────────────────────────── */

static char s_unknown_buf[128];

/* Lookup error code: try JSON first, then fall back to hmsErrCodeInfoTbl. */
static const char *_lookup_msg(uint32_t code, int is_flying) {
    /* Tier 1: JSON database (comprehensive, ~1000+ codes) */
    const char *json_msg = _lookup_msg_json(code, is_flying);
    if (json_msg) return json_msg;

    /* Tier 2: Compiled-in hmsErrCodeInfoTbl (~150 codes) */
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

    /* Tier 3: Give a category-level hint based on the error code prefix */
    uint8_t module_id = (code >> 24) & 0xFF;
    const char *category;
    switch (module_id) {
        case 0x16: category = "Flight control"; break;
        case 0x17: category = "Battery"; break;
        case 0x18: category = "Remote controller / Transmission"; break;
        case 0x19: category = "Avionics system"; break;
        case 0x1A: category = "Payload"; break;
        case 0x1B: category = "RTK"; break;
        case 0x1C: category = "Radar"; break;
        case 0x1D: category = "Vision system"; break;
        case 0x1E: category = "PSDK custom"; break;
        default:   category = "Unknown module"; break;
    }
    snprintf(s_unknown_buf, sizeof(s_unknown_buf),
             "Unknown error 0x%08X (%s)", code, category);
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

    /* Load JSON error code database (best-effort; fallback to built-in table) */
    _load_hms_json();

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

    printf("[hms] initialized (manager_ready=%d, json_loaded=%d)\n",
           s_hms_manager_ready, s_hms_json_root != NULL);
    return 0;
}

void hms_cleanup(void) {
    if (s_hms_manager_ready) {
        DjiHmsManager_DeInit();
    }
    DjiHmsCustomization_DeInit();
    if (s_hms_json_root) {
        cJSON_Delete(s_hms_json_root);
        s_hms_json_root = NULL;
    }
}

#else /* stub */

int hms_init(void) { printf("[hms] stub mode\n"); return 0; }
int hms_get_info(char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"alerts\":[]}");
    return 0;
}
void hms_cleanup(void) {}

#endif
