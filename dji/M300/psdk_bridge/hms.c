#include "hms.h"
#include <stdio.h>
#include <string.h>
#include "cJSON.h"



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

/* ── JSON-based error code lookup ─────────────────────────────────────── */

static cJSON *s_hms_json_root = NULL;

static const char *HMS_JSON_SEARCH_PATHS[] = {
    "data/hms_2023_08_22.json",
    "/opt/dji/M300/psdk_bridge/data/hms_2023_08_22.json",
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
        printf("[hms] JSON database not found, using hmsErrCodeInfoTbl only\n");
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
        printf("[hms] JSON parse failed\n");
        return -1;
    }

    printf("[hms] JSON database loaded (%ld bytes)\n", file_size);
    return 0;
}

static const char *_lookup_msg_json(uint32_t code, int is_flying) {
    if (!s_hms_json_root) return NULL;

    char key[64];
    if (is_flying) {
        snprintf(key, sizeof(key), "fpv_tip_0x%08X_in_the_sky", code);
    } else {
        snprintf(key, sizeof(key), "fpv_tip_0x%08X", code);
    }

    cJSON *error_obj = cJSON_GetObjectItem(s_hms_json_root, key);
    if (!error_obj && is_flying) {
        snprintf(key, sizeof(key), "fpv_tip_0x%08X", code);
        error_obj = cJSON_GetObjectItem(s_hms_json_root, key);
    }
    if (!error_obj) return NULL;

    cJSON *en_msg = cJSON_GetObjectItem(error_obj, "en");
    if (!en_msg || !en_msg->valuestring) return NULL;
    return en_msg->valuestring;
}

/* ── Error code → human-readable message ─────────────────────────────── */

static char s_unknown_buf[128];

static const char *_lookup_msg(uint32_t code, int is_flying) {
    /* Tier 1: JSON database (~1000+ codes) */
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

    /* Tier 3: Category hint from error code prefix */
    uint8_t module_id = (code >> 24) & 0xFF;
    const char *category;
    switch (module_id) {
        case 0x16: category = "Flight control"; break;
        case 0x17: category = "Battery"; break;
        case 0x18: category = "Remote controller"; break;
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

/* ── Public API ──────────────────────────────────────────────────────── */

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

int hms_init(void) {
    _load_hms_json();

    printf("[hms] trying DjiHmsManager_Init()...\n");
    T_DjiReturnCode rc = DjiHmsManager_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[hms] DjiHmsManager_Init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }

    rc = DjiHmsManager_RegHmsInfoCallback(_hms_cb);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[hms] register callback failed: 0x%08llX\n", (unsigned long long)rc);
        DjiHmsManager_DeInit();
        return -1;
    }

    printf("[hms] initialized, callback registered\n");
    return 0;
}

void hms_cleanup(void) {
    DjiHmsManager_DeInit();
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
