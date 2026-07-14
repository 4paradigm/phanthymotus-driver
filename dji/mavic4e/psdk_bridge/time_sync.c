#define _GNU_SOURCE
#include "time_sync.h"
#include "telemetry.h"
#include "error_code.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

/*
 * Time Sync for Mavic 3T — uses FC subscription GPS_DATE + GPS_TIME.
 *
 * DjiTimeSync requires hardware PPS which is not available on Jetson Nano
 * E-Port dev board. Instead, we subscribe to GPS date/time topics from FC
 * and use settimeofday() to sync the local clock.
 */

#ifdef PSDK_ENABLED
#include "dji_fc_subscription.h"

static uint32_t s_gps_date = 0;  /* yyyymmdd */
static uint32_t s_gps_time = 0;  /* hhmmss */

static T_DjiReturnCode _gps_date_cb(const uint8_t *data, uint16_t dataSize,
                                     const T_DjiDataTimestamp *timestamp) {
    (void)dataSize; (void)timestamp;
    s_gps_date = *(const uint32_t *)data;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _gps_time_cb(const uint8_t *data, uint16_t dataSize,
                                     const T_DjiDataTimestamp *timestamp) {
    (void)dataSize; (void)timestamp;
    s_gps_time = *(const uint32_t *)data;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

int time_sync_init(void) {
    T_DjiReturnCode rc;
    rc = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE,
                                          DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, _gps_date_cb);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[time_sync] subscribe GPS_DATE failed: 0x%08llX\n", (unsigned long long)rc);
    }
    rc = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME,
                                          DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, _gps_time_cb);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[time_sync] subscribe GPS_TIME failed: 0x%08llX\n", (unsigned long long)rc);
    }
    printf("[time_sync] initialized (FC subscription GPS date/time)\n");
    return 0;
}

int time_sync_get_aircraft_time(char *buf, size_t buflen) {
    if (s_gps_date == 0 || s_gps_time == 0) {
        snprintf(buf, buflen, "{\"error\":\"GPS time not yet available\",\"recovery\":\"Wait for GPS lock\"}");
        return -1;
    }
    int year = s_gps_date / 10000;
    int month = (s_gps_date / 100) % 100;
    int day = s_gps_date % 100;
    int hour = s_gps_time / 10000;
    int minute = (s_gps_time / 100) % 100;
    int second = s_gps_time % 100;

    snprintf(buf, buflen,
        "{\"time\":\"%04d-%02d-%02dT%02d:%02d:%02dZ\","
        "\"year\":%d,\"month\":%d,\"day\":%d,"
        "\"hour\":%d,\"minute\":%d,\"second\":%d}",
        year, month, day, hour, minute, second,
        year, month, day, hour, minute, second);
    return 0;
}

int time_sync_sync_clock(char *buf, size_t buflen) {
    if (s_gps_date == 0 || s_gps_time == 0) {
        snprintf(buf, buflen, "{\"error\":\"GPS time not yet available\",\"recovery\":\"Wait for GPS lock\"}");
        return -1;
    }

    int sats = telemetry_get_gps_satellite_count();
    if (sats < 4) {
        snprintf(buf, buflen, "{\"error\":\"GPS signal too weak (%d satellites, need >= 4)\","
                 "\"recovery\":\"Move to open area with clear sky view\"}", sats);
        return -1;
    }

    int year = s_gps_date / 10000;
    int month = (s_gps_date / 100) % 100;
    int day = s_gps_date % 100;
    int hour = s_gps_time / 10000;
    int minute = (s_gps_time / 100) % 100;
    int second = s_gps_time % 100;

    if (year < 2024) {
        snprintf(buf, buflen, "{\"error\":\"GPS time stale (cached from %04d-%02d-%02d, not current)\","
                 "\"recovery\":\"Wait for GPS satellite lock (需要 GPS 信号锁定后才能对时)\"}", year, month, day);
        return -1;
    }

    struct tm tm = {0};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = minute;
    tm.tm_sec = second;
    time_t epoch = timegm(&tm);

    struct timeval new_tv = { .tv_sec = epoch, .tv_usec = 0 };
    if (settimeofday(&new_tv, NULL) != 0) {
        snprintf(buf, buflen, "{\"error\":\"Failed to set system clock\",\"recovery\":\"Check container privileges (needs privileged or SYS_TIME)\"}");
        return -1;
    }

    printf("[time_sync] clock synced to %04d-%02d-%02dT%02d:%02d:%02dZ\n",
           year, month, day, hour, minute, second);
    snprintf(buf, buflen, "{\"synced\":\"%04d-%02d-%02dT%02d:%02d:%02dZ\"}",
             year, month, day, hour, minute, second);
    return 0;
}

void time_sync_cleanup(void) {}

#else /* stub */

int time_sync_init(void) { printf("[time_sync] stub mode\n"); return 0; }
int time_sync_get_aircraft_time(char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"error\":\"stub\"}");
    return -1;
}
int time_sync_sync_clock(char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"error\":\"stub\"}");
    return -1;
}
void time_sync_cleanup(void) {}

#endif
