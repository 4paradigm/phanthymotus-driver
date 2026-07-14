#include "time_sync.h"
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

/*
 * PSDK Time Synchronization for Mavic 3T.
 *
 * Uses PPS callback (simplified: system clock as fallback) to sync
 * local time with aircraft GPS time.
 */

#ifdef PSDK_ENABLED
#include "dji_time_sync.h"
#include "dji_platform.h"

static uint64_t s_pps_local_time_us = 0;

/* PPS trigger callback — records local time when PPS edge detected.
 * On Jetson Nano without hardware PPS GPIO, we use system clock. */
static T_DjiReturnCode _get_newest_pps_trigger_time(uint64_t *localTimeUs) {
    if (s_pps_local_time_us == 0) {
        /* No hardware PPS — use current system time as approximation */
        struct timeval tv;
        gettimeofday(&tv, NULL);
        *localTimeUs = (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;
    } else {
        *localTimeUs = s_pps_local_time_us;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

int time_sync_init(void) {
    T_DjiReturnCode rc = DjiTimeSync_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[time_sync] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }

    rc = DjiTimeSync_RegGetNewestPpsTriggerTimeCallback(_get_newest_pps_trigger_time);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[time_sync] register PPS callback failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }

    printf("[time_sync] initialized (software PPS fallback)\n");
    return 0;
}

int time_sync_get_aircraft_time(char *buf, size_t buflen) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t localTimeUs = (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;

    T_DjiTimeSyncAircraftTime aircraftTime = {0};
    T_DjiReturnCode rc = DjiTimeSync_TransferToAircraftTime(localTimeUs, &aircraftTime);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        snprintf(buf, buflen, "{\"error\":\"transfer_failed\",\"code\":\"0x%08llX\"}", (unsigned long long)rc);
        return -1;
    }

    snprintf(buf, buflen,
        "{\"year\":%d,\"month\":%d,\"day\":%d,"
        "\"hour\":%d,\"minute\":%d,\"second\":%d,\"microsecond\":%u}",
        aircraftTime.year, aircraftTime.month, aircraftTime.day,
        aircraftTime.hour, aircraftTime.minute, aircraftTime.second,
        aircraftTime.microsecond);
    return 0;
}

void time_sync_cleanup(void) {}

#else /* stub */

int time_sync_init(void) { printf("[time_sync] stub mode\n"); return 0; }
int time_sync_get_aircraft_time(char *buf, size_t buflen) {
    snprintf(buf, buflen, "{\"error\":\"stub\"}");
    return -1;
}
void time_sync_cleanup(void) {}

#endif
