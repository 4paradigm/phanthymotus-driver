#include "telemetry.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*
 * PSDK Data Subscription for Mavic 3E telemetry.
 *
 * Subscribes to:
 *   - DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION (50Hz)
 *   - DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY (50Hz)
 *   - DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION (50Hz)
 *   - DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS
 *   - DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED
 *   - DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT
 *   - DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT
 *   - DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE
 *   - DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1
 *   - DJI_FC_SUBSCRIPTION_TOPIC_RC
 *   - DJI_FC_SUBSCRIPTION_TOPIC_COMPASS
 *   - DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA
 *
 * Note: Mavic 3E does NOT support:
 *   - POSITION_FUSED, HEIGHT_RELATIVE, GIMBAL_STATUS, HARD_SYNC
 */

#ifdef PSDK_ENABLED
#include "dji_fc_subscription.h"
#include "dji_typedef.h"

static T_DjiFcSubscriptionQuaternion s_quaternion;
static T_DjiFcSubscriptionVelocity s_velocity;
static T_DjiFcSubscriptionGpsPosition s_gps_pos;
static T_DjiFcSubscriptionGpsDetails s_gps_detail;
static T_DjiFcSubscriptionAltitudeFused s_alt_fused;
static T_DjiFcSubscriptionAltitudeOfHomePoint s_alt_home;
static T_DjiFcSubscriptionFlightStatus s_flight_status;
static T_DjiFcSubscriptionDisplaymode s_display_mode;
static T_DjiFcSubscriptionSingleBatteryInfo s_battery;
static T_DjiFcSubscriptionRC s_rc;
static T_DjiFcSubscriptionCompass s_compass;
static T_DjiFcSubscriptionAvoidData s_avoid;

static T_DjiReturnCode _quaternion_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_quaternion))
        memcpy(&s_quaternion, data, sizeof(s_quaternion));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _velocity_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_velocity))
        memcpy(&s_velocity, data, sizeof(s_velocity));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static int s_gps_log_count = 0;
static T_DjiReturnCode _gps_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_gps_pos))
        memcpy(&s_gps_pos, data, sizeof(s_gps_pos));
    if (s_gps_log_count++ % 100 == 0)
        printf("[telemetry] GPS raw: x=%d y=%d z=%d (lon=%.7f lat=%.7f alt=%.1fm)\n",
               s_gps_pos.x, s_gps_pos.y, s_gps_pos.z,
               (double)s_gps_pos.x / 1e7, (double)s_gps_pos.y / 1e7,
               (double)s_gps_pos.z / 1000.0);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _gps_detail_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_gps_detail))
        memcpy(&s_gps_detail, data, sizeof(s_gps_detail));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _alt_fused_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_alt_fused))
        memcpy(&s_alt_fused, data, sizeof(s_alt_fused));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _alt_home_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_alt_home))
        memcpy(&s_alt_home, data, sizeof(s_alt_home));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _flight_status_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_flight_status))
        memcpy(&s_flight_status, data, sizeof(s_flight_status));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _display_mode_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_display_mode))
        memcpy(&s_display_mode, data, sizeof(s_display_mode));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _battery_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_battery))
        memcpy(&s_battery, data, sizeof(s_battery));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _rc_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_rc))
        memcpy(&s_rc, data, sizeof(s_rc));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _compass_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_compass))
        memcpy(&s_compass, data, sizeof(s_compass));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _avoid_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_avoid))
        memcpy(&s_avoid, data, sizeof(s_avoid));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

int telemetry_init(void) {
    T_DjiReturnCode rc;

    rc = DjiFcSubscription_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[telemetry] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }

    /* Subscribe at 10Hz (suitable topics) */
    rc = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _quaternion_cb);
    printf("[telemetry] subscribe QUATERNION → 0x%08llX\n", (unsigned long long)rc);
    rc = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _velocity_cb);
    printf("[telemetry] subscribe VELOCITY → 0x%08llX\n", (unsigned long long)rc);
    rc = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _gps_cb);
    printf("[telemetry] subscribe GPS_POSITION → 0x%08llX\n", (unsigned long long)rc);
    rc = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, _gps_detail_cb);
    printf("[telemetry] subscribe GPS_DETAILS → 0x%08llX\n", (unsigned long long)rc);
    rc = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _alt_fused_cb);
    printf("[telemetry] subscribe ALTITUDE_FUSED → 0x%08llX\n", (unsigned long long)rc);
    rc = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, _alt_home_cb);
    printf("[telemetry] subscribe ALTITUDE_HOME → 0x%08llX\n", (unsigned long long)rc);
    DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _flight_status_cb);
    DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _display_mode_cb);
    DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, _battery_cb);
    DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _rc_cb);
    DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_COMPASS, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _compass_cb);
    DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _avoid_cb);

    printf("[telemetry] subscriptions initialized\n");
    return 0;
}

int telemetry_get_json(char *buf, size_t buflen) {
    /* Convert quaternion to Euler angles */
    double q0 = s_quaternion.q0, q1 = s_quaternion.q1;
    double q2 = s_quaternion.q2, q3 = s_quaternion.q3;
    double roll  = atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2)) * 180.0 / M_PI;
    double pitch = asin(2.0*(q0*q2 - q3*q1)) * 180.0 / M_PI;
    double yaw   = atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3)) * 180.0 / M_PI;

    snprintf(buf, buflen,
        "{"
        "\"position\":{\"latitude\":%.8f,\"longitude\":%.8f,\"altitude\":%.2f,"
        "\"altitude_fused\":%.2f,\"home_altitude\":%.2f,\"relative_height\":%.2f},"
        "\"attitude\":{\"quaternion\":[%.4f,%.4f,%.4f,%.4f],"
        "\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f},"
        "\"velocity\":{\"vx\":%.3f,\"vy\":%.3f,\"vz\":%.3f},"
        "\"battery\":{\"percent\":%d,\"voltage\":%.1f},"
        "\"gps\":{\"satellites\":%d,\"gps_used\":%d,\"glonass_used\":%d,\"fix_type\":%d},"
        "\"compass\":{\"heading\":%.1f},"
        "\"obstacles\":{\"front\":%.1f,\"back\":%.1f,\"left\":%.1f,"
        "\"right\":%.1f,\"up\":%.1f,\"down\":%.1f},"
        "\"rc\":{\"left_stick_x\":%d,\"left_stick_y\":%d,"
        "\"right_stick_x\":%d,\"right_stick_y\":%d},"
        "\"flight_status\":%d,\"flight_mode\":%d"
        "}",
        /* GPS_POSITION: x=Longitude, y=Latitude, z=Altitude(mm) — per PSDK docs */
        (double)s_gps_pos.y / 1e7, (double)s_gps_pos.x / 1e7,
        (double)s_gps_pos.z / 1000.0,
        (double)s_alt_fused, (double)s_alt_home,
        (double)(s_alt_fused - s_alt_home),
        q0, q1, q2, q3, yaw, pitch, roll,
        (double)s_velocity.data.x, (double)s_velocity.data.y, (double)s_velocity.data.z,
        (int)s_battery.batteryCapacityPercent, (double)s_battery.currentVoltage / 1000.0,
        (int)s_gps_detail.totalSatelliteNumberUsed,
        (int)s_gps_detail.gpsSatelliteNumberUsed,
        (int)s_gps_detail.glonassSatelliteNumberUsed,
        (int)s_gps_detail.fixState,
        (double)s_compass.x,
        (double)s_avoid.front, (double)s_avoid.back,
        (double)s_avoid.left, (double)s_avoid.right,
        (double)s_avoid.up, (double)s_avoid.down,
        s_rc.roll, s_rc.pitch,
        s_rc.yaw, s_rc.throttle,
        (int)s_flight_status, (int)s_display_mode
    );
    return 0;
}

int telemetry_get_gps_satellite_count(void) {
    return (int)s_gps_detail.totalSatelliteNumberUsed;
}

int telemetry_get_display_mode(void) {
    return (int)s_display_mode;
}

float telemetry_get_altitude(void) {
    return (float)(s_alt_fused - s_alt_home);
}

int telemetry_get_rc_stick_max(void) {
    int vals[4] = {
        abs(s_rc.roll), abs(s_rc.pitch),
        abs(s_rc.yaw), abs(s_rc.throttle)
    };
    int mx = 0;
    for (int i = 0; i < 4; i++) {
        if (vals[i] > mx) mx = vals[i];
    }
    return mx;
}

void telemetry_cleanup(void) {
    DjiFcSubscription_DeInit();
    printf("[telemetry] cleaned up\n");
}

#else /* !PSDK_ENABLED — stub for build without PSDK */

int telemetry_init(void) {
    printf("[telemetry] stub mode (no PSDK)\n");
    return 0;
}

int telemetry_get_json(char *buf, size_t buflen) {
    snprintf(buf, buflen,
        "{\"position\":{\"latitude\":39.9042,\"longitude\":116.4074,\"altitude\":0},"
        "\"attitude\":{\"quaternion\":[1,0,0,0],\"yaw\":0,\"pitch\":0,\"roll\":0},"
        "\"velocity\":{\"vx\":0,\"vy\":0,\"vz\":0},"
        "\"battery\":{\"percent\":85,\"voltage\":22.8},"
        "\"gps\":{\"satellites\":18,\"fix_type\":5},"
        "\"compass\":{\"heading\":0},"
        "\"obstacles\":{\"front\":10,\"back\":10,\"left\":10,\"right\":10,\"up\":10,\"down\":0},"
        "\"rc\":{\"left_stick_x\":0,\"left_stick_y\":0,\"right_stick_x\":0,\"right_stick_y\":0},"
        "\"flight_status\":0,\"flight_mode\":0}");
    return 0;
}

int telemetry_get_gps_satellite_count(void) { return 0; }
int telemetry_get_display_mode(void) { return 0; }
float telemetry_get_altitude(void) { return 0; }
int telemetry_get_rc_stick_max(void) { return 0; }
void telemetry_cleanup(void) {}

#endif
