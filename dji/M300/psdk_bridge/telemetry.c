#include "telemetry.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>



#ifdef PSDK_ENABLED
#include "dji_fc_subscription.h"
#include "dji_typedef.h"

static T_DjiFcSubscriptionQuaternion s_quaternion;
static T_DjiFcSubscriptionVelocity s_velocity;
static T_DjiFcSubscriptionGpsPosition s_gps_pos;
static T_DjiFcSubscriptionPositionFused s_pos_fused;
static T_DjiFcSubscriptionGpsDetails s_gps_detail;
static T_DjiFcSubscriptionAltitudeFused s_alt_fused;
static T_DjiFcSubscriptionAltitudeOfHomePoint s_alt_home;
static T_DjiFcSubscriptionFlightStatus s_flight_status;
static T_DjiFcSubscriptionDisplaymode s_display_mode;
static T_DjiFcSubscriptionWholeBatteryInfo s_battery_whole;
static T_DjiFcSubscriptionSingleBatteryInfo s_battery1;
static T_DjiFcSubscriptionSingleBatteryInfo s_battery2;
static T_DjiFcSubscriptionRC s_rc;
static T_DjiFcSubscriptionCompass s_compass;
static T_DjiFcSubscriptionAvoidData s_avoid;
static int s_battery_whole_valid = 0;
static int s_battery1_valid = 0;
static int s_battery2_valid = 0;

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

static T_DjiReturnCode _gps_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_gps_pos))
        memcpy(&s_gps_pos, data, sizeof(s_gps_pos));
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _pos_fused_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_pos_fused))
        memcpy(&s_pos_fused, data, sizeof(s_pos_fused));
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

static T_DjiReturnCode _battery_whole_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_battery_whole)) {
        memcpy(&s_battery_whole, data, sizeof(s_battery_whole));
        s_battery_whole_valid = 1;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _battery1_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_battery1)) {
        memcpy(&s_battery1, data, sizeof(s_battery1));
        s_battery1_valid = 1;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _battery2_cb(const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *ts) {
    if (size >= sizeof(s_battery2)) {
        memcpy(&s_battery2, data, sizeof(s_battery2));
        s_battery2_valid = 1;
    }
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

    /* M300 GPS_POSITION is limited to 5Hz.  Log every failure instead of
     * claiming telemetry is ready with a partial or invalid subscription. */
#define SUBSCRIBE(topic, frequency, callback) do { \
    rc = DjiFcSubscription_SubscribeTopic((topic), (frequency), (callback)); \
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) \
        printf("[telemetry] subscribe %s failed: 0x%08llX\\n", #topic, (unsigned long long)rc); \
} while (0)
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _quaternion_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _velocity_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, _gps_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _pos_fused_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, _gps_detail_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _alt_fused_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, _alt_home_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _flight_status_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _display_mode_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, _battery_whole_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, _battery1_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, _battery2_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_RC, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _rc_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_COMPASS, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _compass_cb);
    SUBSCRIBE(DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, _avoid_cb);
#undef SUBSCRIBE

    printf("[telemetry] subscriptions initialized\n");
    return 0;
}

static const char *_flight_status_text(int status) {
    switch (status) {
        case 0: return "stop";
        case 1: return "on_ground";
        case 2: return "in_air";
        default: return "unknown";
    }
}

static const char *_display_mode_text(int mode) {
    switch (mode) {
        case 0: return "manual";
        case 1: return "atti";
        case 6: return "p_gps";
        case 9: return "hotpoint";
        case 17: return "go_home";
        case 33: return "auto_takeoff";
        case 40: return "auto_landing";
        case 41: return "forced_landing";
        default: return "unknown";
    }
}

static int s_local_origin_valid = 0;
static double s_local_origin_lat_deg = 0.0;
static double s_local_origin_lon_deg = 0.0;
static int s_down_origin_valid = 0;
static double s_down_origin = 0.0;

int telemetry_get_json(char *buf, size_t buflen) {
    /* Convert quaternion to Euler angles */
    double q0 = s_quaternion.q0, q1 = s_quaternion.q1;
    double q2 = s_quaternion.q2, q3 = s_quaternion.q3;
    double roll  = atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2)) * 180.0 / M_PI;
    double pitch = asin(2.0*(q0*q2 - q3*q1)) * 180.0 / M_PI;
    double yaw   = atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3)) * 180.0 / M_PI;
    double roll_rad = roll * M_PI / 180.0;
    double pitch_rad = pitch * M_PI / 180.0;
    double yaw_rad = yaw * M_PI / 180.0;

    /* Use POSITION_FUSED as primary source (works in simulator),
     * fall back to GPS_POSITION if fused is 0 */
    double lat_deg, lon_deg, gps_alt;
    if (s_pos_fused.latitude != 0 || s_pos_fused.longitude != 0) {
        lat_deg = s_pos_fused.latitude * 180.0 / M_PI;
        lon_deg = s_pos_fused.longitude * 180.0 / M_PI;
        gps_alt = (double)s_pos_fused.altitude;
    } else {
        lat_deg = (double)s_gps_pos.y / 1e7;
        lon_deg = (double)s_gps_pos.x / 1e7;
        gps_alt = (double)s_gps_pos.z / 1000.0;
    }

    int gps_valid = (lat_deg != 0.0 || lon_deg != 0.0) &&
                    (s_gps_detail.fixState > 0 ||
                     s_gps_detail.totalSatelliteNumberUsed > 0 ||
                     s_pos_fused.visibleSatelliteNumber > 0);
    double relative_alt = (double)s_alt_fused - (double)s_alt_home;
    double down_range = (double)s_avoid.down;
    if ((int)s_flight_status != 2 && down_range > 0.0) {
        s_down_origin = down_range;
        s_down_origin_valid = 1;
    }
    double world_z = relative_alt;
    if (down_range > 0.0) {
        world_z = down_range - (s_down_origin_valid ? s_down_origin : 0.0);
        if (world_z < 0.0)
            world_z = 0.0;
    }
    double local_x = 0.0;
    double local_y = 0.0;
    if (gps_valid) {
        if (!s_local_origin_valid) {
            s_local_origin_lat_deg = lat_deg;
            s_local_origin_lon_deg = lon_deg;
            s_local_origin_valid = 1;
        }
        double lat_rad = lat_deg * M_PI / 180.0;
        double origin_lat_rad = s_local_origin_lat_deg * M_PI / 180.0;
        double dlat_rad = (lat_deg - s_local_origin_lat_deg) * M_PI / 180.0;
        double dlon_rad = (lon_deg - s_local_origin_lon_deg) * M_PI / 180.0;
        double mean_lat = (lat_rad + origin_lat_rad) * 0.5;
        local_x = dlon_rad * cos(mean_lat) * 6378137.0;
        local_y = dlat_rad * 6378137.0;
    }

    int battery_valid = s_battery_whole_valid || s_battery1_valid || s_battery2_valid;
    int battery_percent = 0;
    double battery_voltage = 0.0;
    const char *battery_source = "none";
    if (s_battery_whole_valid) {
        battery_percent = (int)s_battery_whole.percentage;
        battery_voltage = (double)s_battery_whole.voltage / 1000.0;
        battery_source = "whole";
    } else if (s_battery1_valid && s_battery2_valid) {
        battery_percent = ((int)s_battery1.batteryCapacityPercent +
                           (int)s_battery2.batteryCapacityPercent) / 2;
        battery_voltage = ((double)s_battery1.currentVoltage +
                           (double)s_battery2.currentVoltage) / 1000.0;
        battery_source = "single_average";
    } else if (s_battery1_valid) {
        battery_percent = (int)s_battery1.batteryCapacityPercent;
        battery_voltage = (double)s_battery1.currentVoltage / 1000.0;
        battery_source = "single_index1";
    } else if (s_battery2_valid) {
        battery_percent = (int)s_battery2.batteryCapacityPercent;
        battery_voltage = (double)s_battery2.currentVoltage / 1000.0;
        battery_source = "single_index2";
    }

    double compass_heading = atan2((double)s_compass.y, (double)s_compass.x) * 180.0 / M_PI;
    if (compass_heading < 0.0)
        compass_heading += 360.0;

    char lat_json[32], lon_json[32];
    char battery_pct_json[16], battery_volt_json[16];
    char b1_pct_json[16], b1_volt_json[16];
    char b2_pct_json[16], b2_volt_json[16];
    if (gps_valid) {
        snprintf(lat_json, sizeof(lat_json), "%.8f", lat_deg);
        snprintf(lon_json, sizeof(lon_json), "%.8f", lon_deg);
    } else {
        snprintf(lat_json, sizeof(lat_json), "null");
        snprintf(lon_json, sizeof(lon_json), "null");
    }
    if (battery_valid) {
        snprintf(battery_pct_json, sizeof(battery_pct_json), "%d", battery_percent);
        snprintf(battery_volt_json, sizeof(battery_volt_json), "%.1f", battery_voltage);
    } else {
        snprintf(battery_pct_json, sizeof(battery_pct_json), "null");
        snprintf(battery_volt_json, sizeof(battery_volt_json), "null");
    }
    if (s_battery1_valid) {
        snprintf(b1_pct_json, sizeof(b1_pct_json), "%d", (int)s_battery1.batteryCapacityPercent);
        snprintf(b1_volt_json, sizeof(b1_volt_json), "%.1f", (double)s_battery1.currentVoltage / 1000.0);
    } else {
        snprintf(b1_pct_json, sizeof(b1_pct_json), "null");
        snprintf(b1_volt_json, sizeof(b1_volt_json), "null");
    }
    if (s_battery2_valid) {
        snprintf(b2_pct_json, sizeof(b2_pct_json), "%d", (int)s_battery2.batteryCapacityPercent);
        snprintf(b2_volt_json, sizeof(b2_volt_json), "%.1f", (double)s_battery2.currentVoltage / 1000.0);
    } else {
        snprintf(b2_pct_json, sizeof(b2_pct_json), "null");
        snprintf(b2_volt_json, sizeof(b2_volt_json), "null");
    }

    snprintf(buf, buflen,
        "{"
        "\"position\":{\"latitude\":%s,\"longitude\":%s,\"valid\":%s,"
        "\"altitude\":%.2f,\"altitude_gps\":%.2f,\"altitude_fused\":%.2f,"
        "\"home_altitude\":%.2f,\"relative_altitude\":%.2f,"
        "\"world_x\":%.3f,\"world_y\":%.3f,\"world_z\":%.3f},"
        "\"attitude\":{\"quaternion\":[%.4f,%.4f,%.4f,%.4f],"
        "\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,"
        "\"yaw_rad\":%.4f,\"pitch_rad\":%.4f,\"roll_rad\":%.4f},"
        "\"velocity\":{\"vx\":%.3f,\"vy\":%.3f,\"vz\":%.3f},"
        "\"battery\":{\"percent\":%s,\"voltage\":%s,\"valid\":%s,\"source\":\"%s\","
        "\"battery1\":{\"percent\":%s,\"voltage\":%s,\"valid\":%s},"
        "\"battery2\":{\"percent\":%s,\"voltage\":%s,\"valid\":%s}},"
        "\"gps\":{\"satellites\":%d,\"gps_used\":%d,\"glonass_used\":%d,\"fix_type\":%d,\"valid\":%s},"
        "\"compass\":{\"heading\":%.1f,\"raw\":{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f}},"
        "\"obstacles\":{\"front\":%.1f,\"back\":%.1f,\"left\":%.1f,"
        "\"right\":%.1f,\"up\":%.1f,\"down\":%.1f},"
        "\"rc\":{\"left_stick_x\":%d,\"left_stick_y\":%d,"
        "\"right_stick_x\":%d,\"right_stick_y\":%d},"
        "\"flight_status\":%d,\"flight_status_text\":\"%s\","
        "\"flight_mode\":%d,\"flight_mode_text\":\"%s\""
        "}",
        /* GPS_POSITION: x=Longitude, y=Latitude, z=Altitude(mm) — per PSDK docs */
        lat_json, lon_json,
        gps_valid ? "true" : "false",
        gps_alt, gps_alt, (double)s_alt_fused, (double)s_alt_home, relative_alt,
        local_y, local_x, world_z,
        q0, q1, q2, q3, yaw, pitch, roll, yaw_rad, pitch_rad, roll_rad,
        (double)s_velocity.data.x, (double)s_velocity.data.y, (double)s_velocity.data.z,
        battery_pct_json, battery_volt_json,
        battery_valid ? "true" : "false", battery_source,
        b1_pct_json, b1_volt_json,
        s_battery1_valid ? "true" : "false",
        b2_pct_json, b2_volt_json,
        s_battery2_valid ? "true" : "false",
        (int)s_gps_detail.totalSatelliteNumberUsed,
        (int)s_gps_detail.gpsSatelliteNumberUsed,
        (int)s_gps_detail.glonassSatelliteNumberUsed,
        (int)s_gps_detail.fixState,
        gps_valid ? "true" : "false",
        compass_heading,
        (double)s_compass.x, (double)s_compass.y, (double)s_compass.z,
        (double)s_avoid.front, (double)s_avoid.back,
        (double)s_avoid.left, (double)s_avoid.right,
        (double)s_avoid.up, (double)s_avoid.down,
        s_rc.roll, s_rc.pitch,
        s_rc.yaw, s_rc.throttle,
        (int)s_flight_status, _flight_status_text((int)s_flight_status),
        (int)s_display_mode, _display_mode_text((int)s_display_mode)
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
        "{\"position\":{\"latitude\":39.9042,\"longitude\":116.4074,\"valid\":true,"
        "\"altitude\":0,\"altitude_gps\":0,\"altitude_fused\":0,"
        "\"home_altitude\":0,\"relative_altitude\":0,"
        "\"world_x\":0,\"world_y\":0,\"world_z\":0},"
        "\"attitude\":{\"quaternion\":[1,0,0,0],\"yaw\":0,\"pitch\":0,\"roll\":0,"
        "\"yaw_rad\":0,\"pitch_rad\":0,\"roll_rad\":0},"
        "\"velocity\":{\"vx\":0,\"vy\":0,\"vz\":0},"
        "\"battery\":{\"percent\":85,\"voltage\":22.8,\"valid\":true,\"source\":\"stub\","
        "\"battery1\":{\"percent\":85,\"voltage\":22.8,\"valid\":true},"
        "\"battery2\":{\"percent\":85,\"voltage\":22.8,\"valid\":true}},"
        "\"gps\":{\"satellites\":18,\"gps_used\":18,\"glonass_used\":0,\"fix_type\":5,\"valid\":true},"
        "\"compass\":{\"heading\":0,\"raw\":{\"x\":1,\"y\":0,\"z\":0}},"
        "\"obstacles\":{\"front\":10,\"back\":10,\"left\":10,\"right\":10,\"up\":10,\"down\":0},"
        "\"rc\":{\"left_stick_x\":0,\"left_stick_y\":0,\"right_stick_x\":0,\"right_stick_y\":0},"
        "\"flight_status\":0,\"flight_status_text\":\"stop\","
        "\"flight_mode\":0,\"flight_mode_text\":\"manual\"}");
    return 0;
}

int telemetry_get_gps_satellite_count(void) { return 0; }
int telemetry_get_display_mode(void) { return 0; }
float telemetry_get_altitude(void) { return 0; }
int telemetry_get_rc_stick_max(void) { return 0; }
void telemetry_cleanup(void) {}

#endif
