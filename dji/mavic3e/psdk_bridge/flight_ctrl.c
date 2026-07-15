#include "flight_ctrl.h"
#include "error_code.h"
#include "telemetry.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

/*
 * PSDK Flight Controller for Mavic 3E/3T.
 *
 * All functions return 0 on success or the raw PSDK T_DjiReturnCode on failure.
 * move uses a background thread to send joystick commands at 50Hz.
 */

#ifdef PSDK_ENABLED
#include "dji_flight_controller.h"

static int s_has_authority = 0;

/* ── Continuous joystick control ──────────────────────────────────── */
static pthread_t s_move_thread;
static volatile int s_move_active = 0;
static float s_move_vx = 0, s_move_vy = 0, s_move_vz = 0, s_move_vyaw = 0;
static float s_move_duration = -1;  /* -1 = indefinite */
static pthread_mutex_t s_move_mutex = PTHREAD_MUTEX_INITIALIZER;

static void *_move_loop(void *arg) {
    (void)arg;
    T_DjiFlightControllerJoystickMode mode = {
        .horizontalControlMode = DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        .verticalControlMode = DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        .yawControlMode = DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        .horizontalCoordinate = DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        .stableControlMode = DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    DjiFlightController_SetJoystickMode(mode);

    int tick = 0;
    while (s_move_active) {
        pthread_mutex_lock(&s_move_mutex);
        T_DjiFlightControllerJoystickCommand cmd = {
            .x = s_move_vx, .y = s_move_vy, .z = s_move_vz, .yaw = s_move_vyaw,
        };
        float dur = s_move_duration;
        pthread_mutex_unlock(&s_move_mutex);

        DjiFlightController_ExecuteJoystickAction(cmd);
        usleep(20000);  /* 50Hz */
        tick++;

        /* Check duration limit */
        if (dur > 0 && (tick * 0.02f) >= dur) {
            printf("[flight] move duration %.1fs reached, hovering\n", dur);
            s_move_active = 0;
            /* Don't brake — just stop sending commands, aircraft will hover.
             * EmergencyBrake releases joystick authority, breaking future moves. */
            break;
        }
    }
    return NULL;
}

/* ── Init / Authority ─────────────────────────────────────────────── */

int flight_ctrl_init(void) {
    T_DjiFlightControllerRidInfo ridInfo = {0};
    T_DjiReturnCode rc = DjiFlightController_Init(ridInfo);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[flight] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }
    printf("[flight] initialized\n");
    return 0;
}

int64_t flight_ctrl_obtain_authority(void) {
    T_DjiReturnCode rc = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_has_authority = 1;
        return 0;
    }
    return (int64_t)rc;
}

int64_t flight_ctrl_release_authority(void) {
    T_DjiReturnCode rc = DjiFlightController_ReleaseJoystickCtrlAuthority();
    s_has_authority = 0;
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

/* ── Takeoff / Landing ────────────────────────────────────────────── */

int64_t flight_ctrl_takeoff(void) {
    T_DjiReturnCode rc = DjiFlightController_StartTakeoff();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

int64_t flight_ctrl_land(void) {
    T_DjiReturnCode rc = DjiFlightController_StartLanding();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

int64_t flight_ctrl_confirm_landing(void) {
    T_DjiReturnCode rc = DjiFlightController_StartConfirmLanding();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

int64_t flight_ctrl_land_auto_confirm(void) {
    T_DjiReturnCode rc = DjiFlightController_StartLanding();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return (int64_t)rc;

    /* Poll until aircraft reaches low altitude, then confirm */
    for (int i = 0; i < 300; i++) {  /* 30s timeout */
        usleep(100000);
        int mode = telemetry_get_display_mode();
        float alt = telemetry_get_altitude();
        if ((mode == 12 || mode == 33) && alt < 0.6f) {
            usleep(1000000);  /* wait 1s for stabilization */
            printf("[flight] auto-confirm landing at alt=%.2fm\n", alt);
            rc = DjiFlightController_StartConfirmLanding();
            return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
        }
        if (mode != 12 && mode != 33 && i > 20) {
            printf("[flight] landing cancelled (mode=%d)\n", mode);
            return 0;
        }
    }
    printf("[flight] auto-confirm timeout\n");
    rc = DjiFlightController_StartConfirmLanding();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

/* ── Navigation ───────────────────────────────────────────────────── */

int64_t flight_ctrl_go_home(void) {
    T_DjiReturnCode rc = DjiFlightController_StartGoHome();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

int64_t flight_ctrl_cancel_go_home(void) {
    T_DjiReturnCode rc = DjiFlightController_CancelGoHome();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

/* ── Joystick move (continuous) ───────────────────────────────────── */

int64_t flight_ctrl_joystick_move(float vx, float vy, float vz, float vyaw, float duration) {
    /* Join previous thread if it finished */
    if (!s_move_active && s_move_thread) {
        pthread_join(s_move_thread, NULL);
        s_move_thread = 0;
    }

    pthread_mutex_lock(&s_move_mutex);
    s_move_vx = vx;
    s_move_vy = vy;
    s_move_vz = vz;
    s_move_vyaw = vyaw;
    s_move_duration = duration;
    pthread_mutex_unlock(&s_move_mutex);

    if (!s_move_active) {
        s_move_active = 1;
        pthread_create(&s_move_thread, NULL, _move_loop, NULL);
    }
    return 0;
}

int64_t flight_ctrl_stop_move(void) {
    if (s_move_active) {
        s_move_active = 0;
        pthread_join(s_move_thread, NULL);
        s_move_thread = 0;
    }
    T_DjiReturnCode rc = DjiFlightController_ExecuteEmergencyBrakeAction();
    /* EmergencyBrake releases joystick authority — next move will re-obtain */
    s_has_authority = 0;
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

int64_t flight_ctrl_emergency_brake(void) {
    /* Also stop any active move */
    if (s_move_active) {
        s_move_active = 0;
        pthread_join(s_move_thread, NULL);
        s_move_thread = 0;
    }
    T_DjiReturnCode rc = DjiFlightController_ExecuteEmergencyBrakeAction();
    /* EmergencyBrake releases joystick authority — next move will re-obtain */
    s_has_authority = 0;
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

/* ── Motors ────────────────────────────────────────────────────────── */

int64_t flight_ctrl_turn_on_motors(void) {
    T_DjiReturnCode rc = DjiFlightController_TurnOnMotors();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

int64_t flight_ctrl_turn_off_motors(void) {
    T_DjiReturnCode rc = DjiFlightController_TurnOffMotors();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

int64_t flight_ctrl_slow_rotate_start(void) {
    T_DjiReturnCode rc = DjiFlightController_StartSlowRotateMotor();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

int64_t flight_ctrl_slow_rotate_stop(void) {
    T_DjiReturnCode rc = DjiFlightController_StopSlowRotateMotor();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

/* ── Settings ─────────────────────────────────────────────────────── */

int64_t flight_ctrl_set_home(double lat, double lon) {
    T_DjiFlightControllerHomeLocation home = {
        .latitude = lat, .longitude = lon,
    };
    T_DjiReturnCode rc = DjiFlightController_SetHomeLocationUsingGPSCoordinates(home);
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

int64_t flight_ctrl_set_obstacle_avoidance(int enabled, const char *direction) {
    E_DjiFlightControllerObstacleAvoidanceEnableStatus status = enabled
        ? DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE
        : DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;
    T_DjiReturnCode rc;

    if (strcmp(direction, "up") == 0) {
        rc = DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(status);
    } else if (strcmp(direction, "down") == 0) {
        rc = DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(status);
    } else {
        rc = DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(status);
        DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(status);
        DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(status);
    }
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

void flight_ctrl_cleanup(void) {
    if (s_move_active) {
        s_move_active = 0;
        pthread_join(s_move_thread, NULL);
    }
    if (s_has_authority) {
        DjiFlightController_ReleaseJoystickCtrlAuthority();
        s_has_authority = 0;
    }
    DjiFlightController_DeInit();
}

#else /* stub */

int flight_ctrl_init(void) { printf("[flight] stub mode\n"); return 0; }
int64_t flight_ctrl_takeoff(void) { return 0; }
int64_t flight_ctrl_land(void) { return 0; }
int64_t flight_ctrl_confirm_landing(void) { return 0; }
int64_t flight_ctrl_land_auto_confirm(void) { return 0; }
int64_t flight_ctrl_go_home(void) { return 0; }
int64_t flight_ctrl_cancel_go_home(void) { return 0; }
int64_t flight_ctrl_joystick_move(float vx, float vy, float vz, float vyaw, float duration) { return 0; }
int64_t flight_ctrl_stop_move(void) { return 0; }
int64_t flight_ctrl_emergency_brake(void) { return 0; }
int64_t flight_ctrl_turn_on_motors(void) { return 0; }
int64_t flight_ctrl_turn_off_motors(void) { return 0; }
int64_t flight_ctrl_slow_rotate_start(void) { return 0; }
int64_t flight_ctrl_slow_rotate_stop(void) { return 0; }
int64_t flight_ctrl_obtain_authority(void) { return 0; }
int64_t flight_ctrl_release_authority(void) { return 0; }
int64_t flight_ctrl_set_home(double lat, double lon) { return 0; }
int64_t flight_ctrl_set_obstacle_avoidance(int enabled, const char *direction) { return 0; }
void flight_ctrl_cleanup(void) {}

#endif
