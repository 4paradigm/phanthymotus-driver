#include "flight_ctrl.h"
#include "error_code.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/*
 * PSDK Flight Controller for Mavic 3E/3T.
 *
 * All functions return 0 on success or the raw PSDK T_DjiReturnCode on failure.
 * main.c uses error_code_to_json() to format human-readable error messages.
 */

#ifdef PSDK_ENABLED
#include "dji_flight_controller.h"

static int s_has_authority = 0;

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
    /* Wait for aircraft to reach low altitude before confirming */
    usleep(4000000);  /* 4 seconds */
    rc = DjiFlightController_StartConfirmLanding();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

int64_t flight_ctrl_go_home(void) {
    T_DjiReturnCode rc = DjiFlightController_StartGoHome();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

int64_t flight_ctrl_cancel_go_home(void) {
    T_DjiReturnCode rc = DjiFlightController_CancelGoHome();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

int64_t flight_ctrl_joystick_move(float vx, float vy, float vz, float vyaw) {
    T_DjiFlightControllerJoystickMode mode = {
        .horizontalControlMode = DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        .verticalControlMode = DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        .yawControlMode = DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        .horizontalCoordinate = DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        .stableControlMode = DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    DjiFlightController_SetJoystickMode(mode);

    T_DjiFlightControllerJoystickCommand cmd = {
        .x = vx, .y = vy, .z = vz, .yaw = vyaw,
    };
    T_DjiReturnCode rc = DjiFlightController_ExecuteJoystickAction(cmd);
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

int64_t flight_ctrl_emergency_brake(void) {
    T_DjiReturnCode rc = DjiFlightController_ExecuteEmergencyBrakeAction();
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : (int64_t)rc;
}

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
int64_t flight_ctrl_joystick_move(float vx, float vy, float vz, float vyaw) { return 0; }
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
