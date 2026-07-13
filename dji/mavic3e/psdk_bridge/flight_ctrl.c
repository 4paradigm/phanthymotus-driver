#include "flight_ctrl.h"
#include <stdio.h>
#include <string.h>

/*
 * PSDK Flight Controller for Mavic 3E.
 *
 * Key APIs:
 *   DjiFlightController_Init()
 *   DjiFlightController_ObtainJoystickCtrlAuthority()
 *   DjiFlightController_ReleaseJoystickCtrlAuthority()
 *   DjiFlightController_ExecuteJoystickAction()
 *   DjiFlightController_StartTakeoff()
 *   DjiFlightController_StartLanding()
 *   DjiFlightController_StartGoHome()
 *   DjiFlightController_CancelGoHome()
 *   DjiFlightController_ExecuteEmergencyBrakeAction()
 *   DjiFlightController_SetHomePointByGPSCoordinate()
 *   DjiFlightController_SetObstacleAvoidanceEnabled()
 *
 * Joystick control modes for Mavic 3E:
 *   Horizontal: velocity (body frame or ground frame)
 *   Vertical: velocity or position
 *   Yaw: angle rate
 */

#ifdef PSDK_ENABLED
#include "dji_flight_controller.h"

static int s_has_authority = 0;

int flight_ctrl_init(void) {
    /* DjiFlightController_Init requires RID info (latitude/longitude in rad, altitude) */
    T_DjiFlightControllerRidInfo ridInfo = {0};
    /* TODO: fill ridInfo with actual takeoff location for RID compliance */
    T_DjiReturnCode rc = DjiFlightController_Init(ridInfo);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[flight] init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }
    printf("[flight] initialized\n");
    return 0;
}

int flight_ctrl_obtain_authority(void) {
    T_DjiReturnCode rc = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_has_authority = 1;
        printf("[flight] joystick authority obtained\n");
        return 0;
    }
    printf("[flight] obtain authority failed: 0x%08llX\n", (unsigned long long)rc);
    return -1;
}

int flight_ctrl_release_authority(void) {
    T_DjiReturnCode rc = DjiFlightController_ReleaseJoystickCtrlAuthority();
    s_has_authority = 0;
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int flight_ctrl_takeoff(void) {
    return (DjiFlightController_StartTakeoff() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int flight_ctrl_land(void) {
    return (DjiFlightController_StartLanding() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int flight_ctrl_go_home(void) {
    return (DjiFlightController_StartGoHome() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int flight_ctrl_cancel_go_home(void) {
    return (DjiFlightController_CancelGoHome() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int flight_ctrl_joystick_move(float vx, float vy, float vz, float vyaw) {
    T_DjiFlightControllerJoystickMode mode = {
        .horizontalControlMode = DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        .verticalControlMode = DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        .yawControlMode = DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        .horizontalCoordinate = DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        .stableControlMode = DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    DjiFlightController_SetJoystickMode(mode);

    T_DjiFlightControllerJoystickCommand cmd = {
        .x = vx,
        .y = vy,
        .z = vz,
        .yaw = vyaw,
    };
    return (DjiFlightController_ExecuteJoystickAction(cmd) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int flight_ctrl_emergency_brake(void) {
    return (DjiFlightController_ExecuteEmergencyBrakeAction() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int flight_ctrl_set_home(double lat, double lon) {
    T_DjiFlightControllerHomeLocation home = {
        .latitude = lat,
        .longitude = lon,
    };
    return (DjiFlightController_SetHomeLocationUsingGPSCoordinates(home) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
}

int flight_ctrl_set_obstacle_avoidance(int enabled, const char *direction) {
    /* Mavic 3E supports per-direction obstacle avoidance */
    T_DjiReturnCode rc;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus status = enabled
        ? DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE
        : DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;

    if (strcmp(direction, "up") == 0) {
        rc = DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(status);
    } else if (strcmp(direction, "down") == 0) {
        rc = DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(status);
    } else {
        rc = DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(status);
        DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(status);
        DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(status);
    }
    return (rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) ? 0 : -1;
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
int flight_ctrl_takeoff(void) { return 0; }
int flight_ctrl_land(void) { return 0; }
int flight_ctrl_go_home(void) { return 0; }
int flight_ctrl_cancel_go_home(void) { return 0; }
int flight_ctrl_joystick_move(float vx, float vy, float vz, float vyaw) { return 0; }
int flight_ctrl_emergency_brake(void) { return 0; }
int flight_ctrl_obtain_authority(void) { return 0; }
int flight_ctrl_release_authority(void) { return 0; }
int flight_ctrl_set_home(double lat, double lon) { return 0; }
int flight_ctrl_set_obstacle_avoidance(int enabled, const char *direction) { return 0; }
void flight_ctrl_cleanup(void) {}

#endif
