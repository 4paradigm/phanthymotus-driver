#include "error_code.h"
#include <stdio.h>
#include <string.h>

typedef struct {
    uint64_t code;
    const char *description;
    const char *recovery;
} ErrorEntry;

/* Error code table — covers the most common PSDK errors.
 * Codes are 64-bit: upper 32 bits = module index, lower 32 bits = raw code. */
static const ErrorEntry s_error_table[] = {
    /* System module (module=0) */
    {0x0000000000, "Execution successfully.", NULL},
    {0x00000000D4, "Request parameters are invalid.", "Please double-check requested parameters."},
    {0x00000000D7, "A higher priority task is being executed.", "Please stop the higher priority task or try again later."},
    {0x00000000E0, "Operation is not supported.", "Please check input parameters or contact DJI for help."},
    {0x00000000E1, "Execution timeout.", "Please contact DJI for help."},
    {0x00000000E2, "Memory allocation failed.", "Please check system configure."},
    {0x00000000E3, "Input parameters are invalid.", "Please double-check requested parameters."},
    {0x00000000E4, "Operation is not supported in current state.", "Please try again later."},
    {0x00000000EC, "System error.", "Please contact DJI for help."},
    {0x00000000FA, "Hardware error.", "Please contact DJI for help."},
    {0x00000000FB, "Low battery.", "Please replace battery and try again."},
    {0x00000000FF, "Unknown error.", NULL},
    {0x0000000100, "Parameters are not found.", NULL},
    {0x0000000101, "Out of range.", "Please check parameters."},
    {0x0000000102, "System is busy.", "Please try again later."},
    {0x0000000103, "Have existed the same object.", "Please input valid parameters."},
    {0x0000000104, "PSDK adapter do not meet requirements.", "Please try again after replacing PSDK adapter."},

    /* Flight controller basic (module=0x1B) */
    {0x1B00000000, "RC_MODE_ERROR", "Please check the RC mode"},
    {0x1B00000001, "RELEASE_CONTROL_SUCCESS", NULL},
    {0x1B00000002, "OBTAIN_CONTROL_SUCCESS", NULL},
    {0x1B00000003, "OBTAIN_CONTROL_IN_PROGRESS", NULL},
    {0x1B00000004, "RELEASE_CONTROL_IN_PROGRESS", NULL},
    {0x1B00000005, "RC_NEED_MODE_P", "Please switch RC to P mode"},
    {0x1B00000006, "RC_NEED_MODE_F", NULL},
    {0x1B0000FF00, "Activate key error", NULL},
    {0x1B0000FF01, "No authorization", "Please finish activation first"},
    {0x1B0000FF02, "No rights error", NULL},
    {0x1B0000FF03, "Unknown system error", NULL},

    /* Flight controller joystick (module=0x1C) */
    {0x1C00000000, "Obtain/Release joystick authority success", NULL},
    {0x1C00000001, "Device is not allowed to obtain joystick authority", "Please use OSDK/MSDK devices"},
    {0x1C00000003, "Not allowed during takeoff", "Please do it before or after takeoff"},
    {0x1C00000004, "Not allowed during landing", "Please do it before or after landing"},
    {0x1C00000005, "Invalid input command", "Only support 0/1"},
    {0x1C00000006, "RC is not in P_MODE", "Please switch RC to P_MODE"},
    {0x1C00000007, "Invalid command length", "Please input valid length command"},
    {0x1C00000008, "No joystick authority", "Please obtain joystick authority first"},
    {0x1C00000009, "In RC lost action", "Please check RC connection"},

    /* Flight controller action (module=0x1D) */
    {0x1D00000001, "Motor is on", "Please check motor status"},
    {0x1D00000002, "Motor is off", "Please check motor status"},
    {0x1D00000003, "Aircraft is in air", "Please check flight status"},
    {0x1D00000004, "Aircraft is not in air", "Please check flight status"},
    {0x1D00000005, "Home point not set", "Please set home point"},
    {0x1D00000006, "Bad GPS", "Please fly where GPS signal is good"},
    {0x1D00000007, "In simulation", "Please exit simulation first"},
    {0x1D00000011, "Cannot start motor", "Please check motor status"},
    {0x1D00000012, "Low voltage", "Please change battery"},
    {0x1D00000014, "Speed too large", "Please slow down"},

    /* FC home location (module=0x1F) */
    {0x1F00000001, "Set home location fail, unknown reason", NULL},
    {0x1F00000002, "Invalid GPS coordinate", NULL},
    {0x1F00000003, "Home location not recorded", "Please wait for aircraft to record home location"},
    {0x1F00000004, "GPS level < 4", NULL},
    {0x1F00000005, "New home >20km from current home", NULL},

    /* Camera manager (module=0x21) */
    {0x21000000E0, "Command not supported", "Check firmware or command validity"},
    {0x21000000E1, "Camera execution timeout", "Try again or check firmware"},
    {0x21000000E2, "Camera out of memory", "Please contact DJI support"},
    {0x21000000E3, "Camera received invalid parameters", "Check parameter validity"},
    {0x21000000E4, "Camera busy or command not supported in current state", "Check current camera state"},
    {0x21000000E6, "Camera failed to set parameters", "Check if parameter is supported on device"},
    {0x21000000E7, "Camera failed to get parameters", "Check if parameter is supported on device"},
    {0x21000000E8, "SD card missing", "Please install SD card"},
    {0x21000000E9, "SD card full", "Please make sure SD card has enough space"},
    {0x21000000EA, "SD card error", "Please check SD card validity"},
    {0x21000000EB, "Camera sensor error", "Please contact DJI support"},
    {0x21000000EC, "Camera system error", "Please recheck conditions or contact DJI support"},
    {0x21000000F8, "Remote control disconnected", "Please check RC connection"},
    {0x21000000FA, "Camera hardware error", "Please contact DJI support"},
    {0x21000000FC, "Aircraft disconnected", "Please check aircraft connection"},
    {0x21000000FF, "Undefined camera error", "Please contact DJI support"},

    /* Gimbal manager (module=0x22) */
    {0x22000000E0, "Gimbal command not supported", "Check firmware or command validity"},
    {0x22000000E1, "Gimbal execution timeout", "Try again"},
    {0x22000000E3, "Gimbal received invalid parameters", "Check parameter validity"},
    {0x22000000E4, "Gimbal busy or not supported in current state", "Check gimbal state"},
    {0x22000000E6, "Gimbal failed to set parameters", "Check if supported on device"},
    {0x22000000FA, "Gimbal hardware error", "Please contact DJI support"},
    {0x22000000FF, "Undefined gimbal error", "Please contact DJI support"},

    /* Gimbal module (module=0x06) */
    {0x0600000000, "Pitch reached positive limit", "Do not rotate towards positive direction"},
    {0x0600000001, "Pitch reached negative limit", "Do not rotate towards negative direction"},
    {0x0600000002, "Roll reached positive limit", "Do not rotate towards positive direction"},
    {0x0600000003, "Roll reached negative limit", "Do not rotate towards negative direction"},
    {0x0600000004, "Yaw reached positive limit", "Do not rotate towards positive direction"},
    {0x0600000005, "Yaw reached negative limit", "Do not rotate towards negative direction"},
    {0x0600000006, "No gimbal control authority", "Stop controlling gimbal with other high-priority devices"},
};

#define ERROR_TABLE_SIZE (sizeof(s_error_table) / sizeof(s_error_table[0]))

int error_code_to_json(uint64_t code, char *buf, size_t buflen) {
    for (size_t i = 0; i < ERROR_TABLE_SIZE; i++) {
        if (s_error_table[i].code == code) {
            if (s_error_table[i].recovery) {
                return snprintf(buf, buflen,
                    "{\"error\":\"%s\",\"code\":\"0x%010llX\",\"recovery\":\"%s\"}",
                    s_error_table[i].description,
                    (unsigned long long)code,
                    s_error_table[i].recovery);
            } else {
                return snprintf(buf, buflen,
                    "{\"error\":\"%s\",\"code\":\"0x%010llX\"}",
                    s_error_table[i].description,
                    (unsigned long long)code);
            }
        }
    }
    /* Unknown code — return hex with module/raw breakdown */
    uint32_t module = (uint32_t)((code >> 32) & 0xFF);
    uint32_t raw = (uint32_t)(code & 0xFFFFFFFF);
    return snprintf(buf, buflen,
        "{\"error\":\"Unknown error\",\"code\":\"0x%010llX\",\"module\":%u,\"raw_code\":\"0x%08X\"}",
        (unsigned long long)code, module, raw);
}
