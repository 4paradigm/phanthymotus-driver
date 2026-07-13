/*
 * psdk_bridge/main.c — DJI PSDK Bridge main entry point.
 *
 * This is the C process that:
 *   1. Initializes DJI PSDK with app credentials
 *   2. Initializes all PSDK modules (telemetry, flight, camera, gimbal, etc.)
 *   3. Starts IPC server (Unix socket) for Python communication
 *   4. Runs main event loop processing IPC commands + PSDK callbacks
 *
 * Build with PSDK_ENABLED defined to link against libpayloadsdk.a.
 * Without PSDK_ENABLED, builds in stub mode for development/testing.
 *
 * Usage:
 *   ./psdk_bridge [socket_path] [app_id] [app_key] [app_license] [uart_dev] [baud_rate]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

#include "ipc.h"
#include "hal_uart.h"
#include "hal_network.h"
#include "osal_posix.h"
#include "telemetry.h"
#include "flight_ctrl.h"
#include "camera_mgr.h"
#include "gimbal_mgr.h"
#include "liveview.h"
#include "waypoint.h"
#include "perception.h"
#include "speaker.h"
#include "hms.h"

static volatile int s_running = 1;

static void _signal_handler(int sig) {
    printf("[psdk_bridge] signal %d, shutting down\n", sig);
    s_running = 0;
}

/* ── PSDK Core Init ─────────────────────────────────────────────────────── */

#ifdef PSDK_ENABLED
#include "dji_core.h"
#include "dji_platform.h"

/* HAL/OSAL implementations are in hal_uart.c, hal_network.c, osal_posix.c */

static int _psdk_core_init(const char *app_id, const char *app_key,
                           const char *app_license, const char *app_name,
                           const char *uart_dev, uint32_t baud_rate) {
    T_DjiReturnCode rc;

    /* Register HAL (UART, USB, network) */
    T_DjiHalUartHandler uartHandler = {
        /* Fill with platform-specific UART operations */
    };
    rc = DjiPlatform_RegHalUartHandler(&uartHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[psdk] HAL UART registration failed\n");
        return -1;
    }

    /* Register OSAL (thread, mutex, semaphore) */
    T_DjiOsalHandler osalHandler = {
        /* Fill with POSIX implementations */
    };
    rc = DjiPlatform_RegOsalHandler(&osalHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[psdk] OSAL registration failed\n");
        return -1;
    }

    /* Init PSDK core */
    T_DjiUserInfo userInfo = {0};
    strncpy(userInfo.appName, app_name, sizeof(userInfo.appName) - 1);
    strncpy(userInfo.appId, app_id, sizeof(userInfo.appId) - 1);
    strncpy(userInfo.appKey, app_key, sizeof(userInfo.appKey) - 1);
    strncpy(userInfo.appLicense, app_license, sizeof(userInfo.appLicense) - 1);
    strncpy(userInfo.developerAccount, "phanthymotus@4paradigm.com",
            sizeof(userInfo.developerAccount) - 1);

    rc = DjiCore_Init(&userInfo);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[psdk] core init failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }

    rc = DjiCore_SetAlias("PhanthyMotus");
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[psdk] set alias warning: 0x%08llX\n", (unsigned long long)rc);
    }

    rc = DjiCore_ApplicationStart();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[psdk] application start failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }

    printf("[psdk] core initialized (app=%s, id=%s)\n", app_name, app_id);
    return 0;
}
#endif

/* ── IPC Command Dispatcher ─────────────────────────────────────────────── */

static int _dispatch_cmd(const char *raw_json, const char *unused,
                         char *result, size_t result_size) {
    /*
     * Simple JSON command dispatch. In production, use cJSON for proper parsing.
     * For now, use strstr-based matching for the common commands.
     */

    /* Telemetry */
    if (strstr(raw_json, "\"get_telemetry\"")) {
        char telem[4096];
        telemetry_get_json(telem, sizeof(telem));
        snprintf(result, result_size, "{\"ok\":true,\"data\":%s}", telem);
        return 0;
    }

    /* Flight control */
    if (strstr(raw_json, "\"takeoff\"")) {
        int r = flight_ctrl_takeoff();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"land\"")) {
        int r = flight_ctrl_land();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"go_home\"") && !strstr(raw_json, "\"cancel_go_home\"")) {
        int r = flight_ctrl_go_home();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"cancel_go_home\"")) {
        int r = flight_ctrl_cancel_go_home();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"emergency_brake\"")) {
        int r = flight_ctrl_emergency_brake();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"obtain_joystick_authority\"")) {
        int r = flight_ctrl_obtain_authority();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"release_joystick_authority\"")) {
        int r = flight_ctrl_release_authority();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }

    /* Camera */
    if (strstr(raw_json, "\"take_photo\"")) {
        int r = camera_mgr_take_photo("single");
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"start_video\"")) {
        int r = camera_mgr_start_video();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"stop_video\"")) {
        int r = camera_mgr_stop_video();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }

    /* Gimbal */
    if (strstr(raw_json, "\"gimbal_reset\"")) {
        int r = gimbal_mgr_reset();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"gimbal_get_angles\"")) {
        float p, y, r;
        gimbal_mgr_get_angles(&p, &y, &r);
        snprintf(result, result_size,
            "{\"ok\":true,\"data\":{\"pitch\":%.2f,\"yaw\":%.2f,\"roll\":%.2f}}", p, y, r);
        return 0;
    }

    /* Waypoint */
    if (strstr(raw_json, "\"waypoint_start\"")) {
        int r = waypoint_start();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"waypoint_pause\"")) {
        int r = waypoint_pause();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"waypoint_resume\"")) {
        int r = waypoint_resume();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"waypoint_stop\"")) {
        int r = waypoint_stop();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"waypoint_status\"")) {
        char status[256];
        waypoint_get_status(status, sizeof(status));
        snprintf(result, result_size, "{\"ok\":true,\"data\":%s}", status);
        return 0;
    }

    /* HMS */
    if (strstr(raw_json, "\"get_hms_info\"")) {
        char hms_buf[4096];
        hms_get_info(hms_buf, sizeof(hms_buf));
        snprintf(result, result_size, "{\"ok\":true,\"data\":%s}", hms_buf);
        return 0;
    }

    /* Speaker */
    if (strstr(raw_json, "\"speaker_play\"")) {
        speaker_play_tts("test");
        snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        return 0;
    }
    if (strstr(raw_json, "\"speaker_stop\"")) {
        speaker_stop();
        snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        return 0;
    }

    /* Power */
    if (strstr(raw_json, "\"get_power_state\"")) {
        snprintf(result, result_size,
            "{\"ok\":true,\"data\":{\"battery_percent\":85,\"voltage\":22.8,\"current\":5.2,\"eport_power\":true}}");
        return 0;
    }

    /* Liveview */
    if (strstr(raw_json, "\"start_liveview\"")) {
        liveview_start("wide", NULL);
        snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        return 0;
    }
    if (strstr(raw_json, "\"stop_liveview\"")) {
        liveview_stop();
        snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        return 0;
    }

    /* Perception */
    if (strstr(raw_json, "\"start_perception\"")) {
        perception_start("front", NULL);
        snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        return 0;
    }
    if (strstr(raw_json, "\"stop_perception\"")) {
        perception_stop("front");
        snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        return 0;
    }

    /* Aircraft info */
    if (strstr(raw_json, "\"get_aircraft_info\"")) {
        snprintf(result, result_size,
            "{\"ok\":true,\"data\":{\"product_name\":\"Mavic 3 Enterprise\","
            "\"firmware_version\":\"07.01.20.01\",\"serial_number\":\"UNKNOWN\"}}");
        return 0;
    }

    /* Unknown command */
    snprintf(result, result_size, "{\"ok\":false,\"error\":\"unknown command\"}");
    return -1;
}

/* ── Main ───────────────────────────────────────────────────────────────── */

int main(int argc, char *argv[]) {
    const char *socket_path = "/tmp/psdk_bridge.sock";
    const char *app_id = "";
    const char *app_key = "";
    const char *app_license = "";
    const char *app_name = "PhanthyMotus";
    const char *uart_dev = "/dev/ttyACM0";
    uint32_t baud_rate = 921600;

    if (argc >= 2) socket_path = argv[1];
    if (argc >= 3) app_id = argv[2];
    if (argc >= 4) app_key = argv[3];
    if (argc >= 5) app_license = argv[4];
    if (argc >= 6) uart_dev = argv[5];
    if (argc >= 7) baud_rate = (uint32_t)atoi(argv[6]);

    printf("=== DJI PSDK Bridge for Mavic 3E ===\n");
    printf("  Socket: %s\n", socket_path);
    printf("  UART:   %s @ %u\n", uart_dev, baud_rate);

    signal(SIGINT, _signal_handler);
    signal(SIGTERM, _signal_handler);

    /* Initialize HAL layer (platform abstraction) */
    if (HalUart_Init(uart_dev, baud_rate) != 0) {
        printf("[psdk_bridge] WARNING: UART init failed — hardware may not be connected\n");
    }
    HalNetwork_Init();  /* Non-fatal if no USB-Ethernet yet */

#ifdef PSDK_ENABLED
    /* Initialize PSDK core */
    if (_psdk_core_init(app_id, app_key, app_license, app_name, uart_dev, baud_rate) != 0) {
        printf("[psdk_bridge] PSDK core init failed, exiting\n");
        return 1;
    }
#else
    printf("[psdk_bridge] Running in STUB mode (no PSDK)\n");
#endif

    /* Initialize all modules */
    telemetry_init();
    flight_ctrl_init();
    camera_mgr_init();
    gimbal_mgr_init();
    liveview_init();
    waypoint_init();
    perception_init();
    speaker_init();
    hms_init();

    /* Start IPC server */
    if (ipc_init(socket_path) != 0) {
        printf("[psdk_bridge] IPC init failed, exiting\n");
        return 1;
    }
    ipc_set_handler(_dispatch_cmd);

    printf("[psdk_bridge] Ready, entering main loop\n");

    /* Main event loop */
    while (s_running) {
        ipc_process();
        usleep(1000);  /* 1ms — avoids busy-wait */
    }

    /* Cleanup */
    printf("[psdk_bridge] Shutting down...\n");
    hms_cleanup();
    speaker_cleanup();
    perception_cleanup();
    waypoint_cleanup();
    liveview_cleanup();
    gimbal_mgr_cleanup();
    camera_mgr_cleanup();
    flight_ctrl_cleanup();
    telemetry_cleanup();
    ipc_cleanup();
    HalUart_Close();
    HalNetwork_Cleanup();

#ifdef PSDK_ENABLED
    DjiCore_DeInit();
#endif

    printf("[psdk_bridge] Done.\n");
    return 0;
}
