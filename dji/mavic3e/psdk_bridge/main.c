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
#include "dji_payload_camera.h"
#include <pthread.h>
#include <semaphore.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

/* ── UART HAL implementation matching T_DjiHalUartHandler ─────────────── */

static int s_uart_fd = -1;
static const char *s_uart_device = "/dev/ttyUSB0";
static uint32_t s_uart_baud = 921600;

static speed_t _to_speed(uint32_t baud) {
    switch (baud) {
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        default:      return B921600;
    }
}

static T_DjiReturnCode _HalUart_Init(E_DjiHalUartNum uartNum, uint32_t baudRate,
                                      T_DjiUartHandle *uartHandle) {
    (void)uartNum;
    struct termios tty;

    /* Close previous fd if PSDK re-inits with different baud */
    if (s_uart_fd >= 0) {
        close(s_uart_fd);
        s_uart_fd = -1;
    }

    s_uart_fd = open(s_uart_device, O_RDWR | O_NOCTTY);
    if (s_uart_fd < 0) {
        printf("[hal] uart open %s failed: %s\n", s_uart_device, strerror(errno));
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    memset(&tty, 0, sizeof(tty));
    tcgetattr(s_uart_fd, &tty);
    speed_t speed = _to_speed(baudRate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag = CS8 | CLOCAL | CREAD;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;   /* Non-blocking: return immediately with available data */
    tty.c_cc[VTIME] = 5;  /* 500ms timeout — enough for aircraft to respond */
    tcsetattr(s_uart_fd, TCSANOW, &tty);
    tcflush(s_uart_fd, TCIOFLUSH);

    *uartHandle = (T_DjiUartHandle)(intptr_t)s_uart_fd;
    printf("[hal] uart %s opened @ %u baud (fd=%d)\n", s_uart_device, baudRate, s_uart_fd);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _HalUart_DeInit(T_DjiUartHandle uartHandle) {
    int fd = (int)(intptr_t)uartHandle;
    if (fd >= 0) close(fd);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _HalUart_WriteData(T_DjiUartHandle uartHandle,
                                           const uint8_t *buf, uint32_t len, uint32_t *realLen) {
    int fd = (int)(intptr_t)uartHandle;
    ssize_t n = write(fd, buf, len);
    *realLen = (n > 0) ? (uint32_t)n : 0;
    return (n >= 0) ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS : DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
}

static T_DjiReturnCode _HalUart_ReadData(T_DjiUartHandle uartHandle,
                                          uint8_t *buf, uint32_t len, uint32_t *realLen) {
    int fd = (int)(intptr_t)uartHandle;
    ssize_t n = read(fd, buf, len);
    *realLen = (n > 0) ? (uint32_t)n : 0;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _HalUart_GetStatus(E_DjiHalUartNum uartNum, T_DjiUartStatus *status) {
    (void)uartNum;
    status->isConnect = (s_uart_fd >= 0);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _HalUart_GetDeviceInfo(T_DjiHalUartDeviceInfo *deviceInfo) {
    /* FTDI FT232R on E-Port dev board */
    deviceInfo->vid = 0x0403;
    deviceInfo->pid = 0x6001;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* ── OSAL implementation matching T_DjiOsalHandler ────────────────────── */

static T_DjiReturnCode _Osal_TaskCreate(const char *name, void *(*taskFunc)(void *),
                                         uint32_t stackSize, void *arg, T_DjiTaskHandle *task) {
    pthread_t *tid = (pthread_t *)malloc(sizeof(pthread_t));
    if (!tid) return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    if (stackSize > 0 && stackSize >= 64*1024)
        pthread_attr_setstacksize(&attr, stackSize);
    if (pthread_create(tid, &attr, taskFunc, arg) != 0) {
        free(tid);
        pthread_attr_destroy(&attr);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    pthread_attr_destroy(&attr);
    *task = (T_DjiTaskHandle)tid;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _Osal_TaskDestroy(T_DjiTaskHandle task) {
    pthread_t *tid = (pthread_t *)task;
    pthread_cancel(*tid);
    pthread_join(*tid, NULL);
    free(tid);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _Osal_TaskSleepMs(uint32_t timeMs) {
    usleep((useconds_t)timeMs * 1000);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _Osal_MutexCreate(T_DjiMutexHandle *mutex) {
    pthread_mutex_t *m = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
    if (!m) return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    pthread_mutex_init(m, NULL);
    *mutex = (T_DjiMutexHandle)m;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _Osal_MutexDestroy(T_DjiMutexHandle mutex) {
    pthread_mutex_destroy((pthread_mutex_t *)mutex);
    free(mutex);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _Osal_MutexLock(T_DjiMutexHandle mutex) {
    pthread_mutex_lock((pthread_mutex_t *)mutex);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _Osal_MutexUnlock(T_DjiMutexHandle mutex) {
    pthread_mutex_unlock((pthread_mutex_t *)mutex);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _Osal_SemCreate(uint32_t initValue, T_DjiSemaHandle *semaphore) {
    sem_t *s = (sem_t *)malloc(sizeof(sem_t));
    if (!s) return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    sem_init(s, 0, initValue);
    *semaphore = (T_DjiSemaHandle)s;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _Osal_SemDestroy(T_DjiSemaHandle semaphore) {
    sem_destroy((sem_t *)semaphore);
    free(semaphore);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _Osal_SemWait(T_DjiSemaHandle semaphore) {
    sem_wait((sem_t *)semaphore);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _Osal_SemTimedWait(T_DjiSemaHandle semaphore, uint32_t waitTimeMs) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += waitTimeMs / 1000;
    ts.tv_nsec += (waitTimeMs % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
    int ret = sem_timedwait((sem_t *)semaphore, &ts);
    return (ret == 0) ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS : DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
}

static T_DjiReturnCode _Osal_SemPost(T_DjiSemaHandle semaphore) {
    sem_post((sem_t *)semaphore);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _Osal_GetTimeMs(uint32_t *ms) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    *ms = (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _Osal_GetTimeUs(uint64_t *us) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    *us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _Osal_GetRandomNum(uint16_t *randomNum) {
    *randomNum = (uint16_t)(rand() & 0xFFFF);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void *_Osal_Malloc(uint32_t size) { return malloc(size); }
static void _Osal_Free(void *ptr) { free(ptr); }

/* ── Network HAL (configure RNDIS interface via ioctl) ────────────────── */

#define NETWORK_IFACE "rndis0"

static T_DjiReturnCode _HalNetwork_Init(const char *ipAddr, const char *netMask,
                                         T_DjiNetworkHandle *networkHandle) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        printf("[net] socket failed: %s\n", strerror(errno));
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, NETWORK_IFACE, IFNAMSIZ - 1);

    /* Remove rndis0 from any bridge (l4tbr0) — critical for direct routing */
    int br_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (br_sock >= 0) {
        struct ifreq br_ifr;
        memset(&br_ifr, 0, sizeof(br_ifr));
        strncpy(br_ifr.ifr_name, "l4tbr0", IFNAMSIZ - 1);
        ioctl(sock, SIOCGIFINDEX, &ifr);
        br_ifr.ifr_ifindex = ifr.ifr_ifindex;
        ioctl(br_sock, 0x89a3, &br_ifr);
        close(br_sock);
        printf("[net] removed %s from bridge\n", NETWORK_IFACE);
    }

    /* Also remove usb0 from bridge if present */
    {
        struct ifreq usb_ifr;
        memset(&usb_ifr, 0, sizeof(usb_ifr));
        strncpy(usb_ifr.ifr_name, "usb0", IFNAMSIZ - 1);
        int usb_sock = socket(AF_INET, SOCK_STREAM, 0);
        if (usb_sock >= 0) {
            struct ifreq br2;
            memset(&br2, 0, sizeof(br2));
            strncpy(br2.ifr_name, "l4tbr0", IFNAMSIZ - 1);
            ioctl(usb_sock, SIOCGIFINDEX, &usb_ifr);
            br2.ifr_ifindex = usb_ifr.ifr_ifindex;
            ioctl(usb_sock, 0x89a3, &br2);
            close(usb_sock);
        }
    }

    /* Set IP address (don't bring down — keep USB link alive) */
    struct sockaddr_in *addr = (struct sockaddr_in *)&ifr.ifr_addr;
    addr->sin_family = AF_INET;
    inet_pton(AF_INET, ipAddr, &addr->sin_addr);
    if (ioctl(sock, SIOCSIFADDR, &ifr) < 0) {
        printf("[net] set IP %s failed: %s\n", ipAddr, strerror(errno));
    }

    /* Set netmask */
    inet_pton(AF_INET, netMask, &addr->sin_addr);
    if (ioctl(sock, SIOCSIFNETMASK, &ifr) < 0) {
        printf("[net] set mask %s failed: %s\n", netMask, strerror(errno));
    }

    /* Bring interface up */
    ioctl(sock, SIOCGIFFLAGS, &ifr);
    ifr.ifr_flags |= IFF_UP | IFF_RUNNING;
    if (ioctl(sock, SIOCSIFFLAGS, &ifr) < 0) {
        printf("[net] bring up %s failed: %s\n", NETWORK_IFACE, strerror(errno));
    }

    close(sock);
    *networkHandle = (T_DjiNetworkHandle)(intptr_t)1;
    printf("[net] %s configured: %s/%s\n", NETWORK_IFACE, ipAddr, netMask);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _HalNetwork_DeInit(T_DjiNetworkHandle networkHandle) {
    (void)networkHandle;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _HalNetwork_GetDeviceInfo(T_DjiHalNetworkDeviceInfo *deviceInfo) {
    /* Read actual VID/PID from USB gadget configfs */
    FILE *f;
    char buf[16];
    uint16_t vid = 0x0955, pid = 0x7020;  /* default Jetson */

    f = fopen("/sys/kernel/config/usb_gadget/l4t/idVendor", "r");
    if (f) { if (fgets(buf, sizeof(buf), f)) vid = (uint16_t)strtol(buf, NULL, 16); fclose(f); }
    f = fopen("/sys/kernel/config/usb_gadget/l4t/idProduct", "r");
    if (f) { if (fgets(buf, sizeof(buf), f)) pid = (uint16_t)strtol(buf, NULL, 16); fclose(f); }

    deviceInfo->usbNetAdapter.vid = vid;
    deviceInfo->usbNetAdapter.pid = pid;
    printf("[net] device info: vid=0x%04X pid=0x%04X\n", vid, pid);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* ── PSDK init ────────────────────────────────────────────────────────── */

static int _psdk_core_init(const char *app_id, const char *app_key,
                           const char *app_license, const char *app_name,
                           const char *uart_dev, uint32_t baud_rate) {
    T_DjiReturnCode rc;
    s_uart_device = uart_dev;
    s_uart_baud = baud_rate;

    /* Register OSAL first (PSDK needs threads before anything else) */
    T_DjiOsalHandler osalHandler = {
        .TaskCreate = _Osal_TaskCreate,
        .TaskDestroy = _Osal_TaskDestroy,
        .TaskSleepMs = _Osal_TaskSleepMs,
        .MutexCreate = _Osal_MutexCreate,
        .MutexDestroy = _Osal_MutexDestroy,
        .MutexLock = _Osal_MutexLock,
        .MutexUnlock = _Osal_MutexUnlock,
        .SemaphoreCreate = _Osal_SemCreate,
        .SemaphoreDestroy = _Osal_SemDestroy,
        .SemaphoreWait = _Osal_SemWait,
        .SemaphoreTimedWait = _Osal_SemTimedWait,
        .SemaphorePost = _Osal_SemPost,
        .GetTimeMs = _Osal_GetTimeMs,
        .GetTimeUs = _Osal_GetTimeUs,
        .GetRandomNum = _Osal_GetRandomNum,
        .Malloc = _Osal_Malloc,
        .Free = _Osal_Free,
    };
    rc = DjiPlatform_RegOsalHandler(&osalHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[psdk] OSAL registration failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }

    /* Register HAL UART */
    T_DjiHalUartHandler uartHandler = {
        .UartInit = _HalUart_Init,
        .UartDeInit = _HalUart_DeInit,
        .UartWriteData = _HalUart_WriteData,
        .UartReadData = _HalUart_ReadData,
        .UartGetStatus = _HalUart_GetStatus,
        .UartGetDeviceInfo = _HalUart_GetDeviceInfo,
    };
    rc = DjiPlatform_RegHalUartHandler(&uartHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[psdk] HAL UART registration failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }

    /* Register HAL USB Bulk only if FFS endpoints are available (host-side setup done) */
    extern T_DjiHalUsbBulkHandler g_usbBulkHandler;
    if (access("/dev/usb-ffs/bulk1/ep1", F_OK) == 0) {
        rc = DjiPlatform_RegHalUsbBulkHandler(&g_usbBulkHandler);
        if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            printf("[psdk] HAL USB Bulk registration failed: 0x%08llX\n", (unsigned long long)rc);
        } else {
            printf("[psdk] HAL USB Bulk registered OK\n");
        }
    } else {
        printf("[psdk] USB Bulk endpoints not available, skipping (UART-only mode)\n");
    }

    /* Init PSDK core */
    T_DjiUserInfo userInfo = {0};
    strncpy(userInfo.appName, app_name, sizeof(userInfo.appName) - 1);
    strncpy(userInfo.appId, app_id, sizeof(userInfo.appId) - 1);
    strncpy(userInfo.appKey, app_key, sizeof(userInfo.appKey) - 1);
    strncpy(userInfo.appLicense, app_license, sizeof(userInfo.appLicense) - 1);
    strncpy(userInfo.developerAccount, "phanthymotus@4paradigm.com",
            sizeof(userInfo.developerAccount) - 1);
    snprintf(userInfo.baudRate, sizeof(userInfo.baudRate), "%u", baud_rate);

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

    /* Camera init required even for non-camera payloads (Pilot won't detect otherwise) */
    rc = DjiPayloadCamera_Init();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[psdk] payload camera init warning: 0x%08llX\n", (unsigned long long)rc);
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
    if (strstr(raw_json, "\"rotate_start\"") && !strstr(raw_json, "\"slow_rotate_start\"")) {
        int r = flight_ctrl_turn_on_motors();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"rotate_stop\"") && !strstr(raw_json, "\"slow_rotate_stop\"")) {
        int r = flight_ctrl_turn_off_motors();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"slow_rotate_start\"")) {
        int r = flight_ctrl_slow_rotate_start();
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d}}", r == 0 ? "true" : "false", r);
        return 0;
    }
    if (strstr(raw_json, "\"slow_rotate_stop\"")) {
        int r = flight_ctrl_slow_rotate_stop();
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

    /* Power — use real battery data from telemetry */
    if (strstr(raw_json, "\"get_power_state\"")) {
        char telem[4096];
        telemetry_get_json(telem, sizeof(telem));
        /* Extract battery info from telemetry JSON and return as power state */
        snprintf(result, result_size,
            "{\"ok\":true,\"data\":%s}", telem);
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
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

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
