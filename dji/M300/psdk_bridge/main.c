#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

#include "ipc.h"
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
#include "time_sync.h"
#include "error_code.h"

static volatile int s_running = 1;

static volatile int s_psdk_state = 0;

static void _signal_handler(int sig) {
    printf("[psdk_bridge] signal %d, shutting down\n", sig);
    s_running = 0;
}

/* ── PSDK Core Init ─────────────────────────────────────────────────────── */

#ifdef PSDK_ENABLED
#include "dji_core.h"
#include "dji_platform.h"
#include "dji_version.h"
#include "dji_logger.h"
#include "dji_payload_camera.h"
#include "dji_aircraft_info.h"
#include "hal_usb_bulk.h"
#include <pthread.h>
#include <semaphore.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <limits.h>

/* ── UART HAL implementation matching T_DjiHalUartHandler ─────────────── */


#define M300_UART_COUNT 2
typedef struct {
    const char *device;
    uint16_t vid;
    uint16_t pid;
    uint32_t open_handles;
} T_M300Uart;

static T_M300Uart s_uart[M300_UART_COUNT] = {
    [DJI_HAL_UART_NUM_0] = {.device = "/dev/ttyACM0"},
    [DJI_HAL_UART_NUM_1] = {.device = "/dev/ttyACM0"},
};

typedef struct {
    int fd;
    T_M300Uart *uart;
    bool lock_held;
} T_M300UartHandle;

static volatile uint64_t s_uart_tx_bytes;
static volatile uint64_t s_uart_rx_bytes;
static volatile uint32_t s_uart_write_errors;
static volatile uint32_t s_uart_read_errors;

static T_DjiReturnCode _PsdkLogConsole(const uint8_t *data, uint16_t dataLen) {
    if (data == NULL)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    fwrite(data, 1, dataLen, stdout);
    fflush(stdout);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiLoggerConsole s_psdk_debug_console = {
    .func = _PsdkLogConsole,
    .consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_DEBUG,
    .isSupportColor = false,
};
static T_M300Uart *_HalUart_Get(E_DjiHalUartNum uartNum) {
    if (uartNum < DJI_HAL_UART_NUM_0 || uartNum >= M300_UART_COUNT)
        return NULL;
    return &s_uart[uartNum];
}

static T_M300UartHandle *_HalUart_Handle(T_DjiUartHandle handle) {
    return (T_M300UartHandle *)(intptr_t)handle;
}


static void _HalUart_DetectDeviceInfo(T_M300Uart *uart) {
    char path[PATH_MAX], current[PATH_MAX], value[16];
    const char *name = strrchr(uart->device, '/');
    FILE *f;

    name = name ? name + 1 : uart->device;
    uart->vid = 0;
    uart->pid = 0;

    snprintf(path, sizeof(path), "/sys/class/tty/%s/device", name);
    if (realpath(path, current)) {

        while (current[0] != '\0') {
            snprintf(path, sizeof(path), "%s/idVendor", current);
            f = fopen(path, "r");
            if (f) {
                if (fgets(value, sizeof(value), f))
                    uart->vid = (uint16_t)strtoul(value, NULL, 16);
                fclose(f);

                snprintf(path, sizeof(path), "%s/idProduct", current);
                f = fopen(path, "r");
                if (f) {
                    if (fgets(value, sizeof(value), f))
                        uart->pid = (uint16_t)strtoul(value, NULL, 16);
                    fclose(f);
                }
                if (uart->vid != 0 && uart->pid != 0)
                    break;
            }

            char *slash = strrchr(current, '/');
            if (!slash || slash == current)
                break;
            *slash = '\0';
        }
    }

    printf("[psdk][uart] %s VID:PID=%04X:%04X\n", uart->device,
           uart->vid, uart->pid);
}

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
    T_M300Uart *uart = _HalUart_Get(uartNum);
    struct termios tty;

    if (!uart || !uartHandle) {
        printf("[hal] unsupported UART number %d\n", uartNum);
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    /* Each auto-baud probe owns its handle. Closing a previous probe here can
     * close a descriptor which the SDK still deinitializes later. */
    T_M300UartHandle *handle = calloc(1, sizeof(*handle));
    if (handle == NULL)
        return DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED;
    handle->fd = open(uart->device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (handle->fd < 0) {
        printf("[hal] uart%d open %s failed: %s\n", uartNum, uart->device, strerror(errno));
        free(handle);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }


    struct flock lock = {
        .l_type = F_WRLCK,
        .l_whence = SEEK_SET,
        .l_start = 0,
        .l_len = 0,
    };
    if (fcntl(handle->fd, F_GETLK, &lock) != 0 || lock.l_type != F_UNLCK) {
        printf("[psdk][uart] %s is already locked by another process\n", uart->device);
        close(handle->fd);
        free(handle);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    lock.l_type = F_WRLCK;
    lock.l_pid = getpid();
    if (fcntl(handle->fd, F_SETLKW, &lock) != 0) {
        printf("[psdk][uart] lock %s failed: %s\n", uart->device, strerror(errno));
        close(handle->fd);
        free(handle);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    handle->lock_held = true;

    if (tcgetattr(handle->fd, &tty) != 0) {
        printf("[psdk][uart] tcgetattr %s failed: %s\n", uart->device, strerror(errno));
        close(handle->fd);
        free(handle);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    speed_t speed = _to_speed(baudRate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= CLOCAL;
    tty.c_cflag |= CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_iflag &= ~INPCK;
    tty.c_cflag &= ~CSTOPB;
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    tcflush(handle->fd, TCIFLUSH);
    if (tcsetattr(handle->fd, TCSANOW, &tty) != 0) {
        printf("[psdk][uart] tcsetattr %s failed: %s\n", uart->device, strerror(errno));
        close(handle->fd);
        free(handle);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    _HalUart_DetectDeviceInfo(uart);

    handle->uart = uart;
    uart->open_handles++;
    *uartHandle = (T_DjiUartHandle)(intptr_t)handle;
    printf("[hal] uart%d %s opened @ %u baud (fd=%d)\n",
           uartNum, uart->device, baudRate, handle->fd);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _HalUart_DeInit(T_DjiUartHandle uartHandle) {
    T_M300UartHandle *handle = _HalUart_Handle(uartHandle);
    if (handle == NULL)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    if (handle->lock_held) {
        struct flock lock = {
            .l_type = F_UNLCK,
            .l_whence = SEEK_SET,
            .l_start = 0,
            .l_len = 0,
        };
        (void)fcntl(handle->fd, F_SETLK, &lock);
    }
    if (close(handle->fd) != 0) {
        printf("[psdk][uart] close fd=%d failed: %s\n", handle->fd, strerror(errno));
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    if (handle->uart != NULL && handle->uart->open_handles > 0)
        handle->uart->open_handles--;
    free(handle);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _HalUart_WriteData(T_DjiUartHandle uartHandle,
                                           const uint8_t *buf, uint32_t len, uint32_t *realLen) {
    T_M300UartHandle *handle = _HalUart_Handle(uartHandle);
    if (handle == NULL || realLen == NULL)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    int fd = handle->fd;
    uint32_t total = 0;
    while (total < len) {
        ssize_t n = write(fd, buf + total, len - total);
        if (n > 0) {
            total += (uint32_t)n;
            s_uart_tx_bytes += (uint32_t)n;
            continue;
        }
        if (n < 0 && errno == EINTR)
            continue;
        printf("[psdk][uart] write failed after %u/%u bytes: %s\n", total, len,
               n < 0 ? strerror(errno) : "zero-byte write");
        s_uart_write_errors++;
        *realLen = total;
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    *realLen = total;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _HalUart_ReadData(T_DjiUartHandle uartHandle,
                                          uint8_t *buf, uint32_t len, uint32_t *realLen) {
    T_M300UartHandle *handle = _HalUart_Handle(uartHandle);
    if (handle == NULL || realLen == NULL)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    int fd = handle->fd;
    ssize_t n;
    do {
        n = read(fd, buf, len);
    } while (n < 0 && errno == EINTR);
    if (n < 0) {
        *realLen = 0;
        printf("[psdk][uart] read failed: %s\n", strerror(errno));
        s_uart_read_errors++;
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    *realLen = (uint32_t)n;
    if (n > 0)
        s_uart_rx_bytes += (uint32_t)n;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _HalUart_GetStatus(E_DjiHalUartNum uartNum, T_DjiUartStatus *status) {
    T_M300Uart *uart = _HalUart_Get(uartNum);
    if (!uart || !status)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;

    status->isConnect = true;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

#if DJI_VERSION_MINOR >= 16
static T_DjiReturnCode _HalUart_GetDeviceInfo(T_DjiHalUartDeviceInfo *deviceInfo) {
    /* Report the VID/PID of the USB endpoint carrying UART0. */
    T_M300Uart *uart = &s_uart[DJI_HAL_UART_NUM_0];
    if (!deviceInfo || uart->vid == 0 || uart->pid == 0) {
        printf("[psdk][uart] cannot report USB UART VID/PID\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    deviceInfo->vid = uart->vid;
    deviceInfo->pid = uart->pid;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
#endif

static void _HalUart_LogHandshakeStats(void) {
    printf("[psdk][uart] handshake I/O: tx=%llu bytes rx=%llu bytes write_errors=%u read_errors=%u\n",
           (unsigned long long)s_uart_tx_bytes, (unsigned long long)s_uart_rx_bytes,
           s_uart_write_errors, s_uart_read_errors);
}

/* ── PSDK init ────────────────────────────────────────────────────────── */

static int _psdk_core_init(const char *app_id, const char *app_key,
                           const char *app_license, const char *app_name,
                           const char *uart0_dev, uint32_t uart0_baud,
                           const char *uart1_dev) {
    T_DjiReturnCode rc;
    s_uart[DJI_HAL_UART_NUM_0].device = uart0_dev;
    s_uart[DJI_HAL_UART_NUM_1].device = uart1_dev;

    /* Register OSAL first (PSDK needs threads before anything else) */
    T_DjiOsalHandler osalHandler = {
        .TaskCreate = Osal_TaskCreate,
        .TaskDestroy = Osal_TaskDestroy,
        .TaskSleepMs = Osal_TaskSleepMs,
        .MutexCreate = Osal_MutexCreate,
        .MutexDestroy = Osal_MutexDestroy,
        .MutexLock = Osal_MutexLock,
        .MutexUnlock = Osal_MutexUnlock,
        .SemaphoreCreate = Osal_SemaphoreCreate,
        .SemaphoreDestroy = Osal_SemaphoreDestroy,
        .SemaphoreWait = Osal_SemaphoreWait,
        .SemaphoreTimedWait = Osal_SemaphoreTimedWait,
        .SemaphorePost = Osal_SemaphorePost,
        .GetTimeMs = Osal_GetTimeMs,
        .GetTimeUs = Osal_GetTimeUs,
        .GetRandomNum = Osal_GetRandomNum,
        .Malloc = Osal_Malloc,
        .Free = Osal_Free,
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
#if DJI_VERSION_MINOR >= 16
        .UartGetDeviceInfo = _HalUart_GetDeviceInfo,
#endif
    };
    rc = DjiPlatform_RegHalUartHandler(&uartHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[psdk] HAL UART registration failed: 0x%08llX\n", (unsigned long long)rc);
        return -1;
    }


    rc = DjiPlatform_RegHalUsbBulkHandler(&g_usbBulkHandler);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[psdk] HAL USB bulk registration failed: 0x%08llX\n",
               (unsigned long long)rc);
        return -1;
    }

    rc = DjiLogger_AddConsole(&s_psdk_debug_console);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        printf("[psdk] debug logger registration failed: 0x%08llX\n", (unsigned long long)rc);

    /* Init PSDK core */
    T_DjiUserInfo userInfo = {0};
    strncpy(userInfo.appName, app_name, sizeof(userInfo.appName) - 1);
    strncpy(userInfo.appId, app_id, sizeof(userInfo.appId) - 1);
    strncpy(userInfo.appKey, app_key, sizeof(userInfo.appKey) - 1);
    strncpy(userInfo.appLicense, app_license, sizeof(userInfo.appLicense) - 1);
    strncpy(userInfo.developerAccount, "phanthymotus@4paradigm.com",
            sizeof(userInfo.developerAccount) - 1);
    snprintf(userInfo.baudRate, sizeof(userInfo.baudRate), "%u", uart0_baud);

    rc = DjiCore_Init(&userInfo);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        printf("[psdk] core init failed: 0x%08llX\n", (unsigned long long)rc);
        _HalUart_LogHandshakeStats();
        return -1;
    }


    rc = DjiCore_SetAlias("PSDK_APPALIAS");
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        printf("[psdk] set alias failed: 0x%08llX\n", (unsigned long long)rc);
    T_DjiFirmwareVersion firmwareVersion = { .majorVersion = 1, .minorVersion = 0,
                                              .modifyVersion = 0, .debugVersion = 0 };
    rc = DjiCore_SetFirmwareVersion(firmwareVersion);
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        printf("[psdk] set firmware version failed: 0x%08llX\n", (unsigned long long)rc);
    rc = DjiCore_SetSerialNumber("PSDK12345678XX");
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        printf("[psdk] set serial number failed: 0x%08llX\n", (unsigned long long)rc);


    printf("[psdk] core initialized (app=%s, id=%s)\n", app_name, app_id);
    return 0;
}
#endif

static void _init_modules(void) {
    telemetry_init();
    flight_ctrl_init();
    gimbal_mgr_init();
    waypoint_init();
    perception_init();
    speaker_init();
    time_sync_init();
}

#ifdef PSDK_ENABLED
typedef struct {
    const char *app_id;
    const char *app_key;
    const char *app_license;
    const char *app_name;
    const char *uart0_dev;
    uint32_t uart0_baud;
    const char *uart1_dev;
} T_PsdkStartArgs;

static void *_psdk_start_thread(void *arg) {
    T_PsdkStartArgs *args = (T_PsdkStartArgs *)arg;
    if (_psdk_core_init(args->app_id, args->app_key, args->app_license,
                        args->app_name, args->uart0_dev, args->uart0_baud,
                        args->uart1_dev) != 0) {
        s_psdk_state = -1;
        printf("[psdk_bridge] PSDK core init failed; IPC remains available for status/errors\n");
        return NULL;
    }
    T_DjiReturnCode rc = DjiCore_ApplicationStart();
    if (rc != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_psdk_state = -1;
        printf("[psdk_bridge] application start failed: 0x%08llX\n",
               (unsigned long long)rc);
        return NULL;
    }

    (void)Osal_TaskSleepMs(5000);
    if (liveview_init() != 0) {
        printf("[psdk] liveview init unavailable; camera_stream will return an error without crashing\n");
    }

    _init_modules();
    hms_init();
    s_psdk_state = 1;
    printf("[psdk_bridge] PSDK modules ready\n");
    return NULL;
}
#endif

/* ── IPC Command Dispatcher ─────────────────────────────────────────────── */

static const char *_perception_source_from_json(const char *raw_json) {
    static const char *sources[] = {
        "front_left", "front_right",
        "back_left", "back_right",
        "left_left", "left_right",
        "right_left", "right_right",
        "up_left", "up_right",
        "down_left", "down_right",
    };
    static const char *legacy_directions[] = {
        "front", "back", "left", "right", "up", "down",
    };

    /* The IPC client sends the selected source as a JSON string. Match the
     * complete quoted value so e.g. front_left cannot be confused with left. */
    for (size_t i = 0; i < sizeof(sources) / sizeof(sources[0]); ++i) {
        char needle[64];
        snprintf(needle, sizeof(needle), "\"%s\"", sources[i]);
        if (strstr(raw_json, needle)) return sources[i];
    }
    /* Backward compatibility for clients that still send direction=front. */
    for (size_t i = 0; i < sizeof(legacy_directions) / sizeof(legacy_directions[0]); ++i) {
        char needle[32];
        snprintf(needle, sizeof(needle), "\"%s\"", legacy_directions[i]);
        if (strstr(raw_json, needle)) {
            static char legacy_source[32];
            snprintf(legacy_source, sizeof(legacy_source), "%s_left", legacy_directions[i]);
            return legacy_source;
        }
    }
    return "front_left";
}

static int _dispatch_cmd(const char *raw_json, const char *unused,
                         char *result, size_t result_size) {


    /* Do not run any PSDK API before its asynchronous UART handshake has
     * completed.  This is a live bridge response, not a mock fallback. */
    if (s_psdk_state != 1) {
        snprintf(result, result_size,
                 "{\"ok\":false,\"error\":\"PSDK %s\"}",
                 s_psdk_state < 0 ? "initialization failed" : "initializing");
        return -1;
    }
    const char *camera = strstr(raw_json, "\"payload3\"") ? "payload3" :
                         (strstr(raw_json, "\"payload2\"") ? "payload2" : "payload1");

    /* Telemetry */
    if (strstr(raw_json, "\"get_telemetry\"")) {
        char telem[4096];
        telemetry_get_json(telem, sizeof(telem));
        snprintf(result, result_size, "{\"ok\":true,\"data\":%s}", telem);
        return 0;
    }

    /* Flight control */
    if (strstr(raw_json, "\"takeoff\"")) {
        int64_t r = flight_ctrl_takeoff();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"land\"") && !strstr(raw_json, "\"confirm_land")) {
        if (strstr(raw_json, "\"auto_confirm\"")) {
            int64_t r = flight_ctrl_land_auto_confirm();
            if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0,\"message\":\"Landing completed (auto-confirmed)\"}}");
            else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        } else {
            int64_t r = flight_ctrl_land();
            if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0,\"message\":\"Landing initiated. Please confirm on RC when prompted.\"}}");
            else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        }
        return 0;
    }
    if (strstr(raw_json, "\"confirm_landing\"")) {
        int64_t r = flight_ctrl_confirm_landing();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0,\"message\":\"Landing confirmed\"}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"go_home\"") && !strstr(raw_json, "\"cancel_go_home\"")) {
        int64_t r = flight_ctrl_go_home();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"cancel_go_home\"")) {
        int64_t r = flight_ctrl_cancel_go_home();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"emergency_brake\"")) {
        int64_t r = flight_ctrl_emergency_brake();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"rotate_start\"") && !strstr(raw_json, "\"slow_rotate_start\"")) {
        int64_t r = flight_ctrl_turn_on_motors();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"rotate_stop\"") && !strstr(raw_json, "\"slow_rotate_stop\"")) {
        int64_t r = flight_ctrl_turn_off_motors();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"slow_rotate_start\"")) {
        int64_t r = flight_ctrl_slow_rotate_start();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"slow_rotate_stop\"")) {
        int64_t r = flight_ctrl_slow_rotate_stop();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"obtain_joystick_authority\"")) {
        int64_t r = flight_ctrl_obtain_authority();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"release_joystick_authority\"")) {
        int64_t r = flight_ctrl_release_authority();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"joystick_move\"")) {
        float vx = 0, vy = 0, vz = 0, vyaw = 0, duration = 1;
        const char *p;
        if ((p = strstr(raw_json, "\"vx\""))) { p = strchr(p, ':'); if (p) vx = (float)atof(p+1); }
        if ((p = strstr(raw_json, "\"vy\""))) { p = strchr(p, ':'); if (p) vy = (float)atof(p+1); }
        if ((p = strstr(raw_json, "\"vz\""))) { p = strchr(p, ':'); if (p) vz = (float)atof(p+1); }
        if ((p = strstr(raw_json, "\"vyaw\""))) { p = strchr(p, ':'); if (p) vyaw = (float)atof(p+1); }
        if ((p = strstr(raw_json, "\"duration\""))) { p = strchr(p, ':'); if (p) duration = (float)atof(p+1); }
        int64_t r = flight_ctrl_joystick_move(vx, vy, vz, vyaw, duration);
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0,\"message\":\"Moving (duration=%.1fs)\"}}",  duration);
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"stop_move\"")) {
        int64_t r = flight_ctrl_stop_move();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0,\"message\":\"Stopped, hovering\"}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"set_home\"")) {
        double lat = 0, lon = 0;
        const char *p;
        if ((p = strstr(raw_json, "\"lat\""))) { p = strchr(p, ':'); if (p) lat = atof(p+1); }
        if ((p = strstr(raw_json, "\"lon\""))) { p = strchr(p, ':'); if (p) lon = atof(p+1); }
        int64_t r = flight_ctrl_set_home(lat, lon);
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"set_obstacle_avoidance\"")) {
        int enabled = strstr(raw_json, "\"on\"") ? 1 : 0;
        const char *dir = "all";
        if (strstr(raw_json, "\"horizontal\"")) dir = "horizontal";
        else if (strstr(raw_json, "\"upward\"")) dir = "upward";
        else if (strstr(raw_json, "\"downward\"")) dir = "downward";
        int64_t r = flight_ctrl_set_obstacle_avoidance(enabled, dir);
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }

    /* Camera */
    if (strstr(raw_json, "\"take_photo\"")) {
        int r = camera_mgr_take_photo(camera, strstr(raw_json, "\"burst\"") ? "burst" : (strstr(raw_json, "\"interval\"") ? "interval" : "single"));
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"start_video\"")) {
        int r = camera_mgr_start_video(camera);
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"stop_video\"")) {
        int r = camera_mgr_stop_video(camera);
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"set_camera_mode\"")) {
        int r = camera_mgr_set_mode(camera, strstr(raw_json, "\"video\"") ? "video" : "photo");
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"set_zoom\"")) {
        float factor = 1.0f;
        const char *fp = strstr(raw_json, "\"factor\"");
        if (fp) {
            fp = strchr(fp, ':');
            if (fp) factor = (float)atof(fp + 1);
        }
        int r = camera_mgr_set_zoom(camera, factor);
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"set_focus\"")) {
        float x = 0.5f, y = 0.5f;
        const char *xp = strstr(raw_json, "\"x\"");
        const char *yp = strstr(raw_json, "\"y\"");
        if (xp) { xp = strchr(xp, ':'); if (xp) x = (float)atof(xp + 1); }
        if (yp) { yp = strchr(yp, ':'); if (yp) y = (float)atof(yp + 1); }
        int r = camera_mgr_set_focus(camera, x, y);
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"set_exposure\"")) {
        int iso = 0; float aperture = 0, shutter = 0, ev = 0;
        const char *p;
        if ((p = strstr(raw_json, "\"iso\""))) { p = strchr(p, ':'); if (p) iso = atoi(p+1); }
        if ((p = strstr(raw_json, "\"aperture\""))) { p = strchr(p, ':'); if (p) aperture = (float)atof(p+1); }
        if ((p = strstr(raw_json, "\"shutter_speed\""))) { p = strchr(p, ':'); if (p) shutter = (float)atof(p+1); }
        if ((p = strstr(raw_json, "\"ev\""))) { p = strchr(p, ':'); if (p) ev = (float)atof(p+1); }
        int r = camera_mgr_set_exposure(camera, iso, aperture, shutter, ev);
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"get_storage\"")) {
        char storage[256];
        int r = camera_mgr_get_storage(camera, storage, sizeof(storage));
        snprintf(result, result_size, "{\"ok\":%s,\"data\":%s}", r == 0 ? "true" : "false", storage);
        return 0;
    }
    if (strstr(raw_json, "\"ir_temp_point\"")) {
        float x = 0.5f, y = 0.5f;
        const char *p;
        if ((p = strstr(raw_json, "\"x\""))) { p = strchr(p, ':'); if (p) x = (float)atof(p+1); }
        if ((p = strstr(raw_json, "\"y\""))) { p = strchr(p, ':'); if (p) y = (float)atof(p+1); }
        char buf[256];
        int r = camera_mgr_ir_temp_point(camera, x, y, buf, sizeof(buf));
        snprintf(result, result_size, "{\"ok\":%s,\"data\":%s}", r == 0 ? "true" : "false", buf);
        return 0;
    }
    if (strstr(raw_json, "\"ir_temp_area\"")) {
        float ltx = 0.25f, lty = 0.25f, rbx = 0.75f, rby = 0.75f;
        const char *p;
        if ((p = strstr(raw_json, "\"ltx\""))) { p = strchr(p, ':'); if (p) ltx = (float)atof(p+1); }
        if ((p = strstr(raw_json, "\"lty\""))) { p = strchr(p, ':'); if (p) lty = (float)atof(p+1); }
        if ((p = strstr(raw_json, "\"rbx\""))) { p = strchr(p, ':'); if (p) rbx = (float)atof(p+1); }
        if ((p = strstr(raw_json, "\"rby\""))) { p = strchr(p, ':'); if (p) rby = (float)atof(p+1); }
        char buf[256];
        int r = camera_mgr_ir_temp_area(camera, ltx, lty, rbx, rby, buf, sizeof(buf));
        snprintf(result, result_size, "{\"ok\":%s,\"data\":%s}", r == 0 ? "true" : "false", buf);
        return 0;
    }

    /* Gimbal */
    if (strstr(raw_json, "\"gimbal_rotate\"")) {
        float pitch = 0, yaw = 0, roll = 0, duration = 1.0f;
        const char *mode = "absolute";
        const char *p;
        if ((p = strstr(raw_json, "\"pitch\""))) { p = strchr(p, ':'); if (p) pitch = (float)atof(p+1); }
        if ((p = strstr(raw_json, "\"yaw\""))) { p = strchr(p, ':'); if (p) yaw = (float)atof(p+1); }
        if ((p = strstr(raw_json, "\"roll\""))) { p = strchr(p, ':'); if (p) roll = (float)atof(p+1); }
        if ((p = strstr(raw_json, "\"duration\""))) { p = strchr(p, ':'); if (p) duration = (float)atof(p+1); }
        if (strstr(raw_json, "\"relative\"")) mode = "relative";
        int r = gimbal_mgr_rotate(pitch, yaw, roll, mode, duration);
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"gimbal_set_mode\"")) {
        const char *mode = "free";
        if (strstr(raw_json, "\"follow\"")) mode = "follow";
        else if (strstr(raw_json, "\"fpv\"")) mode = "fpv";
        int r = gimbal_mgr_set_mode(mode);
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"gimbal_reset\"")) {
        int r = gimbal_mgr_reset();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
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
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"waypoint_pause\"")) {
        int r = waypoint_pause();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"waypoint_resume\"")) {
        int r = waypoint_resume();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
        return 0;
    }
    if (strstr(raw_json, "\"waypoint_stop\"")) {
        int r = waypoint_stop();
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        else { char eb[256]; error_code_to_json((uint64_t)r, eb, sizeof(eb)); snprintf(result, result_size, "{\"ok\":false,\"data\":%s}", eb); }
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
    if (strstr(raw_json, "\"hms_inject\"")) {
        uint32_t code = 0;
        uint8_t level = 0;
        const char *p;
        if ((p = strstr(raw_json, "\"code\""))) { p = strchr(p, ':'); if (p) code = (uint32_t)strtoul(p + 1, NULL, 16); }
        if ((p = strstr(raw_json, "\"level\""))) { p = strchr(p, ':'); if (p) level = (uint8_t)atoi(p + 1); }
        int r = hms_inject_error(code, level);
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0,\"code\":\"0x%08X\"}}", code);
        else snprintf(result, result_size, "{\"ok\":false,\"error\":\"inject failed\"}");
        return 0;
    }
    if (strstr(raw_json, "\"hms_eliminate\"")) {
        uint32_t code = 0;
        const char *p;
        if ((p = strstr(raw_json, "\"code\""))) { p = strchr(p, ':'); if (p) code = (uint32_t)strtoul(p + 1, NULL, 16); }
        int r = hms_eliminate_error(code);
        if (r == 0) snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0,\"code\":\"0x%08X\"}}", code);
        else snprintf(result, result_size, "{\"ok\":false,\"error\":\"eliminate not found\"}");
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
        const char *cam = "fpv";
        int r = liveview_start(cam, NULL);
        if (r == 0) {
            snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0,\"camera\":\"%s\"}}", cam);
        } else {
            snprintf(result, result_size, "{\"ok\":false,\"error\":\"failed to start liveview for %s\"}", cam);
        }
        return 0;
    }
    if (strstr(raw_json, "\"stop_liveview\"")) {
        liveview_stop("fpv");
        snprintf(result, result_size, "{\"ok\":true,\"data\":{\"ret\":0}}");
        return 0;
    }

    /* Perception */
    if (strstr(raw_json, "\"start_perception\"")) {
        const char *source = _perception_source_from_json(raw_json);
        int r = perception_start(source, NULL);
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d,\"source\":\"%s\"}}", r == 0 ? "true" : "false", r, source);
        return 0;
    }
    if (strstr(raw_json, "\"stop_perception\"")) {
        const char *source = _perception_source_from_json(raw_json);
        int r = perception_stop(source);
        snprintf(result, result_size, "{\"ok\":%s,\"data\":{\"ret\":%d,\"source\":\"%s\"}}", r == 0 ? "true" : "false", r, source);
        return 0;
    }

    /* Aircraft info */
    if (strstr(raw_json, "\"get_aircraft_info\"")) {
        T_DjiAircraftInfoBaseInfo baseInfo = {0};
        T_DjiAircraftVersion version = {0};
        bool connected = false;
        T_DjiReturnCode base_rc = DjiAircraftInfo_GetBaseInfo(&baseInfo);
        T_DjiReturnCode connected_rc = DjiAircraftInfo_GetConnectionStatus(&connected);
        T_DjiReturnCode version_rc = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        bool version_attempted = true;

        const char *type_name = "Unknown";
        switch (baseInfo.aircraftType) {
            case 44: type_name = "Matrice 200 V2"; break;
            case 45: type_name = "Matrice 210 V2"; break;
            case 46: type_name = "Matrice 210 RTK V2"; break;
            case 60: type_name = "Matrice 300 RTK"; break;
            case 67: type_name = "Matrice 30"; break;
            case 68: type_name = "Matrice 30T"; break;
            case 77: type_name = "Mavic 3E"; break;
            case 78: type_name = "FlyCart 30"; break;
            case 79: type_name = "Mavic 3T"; break;
            case 80: type_name = "Mavic 3TA"; break;
            case 89: type_name = "Matrice 350 RTK"; break;
            case 91: type_name = "Matrice 3D"; break;
            case 93: type_name = "Matrice 3TD"; break;
            default: break;
        }
        const char *series_name = "Unknown";
        switch (baseInfo.aircraftSeries) {
            case 1: series_name = "M200 V2"; break;
            case 2: series_name = "M300"; break;
            case 3: series_name = "M30"; break;
            case 4: series_name = "M3"; break;
            case 5: series_name = "M350"; break;
            case 6: series_name = "M3D"; break;
            case 7: series_name = "FC30"; break;
            default: break;
        }
        const char *mount_name = "Unknown";
        switch (baseInfo.mountPosition) {
            case 1: mount_name = "Payload Port No.1"; break;
            case 2: mount_name = "Payload Port No.2"; break;
            case 3: mount_name = "Payload Port No.3"; break;
            case 4: mount_name = "Extension Port"; break;
            case 5: mount_name = "Extension Lite Port"; break;
            case 6: mount_name = "Extension Port V2 No.5 (USB Hub 1)"; break;
            case 7: mount_name = "Extension Port V2 No.6 (USB Hub 2)"; break;
            case 8: mount_name = "Extension Port V2 No.7 (USB Hub 3)"; break;
            default: break;
        }

        if (baseInfo.aircraftType == 60 && baseInfo.mountPosition == 4) {
            version_attempted = false;
        } else {
            version_rc = DjiAircraftInfo_GetAircraftVersion(&version);
        }

        if (version_attempted && version_rc == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            snprintf(result, result_size,
                "{\"ok\":true,\"data\":{\"aircraft_type\":\"%s\","
                "\"aircraft_series\":\"%s\","
                "\"firmware_version\":\"%d.%d.%d.%d\","
                "\"mount_position\":\"%s\",\"connected\":%s,"
                "\"base_info_ret\":%llu,\"connection_ret\":%llu}}",
                type_name, series_name,
                version.majorVersion, version.minorVersion, version.modifyVersion, version.debugVersion,
                mount_name, connected ? "true" : "false",
                (unsigned long long)base_rc,
                (unsigned long long)connected_rc);
        } else {
            snprintf(result, result_size,
                "{\"ok\":true,\"data\":{\"aircraft_type\":\"%s\","
                "\"aircraft_series\":\"%s\","
                "\"mount_position\":\"%s\",\"connected\":%s,"
                "\"base_info_ret\":%llu,\"connection_ret\":%llu}}",
                type_name, series_name,
                mount_name, connected ? "true" : "false",
                (unsigned long long)base_rc,
                (unsigned long long)connected_rc);
        }
        return 0;
    }

    /* Time sync */
    if (strstr(raw_json, "\"get_aircraft_time\"")) {
        char time_buf[256];
        int r = time_sync_get_aircraft_time(time_buf, sizeof(time_buf));
        snprintf(result, result_size, "{\"ok\":%s,\"data\":%s}", r == 0 ? "true" : "false", time_buf);
        return 0;
    }
    if (strstr(raw_json, "\"sync_clock\"")) {
        char time_buf[256];
        int r = time_sync_sync_clock(time_buf, sizeof(time_buf));
        snprintf(result, result_size, "{\"ok\":%s,\"data\":%s}", r == 0 ? "true" : "false", time_buf);
        return 0;
    }

    /* Unknown command */
    snprintf(result, result_size, "{\"ok\":false,\"error\":\"unknown command\"}");
    return -1;
}

/* ── Main ───────────────────────────────────────────────────────────────── */

int main(int argc, char *argv[]) {
    /* IPC peers may disconnect while a stream command is in flight.  Never
     * let a write to that socket terminate the bridge process. */
    signal(SIGPIPE, SIG_IGN);
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    const char *socket_path = "/tmp/psdk_bridge.sock";
    const char *app_id = "";
    const char *app_key = "";
    const char *app_license = "";
    const char *app_name = "PhanthyMotus";
    const char *uart0_dev = "/dev/ttyTHS0";
    uint32_t uart0_baud = 460800;
    const char *uart1_dev = "/dev/ttyACM0";

    if (argc >= 2) socket_path = argv[1];
    if (argc >= 3) app_id = argv[2];
    if (argc >= 4) app_key = argv[3];
    if (argc >= 5) app_license = argv[4];
    if (argc >= 6) uart0_dev = argv[5];
    if (argc >= 7) uart0_baud = (uint32_t)atoi(argv[6]);
    if (argc >= 8) uart1_dev = argv[7];

    printf("=== DJI PSDK Bridge for Matrice 300 RTK ===\n");
    printf("  Socket: %s\n", socket_path);
    printf("  UART0:  %s @ %u baud\n", uart0_dev, uart0_baud);
    printf("  UART1:  %s @ 921600 baud\n", uart1_dev);

    signal(SIGINT, _signal_handler);
    signal(SIGTERM, _signal_handler);



    if (ipc_init(socket_path) != 0) {
        printf("[psdk_bridge] IPC init failed, exiting\n");
        return 1;
    }
    ipc_set_handler(_dispatch_cmd);

#ifdef PSDK_ENABLED
    T_PsdkStartArgs start_args = {
        .app_id = app_id,
        .app_key = app_key,
        .app_license = app_license,
        .app_name = app_name,
        .uart0_dev = uart0_dev,
        .uart0_baud = uart0_baud,
        .uart1_dev = uart1_dev,
    };
    pthread_t psdk_thread;
    if (pthread_create(&psdk_thread, NULL, _psdk_start_thread, &start_args) != 0) {
        printf("[psdk_bridge] failed to start PSDK initialization thread\n");
        ipc_cleanup();
        return 1;
    }
    pthread_detach(psdk_thread);
#else
    printf("[psdk_bridge] Running in STUB mode (no PSDK)\n");
    _init_modules();
    s_psdk_state = 1;
#endif

    printf("[psdk_bridge] IPC ready, entering main loop\n");

    /* Main event loop */
    while (s_running) {
        ipc_process();
        usleep(1000);  /* 1ms — avoids busy-wait */
    }

    /* Cleanup */
    printf("[psdk_bridge] Shutting down...\n");
    if (s_psdk_state == 1) {
        hms_cleanup();
        speaker_cleanup();
        perception_cleanup();
        waypoint_cleanup();
        liveview_cleanup();
        gimbal_mgr_cleanup();
        camera_mgr_cleanup();
        flight_ctrl_cleanup();
        telemetry_cleanup();
    }
    ipc_cleanup();
#ifdef PSDK_ENABLED
    if (s_psdk_state == 1)
        DjiCore_DeInit();
#endif

    printf("[psdk_bridge] Done.\n");
    return 0;
}
