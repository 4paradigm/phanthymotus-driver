#include "hal_usb_bulk.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#ifdef PSDK_ENABLED
#include "dji_platform.h"

/*
 * USB Bulk HAL for DJI PSDK — FunctionFS endpoints.
 *
 * Each bulk channel has:
 *   ep1 = IN  (device→host, i.e., Jetson→Aircraft)
 *   ep2 = OUT (host→device, i.e., Aircraft→Jetson)
 *
 * For liveview/perception, we READ from ep1 (data coming from aircraft).
 */

#define USB_BULK_EP_IN_FMT  "/dev/usb-ffs/bulk%d/ep1"
#define USB_BULK_EP_OUT_FMT "/dev/usb-ffs/bulk%d/ep2"

typedef struct {
    int fd_in;
    int fd_out;
    int channel;  /* 1, 2, or 3 */
} BulkHandle_t;

static T_DjiReturnCode _UsbBulk_Init(T_DjiHalUsbBulkInfo usbBulkInfo,
                                      T_DjiUsbBulkHandle *usbBulkHandle) {
    /* Determine channel from endpoint address */
    int channel;
    if (usbBulkInfo.channelInfo.endPointIn == 0x81) channel = 1;
    else if (usbBulkInfo.channelInfo.endPointIn == 0x82) channel = 2;
    else if (usbBulkInfo.channelInfo.endPointIn == 0x83) channel = 3;
    else channel = 1;  /* fallback */
    char ep_in[64], ep_out[64];
    snprintf(ep_in, sizeof(ep_in), USB_BULK_EP_IN_FMT, channel);
    snprintf(ep_out, sizeof(ep_out), USB_BULK_EP_OUT_FMT, channel);

    BulkHandle_t *h = (BulkHandle_t *)malloc(sizeof(BulkHandle_t));
    if (!h) return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;

    h->channel = channel;
    h->fd_in = open(ep_in, O_RDWR);
    if (h->fd_in < 0) {
        printf("[usb_bulk] open %s failed: %s\n", ep_in, strerror(errno));
        free(h);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    h->fd_out = open(ep_out, O_RDWR);
    if (h->fd_out < 0) {
        printf("[usb_bulk] open %s failed: %s\n", ep_out, strerror(errno));
        close(h->fd_in);
        free(h);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    *usbBulkHandle = (T_DjiUsbBulkHandle)h;
    printf("[usb_bulk] channel %d opened (in=%s, out=%s)\n", channel, ep_in, ep_out);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _UsbBulk_DeInit(T_DjiUsbBulkHandle usbBulkHandle) {
    BulkHandle_t *h = (BulkHandle_t *)usbBulkHandle;
    if (h) {
        if (h->fd_in >= 0) close(h->fd_in);
        if (h->fd_out >= 0) close(h->fd_out);
        free(h);
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _UsbBulk_WriteData(T_DjiUsbBulkHandle usbBulkHandle,
                                           const uint8_t *buf, uint32_t len, uint32_t *realLen) {
    BulkHandle_t *h = (BulkHandle_t *)usbBulkHandle;
    ssize_t n = write(h->fd_out, buf, len);
    *realLen = (n > 0) ? (uint32_t)n : 0;
    return (n >= 0) ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS : DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
}

static T_DjiReturnCode _UsbBulk_ReadData(T_DjiUsbBulkHandle usbBulkHandle,
                                          uint8_t *buf, uint32_t len, uint32_t *realLen) {
    BulkHandle_t *h = (BulkHandle_t *)usbBulkHandle;
    ssize_t n = read(h->fd_in, buf, len);
    *realLen = (n > 0) ? (uint32_t)n : 0;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _UsbBulk_GetDeviceInfo(T_DjiHalUsbBulkDeviceInfo *deviceInfo) {
    deviceInfo->vid = 0x2CA3;
    deviceInfo->pid = 0xF001;
    /* Channel endpoint addresses */
    deviceInfo->channelInfo[0].endPointIn = 0x81;
    deviceInfo->channelInfo[0].endPointOut = 0x01;
    deviceInfo->channelInfo[1].endPointIn = 0x82;
    deviceInfo->channelInfo[1].endPointOut = 0x02;
    deviceInfo->channelInfo[2].endPointIn = 0x83;
    deviceInfo->channelInfo[2].endPointOut = 0x03;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Public: register USB Bulk HAL with PSDK */
T_DjiHalUsbBulkHandler g_usbBulkHandler = {
    .UsbBulkInit = _UsbBulk_Init,
    .UsbBulkDeInit = _UsbBulk_DeInit,
    .UsbBulkWriteData = _UsbBulk_WriteData,
    .UsbBulkReadData = _UsbBulk_ReadData,
    .UsbBulkGetDeviceInfo = _UsbBulk_GetDeviceInfo,
};

#endif /* PSDK_ENABLED */
