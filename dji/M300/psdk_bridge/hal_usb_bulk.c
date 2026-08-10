#include "hal_usb_bulk.h"

#ifdef PSDK_ENABLED

#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>

#include "dji_platform.h"

/*
 * M300 presents itself to the Jetson as a USB device (VID:PID 2ca3:001f).
 * PSDK supplies the interface number and endpoints for each stream channel;
 * the host must claim those interfaces with libusb.  FunctionFS is the
 * inverse topology (Jetson acting as USB device) and cannot receive M300
 * liveview data.
 */
typedef struct {
    libusb_device_handle *device;
    T_DjiHalUsbBulkInfo info;
} T_UsbBulkHandle;

static T_DjiReturnCode _UsbBulk_Init(T_DjiHalUsbBulkInfo info,
                                     T_DjiUsbBulkHandle *out_handle) {
    T_UsbBulkHandle *handle;
    int rc;

    if (!out_handle || !info.isUsbHost)
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;

    /* The M300's PSDK 3.11 liveview request reports vendor interface 6
     * (0x05/0x87), but the aircraft exposes the actual H.264 bulk channel on
     * interface 7 (0x06/0x88).  The official on-board PSDK 3.8 demo uses
     * interface 7 successfully.  Keep the SDK-provided mapping for every
     * other channel; only remap this known M300 liveview tuple. */
    if (info.vid == 0x2ca3 && info.pid == 0x001f &&
        info.channelInfo.interfaceNum == 6 &&
        info.channelInfo.endPointOut == 0x05 && info.channelInfo.endPointIn == 0x87) {
        info.channelInfo.interfaceNum = 7;
        info.channelInfo.endPointOut = 0x06;
        info.channelInfo.endPointIn = 0x88;
        printf("[usb_bulk] remapped M300 liveview to interface=7 in=0x88 out=0x06\n");
    }

    handle = calloc(1, sizeof(*handle));
    if (!handle)
        return DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED;

    rc = libusb_init(NULL);
    if (rc != LIBUSB_SUCCESS)
        goto fail;

    handle->device = libusb_open_device_with_vid_pid(NULL, info.vid, info.pid);
    if (!handle->device) {
        printf("[usb_bulk] DJI USB %04x:%04x not found\n", info.vid, info.pid);
        goto fail_exit;
    }

    /* The M300 liveview endpoints are vendor-specific, but auto-detach keeps
     * this safe on boards where a generic driver has bound the interface. */
    (void)libusb_set_auto_detach_kernel_driver(handle->device, 1);
    rc = libusb_claim_interface(handle->device, info.channelInfo.interfaceNum);
    if (rc != LIBUSB_SUCCESS) {
        printf("[usb_bulk] claim interface %u failed: %s\n",
               info.channelInfo.interfaceNum, libusb_error_name(rc));
        goto fail_close;
    }

    handle->info = info;
    *out_handle = handle;
    printf("[usb_bulk] host %04x:%04x interface=%u in=0x%02x out=0x%02x\n",
           info.vid, info.pid, info.channelInfo.interfaceNum,
           info.channelInfo.endPointIn, info.channelInfo.endPointOut);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

fail_close:
    libusb_close(handle->device);
fail_exit:
    libusb_exit(NULL);
fail:
    free(handle);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
}

static T_DjiReturnCode _UsbBulk_DeInit(T_DjiUsbBulkHandle opaque) {
    T_UsbBulkHandle *handle = opaque;
    if (!handle)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    libusb_release_interface(handle->device, handle->info.channelInfo.interfaceNum);
    libusb_close(handle->device);
    libusb_exit(NULL);
    free(handle);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _UsbBulk_Transfer(T_DjiUsbBulkHandle opaque, uint8_t endpoint,
                                         uint8_t *data, uint32_t len, uint32_t *real_len,
                                         unsigned int timeout_ms) {
    T_UsbBulkHandle *handle = opaque;
    int actual_len = 0;
    int rc;

    if (!handle || !data || !real_len)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    rc = libusb_bulk_transfer(handle->device, endpoint, data, (int)len,
                              &actual_len, timeout_ms);
    if (rc != LIBUSB_SUCCESS) {
        *real_len = 0;
        if (rc != LIBUSB_ERROR_TIMEOUT)
            printf("[usb_bulk] transfer endpoint 0x%02x failed: %s\n",
                   endpoint, libusb_error_name(rc));
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }
    *real_len = (uint32_t)actual_len;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode _UsbBulk_WriteData(T_DjiUsbBulkHandle handle,
                                          const uint8_t *data, uint32_t len,
                                          uint32_t *real_len) {
    T_UsbBulkHandle *bulk = handle;
    if (!bulk)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    return _UsbBulk_Transfer(handle, (uint8_t)bulk->info.channelInfo.endPointOut,
                             (uint8_t *)data, len, real_len, 50);
}

static T_DjiReturnCode _UsbBulk_ReadData(T_DjiUsbBulkHandle handle,
                                         uint8_t *data, uint32_t len,
                                         uint32_t *real_len) {
    T_UsbBulkHandle *bulk = handle;
    if (!bulk)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    return _UsbBulk_Transfer(handle, (uint8_t)bulk->info.channelInfo.endPointIn,
                             data, len, real_len, 0);
}

static T_DjiReturnCode _UsbBulk_GetDeviceInfo(T_DjiHalUsbBulkDeviceInfo *info) {
    /* This callback is only used when the payload computer is a USB gadget.
     * M300 is host-connected, so the SDK passes the aircraft VID/PID to Init. */
    if (!info)
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiHalUsbBulkHandler g_usbBulkHandler = {
    .UsbBulkInit = _UsbBulk_Init,
    .UsbBulkDeInit = _UsbBulk_DeInit,
    .UsbBulkWriteData = _UsbBulk_WriteData,
    .UsbBulkReadData = _UsbBulk_ReadData,
    .UsbBulkGetDeviceInfo = _UsbBulk_GetDeviceInfo,
};

#endif
