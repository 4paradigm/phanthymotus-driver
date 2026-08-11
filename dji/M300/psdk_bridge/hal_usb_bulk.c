#include "hal_usb_bulk.h"

#ifdef PSDK_ENABLED

#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>

#include "dji_platform.h"


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
    /* hzhy leaves a short settle interval after releasing the M300 bulk
     * interface.  Without it a following liveview retry can race teardown. */
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    if (osal && osal->TaskSleepMs)
        (void)osal->TaskSleepMs(100);
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
                             data, len, real_len, (unsigned int)-1);
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
