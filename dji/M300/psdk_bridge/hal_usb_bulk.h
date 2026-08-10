#ifndef HAL_USB_BULK_H
#define HAL_USB_BULK_H

#include <stdint.h>
#include <stddef.h>

#ifdef PSDK_ENABLED
#include "dji_platform.h"
extern T_DjiHalUsbBulkHandler g_usbBulkHandler;
#endif

int HalUsbBulk_Init(void);
void HalUsbBulk_Cleanup(void);

#endif
