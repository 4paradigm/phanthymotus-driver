#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stddef.h>



typedef struct {
    uint16_t vid;   /* USB Vendor ID (DJI: 0x2CA3) */
    uint16_t pid;   /* USB Product ID */
} T_HalUartDeviceInfo;


int HalUart_Init(const char *device, uint32_t baudRate);


int HalUart_Write(const uint8_t *data, uint32_t len);


int HalUart_Read(uint8_t *buf, uint32_t len, uint32_t timeout_ms);


int HalUart_GetDeviceInfo(T_HalUartDeviceInfo *info);

/* Close UART and release resources. */
void HalUart_Close(void);

#endif /* HAL_UART_H */
