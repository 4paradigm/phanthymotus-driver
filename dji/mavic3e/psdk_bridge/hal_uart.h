#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stddef.h>

/**
 * HAL UART — Linux termios implementation for DJI PSDK.
 *
 * Provides serial communication with Mavic 3E via E-Port USB serial device.
 * Device typically appears as /dev/ttyACM0 (USB CDC ACM class).
 */

typedef struct {
    uint16_t vid;   /* USB Vendor ID (DJI: 0x2CA3) */
    uint16_t pid;   /* USB Product ID */
} T_HalUartDeviceInfo;

/* Initialize UART — open device, configure baud/parity/flow.
 * @param device   Serial device path (e.g., "/dev/ttyACM0")
 * @param baudRate Baud rate (typically 921600)
 * @return 0 on success, -1 on error */
int HalUart_Init(const char *device, uint32_t baudRate);

/* Write data to UART.
 * @param data    Pointer to data buffer
 * @param len     Number of bytes to write
 * @return Number of bytes written, or -1 on error */
int HalUart_Write(const uint8_t *data, uint32_t len);

/* Read data from UART (blocking with timeout).
 * @param buf     Buffer to receive data
 * @param len     Maximum bytes to read
 * @param timeout_ms  Read timeout in milliseconds (0 = non-blocking)
 * @return Number of bytes read, or -1 on error/timeout */
int HalUart_Read(uint8_t *buf, uint32_t len, uint32_t timeout_ms);

/* Get USB device info (VID/PID) for E-Port identification.
 * Called by PSDK to validate connection to correct aircraft.
 * @param info    Output device info struct
 * @return 0 on success */
int HalUart_GetDeviceInfo(T_HalUartDeviceInfo *info);

/* Close UART and release resources. */
void HalUart_Close(void);

#endif /* HAL_UART_H */
