#ifndef HAL_NETWORK_H
#define HAL_NETWORK_H

#include <stdint.h>

/**
 * HAL Network — USB-Ethernet (RNDIS/NCM) for PSDK high-bandwidth data.
 *
 * E-Port provides a USB network interface (RNDIS/CDC-NCM) for:
 *   - Camera liveview streaming (H.264)
 *   - Perception image data
 *   - High-speed data transmission channel
 *
 * On Jetson NX, this typically appears as usb0 or eth1.
 */

typedef struct {
    char ifname[32];     /* Network interface name (e.g., "usb0") */
    char ip_addr[16];    /* IP address assigned to interface */
    uint32_t mtu;        /* MTU size */
} T_HalNetworkInfo;

/* Initialize network HAL — detect USB-Ethernet interface from E-Port.
 * @return 0 on success, -1 if no USB-Ethernet interface found */
int HalNetwork_Init(void);

/* Get network interface info.
 * @param info  Output struct with interface details
 * @return 0 on success */
int HalNetwork_GetInfo(T_HalNetworkInfo *info);

/* Get the interface name for PSDK network operations.
 * @return pointer to static string with interface name */
const char *HalNetwork_GetInterfaceName(void);

/* Cleanup network HAL. */
void HalNetwork_Cleanup(void);

#endif /* HAL_NETWORK_H */
