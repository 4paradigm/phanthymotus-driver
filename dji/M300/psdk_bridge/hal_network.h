#ifndef HAL_NETWORK_H
#define HAL_NETWORK_H

#include <stdint.h>



typedef struct {
    char ifname[32];     /* Network interface name (e.g., "usb0") */
    char ip_addr[16];    /* IP address assigned to interface */
    uint32_t mtu;        /* MTU size */
} T_HalNetworkInfo;

/* Initialize network HAL — detect USB-Ethernet interface from E-Port.
 * @return 0 on success, -1 if no USB-Ethernet interface found */
int HalNetwork_Init(void);


int HalNetwork_GetInfo(T_HalNetworkInfo *info);

/* Get the interface name for PSDK network operations.
 * @return pointer to static string with interface name */
const char *HalNetwork_GetInterfaceName(void);

/* Cleanup network HAL. */
void HalNetwork_Cleanup(void);

#endif /* HAL_NETWORK_H */
