#include "hal_network.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>

/*
 * E-Port USB-Ethernet detection strategy:
 *
 * 1. Scan /sys/class/net/ for interfaces backed by USB subsystem
 * 2. Match by USB VID (0x2CA3 = DJI) or by interface naming pattern
 * 3. Common interface names: usb0, usb1, enx* (predictable naming)
 *
 * On Jetson NX with E-Port connected, the RNDIS/NCM device typically
 * creates a "usb0" interface with a 192.168.x.x address.
 */

static T_HalNetworkInfo s_net_info = {0};
static int s_initialized = 0;

/* Check if a network interface is backed by USB */
static int _is_usb_interface(const char *ifname) {
    char path[256];
    char link_target[256];

    /* Check /sys/class/net/<ifname>/device — should resolve to USB path */
    snprintf(path, sizeof(path), "/sys/class/net/%s/device", ifname);
    ssize_t len = readlink(path, link_target, sizeof(link_target) - 1);
    if (len < 0) return 0;
    link_target[len] = '\0';

    /* USB devices have "usb" in their sysfs path */
    return (strstr(link_target, "usb") != NULL) ? 1 : 0;
}

/* Check if interface has a valid IP address */
static int _get_interface_ip(const char *ifname, char *ip_buf, size_t ip_buf_len) {
    struct ifaddrs *ifap, *ifa;
    int found = 0;

    if (getifaddrs(&ifap) != 0) return 0;

    for (ifa = ifap; ifa; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) continue;
        if (strcmp(ifa->ifa_name, ifname) != 0) continue;
        if (ifa->ifa_addr->sa_family != AF_INET) continue;

        struct sockaddr_in *sin = (struct sockaddr_in *)ifa->ifa_addr;
        inet_ntop(AF_INET, &sin->sin_addr, ip_buf, ip_buf_len);
        found = 1;
        break;
    }

    freeifaddrs(ifap);
    return found;
}

int HalNetwork_Init(void) {
    DIR *dir = opendir("/sys/class/net");
    if (!dir) {
        printf("[hal_network] cannot open /sys/class/net\n");
        return -1;
    }

    struct dirent *entry;
    int found = 0;

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_name[0] == '.') continue;
        if (strcmp(entry->d_name, "lo") == 0) continue;

        /* Priority 1: interface named "usb*" */
        if (strncmp(entry->d_name, "usb", 3) == 0) {
            strncpy(s_net_info.ifname, entry->d_name, sizeof(s_net_info.ifname) - 1);
            found = 1;
            break;
        }

        /* Priority 2: USB-backed interface with IP in 192.168.x.x range */
        if (_is_usb_interface(entry->d_name)) {
            char ip[16] = {0};
            if (_get_interface_ip(entry->d_name, ip, sizeof(ip))) {
                if (strncmp(ip, "192.168.", 8) == 0) {
                    strncpy(s_net_info.ifname, entry->d_name, sizeof(s_net_info.ifname) - 1);
                    strncpy(s_net_info.ip_addr, ip, sizeof(s_net_info.ip_addr) - 1);
                    found = 1;
                    break;
                }
            }
        }
    }
    closedir(dir);

    if (!found) {
        /* Fallback: use "usb0" as default (may appear later) */
        strncpy(s_net_info.ifname, "usb0", sizeof(s_net_info.ifname) - 1);
        printf("[hal_network] no USB-Ethernet found, defaulting to usb0\n");
    }

    /* Get IP if not already set */
    if (s_net_info.ip_addr[0] == '\0') {
        _get_interface_ip(s_net_info.ifname, s_net_info.ip_addr, sizeof(s_net_info.ip_addr));
    }

    /* Get MTU */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock >= 0) {
        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, s_net_info.ifname, IFNAMSIZ - 1);
        if (ioctl(sock, SIOCGIFMTU, &ifr) == 0) {
            s_net_info.mtu = (uint32_t)ifr.ifr_mtu;
        }
        close(sock);
    }
    if (s_net_info.mtu == 0) s_net_info.mtu = 1500;

    s_initialized = 1;
    printf("[hal_network] initialized: iface=%s ip=%s mtu=%u\n",
           s_net_info.ifname,
           s_net_info.ip_addr[0] ? s_net_info.ip_addr : "(no ip)",
           s_net_info.mtu);
    return 0;
}

int HalNetwork_GetInfo(T_HalNetworkInfo *info) {
    if (!info) return -1;
    if (!s_initialized) HalNetwork_Init();
    memcpy(info, &s_net_info, sizeof(T_HalNetworkInfo));
    return 0;
}

const char *HalNetwork_GetInterfaceName(void) {
    if (!s_initialized) HalNetwork_Init();
    return s_net_info.ifname;
}

void HalNetwork_Cleanup(void) {
    s_initialized = 0;
    memset(&s_net_info, 0, sizeof(s_net_info));
}
