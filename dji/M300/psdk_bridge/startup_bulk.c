/*
 * startup_bulk.c — Initialize USB FunctionFS bulk endpoint.
 *
 * Based on DJI PSDK official sample + community fix for Little Endian.
 * Writes USB descriptors to FFS ep0, making endpoints ready for PSDK.
 *
 * Usage: startup_bulk /dev/usb-ffs/bulk1
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>
#include <endian.h>
#include <linux/usb/functionfs.h>

/******************** Little Endian Handling ********************************/
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define cpu_to_le16(x) (x)
#define cpu_to_le32(x) (x)
#else
#define cpu_to_le16(x) ((((x) >> 8) & 0xffu) | (((x) & 0xffu) << 8))
#define cpu_to_le32(x) \
    ((((x) & 0xff000000u) >> 24) | (((x) & 0x00ff0000u) >> 8) | \
     (((x) & 0x0000ff00u) << 8) | (((x) & 0x000000ffu) << 24))
#endif

#define STR_INTERFACE "DJI USB Bulk Interface"

struct usb_ffs_desc {
    struct {
        __le32 magic;
        __le32 length;
        __le32 flags;
        __le32 fs_count;
        __le32 hs_count;
    } header;
    struct {
        struct usb_interface_descriptor intf;
        struct usb_endpoint_descriptor_no_audio ep_in;
        struct usb_endpoint_descriptor_no_audio ep_out;
    } fs_descs, hs_descs;
} __attribute__((packed));

static struct usb_ffs_desc descriptors = {
    .header = {
        .magic = cpu_to_le32(FUNCTIONFS_DESCRIPTORS_MAGIC_V2),
        .flags = cpu_to_le32(FUNCTIONFS_HAS_FS_DESC | FUNCTIONFS_HAS_HS_DESC),
        .fs_count = cpu_to_le32(3),
        .hs_count = cpu_to_le32(3),
    },
    .fs_descs = {
        .intf = {
            .bLength = sizeof(struct usb_interface_descriptor),
            .bDescriptorType = USB_DT_INTERFACE,
            .bNumEndpoints = 2,
            .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
            .iInterface = 1,
        },
        .ep_in = {
            .bLength = sizeof(struct usb_endpoint_descriptor_no_audio),
            .bDescriptorType = USB_DT_ENDPOINT,
            .bEndpointAddress = 1 | USB_DIR_IN,
            .bmAttributes = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize = cpu_to_le16(64),
        },
        .ep_out = {
            .bLength = sizeof(struct usb_endpoint_descriptor_no_audio),
            .bDescriptorType = USB_DT_ENDPOINT,
            .bEndpointAddress = 1 | USB_DIR_OUT,
            .bmAttributes = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize = cpu_to_le16(64),
        },
    },
    .hs_descs = {
        .intf = {
            .bLength = sizeof(struct usb_interface_descriptor),
            .bDescriptorType = USB_DT_INTERFACE,
            .bNumEndpoints = 2,
            .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
            .iInterface = 1,
        },
        .ep_in = {
            .bLength = sizeof(struct usb_endpoint_descriptor_no_audio),
            .bDescriptorType = USB_DT_ENDPOINT,
            .bEndpointAddress = 1 | USB_DIR_IN,
            .bmAttributes = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize = cpu_to_le16(512),
        },
        .ep_out = {
            .bLength = sizeof(struct usb_endpoint_descriptor_no_audio),
            .bDescriptorType = USB_DT_ENDPOINT,
            .bEndpointAddress = 1 | USB_DIR_OUT,
            .bmAttributes = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize = cpu_to_le16(512),
        },
    },
};

#define STR_INTERFACE_LEN (sizeof(STR_INTERFACE))

struct usb_ffs_strings {
    struct usb_functionfs_strings_head header;
    __le16 lang;
    char str[STR_INTERFACE_LEN];
} __attribute__((packed));

static struct usb_ffs_strings strings = {
    .header = {
        .magic = cpu_to_le32(FUNCTIONFS_STRINGS_MAGIC),
        .length = cpu_to_le32(sizeof(struct usb_ffs_strings)),
        .str_count = cpu_to_le32(1),
        .lang_count = cpu_to_le32(1),
    },
    .lang = cpu_to_le16(0x0409),
    .str = STR_INTERFACE,
};

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s /dev/usb-ffs/bulkN\n", argv[0]);
        return 1;
    }

    char ep0_path[256];
    snprintf(ep0_path, sizeof(ep0_path), "%s/ep0", argv[1]);

    int fd = open(ep0_path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[startup_bulk] open %s failed: %s\n", ep0_path, strerror(errno));
        return 1;
    }

    /* Compute total length of descriptors */
    descriptors.header.length = cpu_to_le32(sizeof(descriptors));

    /* Write descriptors */
    ssize_t n = write(fd, &descriptors, sizeof(descriptors));
    if (n < 0) {
        fprintf(stderr, "[startup_bulk] write descriptors to %s failed: %s\n",
                ep0_path, strerror(errno));
        close(fd);
        return 1;
    }

    /* Write strings */
    n = write(fd, &strings, sizeof(strings));
    if (n < 0) {
        fprintf(stderr, "[startup_bulk] write strings to %s failed: %s\n",
                ep0_path, strerror(errno));
        close(fd);
        return 1;
    }

    printf("[startup_bulk] %s initialized OK\n", argv[1]);

    /* Keep ep0 open — FunctionFS needs it alive */
    while (1) {
        sleep(3600);
    }

    close(fd);
    return 0;
}
