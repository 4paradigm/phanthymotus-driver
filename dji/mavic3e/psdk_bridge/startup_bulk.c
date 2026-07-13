/*
 * startup_bulk.c — Initialize USB FunctionFS bulk endpoint.
 *
 * Writes USB descriptors (endpoint descriptors) to the FFS ep0 file,
 * making the endpoint ready for PSDK to use.
 *
 * Usage: startup_bulk /dev/usb-ffs/bulk1
 *
 * Reference: DJI PSDK samples/sample_c/platform/linux/raspberry_pi/
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>
#include <linux/usb/functionfs.h>

/* USB Bulk endpoint descriptors */
struct usb_ffs_descriptors {
    struct usb_functionfs_descs_head_v2 header;
    __le32 fs_count;
    __le32 hs_count;
    /* Full-speed descriptors */
    struct usb_interface_descriptor fs_intf;
    struct usb_endpoint_descriptor_no_audio fs_ep_in;
    struct usb_endpoint_descriptor_no_audio fs_ep_out;
    /* High-speed descriptors */
    struct usb_interface_descriptor hs_intf;
    struct usb_endpoint_descriptor_no_audio hs_ep_in;
    struct usb_endpoint_descriptor_no_audio hs_ep_out;
} __attribute__((packed));

struct usb_ffs_strings {
    struct usb_functionfs_strings_head header;
    uint16_t lang;
    char str[];
} __attribute__((packed));

static struct usb_ffs_descriptors s_descriptors = {
    .header = {
        .magic = FUNCTIONFS_DESCRIPTORS_MAGIC_V2,
        .length = sizeof(struct usb_ffs_descriptors),
        .flags = FUNCTIONFS_HAS_FS_DESC | FUNCTIONFS_HAS_HS_DESC,
    },
    .fs_count = 3,  /* 1 interface + 2 endpoints */
    .hs_count = 3,
    /* Full-speed */
    .fs_intf = {
        .bLength = sizeof(struct usb_interface_descriptor),
        .bDescriptorType = USB_DT_INTERFACE,
        .bInterfaceNumber = 0,
        .bNumEndpoints = 2,
        .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = 0,
    },
    .fs_ep_in = {
        .bLength = sizeof(struct usb_endpoint_descriptor_no_audio),
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 1 | USB_DIR_IN,
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize = 64,
    },
    .fs_ep_out = {
        .bLength = sizeof(struct usb_endpoint_descriptor_no_audio),
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 1 | USB_DIR_OUT,
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize = 64,
    },
    /* High-speed */
    .hs_intf = {
        .bLength = sizeof(struct usb_interface_descriptor),
        .bDescriptorType = USB_DT_INTERFACE,
        .bInterfaceNumber = 0,
        .bNumEndpoints = 2,
        .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = 0,
    },
    .hs_ep_in = {
        .bLength = sizeof(struct usb_endpoint_descriptor_no_audio),
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 1 | USB_DIR_IN,
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize = 512,
    },
    .hs_ep_out = {
        .bLength = sizeof(struct usb_endpoint_descriptor_no_audio),
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 1 | USB_DIR_OUT,
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize = 512,
    },
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

    /* Write descriptors */
    ssize_t n = write(fd, &s_descriptors, sizeof(s_descriptors));
    if (n < 0) {
        fprintf(stderr, "[startup_bulk] write descriptors to %s failed: %s\n", ep0_path, strerror(errno));
        close(fd);
        return 1;
    }

    /* Write empty strings (required by FunctionFS) */
    struct {
        struct usb_functionfs_strings_head header;
    } __attribute__((packed)) strings = {
        .header = {
            .magic = FUNCTIONFS_STRINGS_MAGIC,
            .length = sizeof(strings),
            .str_count = 0,
            .lang_count = 0,
        },
    };

    n = write(fd, &strings, sizeof(strings));
    if (n < 0) {
        fprintf(stderr, "[startup_bulk] write strings to %s failed: %s\n", ep0_path, strerror(errno));
        close(fd);
        return 1;
    }

    printf("[startup_bulk] %s initialized\n", argv[1]);

    /* Keep ep0 open — FunctionFS needs it alive */
    /* Block until killed */
    while (1) {
        sleep(3600);
    }

    close(fd);
    return 0;
}
