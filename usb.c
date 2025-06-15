#include <sys/types.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libusb.h>
#include "oscompat.h"
#include "qdl.h"

#define DEFAULT_OUT_CHUNK_SIZE (1024 * 1024)

struct qdl_device_usb {
    struct qdl_device base;
    struct libusb_device_handle *usb_handle;
    int in_ep;
    int out_ep;
    size_t in_maxpktsize;
    size_t out_maxpktsize;
    size_t out_chunk_size;
};

/*
 * libusb commit f0cce43f882d split transfer type and endpoint transfer types.
 * Provide an alias for compatibility with older libusb versions.
 */
#ifndef LIBUSB_ENDPOINT_TRANSFER_TYPE_BULK
#define LIBUSB_ENDPOINT_TRANSFER_TYPE_BULK LIBUSB_TRANSFER_TYPE_BULK
#endif

static bool usb_match_usb_serial(struct libusb_device_handle *handle, const char *serial,
                                const struct libusb_device_descriptor *desc) {
    char buf[128];
    char *p;
    int ret;

    if (!serial)
        return true;

    ret = libusb_get_string_descriptor_ascii(handle, desc->iProduct, (unsigned char *)buf, sizeof(buf));
    if (ret < 0) {
        warnx("failed to read iProduct descriptor: %s", libusb_strerror(ret));
        return false;
    }

    p = strstr(buf, "_SN:");
    if (!p)
        return false;

    p += strlen("_SN:");
    p[strcspn(p, " _")] = '\0';

    return strcmp(p, serial) == 0;
}

static int usb_try_open_with_fd(struct qdl_device_usb *qdl, int fd, const char *serial) {
    struct libusb_device_handle *handle;
    struct libusb_device *dev;
    struct libusb_config_descriptor *config;
    struct libusb_device_descriptor desc;
    const struct libusb_endpoint_descriptor *endpoint;
    const struct libusb_interface_descriptor *ifc;
    size_t out_size = 0;
    size_t in_size = 0;
    uint8_t type;
    int ret;
    int in = -1;
    int out = -1;
    int k, l;

    /* Initialize libusb with Android-specific context */
    ret = libusb_init(NULL);
    if (ret < 0) {
        warnx("failed to initialize libusb: %s", libusb_strerror(ret));
        return -1;
    }

    /* Use the provided file descriptor to open the device */
    ret = libusb_open_device_with_fd(NULL, fd, &handle);
    if (ret < 0 || !handle) {
        warnx("failed to open USB device with fd %d: %s", fd, libusb_strerror(ret));
        libusb_exit(NULL);
        return -1;
    }

    dev = libusb_get_device(handle);
    if (!dev) {
        warnx("failed to get USB device from handle");
        libusb_close(handle);
        libusb_exit(NULL);
        return -1;
    }

    ret = libusb_get_device_descriptor(dev, &desc);
    if (ret < 0) {
        warnx("failed to get USB device descriptor: %s", libusb_strerror(ret));
        libusb_close(handle);
        libusb_exit(NULL);
        return -1;
    }

    /* Verify vendor and product IDs for Qualcomm EDL */
    if (desc.idVendor != 0x05c6 || (desc.idProduct != 0x9008 && desc.idProduct != 0x900e && desc.idProduct != 0x901d)) {
        warnx("device does not match Qualcomm EDL VID:PID");
        libusb_close(handle);
        libusb_exit(NULL);
        return 0;
    }

    /* Verify serial number if provided */
    if (!usb_match_usb_serial(handle, serial, &desc)) {
        warnx("serial number mismatch");
        libusb_close(handle);
        libusb_exit(NULL);
        return 0;
    }

    ret = libusb_get_active_config_descriptor(dev, &config);
    if (ret < 0) {
        warnx("failed to acquire USB device's active config descriptor: %s", libusb_strerror(ret));
        libusb_close(handle);
        libusb_exit(NULL);
        return -1;
    }

    for (k = 0; k < config->bNumInterfaces; k++) {
        ifc = config->interface[k].altsetting;

        in = -1;
        out = -1;
        in_size = 0;
        out_size = 0;

        for (l = 0; l < ifc->bNumEndpoints; l++) {
            endpoint = &ifc->endpoint[l];
            type = endpoint->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK;
            if (type != LIBUSB_ENDPOINT_TRANSFER_TYPE_BULK)
                continue;

            if (endpoint->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                in = endpoint->bEndpointAddress;
                in_size = endpoint->wMaxPacketSize;
            } else {
                out = endpoint->bEndpointAddress;
                out_size = endpoint->wMaxPacketSize;
            }
        }

        if (ifc->bInterfaceClass != 0xff || ifc->bInterfaceSubClass != 0xff)
            continue;

        if (ifc->bInterfaceProtocol != 0xff && ifc->bInterfaceProtocol != 16 && ifc->bInterfaceProtocol != 17)
            continue;

        ret = libusb_claim_interface(handle, ifc->bInterfaceNumber);
        if (ret < 0) {
            warnx("failed to claim USB interface: %s", libusb_strerror(ret));
            libusb_close(handle);
            continue;
        }

        qdl->usb_handle = handle;
        qdl->in_ep = in;
        qdl->out_ep = out;
        qdl->in_maxpktsize = in_size;
        qdl->out_maxpktsize = out_size;

        if (qdl->out_chunk_size && qdl->out_chunk_size % out_size) {
            ux_err("WARNING: requested out-chunk-size must be multiple of wMaxPacketSize %ld, using %ld\n",
                   out_size, out_size);
            qdl->out_chunk_size = out_size;
        } else if (!qdl->out_chunk_size) {
            qdl->out_chunk_size = DEFAULT_OUT_CHUNK_SIZE;
        }

        ux_debug("USB: using out-chunk-size of %ld\n", qdl->out_chunk_size);
        break;
    }

    libusb_free_config_descriptor(config);

    if (!qdl->usb_handle) {
        libusb_close(handle);
        libusb_exit(NULL);
        return -1;
    }

    return 1;
}

static int usb_open(struct qdl_device *qdl, const char *serial, int fd) {
    struct qdl_device_usb *qdl_usb = container_of(qdl, struct qdl_device_usb, base);
    int ret;

    if (fd < 0) {
        warnx("invalid file descriptor provided");
        return -1;
    }

    ret = usb_try_open_with_fd(qdl_usb, fd, serial);
    if (ret <= 0) {
        warnx("failed to open USB device with provided file descriptor");
        return -1;
    }

    return 0;
}

static void usb_close(struct qdl_device *qdl) {
    struct qdl_device_usb *qdl_usb = container_of(qdl, struct qdl_device_usb, base);
    if (qdl_usb->usb_handle) {
        libusb_release_interface(qdl_usb->usb_handle, 0);
        libusb_close(qdl_usb->usb_handle);
    }
    libusb_exit(NULL);
}

static int usb_read(struct qdl_device *qdl, void *buf, size_t len, unsigned int timeout) {
    struct qdl_device_usb *qdl_usb = container_of(qdl, struct qdl_device_usb, base);
    int actual;
    int ret;

    ret = libusb_bulk_transfer(qdl_usb->usb_handle, qdl_usb->in_ep, buf, len, &actual, timeout);
    if ((ret != 0 && ret != LIBUSB_ERROR_TIMEOUT) ||
        (ret == LIBUSB_ERROR_TIMEOUT && actual == 0)) {
        warnx("bulk read failed: %s", libusb_strerror(ret));
        return -1;
    }

    return actual;
}

static int usb_write(struct qdl_device *qdl, const void *buf, size_t len) {
    unsigned char *data = (unsigned char*) buf;
    struct qdl_device_usb *qdl_usb = container_of(qdl, struct qdl_device_usb, base);
    unsigned int count = 0;
    int actual;
    int xfer;
    int ret;

    while (len > 0) {
        xfer = (len > qdl_usb->out_chunk_size) ? qdl_usb->out_chunk_size : len;
        ret = libusb_bulk_transfer(qdl_usb->usb_handle, qdl_usb->out_ep, data,
                                   xfer, &actual, 1000);
        if ((ret != 0 && ret != LIBUSB_ERROR_TIMEOUT) ||
            (ret == LIBUSB_ERROR_TIMEOUT && actual == 0)) {
            warnx("bulk write failed: %s", libusb_strerror(ret));
            return -1;
        }

        count += actual;
        len -= actual;
        data += actual;
    }

    if (len % qdl_usb->out_maxpktsize == 0) {
        ret = libusb_bulk_transfer(qdl_usb->usb_handle, qdl_usb->out_ep, NULL,
                                   0, &actual, 1000);
        if (ret < 0) {
            warnx("zero-length packet failed: %s", libusb_strerror(ret));
            return -1;
        }
    }

    return count;
}

static void usb_set_out_chunk_size(struct qdl_device *qdl, long size) {
    struct qdl_device_usb *qdl_usb = container_of(qdl, struct qdl_device_usb, base);
    qdl_usb->out_chunk_size = size;
}

struct qdl_device *usb_init(void) {
    struct qdl_device_usb *qdl_usb = malloc(sizeof(struct qdl_device_usb));
    if (!qdl_usb)
        return NULL;

    struct qdl_device *qdl = &qdl_usb->base;
    qdl->dev_type = QDL_DEVICE_USB;
    qdl->open = usb_open;
    qdl->read = usb_read;
    qdl->write = usb_write;
    qdl->close = usb_close;
    qdl->set_out_chunk_size = usb_set_out_chunk_size;

    return qdl;
}
