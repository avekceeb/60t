#if openwrtbuild 
#include <libusb-1.0/libusb.h>
#else
#include <libusb.h>
#endif

#define usb_rq_in (LIBUSB_RECIPIENT_DEVICE|LIBUSB_REQUEST_TYPE_VENDOR|LIBUSB_ENDPOINT_IN)
#define usb_rq_out (LIBUSB_RECIPIENT_DEVICE|LIBUSB_REQUEST_TYPE_VENDOR|LIBUSB_ENDPOINT_OUT)

#define shared_vid 0x16C0
#define shared_pid 0x05DC

#define usb_timeout 5000

libusb_device_handle* get_usb_device();

int usb_send (libusb_device_handle * handle,
        char * buffer,
        size_t size);

int usb_receive (libusb_device_handle * handle,
        char * buffer,
        size_t size);

void close_usb_device(libusb_device_handle* handle);
