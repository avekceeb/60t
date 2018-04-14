/**
 * License: GNU GPL v3
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <libusb-1.0/libusb.h>
#include "60T.h" 

#define RQIN (LIBUSB_RECIPIENT_DEVICE|LIBUSB_REQUEST_TYPE_VENDOR|LIBUSB_ENDPOINT_IN)
#define RQOUT (LIBUSB_RECIPIENT_DEVICE|LIBUSB_REQUEST_TYPE_VENDOR|LIBUSB_ENDPOINT_OUT)
#define usb_timeout_ms 3000

void usage() {
    printf("Usage:\n"
        "\t60T2 <cmd> <options>\n"
        "\tcmd: ...\n"
        "\toptions: [f|b|l|r|s...]\n"
    );
    exit(1);
}

void print_usb_status(int retcode) {
    switch(retcode) {
        case LIBUSB_ERROR_TIMEOUT:
            printf("transfer timed out\n");
            break; 
        case LIBUSB_ERROR_PIPE:
            printf("control request was not supported by the device\n");
            break;
        case LIBUSB_ERROR_NO_DEVICE:
            printf("device has been disconnected\n");
            break;
        default:
            printf("%d bytes transferred\n", retcode);
        }
}

int main(int argc, char **argv) {
    libusb_device_handle *handle = NULL;
    int bts = 0;
    uint8_t buffer[256];
    struct Report *report;
    int res = 0;
    uint8_t cmd;
    int rq;

    if(argc < 2) {
        usage();
    }

    res = libusb_init(0);
    if (res != 0) {
        printf("Error initialising libusb.\n");
        return 1;
    }

    handle = libusb_open_device_with_vid_pid(0, 0x16C0, 0x05DC);
    if (!handle) {
        printf("Unable to open device\n");
        return 1;
    }

    if (libusb_kernel_driver_active(handle, 0)) {
        res = libusb_detach_kernel_driver(handle, 0);
        if (res) {
          printf("Error detaching kernel driver.\n");
          return 1;
        }
    }

    res = libusb_claim_interface(handle, 0);
    if (res != 0) {
        printf("Error claiming interface.\n");
        return 1;
    }

    for (int a=1; a<argc; a++) {
        switch ((uint8_t)argv[a][0]) {
            case 'f':
                cmd = cmd_test_fwd;
                break;
            case 'b':
                cmd = cmd_test_bkw;
                break;
            case 'l':
                cmd = cmd_test_left;
                break;
            case 'r':
                cmd = cmd_test_right;
                break;
            case 'S':
            case 's':
                cmd = cmd_stop;
                break;
            case 'F':
                cmd = cmd_fwd;
                break;
            case 'B':
                cmd = cmd_bkw;
                break;
            case 'L':
                cmd = cmd_left;
                break;
            case 'R':
                cmd = cmd_right;
                break;
            default:
                printf("wrong command!\n");
                goto end;
        }

        buffer[0] = (uint8_t)(cmd);
        printf ("Executing command: %c (0x%02x)\n", argv[a][0], cmd);

        bts = libusb_control_transfer(handle, RQOUT, usb_cmd_set,
                0, 0, (uint8_t*)buffer, 1/*len*/, usb_timeout_ms);
        print_usb_status(bts);

        printf("press button...");
        rq = getchar();

        bts = libusb_control_transfer(handle, RQIN, usb_cmd_get,
                0, 0, (uint8_t*)buffer, 16/*sizeof(buffer)*/, usb_timeout_ms);
        print_usb_status(bts);
        if (bts > 0 && bts < 255) {
            report = (struct Report *)(buffer);
            printf("------------------\n"
                "\tCommand = 0x%02x\n"
                "\tRunning = %d\n"
                "\tDone    = %d\n"
                "\tBridge  = 0x%02x\n"
                "\tTicks L = %d\n"
                "\tTicks R = %d\n"
                "\tSpeed L = %d\n"
                "\tSpeed R = %d\n"
                "\tFwd     = %d\n"
                "\tBkw     = %d\n"
                "\tTurn    = %d\n",
                report->command,
                report->running,
                report->step_done,
                report->direction,
                report->ticks_l,
                report->ticks_r,
                report->speed_l,
                report->speed_r,
                report->step_bkw,
                report->step_fwd,
                report->step_turn
                );
        printf("press button...");
        rq = getchar();

        }
    }

end:
    libusb_close(handle);
    return 0;
}
