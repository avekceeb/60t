# 60T.  Remote controlled car 

## 60T1

    TPLink MR3020 router with OpenWRT connected via USB to AVR ATtiny2313
    drives 2 brush motors via L293D

## 60T2

    2 x ATmega8 controlled via NEC protocol

-----------------

### Build

    apt-get install gcc-avr avr-libc binutils-avr avrdude
    # 60T2 - optionally (for usb connection)
    apt-get install libusb-1.0-0-dev
    cd 60t/60t2 ; make clean ; make
    # to build usb cli also:
    make usbcli=1
    # to build with LCD debug:
    make use_led=1

### TODO:
- NEC as separate module
- Proximity sensor
- up-down both pwm
- l-r speed ?
- receiver: cmd ; wait for finish (req queue)
- increase frequency of shaft encoder
 
