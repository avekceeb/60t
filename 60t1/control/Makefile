
USBFLAGS=`pkg-config --cflags libusb-1.0`
USBLIBS=`pkg-config --libs libusb-1.0`

CFLAGS = -I../include $(USBFLAGS)

demonize = 1 

ALL:=60t

all: $(ALL) 

pkg: daemon.c 
	mkdir -p 60t-pkg/src
	cp openwrt-pkg-makefile 60t-pkg/Makefile
	cp openwrt-makefile 60t-pkg/src/Makefile
	cp *.init *.c *.html *.js 60t-pkg/src
	cp -r ../include 60t-pkg/src

clean:
	rm -f *.o $(ALL)
	rm -rf 60t-pkg

60t: daemon.c
	gcc $(CFLAGS) -Ddemonize=$(demonize) -o$@ $^ $(USBLIBS)

