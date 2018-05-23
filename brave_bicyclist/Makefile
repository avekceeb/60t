.PHONY: clean

use_lcd = 1
use_usb = 0

istty = 1
mc = atmega8
programmer = nikolaew
serialport = /dev/ttyS0
VUSB = ../vusb
NEC = ../nec
LCD = ../lcd
inc60t = -I$(NEC) -I$(LCD) -I$(VUSB) \
		-Duse_lcd=$(use_lcd) -Duse_usb=$(use_usb)


CC = avr-gcc
COMMON = -mmcu=$(mc)
CFLAGS = $(COMMON)
CFLAGS += -Wall -Os $(inc60t)
LDFLAGS = $(COMMON)
HEX_FLASH_FLAGS = -R .eeprom
HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0

TARGETS = main.hex receiver.hex

ifeq ($(istty),1)
	_Y:=\\033[93m
	_R:=\\033[31m
	_G:=\\033[92m
	_N:=\\033[m
endif

define green
	printf "$(_G)%s$(_N)\n" "$(1)"
endef

ifeq ($(usbcli),1)
	TARGETS += 60t2-usb-cli
endif

define buildflash
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS) $(1).elf $(1).hex
	avr-objdump -h -S $(1).elf > $(1).lss
	$(CC) $(INCLUDES) $(CFLAGS) -S $(1).c -o $(1).S
	avr-objcopy -I ihex $(1).hex -O binary $(1).bin
	@$(call green,`stat -c '%n : %s bytes' $(1).bin`)
endef

receiverobjects = $(NEC)/ir-nec.o receiver.o
mainobjects = main.o

ifeq ($(use_usb),1)
	receiverobjects = $(VUSB)/usbdrvasm.o $(VUSB)/usbdrv.o $(VUSB)/oddebug.o $(NEC)/ir-nec.o receiver.o
endif

ifeq ($(use_lcd),1)
	mainobjects += ../lcd/lcd.o
else
	inc60t += -Duse_service_mode=0
endif

all: $(TARGETS)

$(receiverobjects): $(VUSB)/usbconfig.h

main.hex: $(mainobjects)
	$(CC) $(LDFLAGS) $^ $(LIBDIRS) $(LIBS) -o main.elf
	$(call buildflash,main)

receiver.hex: $(receiverobjects)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LIBDIRS) $(LIBS) -o receiver.elf
	$(call buildflash,receiver)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $< -o $@

clean:
	-rm -rf $(receiverobjects) $(mainobjects) $(NEC)/*.o $(LCD)/*.o *.lss *.S *.elf dep/ *.hex *.eep *.bin 60t2-usb-cli

-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

checkname:
ifndef program
	$(error 'program' is not set. Use 'make program=<name of program>')
endif

flash: checkname $(program).hex
	stty 9600 ignbrk -brkint -icrnl -imaxbel -opost -isig -icanon -iexten -echo noflsh <$(serialport)
	avrdude -b 9660 -p $(mc) -c $(programmer) -P $(serialport) -v -U flash:w:$(program).hex:i

60t2-usb-cli: 60t2-usb-cli.c
	gcc -o $@ $^ $(inc60t) -lusb-1.0 

dist:
	mv main.hex working-firmware/main.`date +%Y-%m-%d`
	mv receiver.hex working-firmware/receiver.`date +%Y-%m-%d`
