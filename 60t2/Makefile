.PHONY: clean

mc = atmega8
VUSB = ../vusb

CC = avr-gcc
COMMON = -mmcu=$(mc)
CFLAGS = $(COMMON)
CFLAGS += -Wall -Os -I$(VUSB) -I../include
LDFLAGS = $(COMMON)
HEX_FLASH_FLAGS = -R .eeprom
HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0


define buildflash
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS) $(1).elf $(1).hex
	avr-objdump -h -S $(1).elf > $(1).lss
	$(CC) $(INCLUDES) $(CFLAGS) -S $(1).c -o $(1).S
	avr-objcopy -I ihex $(1).hex -O binary $(1).bin
	stat -c "%n : %s bytes"  $(1).bin
endef

receiverobjects = $(VUSB)/usbdrvasm.o $(VUSB)/usbdrv.o $(VUSB)/oddebug.o receiver.o

all: main.hex receiver.hex

$(receiverobjects): $(VUSB)/usbconfig.h

main.hex: main.o
	$(CC) $(LDFLAGS) $< $(LIBDIRS) $(LIBS) -o main.elf
	$(call buildflash,main)

receiver.hex: $(receiverobjects)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ $(LIBDIRS) $(LIBS) -o receiver.elf
	$(call buildflash,receiver)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $< -o $@

clean:
	-rm -rf $(receiverobjects) *.o *.lss *.S *.elf dep/ *.hex *.eep *.bin

-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

checkname:
ifndef program
	$(error 'program' is not set. Use 'make program=<name of program>')
endif

flash: checkname $(program).hex
	stty 9600 ignbrk -brkint -icrnl -imaxbel -opost -isig -icanon -iexten -echo noflsh </dev/ttyS0
	avrdude -b 9660 -p $(mc) -c nikolaew -P /dev/ttyS0 -v -U flash:w:$(program).hex:i

60tcli:
	gcc -o $@ 60t-usb-cli.c -lusb-1.0 

