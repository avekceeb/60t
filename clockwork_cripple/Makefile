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


TARGETS = cripple.hex


ifeq ($(istty),1)
	_Y:=\\033[93m
	_R:=\\033[31m
	_G:=\\033[92m
	_N:=\\033[m
endif


define green
	printf "$(_G)%s$(_N)\n" "$(1)"
endef


define buildflash
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS) $(1).elf $(1).hex
	avr-objdump -h -S $(1).elf > $(1).lss
	$(CC) $(INCLUDES) $(CFLAGS) -S $(1).c -o $(1).S
	avr-objcopy -I ihex $(1).hex -O binary $(1).bin
	@$(call green,`stat -c '%n : %s bytes' $(1).bin`)
endef


mainobjects = cripple.o


ifeq ($(use_lcd),1)
	mainobjects += ../lcd/lcd.o
else
	inc60t += -Duse_service_mode=0
endif


all: $(TARGETS)

cripple.hex: $(mainobjects)
	$(CC) $(LDFLAGS) $^ $(LIBDIRS) $(LIBS) -o cripple.elf
	$(call buildflash,cripple)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $< -o $@

clean:
	-rm -rf $(mainobjects) *.lss *.S *.elf dep/ *.hex *.eep *.bin

-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

