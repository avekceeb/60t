# WinAVR cross-compiler toolchain is used here
CC = avr-gcc
OBJCOPY = avr-objcopy
DUDE = avrdude

# If you are not using ATtiny2313 and the USBtiny programmer, 
# update the lines below to match your configuration
CFLAGS = -Wall -Os -Iusbdrv -I../include -mmcu=attiny2313
OBJFLAGS = -j .text -j .data -O ihex
DUDEFLAGS = -b 9660 -p attiny2313 -c gromoff -P /dev/ttyS0 -v

# Object files for the firmware (usbdrv/oddebug.o not strictly needed I think)
OBJECTS = usbdrv/usbdrv.o usbdrv/oddebug.o usbdrv/usbdrvasm.o main.o

# By default, build the firmware and command-line client, but do not flash
all: main.hex

# With this, you can flash the firmware by just typing "make flash" on command-line
flash: main.hex
	stty -F /dev/ttyS0 9600 ignbrk -brkint -icrnl -imaxbel -opost -isig -icanon -iexten -echo noflsh
	$(DUDE) $(DUDEFLAGS) -U flash:w:$<:i

# Housekeeping if you want it
clean:
	$(RM) *.o *.hex *.elf usbdrv/*.o

# From .elf file to .hex
%.hex: %.elf
	$(OBJCOPY) $(OBJFLAGS) $< $@

# Main.elf requires additional objects to the firmware, not just main.o
main.elf: $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o $@

# Without this dependance, .o files will not be recompiled if you change 
# the config! I spent a few hours debugging because of this...
$(OBJECTS): usbdrv/usbconfig.h

# From C source to .o object file
%.o: %.c	
	$(CC) $(CFLAGS) -c $< -o $@

# From assembler source to .o object file
%.o: %.S
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $< -o $@
