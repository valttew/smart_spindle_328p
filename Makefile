
MCU=atmega328p
F_CPU=16000000UL
PORT=/dev/ttyUSB0
BAUD=115200
CFLAGS=-Wall -Os -DF_CPU=$(F_CPU)
CPPFLAGS=$(CFLAGS)
all: spindle_control.hex
spindle_control.elf: spindle_control.cpp
	avr-g++ $(CPPFLAGS) -mmcu=$(MCU) -o $@ $<
spindle_control.hex: spindle_control.elf
	avr-objcopy -O ihex -R .eeprom $< $@
flash: spindle_control.hex
	avrdude -V -patmega328p -carduino -P$(PORT) -b$(BAUD) -D -Uflash:w:$<:i
clean:
	rm -f *.elf *.hex
