MCU=atmega32a
F_CPU=8000000
CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-std=c99 -Wall -Os -mmcu=${MCU} -DF_CPU=${F_CPU} -I.
CFLAGS+=-funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums 
CFLAGS+=-Wstrict-prototypes
CFLAGS+=-ffunction-sections -fdata-sections -Wl,--gc-sections -Wl,--relax

TARGET=fw
SRCS=$(wildcard *.c)
OBJS=$(wildcard *.o)

all:
	${CC} ${CFLAGS} ${SRCS} -o ${TARGET}.elf
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.elf ${TARGET}.hex
	rm -f *.elf
flash:
	avrdude -p ${MCU} -c usbasp -U flash:w:${TARGET}.hex:i -F -P usb
ucheck:
	avrdude -p ${MCU} -c usbasp -P usb
clean:
	rm -f *.bin *.hex *.elf *.o