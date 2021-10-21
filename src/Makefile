MCU=atmega328p
F_CPU=11059200
CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Wall -g -O2 -mmcu=${MCU} -DF_CPU=${F_CPU} -I.
TARGET=main
SRCS=main.c

all:
	${CC} ${CFLAGS} -o ${TARGET}.bin ${SRCS}
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.bin ${TARGET}.hex
	avr-size --mcu=${MCU} -C main.bin

flash:
	avrdude.exe -p ${MCU} -c usbasp -U flash:w:${TARGET}.hex:i -F -P usb

reset:
	avrdude.exe -p ${MCU} -c usbasp -P usb

clean:
	rm -f *.bin *.hex *.o
