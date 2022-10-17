.PHONY: all
all: sos_blinky.bin

sos_blinky.bin: sos_blinky.elf
	arm-none-eabi-objcopy -O binary -S sos_blinky.elf sos_blinky.bin

sos_blinky.elf: sos_blinky.o
	arm-none-eabi-gcc sos_blinky.o -nostartfiles -Wl,-Ttext=0x8000000 -g -o sos_blinky.elf

sos_blinky.o: sos_blinky.S
	arm-none-eabi-gcc sos_blinky.S -mcpu=cortex-m4 -mthumb -c -g -o sos_blinky.o

.PHONY: clean
clean:
	rm -f sos_blinky.o sos_blinky.elf sos_blinky.bin

.PHONY: upload
upload: sos_blinky.bin
	cp sos_blinky.bin /media/nmik/NODE_L432KC