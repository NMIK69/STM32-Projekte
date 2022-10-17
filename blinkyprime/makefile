.PHONY:all
all:head.o utilities.o blinkyprime.bin blinkyprime.o blinkyprime.elf

head.o: head.s
	arm-none-eabi-gcc head.s -mcpu=cortex-m4 -mthumb -c -g -o head.o

utilities.o: utilities.c
	arm-none-eabi-gcc utilities.c -mcpu=cortex-m4 -mthumb -c -g -o utilities.o

blinkyprime.bin: blinkyprime.elf 
	arm-none-eabi-objcopy -O binary -S blinkyprime.elf blinkyprime.bin

blinkyprime.o: blinkyprime.c
	arm-none-eabi-gcc blinkyprime.c -mcpu=cortex-m4 -mthumb -c -g -o blinkyprime.o
 
blinkyprime.elf: blinkyprime.o head.o utilities.o
	arm-none-eabi-gcc head.o utilities.o blinkyprime.o -mcpu=cortex-m4 -mthumb -nostartfiles -Wl,-Ttext=0x8000000 -o blinkyprime.elf



.PHONY:clean
clean:
	rm -f head.o blinkyprime.o blinkyprime.elf blinkyprime.bin utilities.o 


.PHONY:upload
upload: blinkyprime.bin
	cp blinkyprime.bin /media/nmik/NODE_L432KC
