.PHONY:all
all:head.o utilities.o stopwatch.bin stopwatch.o stopwatch.elf

head.o: head.s
	arm-none-eabi-gcc head.s -mcpu=cortex-m4 -mthumb -c -g -o head.o

utilities.o: utilities.c
	arm-none-eabi-gcc utilities.c -mcpu=cortex-m4 -mthumb -c -g -o utilities.o

stopwatch.bin: stopwatch.elf 
	arm-none-eabi-objcopy -O binary -S stopwatch.elf stopwatch.bin

stopwatch.o: stopwatch.c
	arm-none-eabi-gcc stopwatch.c -mcpu=cortex-m4 -mthumb -c -g -o stopwatch.o
 
stopwatch.elf: stopwatch.o head.o utilities.o
	arm-none-eabi-gcc head.o utilities.o stopwatch.o -mcpu=cortex-m4 -mthumb -nostartfiles -Wl,-Ttext=0x8000000,-Tbss=0x20000068 -o stopwatch.elf



.PHONY:clean
clean:
	rm -f head.o stopwatch.o stopwatch.elf stopwatch.bin utilities.o 


.PHONY:upload
upload: stopwatch.bin
	cp stopwatch.bin /media/nmik/NODE_L432KC
