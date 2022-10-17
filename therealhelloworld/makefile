.PHONY:all
all:head.o usart.o therealHW.bin therealHW.o therealHW.elf

head.o: head.s
	arm-none-eabi-gcc head.s -mcpu=cortex-m4 -mthumb -c -g -o head.o

usart.o: usart.c
	arm-none-eabi-gcc usart.c -mcpu=cortex-m4 -mthumb -c -g -o usart.o

therealHW.bin: therealHW.elf
	arm-none-eabi-objcopy -O binary -S therealHW.elf therealHW.bin

therealHW.o: therealHW.c
	arm-none-eabi-gcc therealHW.c -mcpu=cortex-m4 -mthumb -c -g -o therealHW.o
 
therealHW.elf: therealHW.o head.o usart.o
	arm-none-eabi-gcc head.o usart.o therealHW.o -mcpu=cortex-m4 -mthumb -nostartfiles -Wl,-Ttext=0x8000000 -o therealHW.elf



.PHONY:clean
clean:
	rm -f head.o therealHW.o therealHW.elf therealHW.bin usart.o 


.PHONY:upload
upload: therealHW.bin
	cp therealHW.bin /media/nmik/NODE_L432KC
