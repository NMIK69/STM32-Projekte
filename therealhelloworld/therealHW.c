#include <stdint.h>
#include "usart.h"

void _start() {

    configure_sys_clock();
    usart2_enable();
    usart2_puts("Hello, World!", 13);


    while(1) {
        __asm__("wfi");
    }
}

