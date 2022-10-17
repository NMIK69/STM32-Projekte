#include <stdint.h>
#include "utilities.h"


void systick_handler() {
    // this function gets called every second

    toggle_led();

    return;
}


void _start() {

    // configure clock
    configure_sys_clock();

    // configure led
    led_enable();

    // configure systick interrupt
    enable_systick();


    while(1) {
        __asm__("wfi");
    }
}

