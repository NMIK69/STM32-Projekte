#include <stdint.h>
#include "utilities.h"

volatile uint32_t counter; // counts the elapsed time in ms after the button is pressed
volatile uint32_t button_pressed; // counts the amount of times the button is pressed (for debouncing)
volatile uint32_t button_not_pressed; // counts the amount of times the button is not pressed (for debouncing)
volatile uint8_t stopwatch_mode_flag; // this flag tells if we are currently in stopwatchmode or not
volatile uint32_t blink_counter; // counts the time in ms to toggle the led every second 


void systick_handler() {
    // this function gets called every ms 

    // check if we are in sopwatch mode
    if(stopwatch_mode_flag == 1) {
        
        // toggle the led every ms or right after the button was pressed
        blink_counter += 1;
        if(blink_counter >= 1000 || counter == 0) {
            toggle_led();
            blink_counter = 0;
        }

        // count the ms 
        counter += 1;
    }

    //turn on led to indicate that we are not in stopwatch mode
    else if(stopwatch_mode_flag == 0) {
        turn_on_led();
    }

    // check if the button was pressed 
    if(get_btn_pin_state() == 1) {
        button_pressed += 1; 
    }
    else {
        button_not_pressed += 1;
    }


    /* the next section is for debouncing the button */
    // if the button was not pressed
    if(button_not_pressed == 50) {

        // reset both counters to zero
        button_not_pressed = 0;
        button_pressed = 0;
    }

    // if the button was pressed 
    else if(button_pressed == 50) {

        // if the current mode was stopwatch mode then transmit counted time and exit this mode
        if(stopwatch_mode_flag == 1) {
            // transmit time elapsed
            send_time(counter);

            counter = 0;
            stopwatch_mode_flag = 0;
        }


        // if the current mode was standby mode enter counting mode
        else {
            stopwatch_mode_flag = 1;
        }

    }


    return;
}


void _start() {

    // initialize variables
    blink_counter = 0;
    counter = 0;
    button_not_pressed = 0;
    button_pressed = 0;
    stopwatch_mode_flag = 0;

    // configure clock
    configure_sys_clock();

    // configure led
    led_enable();

    // configure usart
    usart2_enable();

    // configure button pin
    enable_btn_pin();

    // configure systick interrupt
    enable_systick();


    while(1) {
        __asm__("wfi");
    }
}

