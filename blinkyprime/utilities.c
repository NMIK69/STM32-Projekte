#include "utilities.h"


// Base adresses
uint32_t const RCC_BASE = 0x40021000;
uint32_t const GPIOB_BASE = 0x48000400;
uint32_t const STK_BASE = 0xE000E010;


// gpiob adresses
static volatile uint32_t * const GPIOB_MODER = (uint32_t *)(GPIOB_BASE + 0x00);
static volatile uint32_t * const GPIOB_ODR = (uint32_t *)(GPIOB_BASE + 0x14);

// rcc adresses
static volatile uint32_t * const RCC_AHB2ENR = (uint32_t *)(RCC_BASE + 0x4C);
static volatile uint32_t * const RCC_CR = (uint32_t *)(RCC_BASE + 0x00);
static volatile uint32_t * const RCC_CFGR = (uint32_t *)(RCC_BASE + 0x08);


// systick timer adresses
static volatile uint32_t * const STK_CTRL = (uint32_t *)(STK_BASE + 0x00);
static volatile uint32_t * const STK_LOAD = (uint32_t *)(STK_BASE + 0x04); 
static volatile uint32_t * const STK_VAL = (uint32_t *)(STK_BASE + 0x08);


/*SYSTEM CLOCK CONFIGURE FUCNTIONS START*/
void configure_sys_clock() {
    //set clock to hsi 16 MHz
    *RCC_CR |= (1<<8);

    // wait untiil ready
    while(!(*RCC_CR & (0x0400)));

    // switch clock
    *RCC_CFGR &= ~(0x03);
    *RCC_CFGR |= (0x01);

    // wait until switched
    while((*RCC_CFGR & (0x0C)) != 0x04);
}
/*SYSTEM CLOCK CONFIGURE FUCNTIONS END*/


/*LED FUNTCIONS START*/
void led_enable() {
    // enable clock for port b
    *RCC_AHB2ENR |= (1<<1);

    // set mode to output
    *GPIOB_MODER &= ~(3<<6);
    *GPIOB_MODER |= (1<<6);
}

void turn_on_led() {
    *GPIOB_ODR |= (1<<3);
}

void turn_off_led(){
    *GPIOB_ODR &= ~(1<<3);
}

void toggle_led() {
    *GPIOB_ODR ^= (1<<3);
}
/*LED FUNTCIONS END*/



/* SYSTICK HANDLER FUNCTIONS START*/
void enable_systick() {

    // set reload value to 16_000_000 for a 1 Hz delay and reset current value
    *STK_LOAD = 0x0F42400;
    *STK_VAL = 0x00;

    // enable counter and tick interrupt and set clock to processor clock 
    *STK_CTRL |= ( (1<<0) | (1<<1) | (1<<2));
}
/* SYSTICK HANDLER FUNCTIONS END*/