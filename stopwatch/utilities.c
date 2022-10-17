#include "utilities.h"


uint16_t baud_rate = 16000000 / 115200; 

// Base adresses
uint32_t const GPIOA_BASE = 0x48000000;
uint32_t const USART2_BASE = 0x40004400;
uint32_t const RCC_BASE = 0x40021000;
uint32_t const GPIOB_BASE = 0x48000400;
uint32_t const STK_BASE = 0xE000E010;


// gpiob adresses
static volatile uint32_t * const GPIOB_MODER = (uint32_t *)(GPIOB_BASE + 0x00);
static volatile uint32_t * const GPIOB_ODR = (uint32_t *)(GPIOB_BASE + 0x14);

// rcc adresses
static volatile uint32_t * const RCC_AHB2ENR = (uint32_t *)(RCC_BASE + 0x4C);
static volatile uint32_t * const RCC_APB1ENR1 = (uint32_t *)(RCC_BASE + 0x58);
static volatile uint32_t * const RCC_CR = (uint32_t *)(RCC_BASE + 0x00);
static volatile uint32_t * const RCC_CFGR = (uint32_t *)(RCC_BASE + 0x08);

// usart2 adresses
static volatile uint32_t * const USART2_CR1 = (uint32_t *)(USART2_BASE + 0x00);
static volatile uint32_t * const USART2_BRR = (uint32_t *)(USART2_BASE + 0x0C);
static volatile uint32_t * const USART2_CR2 = (uint32_t *)(USART2_BASE + 0x04);
static volatile uint32_t * const USART2_CR3 = (uint32_t *)(USART2_BASE + 0x08);

static volatile uint32_t * const USART2_TDR = (uint32_t *)(USART2_BASE + 0x28);
static volatile uint32_t * const USART2_ISR = (uint32_t *)(USART2_BASE + 0x1C);

// gpioa adresses
static volatile uint32_t * const GPIOA_MODER = (uint32_t *)(GPIOA_BASE + 0x00);
static volatile uint32_t * const GPIOA_IDR = (uint32_t *)(GPIOA_BASE + 0x10);
static volatile uint32_t * const GPIOA_OSPEEDR = (uint32_t *)(GPIOA_BASE + 0x08);
static volatile uint32_t * const GPIOA_AFRL = (uint32_t *)(GPIOA_BASE + 0x20);
static volatile uint32_t * const GPIOA_AFRH = (uint32_t *)(GPIOA_BASE + 0x24);
static volatile uint32_t * const GPIOA_OTYPER = (uint32_t *)(GPIOA_BASE + 0x04);


// systick timer adresses
static volatile uint32_t * const STK_CTRL = (uint32_t *)(STK_BASE + 0x00);
static volatile uint32_t * const STK_LOAD = (uint32_t *)(STK_BASE + 0x04); 
static volatile uint32_t * const STK_VAL = (uint32_t *)(STK_BASE + 0x08);



/*SYSTEM CLOCK CONFIGURE FUCNTIONS START*/
void configure_sys_clock() {
    //set clock to hsi 16 mhz
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



/* USART AND TRANSMIT FUNCTIONS SECTION START */
void usart2_putc(uint8_t c) {

    // transmitt a character
    while(!(*(USART2_ISR) & (1<<7))); // wait untill the TDR is empty to not overwrite previous data

    *USART2_TDR = c; // transmit 

    while(!(*(USART2_ISR) & (1<<6))); // wait untill transmition is complete
}

void usart2_puts(const char* s, unsigned int size) {

    // transmit a string 
    for(unsigned int i = 0; i < size; i++) {
        usart2_putc(s[i]);
    }

}

void usart2_putx(uint32_t data) {
    // convter data (integer) to a string (ASCII)

    uint8_t data_as_string[33];
	data_as_string[0] = '0';
	uint16_t i = 0;
	
	//turn the data to ascii
	while(data > 0) {
		uint8_t last_digit = data % 10;
		data = data / 10;

		data_as_string[i] = '0' + last_digit;
		i++;
	}

	int j = 0;
	int k = i - 1;


	// reverse the array so that the numbers are in the correct order
	while(j < k) {
		uint8_t temp = data_as_string[j];
		data_as_string[j] = data_as_string[k];
		data_as_string[k] = temp;

		j++;
		k--;
	}

	if(i == 0) i = 1;

    // send the data via USART
    usart2_puts(data_as_string, i);
}

void usart2_enable() {

    // enable clock for UART
    *RCC_APB1ENR1 |= (1<<17);

    // enable clock for GPIOA 
    *RCC_AHB2ENR |= (1<<0);

    // enable TX Pin (Pin 2 to alternate function)
    *GPIOA_MODER &= ~(3 << (4));
    *GPIOA_MODER |= (2 << 4);

    // set output type to high
    *GPIOA_OTYPER |= (0x01 << 2);

    // set ouput speed to high
    *GPIOA_OSPEEDR &= ~(3 << 4);
    *GPIOA_OSPEEDR |= (3 << 4);

    // set alternate function 
    *GPIOA_AFRL &= ~( (0x0F << 8));
    *GPIOA_AFRL |= (0x07 << 8);


    // set baud rate
    *USART2_BRR = baud_rate;

    // enable USART2 and  transmiter
    *USART2_CR1 |= ( (1<<0) | (1<<3) );
}

void send_time(uint32_t data) {
    // convter data (integer) to a string (ASCII)

    uint8_t data_as_string[34];
	data_as_string[0] = '0';
    data_as_string[1] = '0';
    data_as_string[2] = '0';
    data_as_string[3] = '.';
    data_as_string[4] = '0';
	uint16_t i = 0;
	
	//turn the data to ascii
	while(data > 0) {
		uint8_t last_digit = data % 10;
		data = data / 10;

        if(i == 3) i = 4;
		data_as_string[i] = '0' + last_digit;
		i++;
	}

    // set boundaries for the coming reversal 
	int j = 0;
	int k = i - 1;
    if(i < 4) {
        k = 4;
        i = 5;
    }


	// reverse the array so that the numbers are in the correct order
	while(j < k) {
		uint8_t temp = data_as_string[j];
		data_as_string[j] = data_as_string[k];
		data_as_string[k] = temp;

		j++;
		k--;
	}

	if(i == 0) i = 1;


    // send the data via USART
    usart2_puts(data_as_string, i);
    usart2_putc('\n');
}
/* USART AND TRANSMIT FUNCTIONS SECTION END */



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

    // set reload value to 16_000 (so systick handler gets called every ms) 
    *STK_LOAD = 0x03E80;
    
    // reset current value
    *STK_VAL = 0x00;

    // enable counter and tick interrupt
    *STK_CTRL |= ( (1<<0) | (1<<1) | (1<<2));
}
/* SYSTICK HANDLER FUNCTIONS END*/



/*BUTTON HANDLER FUNCTIONS START*/
void enable_btn_pin() {

    // set gpioa pin 1 as input
    *GPIOA_MODER &= ~(0x03 << 2);
}

uint8_t get_btn_pin_state() {
    if((*GPIOA_IDR & (1<<1)) == 0) {
        return 0;
    }
    else {
        return 1;
    }
}
/*BUTTON HANDLER FUNCTIONS END*/
