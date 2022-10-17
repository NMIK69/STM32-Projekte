#ifndef UTILITIES_H
#define UTILITIES_H

#include <stdint.h>

/* Base adresses */
uint32_t const GPIOA_BASE;
uint32_t const GPIOB_BASE;
uint32_t const USART2_BASE;
uint32_t const RCC_BASE;

/* GPIOB adresses */
static volatile uint32_t * const GPIOB_MODER;
static volatile uint32_t * const GPIOB_ODR;

/* RCC adresses */
static volatile uint32_t * const RCC_AHB2ENR;
static volatile uint32_t * const RCC_APB1ENR1;
static volatile uint32_t * const RCC_CR;
static volatile uint32_t * const RCC_CFGR;

/* USART2 adresses */
static volatile uint32_t * const USART2_CR1;
static volatile uint32_t * const USART2_BRR;
static volatile uint32_t * const USART2_CR2;
static volatile uint32_t * const USART2_CR3;

static volatile uint32_t * const USART2_TDR;
static volatile uint32_t * const USART2_ISR;

/* GPIOA adresses */
static volatile uint32_t * const GPIOA_MODER;
static volatile uint32_t * const GPIOA_IDR;
static volatile uint32_t * const GPIOA_OSPEED;
static volatile uint32_t * const GPIOA_AFRL;
static volatile uint32_t * const GPIOA_AFRH;
static volatile uint32_t * const GPIOA_OTYPER;

/* Systick adresses */
static volatile uint32_t * const STK_CTRL;
static volatile uint32_t * const STK_LOAD; 
static volatile uint32_t * const STK_VAL;


void configure_sys_clock(); /* confugre the system clock the HSI 16MHz*/


void usart2_putc(uint8_t c); /* transmit a single 8 bit character via usart2*/
void usart2_puts(const char* str, unsigned int size); /* transmit a string via usart2 */
void usart2_putx(uint32_t val); /* transmit a digit as ASCII via usart2*/
void usart2_enable();    /* enabeles communication via usart2*/
void send_time(); /*send time data more readable*/

void led_enable(); /* enables the GPIOB Port and configures PIN 3 as output (LED)*/
void turn_on_led(); /* turns on the GPIOB PIN 3 (LED)*/
void turn_off_led(); /* turns off the GPIOB PIN 3 (LED)*/
void toggle_led(); /* toggle the GPIOB PIN 3 (LED)*/


uint8_t get_btn_pin_state(); /* get the state of the Button Pin*/
void enable_btn_pin(); /* enable the GPIOA PIN 1 as input (Button Pin)*/


void enable_systick(); /* enables and configures the systick interrupt */

#endif