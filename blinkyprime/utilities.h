#ifndef UTILITIES_H
#define UTILITIES_H

#include <stdint.h>

/* Base adresses */
uint32_t const GPIOB_BASE;
uint32_t const RCC_BASE;

/* GPIOB adresses */
static volatile uint32_t * const GPIOB_MODER;
static volatile uint32_t * const GPIOB_ODR;

/* RCC adresses */
static volatile uint32_t * const RCC_AHB2ENR;
static volatile uint32_t * const RCC_CR;
static volatile uint32_t * const RCC_CFGR;

/* Systick adresses */
static volatile uint32_t * const STK_CTRL;
static volatile uint32_t * const STK_LOAD; 
static volatile uint32_t * const STK_VAL;



void led_enable(); /* enables the GPIOB Port and configures PIN 3 as output (LED)*/
void turn_on_led(); /* turns on the GPIOB PIN 3 (LED)*/
void turn_off_led(); /* turns off the GPIOB PIN 3 (LED)*/
void toggle_led(); /* toggle the GPIOB PIN 3 (LED)*/

void configure_sys_clock(); /*configure clock*/
void enable_systick(); /* enables and configures the systick interrupt */

#endif