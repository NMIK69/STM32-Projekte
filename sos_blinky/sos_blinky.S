.global _start

    .syntax unified

    .equ STACK_HIGH,          0x20010000

    /* Relevant addresses and values for the RCC register bank */
    .equ RCC_BASE,            0x40021000
    .equ RCC_AHB2ENR,         RCC_BASE + 0x4c
    .equ RCC_AHB2ENR_PB,      (1<<1)

    /* Relevant addresses and values of the GPIOB register bank */
    .equ GPIOB_BASE,          0x48000400
    .equ GPIOB_MODER,         GPIOB_BASE + 0x00
    .equ GPIO_MODER_PIN0,     1
    .equ GPIOB_ODR,           GPIOB_BASE + 0x14
    .equ GPIO_PIN3_HIGH,      1
    .equ GPIO_PIN3_LOW,       0x0

    
    // Short signal 250 ms 
    .equ SHORT_DELAY,   0x3D090

    // Long signal 750 ms
    .equ LONG_DELAY,    0xB71B0

    // Pause Signal 1750 ms
    .equ PAUSE_DELAY,   0x1AB3F0

    

Vectors:
    .word STACK_HIGH  /* initial value of the sp */
    .word _start + 1
  

_start:
    /* Get contents of AHB1 bus clock enable register */
    ldr     r0, =RCC_AHB2ENR
    ldr     r1, [r0]    
    /* Set bit to enable clock for GPIO port B */
    ldr     r2, =RCC_AHB2ENR_PB
    orrs    r1, r2
    str     r1, [r0]    
    /* Configure GPIO pin as output */
    ldr     r0, =GPIOB_MODER
    ldr     r1, =GPIO_MODER_PIN0
    str     r1, [r0]    
    /* Load address of GPIOB output data register in r0 */
    ldr     r0, =GPIOB_ODR
    /* Load bitmask of gpio pin3 in r1 */
    ldr     r1, =GPIO_PIN3_HIGH



    b .sos


.sos:

    bl morse_S
    bl morse_O
    bl morse_S

    bl wait_pause

    b .sos



.type turn_on_LED, %function
turn_on_LED:

    ldr r1, =GPIO_PIN3_HIGH
    ldr r0, =GPIOB_ODR
    str r1, [r0]

    bx lr


.type turn_off_LED, %function
turn_off_LED:

    ldr r1, =GPIO_PIN3_LOW
    ldr r0, =GPIOB_ODR
    str r1, [r0]

    bx lr


.type wait_short, %function
wait_short:

    ldr r2, =SHORT_DELAY
    .loop:
        subs r2, r2, #1
        bne .loop
    
    bx lr 


.type wait_long, %function
wait_long:

    ldr r2, =LONG_DELAY
    .loop1:
        subs r2, r2, #1
        bne .loop1
    
    bx lr 


.type wait_pause, %function
wait_pause:

    ldr r2, =PAUSE_DELAY
    .loop2:
        subs r2, r2, #1
        bne .loop2
    
    bx lr 


.type morse_S, %function
morse_S:
    push {lr}

    bl turn_on_LED
    bl wait_short
    bl turn_off_LED
    bl wait_short

    bl turn_on_LED
    bl wait_short
    bl turn_off_LED
    bl wait_short

    bl turn_on_LED
    bl wait_short
    bl turn_off_LED
    bl wait_short

    pop {lr}
    bx lr


.type morse_O, %function
morse_O:

    push {lr}

    bl turn_on_LED
    bl wait_long
    bl turn_off_LED
    bl wait_long


    bl turn_on_LED
    bl wait_long
    bl turn_off_LED
    bl wait_long

    bl turn_on_LED
    bl wait_long
    bl turn_off_LED
    bl wait_long

    pop {lr}
    bx lr
