.global _start
.syntax unified


.equ STACK_HIGH,          0x20010000
/* Relevant addresses and values for the RCC register bank */
.equ RCC_BASE,            0x40021000
.equ RCC_AHB2ENR,         RCC_BASE + 0x4c
.equ RCC_AHB2ENR_PBA,     0b11


/* Relevant addresses and values of the GPIOB register bank */
.equ GPIOB_BASE,          0x48000400
.equ GPIOB_MODER,         GPIOB_BASE + 0x00
.equ GPIOB_MODER_PIN0,    0b1
.equ GPIOB_ODR,           GPIOB_BASE + 0x14

.equ GPIOB_PIN0_HIGH,      0b1
.equ GPIOB_PIN0_LOW,       0x0


// Relevant addresses and values of the GPIOA register bank
.equ GPIOA_BASE,        0x48000000
.equ GPIOA_MODER,       GPIOA_BASE + 0x00
.equ GPIOA_MODER_PIN1_INVERSE,   0b1100
.equ GPIOA_MODER_RVAL,           0xABFFFFFF // reset value for moder

.equ GPIOA_IDR,         GPIOA_BASE + 0x10
.equ GPIOA_PIN1_ON,     (1<<1)


// Short signal 250 ms 
.equ SHORT_DELAY,   0x3D090
// Long signal 750 ms
.equ LONG_DELAY,    0xB71B0
// Pause Signal 1750 ms
.equ PAUSE_DELAY,   0x1AB3F0


/*Systick enbale adresses */
.equ STK_CTRL, 0xE000E010
.equ STK_LOAD, 0xE000E014
.equ STK_VAL, 0xE000E018



Vectors:
    .word STACK_HIGH
    .word _start + 1
    .space 0x34
    .word systick_handler


    

_start:

    ldr r5, =0 // counts the times the button is pressed
    ldr r6, =0 // counts the times the button is not pressed
    ldr r7, =0 // holds the mode flag
    ldr r8, =0 // holds the button currently pressed flag
    ldr r9, =0 // holds the mode change flag

    bl enable_GPIO
    bl enable_systick


    b .main



.type enable_GPIO, %function
enable_GPIO:
    /* Get contents of AHB1 bus clock enable register */
    ldr     r0, =RCC_AHB2ENR
    ldr     r1, [r0]    
    /* Set bit to enable clock for GPIO port B and A */
    ldr     r2, =RCC_AHB2ENR_PBA
    orrs    r1, r2
    str     r1, [r0]  



    /* Configure GPIOB pin 3 as output */
    ldr     r0, =GPIOB_MODER
    ldr     r1, =GPIOB_MODER_PIN0
    str     r1, [r0]



    // configure GPIOA pin 1 as input
    ldr r0, =GPIOA_MODER
    ldr r1, =GPIOA_MODER_PIN1_INVERSE
    ldr r2, =GPIOA_MODER_RVAL
    BIC r3, r2, r1
    str r3, [r0]


    bx lr



.type enable_systick, %function
enable_systick:

    // set reload register
    ldr r1, =STK_LOAD
    ldr r0, =0x2710
    str r0, [r1]

    // set current value
    ldr r1, =STK_VAL
    ldr r0, =0x00
    str r0, [r1]

    /* set control and status regiter
    set tickinit to 1 (bit 1)
    set enable counter to 1 (bit 0)
    set clocksource to processor clock (bit 2 = 1)
    so the first 3 bits need to be set 0b111 = 0x7  */
    ldr r1, =STK_CTRL
    ldr r0, =0x7
    str r0, [r1]

    bx lr

.type systick_handler, %function
.global systick_handler
systick_handler:


    push {lr}

    // check if the button is pressed
    ldr r0, =GPIOA_IDR
    // load content of the idr
    ldr r1, [r0]
    // load the mask 
    ldr r2, =GPIOA_PIN1_ON
    AND r3, r1, r2

    // check if pin is on high
    cmp r2, r3


    // if pin is on high add one to the high register
    it eq
    addeq r5, #1

    // else add one to the low register
    it ne
    addne r6, #1


    // if the high register is at cap then switch mode
    cmp r5, #50

    it eq
    bleq change_mode
    


    /* if the low register is at cap then reset both 
    registers to zero and set button press register to 0*/

    cmp r6, #50
    it eq
    bleq reset_registers


    pop {lr}
    bx lr




.type change_mode, %function
change_mode:

    // if the button is currently pressed dont change mode
    cmp r8, #1
    it eq
    bxeq lr

    /* switch modes
    if the mode value is bigger then 1 set it back to 0 */
    add r7, r7, #1
    cmp r7, #2

    it eq
    moveq r7, #0


    bx lr

.type sos, %function
sos:

    push {lr}

    // morse the S
    bl morse_S

    /* if the button has been presed recently return
    from this function and reset mode change register */
    cmp r7, #0
    itt eq
    popeq {lr}
    bxeq lr

    // morse the O
    bl morse_O

    cmp r7, #0
    itt eq
    popeq {lr}
    bxeq lr

    // morese the S
    bl morse_S

    // wait after signal
    bl wait_pause

    pop {lr}
    bx lr



.type turn_on_LED, %function
turn_on_LED:

    ldr r1, =GPIOB_PIN0_HIGH
    ldr r0, =GPIOB_ODR
    str r1, [r0]

    bx lr


.type turn_off_LED, %function
turn_off_LED:

    ldr r1, =GPIOB_PIN0_LOW
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


.type reset_registers, %function
reset_registers:
    mov r5, #0
    mov r6, #0
    mov r8, #0

    bx lr

.main:


    .mainloop:

        /* check if the mode register is on one or 0
        if on 0 sleep
        if on 1 blink sos */

        cmp r7, #1

        it eq
        bleq sos

        it ne
        blne turn_off_LED


        b .mainloop

   
    b .main
