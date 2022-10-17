.global _start
.global systick_handler
.syntax unified

.equ STACK_HIGH, 0x20010000


Vectors:
    .word STACK_HIGH
    .word _start + 1
    .space 0x34
    .word systick_handler


