.global _start
.syntax unified

.equ STACK_HIGH, 0x20010000


Vectors:
    .word STACK_HIGH
    .word _start + 1


