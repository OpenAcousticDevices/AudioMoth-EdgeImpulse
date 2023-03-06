.syntax unified


.section .text
.thumb
.global topERAM
.global maxHeapAddress
.global external_ei_classify
external_ei_classify:
    PUSH {r4-r8}
    LDR r7, =maxHeapAddress @ load address of global variable to %r7
    LDR r8, =topERAM @ load address of global variable to %r8
    MOV r4, sp @ copy %sp to %r4
    MOV r5, lr @ copy %lr to $r5
    STR r13, [r7] @ store %sp value to global variable 
    LDR r13, [r8] @ move stack pointer to top of ERAM
    BLX ei_classify
    MOV sp, r4 @ copy %r4 to %sp
    MOV lr, r5 @ copy %r5 to %lr
    POP {r4-r8}
    BX lr
