	//rsb -- DEPENDENT INSTRUCTIONS

#include "amber_registers.h"

	.section .text
	.globl   main

main:
	mov r0, #16
	mov r1, #32

	rsb r2, r0, r1
	//r2 should contain 16

	rsb r3, r1, r2
	//r3 should contain -16
	
	cmp r2, #16
	moveq r5, #0x1
	cmp r3, #-16
	moveq r6, #0x1
