	//Odd arithmetic instructions
	//ADC, RSB, RSC, SBC, SUB
	//DEPENDENT INSTRUCTIONS

#include "amber_registers.h"

	.section .text
	.globl   main

main:
	mov r0, #10
	mov r2, #5
	mov r3, #0
	mov r4, #1
	//setting up variables
	adc r5, r0, r4
	//no carry flag, so r5 should just have 11
	sbc r6, r5, r4
	//no carry flag, so r6 should have 9 (if carry flag is clear, subtract 1)
	rsc r7, r4, r5
	//no carry flag, so r7 should have 9

	cmp r5, #11
	bne testfail
	cmp r6, #9
	bne testfail
	cmp r7, #9
	bne testfail
	
	b testpass
