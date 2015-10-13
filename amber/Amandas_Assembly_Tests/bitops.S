// Amanda's Amber 2 Core Instruction Test
//
// Description:
//	Tests these bitwise operators: 
//		bic (bit clear)
//		eor (exclusive or)
//		orr (bitwise or)
//
// Author(s):
//	- Amanda Marano, amarano@andrew.cmu.edu
//

#include "amber_registers.h"	
#include "amber_macros.h"

	.section .text
	.global main

main:
	mov r1, #0x00000005

	bics r2, r1, #0xfffffffa
	bne testfail

	mov r6, #0x1
	bics r7, r6, #0x0
	cmp r6, r7
	bne testfail

	orr r3, r1, #0x0000000a
	cmp r3, #0xf
	bne testfail

	mov r4, #0x80000000
	eors r5, r4, #0x80000000
	bne testfail

	b testpass


testfail:
	ldr   r11, AdrTestStatus
	str   r10, [r11]
        b     testfail

	
testpass:
	ldr   r11, AdrTestStatus
	mov   r10, #17
	str   r10, [r11]
	b     testpass

AdrTestStatus:	.word ADR_AMBER_TEST_STATUS
