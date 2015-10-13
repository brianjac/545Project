//Amanda's Amber 2 Core Instruction Test
//
// Description:
// Tests compare and compare negative
//
//
// Author(s):
//     - Amanda Marano, amarano@andrew.cmu.edu
//


#include "amber_registers.h"
#include "amber_macros.h"

	.section .text
	.global main

main:
	//check that subtracting 1 from lowest negative gets highest positive
        mov r1, #0x8000000
	cmp r1, #0x1
	bmi testfail
	bcs testfail

	cmp r1, #0x8000000
	bne testfail

	mov r1, #0x7fffffff
	cmn r1, 0x1
	bpl testfail
	bcc testfail

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
