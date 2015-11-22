	//MOV instruction with flags
	//independent and dependent instructions

#include "amber_registers.h"

	.section .text
	.globl   main

main:
	mov r0, #0xffff
	mov r1, #0x1
	mov r0, r0, LSL #4
	//r0 should now contain 0xffff0
	mov r1, r1, LSL #1
	//r1 should now contain 0x2
	cmp r0, #0xffff0
	bne testfail
	cmp r1, #0x2
	bne testfail

	b testpass
