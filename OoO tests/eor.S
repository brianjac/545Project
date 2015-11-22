	//Bitwise operations
	//all independent except for initial setup

#include "amber_registers.h"

	.section .text
	.globl   main

main:
	mov r0, #0xffff
	mov r1, #0x1
	mvn r4, r1
	//r4 should contain 0xfffffffe
	//set up variables -- done

	eor r3, r0, r1
	//r3 should now contain 0xfffe
	eor r5, r0, r4
	//r5 should now contain 0xffff0001
	
	cmp r4, #0xfffffffe
	bne testfail
	cmp r3, #0xfffe
	bne testfail
	cmp r5, #0xffff0001
	bne testfail
	
	b testpass
