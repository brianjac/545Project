	//Bitwise BIC
	//all independent except for initial setup

#include "amber_registers.h"

	.section .text
	.globl   main

main:
	mov r0, #0xffff
	mov r1, #0x1
	//set up variables -- done
	bic r2, r0, r1
	//r2 should now contain 0xfffe	

	cmp r2, #0xfffe
	bne testfail

	mov r3, #0xA
	mov r4, #0x5
	bic r5, r3, r4
	//r5 should now have 0xA
	bic r6, r4, r3
	//r6 should now have 0x5
	
	cmp r5, #0xA
	bne testfail
	cmp r6, #0x5
	bne testfail
	
	b testpass
