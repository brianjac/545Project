	//MOV, ADD, CMP instructions with flags
	//dependent instructions (ADD dependent on MOVs)

#include "amber_registers.h"

	.section .text
	.globl   main

main:
	mov r0, #0xffff
	mov r1, #0x1
	adds r2, r1, r0
	//carry out flag should be set
	cmp r2, #0x10000
	beq testpass

	b testfail
	
