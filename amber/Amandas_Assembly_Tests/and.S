/*****************************************************************
//                                                              //
//  Amber 2 Core Instruction Test                               //
//                                                              //
//  This file is part of the Amber project                      //
//  http://www.opencores.org/project,amber                      //
//                                                              //
//  Description                                                 //
//  Tests add with carry                                        //
//                                                              //
//  Author(s):                                                  //
//      - Conor Santifort, csantifort.amber@gmail.com           //
//                                                              //
//////////////////////////////////////////////////////////////////
//                                                              //
// Copyright (C) 2010 Authors and OPENCORES.ORG                 //
//                                                              //
// This source file may be used and distributed without         //
// restriction provided that this copyright statement is not    //
// removed from the file and that any derivative work contains  //
// the original copyright notice and the associated disclaimer. //
//                                                              //
// This source file is free software; you can redistribute it   //
// and/or modify it under the terms of the GNU Lesser General   //
// Public License as published by the Free Software Foundation; //
// either version 2.1 of the License, or (at your option) any   //
// later version.                                               //
//                                                              //
// This source is distributed in the hope that it will be       //
// useful, but WITHOUT ANY WARRANTY; without even the implied   //
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU Lesser General Public License for more //
// details.                                                     //
//                                                              //
// You should have received a copy of the GNU Lesser General    //
// Public License along with this source; if not, download it   //
// from http://www.opencores.org/lgpl.shtml                     //
//                                                              //
*****************************************************************/

#include "amber_registers.h"
#include "amber_macros.h"

	.section .text
	.globl	main        
main:
        // sets all four flags
        // Cant use p version of instrustion in 32-bit CPU because it writes the upper 4 bits of PC
        teqp    pc, #0xf0000000

        // 
        // Check that they are set
        bvc     testfail  // V flag 
        bpl     testfail  // N flag
        bne     testfail  // Z flag
        bcc     testfail  // C flag

        mov     r1, #0x0000005a

        // should not unset the V flag value
        // the carry flag is set to the shifter carry out
        ands    r2, r1, #0x00000055

        // Check the V flag is still set, the Z flag is clear, the C flag 
        // is unchanged, and the N flag gets cleared
        bvc     testfail //V flag
        beq     testfail //Z flag
        bcc     testfail //C flag
        bmi     testfail //N flag

        tst     r2, #0x00000050
        beq     testfail

	//sets all 4 flags
	teqp    pc, #0xf0000000

	//it was already checked that this set all of the flags

	mov r3, #0x80000000

	// should not unset the V flag value
	// the carry flag is set to the shifter carry out
	ands r4, r3, #0x80000000

	//check the V flag is still set, the Z flag is clear,
	//the C flag is unchanged, the N flag is still set
	bvc    testfail //V flag
	beq    testfail //Z flag
	bcc    testfail //C flag
	bpl    testfail //N flag

	tst    r4, #0x80000000
	beq    testfail

		//sets all 4 flags
	teqp    pc, #0xf0000000

	//it was already checked that this set all of the flags

	mov r3, #0x80000000

	// should not unset the V flag value
	// the carry flag is set to the shifter carry out
	ands r4, r3, #0x7fffffff

	//check the V flag is still set, the Z flag is still set,
	//the C flag is unchanged, the N flag is clear
	bvc    testfail //V flag
	bne    testfail //Z flag
	bcc    testfail //C flag
	bmi    testfail //N flag

	tst    r4, #0x00000000
	beq    testfail
	
        b       testpass
                
testfail:
        ldr     r11, AdrTestStatus
        str     r10, [r11]
        b       testfail
        
testpass:             
        ldr     r11, AdrTestStatus
        mov     r10, #17
        str     r10, [r11]
        b       testpass
        

AdrTestStatus:  .word  ADR_AMBER_TEST_STATUS

