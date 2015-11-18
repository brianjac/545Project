//////////////////////////////////////////////////////////////////
//                                                              //
//  Execute stage of Amber 25 Core                              //
//                                                              //
//  This file is part of the Amber project                      //
//  http://www.opencores.org/project,amber                      //
//                                                              //
//  Description                                                 //
//  Executes instructions. Instantiates the register file, ALU  //
//  multiplication unit and barrel shifter. This stage is       //
//  relitively simple. All the complex stuff is done in the     //
//  decode stage.                                               //
//                                                              //
//  Author(s):                                                  //
//      - Conor Santifort, csantifort.amber@gmail.com           //
//                                                              //
//////////////////////////////////////////////////////////////////
//                                                              //
// Copyright (C) 2011 Authors and OPENCORES.ORG                 //
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
//////////////////////////////////////////////////////////////////


module b01_execute_alu (

input                       i_clk,
input						i_rst,
input                       i_core_stall,               // stall all stages of the Amber core at the same time

input logic					i_instr_valid,				//make outputs benign if no incoming instruction
//o_rd_sel no longer needed. this is handled now by tag comparison
//output logic  [8:0]         o_rd_sel, 			      	// The destination register for a load instruction
output logic				o_rd_valid, //whether or not the data on the output of this stage is valid
output logic [5:0]			o_rd_tag,
output logic [31:0]			o_rd_data,
output logic [3:0]			o_alu_flags,
output logic				o_pc_wen,

//TODO confirm the op1/rn,op2/rs,op3/rm naming scheme as per Dispatch definitions/logic and modify if needed
//Note that we don't care about the operand tags and valid bits; we only care about the dest reg tag.
//Operand 1 data
input logic [31:0] i_rn,

//Operand 2 data
input logic [31:0] i_rs,

//Operand 3 data (used if we write to a reg, one operand is a reg, and the other operand is a reg shifted by another reg)
input logic [31:0] i_rm,

//CCR data, for carry bit
input logic [3:0] i_status_bits_flags,
input logic i_use_carry_in,

//Control signals for the ALU stage
input logic [31:0] i_imm32,
input logic [4:0] i_imm_shift_amount,
input logic i_shift_imm_zero,
//input logic [8:0] i_exec_load_rd,
input logic [1:0] i_barrel_shift_amount_sel,
input logic [1:0] i_barrel_shift_data_sel,
input logic [1:0] i_barrel_shift_function,
input logic [8:0] i_alu_function,
input logic i_pc_wen, //will need to be passed through to the output
//input logic [14:0] i_reg_bank_wen,
//input logic i_status_bits_flags_wen, //shouldn't be needed in this module anymore
/*input logic i_status_bits_mode_wen,
input logic i_status_bits_irq_mask_wen,
input logic i_status_bits_firq_mask_wen*/

input logic [5:0] i_rd_tag //TODO note: might need to change the reservation station structure to add this

);

`include "b01_localparams.vh"
`include "b01_functions.vh"

// ========================================================
// Internal signals
// ========================================================
wire [31:0]         alu_out;
wire [3:0]          alu_flags;
wire [7:0]          shift_amount;
wire [31:0]         barrel_shift_in;
wire [31:0]         barrel_shift_out;
wire                barrel_shift_carry;
wire                barrel_shift_stall;
wire                barrel_shift_carry_alu;


//TODO note: need to handle LDM instruction stuff purely in Mem stage. Retirement of this one will be tricky.
//What I'll probably do is just have the Mem stage do everything purely in an internal state machine and only "retire" the instruction after it's done all the loads/stores.

wire                carry_in;


// ========================================================
// Barrel Shift Amount Select
// ========================================================
// An immediate shift value of 0 is translated into 32
assign shift_amount = i_barrel_shift_amount_sel == 2'd0 ? 8'd0                         :
                      i_barrel_shift_amount_sel == 2'd1 ? i_rs[7:0]                    :
                                                          {3'd0, i_imm_shift_amount  } ;


// ========================================================
// Barrel Shift Data Select
// ========================================================
assign barrel_shift_in = i_barrel_shift_data_sel == 2'd0 ? i_imm32 : i_rm ;


// ========================================================
// Register Update
// ========================================================

//note that predicated execution is handled in Dispatch, by simply not adding the instruction to a reservation station
always_ff @(posedge i_rst, posedge i_clk) begin
	if (i_rst) begin
		o_rd_valid <= 1'b0;
		o_rd_tag <= '0;
		o_rd_data <= '0;
		o_alu_flags <= 4'b1111;
	end
	else begin
		o_rd_valid <= i_core_stall ? 1'b0 : i_instr_valid; //TODO ensure this is okay not being combinational in the case of a core stall
		o_rd_tag <= i_core_stall ? o_rd_tag : i_rd_tag;
		o_rd_data <= i_core_stall ? o_rd_data : alu_out;
		o_alu_flags <= i_core_stall ? o_alu_flags : alu_flags;
	end
end


// ========================================================
// Instantiate Barrel Shift
// ========================================================
assign carry_in = i_use_carry_in ? i_status_bits_flags[1] : 1'd0;

b01_barrel_shift u_barrel_shift  (
    .i_clk            ( i_clk                     ),
    .i_in             ( barrel_shift_in           ),
    .i_carry_in       ( carry_in                  ),
    .i_shift_amount   ( shift_amount              ),
    .i_shift_imm_zero ( i_shift_imm_zero          ),
    .i_function       ( i_barrel_shift_function   ),

    .o_out            ( barrel_shift_out          ),
    .o_carry_out      ( barrel_shift_carry        ),
    .o_stall          ( barrel_shift_stall        ));


// ========================================================
// Instantiate ALU
// ========================================================
assign barrel_shift_carry_alu =  i_barrel_shift_data_sel == 2'd0 ?
                                  (i_imm_shift_amount[4:1] == 0 ? i_status_bits_flags[1] : i_imm32[31]) :
                                   barrel_shift_carry;

b01_alu u_alu (
    .i_a_in                 ( i_rn                    ),
    .i_b_in                 ( barrel_shift_out        ),
    .i_barrel_shift_carry   ( barrel_shift_carry_alu  ),
    .i_status_bits_carry    ( status_bits_flags[1]    ),
    .i_function             ( i_alu_function          ),

    .o_out                  ( alu_out                 ),
    .o_flags                ( alu_flags               ));


endmodule


