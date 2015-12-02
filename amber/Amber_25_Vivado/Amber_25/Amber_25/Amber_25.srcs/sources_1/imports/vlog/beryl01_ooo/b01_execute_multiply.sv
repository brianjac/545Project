//////////////////////////////////////////////////////////////////
//                                                              //
//  Multiplication Module for Amber 25 Core                     //
//                                                              //
//  This file is part of the Amber project                      //
//  http://www.opencores.org/project,amber                      //
//                                                              //
//  Description                                                 //
//  64-bit Booth signed or unsigned multiply and                //
//  multiply-accumulate supported. It takes about 38 clock      //
//  cycles to complete an operation.                            //
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



// bit 0 go, bit 1 accumulate
// Command:
//  4'b01 :  MUL   - 32 bit multiplication
//  4'b11 :  MLA   - 32 bit multiply and accumulate
//
//  34-bit Booth adder
//  The adder needs to be 34 bit to deal with signed and unsigned 32-bit
//  multiplication inputs. This adds 1 extra bit. Then to deal with the
//  case of two max negative numbers another bit is required.
//
/*.o_instr_valid_mult(o_instr_valid_mult),
	.o_rn_mult(o_rn_mult),
	.o_rs_mult(o_rs_mult),
	.o_rm_mult(o_rm_mult),
	.o_multiply_function_mult(o_multiply_function_mult),
	.o_use_carry_in_mult(o_use_carry_in_mult), //TODO remove from reservation station
	.o_pc_wen_mult(o_pc_wen_mult), //TODO remove from reservation station
	.o_status_bits_flags_wen_mult(o_status_bits_flags_wen_mult), //TODO remove from reservation station
	.o_rd_tag_mult(o_rd_tag_mult),
	.i_mult_valid(i_mult_valid),
	.i_mult_tag(i_mult_tag),
	.i_mult_data(i_mult_data),*/
	//TODO ensure status_bits_mult exists
	//note: i don't think we actually care about carry-in! :-D
	//further note: need to be careful to only update N and Z flags and only if the S bit is set

//Note that the spec guarantees we'll never have r15 as the destination register for a MUL or MLA

typedef struct packed {
	logic valid;
	logic fctn;
	logic [31:0] rn;
	logic [5:0] tag;
} multiply_line;

module b01_execute_multiply (
input logic                 i_clk,
input logic					i_rst,
//input                       i_core_stall, //we don't care about an upstream core stall as we still want to retire and dispatch waiting instructions
input logic					i_instr_valid,

input logic 	[31:0]      i_rs,         // input 'a'
input logic 	[31:0]      i_rm,         // input 'b'
input logic 	[31:0]		i_rn,		  // input for optional accumulate value
input logic 	           	i_function,   // 0 is multiply, 1 is multiply_accumulate
input logic		[5:0]		i_rd_tag,

output logic				o_valid,
output logic	[5:0]		o_tag,
output logic	[31:0]      o_data,
output logic	[1:0]       o_flags       // [1] = N, [0] = Z
);

logic [63:0] mult_result_full;
logic [31:0] mult_result;
logic [31:0] data_nxt;
multiply_line [4:0] mult_pipeline_data;

mult_gen_0 multiplier(	.CLK(i_clk),
						.A(i_rs),
						.B(i_rm),
						.P(mult_result_full));
						
assign mult_result = mult_result_full[31:0];
assign data_nxt = mult_pipeline_data[0].fctn ? mult_result+mult_pipeline_data[0].rn : mult_result;

//State machine for multiply line
always_ff @(posedge i_clk, posedge i_rst) begin
	if (i_rst) begin
		mult_pipeline_data <= 'd0;
		o_valid <= 'd0;
		o_tag <= 'd0;
		o_data <= 'd0;
		o_flags <= 2'b11;
	end
	else begin
		//update pipeline data
		mult_pipeline_data[4].valid <= i_instr_valid;
		mult_pipeline_data[4].fctn <= i_function;
		mult_pipeline_data[4].rn <= i_rn;
		mult_pipeline_data[4].tag <= i_rd_tag;
		for (int i=0; i<4; i=i+1) mult_pipeline_data[i] <= mult_pipeline_data[i+1];
		
		//module output
		o_valid <= mult_pipeline_data[0].valid;
		o_tag <= mult_pipeline_data[0].tag;
		o_data <= mult_pipeline_data[0].fctn ? mult_result+mult_pipeline_data[0].rn : mult_result;
		o_flags[1] <= data_nxt[31]; //N bit
		o_flags[0] <= (data_nxt == 32'd0); //Z bit
	end
end

//TODO add more-efficient (and combinational) adder instead of simple '+', or else make that another pipeline stage at the end (to either accumulate or not, but do the add-or-don't-add on another clock cycle)

endmodule