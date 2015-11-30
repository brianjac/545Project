//////////////////////////////////////////////////////////////////
//                                                              //
//  Memory Access - Instantiates the memory access stage        //
//  sub-modules of the Amber 25 Core                            //
//                                                              //
//  This file is part of the Amber project                      //
//  http://www.opencores.org/project,amber                      //
//                                                              //
//  Description                                                 //
//  Instantiates the Data Cache                                 //
//  Also contains a little bit of logic to decode memory        //
//  accesses to decide if they are cached or not                //
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
/*

	.o_instr_valid_mem(o_instr_valid_mem),
	.o_rn_mem(o_rn_mem), //TODO make this just i_address_mem
	.o_rs_mem(o_rs_mem), //TODO remove
	.o_rm_mem(o_rm_mem), //TODO remove
	.o_exclusive_mem(o_exclusive_mem), //TODO remove
	.o_pc_wen_mem(o_pc_wen_mem),
	.o_status_bits_flags_wen_mem(o_status_bits_flags_wen_mem),
	.o_byte_enable_sel_mem(o_byte_enable_sel_mem),
	.o_rd_tag_mem(o_rd_tag_mem),
	.i_mem_valid(i_mem_valid),
	.i_mem_tag(i_mem_tag),
	.i_mem_data(i_mem_data)
	*/
	
	/*
	New interface:
	o_instr_valid_mem
	o_address_mem [31:0]
	o_write_data_mem [31:0]
	o_op_type_mem [1:0] //00=read, 01=write, 10=swap
	o_byte_enable_sel_mem [3:0]
	o_rd_tag_mem [5:0]
	i_mem_ready
	i_mem_valid
	i_mem_tag [5:0]
	i_mem_data [31:0]
	i_mem_flags [3:0]
	*/
	
	/*
	Supported:
	ldm
	stm
	ldr
	ldrb
	str
	strb
	*/

module b01_execute_mem
(
input logic 			i_clk,
input logic				i_rst,
output logic 			o_ready,            // asserted high on the cycle when data_out_valid goes high and held high until another memop is requested

input logic				i_instr_valid,
input logic	 [31:0]		i_address,
input logic	 [31:0]		i_write_data,
input logic  [1:0]		i_op_type,
input logic  [3:0]		i_byte_enable,
input logic  [5:0]		i_rd_tag,

output logic			o_valid,
output logic [5:0]		o_tag,
output logic [31:0]		o_data,

// Wishbone accesses                                                         
output                      o_wb_req,      // Unached Request
output                      o_wb_write,             // Read=0, Write=1
output     [15:0]           o_wb_byte_enable,       // byte eable
output     [127:0]          o_wb_write_data,
output     [31:0]           o_wb_address,           // wb bus                                 
input                       i_wb_ready     // wishbone access complete and read data valid
input      [127:0]          i_wb_rdata,    // wb bus                          
);

`include "memory_configuration.vh"

//*** TODO note that memops don't set flags! (thus, address calculation ops shouldn't update flags either!)

//internal signals and registers
logic op_in_progress;
logic [1:0] op_type_current; //doesn't really need to be 2 bits, but w/e for now
logic is_swap_current; //op_type_current starts as R and changes to W if is_swap_current is latched high
logic [31:0] address_current;
logic [31:0] write_data_current;
logic [3:0] byte_enable_current;
logic [5:0] rd_tag_current;
logic swap_op_wstart; //high on the cycle when the write portion of a swap operation begins

assign swap_op_wstart = (op_type_current==2'b00 && is_swap_current && i_wb_ready);

//output logic
always_comb begin
	o_ready = ~i_instr_valid && (~op_in_progress || (op_in_progress && i_wb_ready && (~is_swap_current || op_type_current==2'b01)));
	o_valid = op_in_progress && ~op_type_current[0] && i_wb_ready;
	o_tag = rd_tag_current;
	o_data = 	address_current[3:2] == 2'd0 ? i_wb_rdata[ 31: 0] : //TODO confirm this does the right thing wrt loading individual bytes or if add'l logic will be required
				address_current[3:2] == 2'd1 ? i_wb_rdata[ 63:32] :
				address_current[3:2] == 2'd2 ? i_wb_rdata[ 95:64] :
											   i_wb_rdata[127:96] ;
	o_wb_req = i_instr_valid || swap_op_wstart;
	o_wb_write = (i_instr_valid && i_op_type[0]) || swap_op_wstart;
	if (i_instr_valid)	o_wb_byte_enable = i_address[3:2] == 2'd0 ? {12'd0, i_byte_enable       } :
										   i_address[3:2] == 2'd1 ? { 8'd0, i_byte_enable,  4'd0} :
										   i_address[3:2] == 2'd2 ? { 4'd0, i_byte_enable,  8'd0} :
																	{       i_byte_enable, 12'd0} ;
	else	o_wb_byte_enable = 	address_current[3:2] == 2'd0 ? {12'd0, byte_enable_current       } :
								address_current[3:2] == 2'd1 ? { 8'd0, byte_enable_current,  4'd0} :
								address_current[3:2] == 2'd2 ? { 4'd0, byte_enable_current,  8'd0} :
															   {       byte_enable_current, 12'd0} ;
	o_wb_write_data = i_instr_valid ? {4{i_write_data}} : {4{write_data_current}};
	o_wb_address = i_instr_valid ? {i_address[31:2],2'd0} : {address_current[31:2],2'd0};
end

/*
	Operation:
	- if no op in progress and no input ready: do nothing
	- if no op in progress and input ready:
		- latch operands
		- forward appropriate data directly to mem
	- if op in progress:
		- if op incomplete:
			- wait
		- if op complete:
			- latch outputs: ready, data, tag, valid, etc.
	
	- if op is typical read or write: nothing special
		- read: need to latch ready, data, tag, valid, and flags at end
		- write: only need to latch ready at end
	- if op is swap:
		- latch all data and begin read op
		- when read op done, put data on module output and start write op
		- when write op done, set "ready" output
*/
always_ff @(posedge i_clk, posedge i_rst) begin
	if (i_rst) begin
		//all internal registers to "safe" state
		op_in_progress <= 'd0;
		op_type_current <= 'd0;
		is_swap_current <= 'd0;
		address_current <= MAIN_BASE; //defined in memory_configuration.vh. TODO: map this appropriately for the Harvard architecture!
		write_data_current <= 'd0;
		byte_enable_current <= 'd0;
		rd_tag_current <= 'd0;
	end
	else begin
		if (op_in_progress) begin
			if (i_wb_ready) begin //regardless of op type, we need to wait for ack
				//note that output logic is to be combinational based on the current state of i_wb_ready, op_type_current, and i_wb_rdata
				if (op_type_current[1] || ~is_swap_current) begin //last part of swap, or any non-swap op
					op_type_current <= 'd0;
					op_in_progress <= 1'b0;
					is_swap_current <= 1'b0;
					address_current <= MAIN_BASE;
					write_data_current <= 'd0;
					byte_enable_current <= 'd0;
					rd_tag_current <= 'd0;
				end
				else begin //just finished the read part of swap; go to write
					op_type_current <= 2'b01;
					//the following not strictly necessary as they should already be latched
					/*op_in_progress <= 1'b1;
					is_swap_current <= 1'b1;
					address_current <= address_current;
					write_data_current <= write_data_current;
					byte_enable_current <= byte_enable_current;
					rd_tag_current <= rd_tag_current;*/
				end
			end
			else begin
				//it was a read op; we need to wait until i_wb_ready is asserted
				//do nothing, leave all as-is
			end
		end
		else begin //no op in progress
			if (i_instr_valid) begin
				if (i_op_type == 2'b11) begin
					//invalid op type, should never happen. treat as NOP.
					op_in_progress <= 'd0;
					op_type_current <= 'd0;
					is_swap_current <= 'd0;
					address_current <= MAIN_BASE; //defined in memory_configuration.vh. TODO: map this appropriately for the Harvard architecture!
					write_data_current <= 'd0;
					byte_enable_current <= 'd0;
					rd_tag_current <= 'd0;
				end
				else begin
					//valid read, write, or swap instruction
					if (i_op_type[1]) begin
						//swap instruction, start with the read
						op_type_current <= 2'b00;
					end
					else begin
						//"normal" read or write
						op_type_current <= i_op_type;
					end
					op_in_progress <= 1'b1;
					is_swap_current <= i_op_type[1];
					address_current <= i_address;
					write_data_current <= i_write_data;
					byte_enable_current <= i_byte_enable;
					rd_tag_current <= i_rd_tag;
				end
			end
			else begin //no op in progress and no incoming instruction
				//go to safe state
				op_in_progress <= 'd0;
				op_type_current <= 'd0;
				is_swap_current <= 'd0;
				address_current <= MAIN_BASE; //defined in memory_configuration.vh. TODO: map this appropriately for the Harvard architecture!
				write_data_current <= 'd0;
				byte_enable_current <= 'd0;
				rd_tag_current <= 'd0;
			end
		end
	end
end

endmodule
