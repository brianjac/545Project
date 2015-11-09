//////////////////////////////////////////////////////////////////
//                                                              //
//  Register Bank for Amber 25 Core                             //
//                                                              //
//  This file is part of the Amber project                      //
//  http://www.opencores.org/project,amber                      //
//                                                              //
//  Description                                                 //
//  Contains 37 32-bit registers, 16 of which are visible       //
//  ina any one operating mode. Registers use real flipflops,   //
//  rather than SRAM. This makes sense for an FPGA              //
//  implementation, where flipflops are plentiful.              //
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

typedef struct reg_line {
	reg			valid = 1'd1;
	reg [5:0] 	tag = 6'd0;
	reg [31:0] 	data = 32'hdead_beef;
} reg_line;

typedef struct pc_line {
	reg 		valid = 1'd1;
	reg [5:0]	tag = 1'd0;
	reg [23:0]	data = 24'h00_0000;
} pc_line;

module a25_register_bank (

input                       i_clk,
input                       i_core_stall,
input                       i_mem_stall,

input       [1:0]           i_mode_idec,            // user, supervisor, irq_idec, firq_idec etc.
                                                    // Used for register writes
input       [1:0]           i_mode_exec,            // 1 periods delayed from i_mode_idec
                                                    // Used for register reads
input       [3:0]           i_mode_rds_exec,        // Use one-hot version specifically for rds, 
                                                    // includes i_user_mode_regs_store
input                       i_firq_not_user_mode,
input       [3:0]           i_rm_sel,
input       [3:0]           i_rs_sel,
input       [3:0]           i_rn_sel,

input                       i_pc_wen,
input       [14:0]          i_reg_bank_wen,

input       [23:0]          i_pc,                   // program counter [25:2]
input       [31:0]          i_reg,

input       [31:0]          i_wb_read_data,
input                       i_wb_read_data_valid,
input       [3:0]           i_wb_read_data_rd,
input       [1:0]           i_wb_mode,

input       [3:0]           i_status_bits_flags,
input                       i_status_bits_irq_mask,
input                       i_status_bits_firq_mask,

output      [31:0]          o_rm,
output reg  [31:0]          o_rs,
output reg  [31:0]          o_rd,
output      [31:0]          o_rn,
output      [31:0]          o_pc,

//items for OOO
input						i_alu_valid,
input						i_mult_valid,
input						i_mem_valid,
input		[5:0]			i_alu_tag,
input		[5:0]			i_mult_tag,
input		[5:0]			i_mem_tag,
input		[31:0]			i_alu_data,
input		[31:0]			i_mult_data,
input		[31:0]			i_mem_data,
input		[3:0]			i_alu_flags,
input		[3:0]			i_mult_flags,
input		[3:0]			i_mem_flags,

output						o_rm_valid,
output						o_rs_valid,
output						o_rd_valid,
output						o_rn_valid,
output						o_pc_valid,
output		[5:0]			o_rm_tag,
output		[5:0]			o_rs_tag,
output		[5:0]			o_rd_tag,
output		[5:0]			o_rn_tag,
output		[5:0]			o_pc_tag,

input		[5:0]			i_rd_tag,

output logic [7:0] led,
input [7:0] sw

);

`include "a25_localparams.vh"
`include "a25_functions.vh"


// User Mode Registers
reg_line r0;
reg_line r1;
reg_line r2;
reg_line r3;
reg_line r4;
reg_line r5;
reg_line r6;
reg_line r7;
reg_line r8;
reg_line r9;
reg_line r10;
reg_line r11;
reg_line r12;
reg_line r13;
reg_line r14;
//reg  [23:0] r15 = 24'hc0_ffee;
pc_line r15;

wire  [31:0] r0_out;
wire  [31:0] r1_out;
wire  [31:0] r2_out;
wire  [31:0] r3_out;
wire  [31:0] r4_out;
wire  [31:0] r5_out;
wire  [31:0] r6_out;
wire  [31:0] r7_out;
wire  [31:0] r8_out;
wire  [31:0] r9_out;
wire  [31:0] r10_out;
wire  [31:0] r11_out;
wire  [31:0] r12_out;
wire  [31:0] r13_out;
wire  [31:0] r14_out;
wire  [31:0] r15_out_rm;
wire  [31:0] r15_out_rm_nxt;
wire  [31:0] r15_out_rn;

wire  [31:0] r8_rds;
wire  [31:0] r9_rds;
wire  [31:0] r10_rds;
wire  [31:0] r11_rds;
wire  [31:0] r12_rds;
wire  [31:0] r13_rds;
wire  [31:0] r14_rds;

// Supervisor Mode Registers
reg_line r13_svc;
reg_line r14_svc;

// Interrupt Mode Registers
reg_line r13_irq;
reg_line r14_irq;

// Fast Interrupt Mode Registers
reg_line r8_firq;
reg_line r9_firq;
reg_line r10_firq;
reg_line r11_firq;
reg_line r12_firq;
reg_line r13_firq;
reg_line r14_firq;

wire        usr_exec;
wire        svc_exec;
wire        irq_exec;
wire        firq_exec;

wire        usr_idec;
wire        svc_idec;
wire        irq_idec;
wire        firq_idec;
wire [14:0] read_data_wen;
wire [14:0] reg_bank_wen_c;
wire        pc_wen_c;
wire        pc_dmem_wen;


    // Write Enables from execute stage
assign usr_idec  = i_mode_idec == USR;
assign svc_idec  = i_mode_idec == SVC;
assign irq_idec  = i_mode_idec == IRQ;

// pre-encoded in decode stage to speed up long path
assign firq_idec = i_firq_not_user_mode;

    // Read Enables from stage 1 (fetch)
assign usr_exec  = i_mode_exec == USR;
assign svc_exec  = i_mode_exec == SVC;
assign irq_exec  = i_mode_exec == IRQ;
assign firq_exec = i_mode_exec == FIRQ;

assign read_data_wen = {15{i_wb_read_data_valid & ~i_mem_stall}} & decode (i_wb_read_data_rd);

assign reg_bank_wen_c = {15{~i_core_stall}} & i_reg_bank_wen;
assign pc_wen_c       = ~i_core_stall & i_pc_wen;
assign pc_dmem_wen    = i_wb_read_data_valid & ~i_mem_stall & i_wb_read_data_rd == 4'd15;


// ========================================================
// Register Update
// ========================================================

//note that the same-indexed bit will never be 1 in more than one of these at a time
logic [15:0] tag_match_alu;
logic [15:0] tag_match_mult;
logic [15:0] tag_match_mem;

//Tag comparison at each register's tag and valid bits with data being retired this cycle
always_comb begin
	//defaults
	tag_match_alu = 16'd0;
	tag_match_mult = 16'd0;
	tag_match_mem = 16'd0;
	
	//ALU
	//r0-r7, used in all modes
	if (!r0.valid && i_alu_valid && r0.tag == i_alu_tag) tag_match_alu[0 ] = 1'b1;
	if (!r1.valid && i_alu_valid && r1.tag == i_alu_tag) tag_match_alu[1 ] = 1'b1;
	if (!r2.valid && i_alu_valid && r2.tag == i_alu_tag) tag_match_alu[2 ] = 1'b1;
	if (!r3.valid && i_alu_valid && r3.tag == i_alu_tag) tag_match_alu[3 ] = 1'b1;
	if (!r4.valid && i_alu_valid && r4.tag == i_alu_tag) tag_match_alu[4 ] = 1'b1;
	if (!r5.valid && i_alu_valid && r5.tag == i_alu_tag) tag_match_alu[5 ] = 1'b1;
	if (!r6.valid && i_alu_valid && r6.tag == i_alu_tag) tag_match_alu[6 ] = 1'b1;
	if (!r7.valid && i_alu_valid && r7.tag == i_alu_tag) tag_match_alu[7 ] = 1'b1;
	
	//r8-r12, depending on if we're in FIRQ mode or not
	if (i_wb_mode != FIRQ) begin
		if (!r8.valid && i_alu_valid && r8.tag == i_alu_tag) tag_match_alu[8 ] = 1'b1;
		if (!r9.valid && i_alu_valid && r9.tag == i_alu_tag) tag_match_alu[9 ] = 1'b1;
		if (!r10.valid && i_alu_valid && r10.tag == i_alu_tag) tag_match_alu[10] = 1'b1;
		if (!r11.valid && i_alu_valid && r11.tag == i_alu_tag) tag_match_alu[11] = 1'b1;
		if (!r12.valid && i_alu_valid && r12.tag == i_alu_tag) tag_match_alu[12] = 1'b1;
	end
	else begin
		if (!r8_firq.valid && i_alu_valid && r8_firq.tag == i_alu_tag) tag_match_alu[8 ] = 1'b1;
		if (!r9_firq.valid && i_alu_valid && r9_firq.tag == i_alu_tag) tag_match_alu[9 ] = 1'b1;
		if (!r10_firq.valid && i_alu_valid && r10_firq.tag == i_alu_tag) tag_match_alu[10] = 1'b1;
		if (!r11_firq.valid && i_alu_valid && r11_firq.tag == i_alu_tag) tag_match_alu[11] = 1'b1;
		if (!r12_firq.valid && i_alu_valid && r12_firq.tag == i_alu_tag) tag_match_alu[12] = 1'b1;
	end
	
	//r13-r14, based on our mode
	if (i_wb_mode == USR) begin
		if (!r13.valid && i_alu_valid && r13.tag == i_alu_tag) tag_match_alu[13] = 1'b1;
		if (!r14.valid && i_alu_valid && r14.tag == i_alu_tag) tag_match_alu[14] = 1'b1;
	end
	else if (i_wb_mode == SVC) begin
		if (!r13_svc.valid && i_alu_valid && r13_svc.tag == i_alu_tag) tag_match_alu[13] = 1'b1;
		if (!r14_svc.valid && i_alu_valid && r14_svc.tag == i_alu_tag) tag_match_alu[14] = 1'b1;
	end
	else if (i_wb_mode == IRQ) begin
		if (!r13_irq.valid && i_alu_valid && r13_irq.tag == i_alu_tag) tag_match_alu[13] = 1'b1;
		if (!r14_irq.valid && i_alu_valid && r14_irq.tag == i_alu_tag) tag_match_alu[14] = 1'b1;
	end
	else if (i_wb_mode == FIRQ) begin
		if (!r13_firq.valid && i_alu_valid && r13_firq.tag == i_alu_tag) tag_match_alu[13] = 1'b1;
		if (!r14_firq.valid && i_alu_valid && r14_firq.tag == i_alu_tag) tag_match_alu[14] = 1'b1;
	end
	
	//r15 (PC), handled the same in all modes
	if (!r15.valid && i_alu_valid && r15.tag == i_alu_tag) tag_match_alu[15] = 1'b1;
	
	
	//Mult
	//r0-r7, used in all modes
	if (!r0.valid && i_mult_valid && r0.tag == i_mult_tag) tag_match_mult[0 ] = 1'b1;
	if (!r1.valid && i_mult_valid && r1.tag == i_mult_tag) tag_match_mult[1 ] = 1'b1;
	if (!r2.valid && i_mult_valid && r2.tag == i_mult_tag) tag_match_mult[2 ] = 1'b1;
	if (!r3.valid && i_mult_valid && r3.tag == i_mult_tag) tag_match_mult[3 ] = 1'b1;
	if (!r4.valid && i_mult_valid && r4.tag == i_mult_tag) tag_match_mult[4 ] = 1'b1;
	if (!r5.valid && i_mult_valid && r5.tag == i_mult_tag) tag_match_mult[5 ] = 1'b1;
	if (!r6.valid && i_mult_valid && r6.tag == i_mult_tag) tag_match_mult[6 ] = 1'b1;
	if (!r7.valid && i_mult_valid && r7.tag == i_mult_tag) tag_match_mult[7 ] = 1'b1;
	
	//r8-r12, depending on if we're in FIRQ mode or not
	if (i_wb_mode != FIRQ) begin
		if (!r8.valid && i_mult_valid && r8.tag == i_mult_tag) tag_match_mult[8 ] = 1'b1;
		if (!r9.valid && i_mult_valid && r9.tag == i_mult_tag) tag_match_mult[9 ] = 1'b1;
		if (!r10.valid && i_mult_valid && r10.tag == i_mult_tag) tag_match_mult[10] = 1'b1;
		if (!r11.valid && i_mult_valid && r11.tag == i_mult_tag) tag_match_mult[11] = 1'b1;
		if (!r12.valid && i_mult_valid && r12.tag == i_mult_tag) tag_match_mult[12] = 1'b1;
	end
	else begin
		if (!r8_firq.valid && i_mult_valid && r8_firq.tag == i_mult_tag) tag_match_mult[8 ] = 1'b1;
		if (!r9_firq.valid && i_mult_valid && r9_firq.tag == i_mult_tag) tag_match_mult[9 ] = 1'b1;
		if (!r10_firq.valid && i_mult_valid && r10_firq.tag == i_mult_tag) tag_match_mult[10] = 1'b1;
		if (!r11_firq.valid && i_mult_valid && r11_firq.tag == i_mult_tag) tag_match_mult[11] = 1'b1;
		if (!r12_firq.valid && i_mult_valid && r12_firq.tag == i_mult_tag) tag_match_mult[12] = 1'b1;
	end
	
	//r13-r14, based on our mode
	if (i_wb_mode == USR) begin
		if (!r13.valid && i_mult_valid && r13.tag == i_mult_tag) tag_match_mult[13] = 1'b1;
		if (!r14.valid && i_mult_valid && r14.tag == i_mult_tag) tag_match_mult[14] = 1'b1;
	end
	else if (i_wb_mode == SVC) begin
		if (!r13_svc.valid && i_mult_valid && r13_svc.tag == i_mult_tag) tag_match_mult[13] = 1'b1;
		if (!r14_svc.valid && i_mult_valid && r14_svc.tag == i_mult_tag) tag_match_mult[14] = 1'b1;
	end
	else if (i_wb_mode == IRQ) begin
		if (!r13_irq.valid && i_mult_valid && r13_irq.tag == i_mult_tag) tag_match_mult[13] = 1'b1;
		if (!r14_irq.valid && i_mult_valid && r14_irq.tag == i_mult_tag) tag_match_mult[14] = 1'b1;
	end
	else if (i_wb_mode == FIRQ) begin
		if (!r13_firq.valid && i_mult_valid && r13_firq.tag == i_mult_tag) tag_match_mult[13] = 1'b1;
		if (!r14_firq.valid && i_mult_valid && r14_firq.tag == i_mult_tag) tag_match_mult[14] = 1'b1;
	end
	
	//r15 (PC), handled the same in all modes
	if (!r15.valid && i_mult_valid && r15.tag == i_mult_tag) tag_match_mult[15] = 1'b1;
	
	
	
	//Mem
	//r0-r7, used in all modes
	if (!r0.valid && i_mem_valid && r0.tag == i_mem_tag) tag_match_mem[0 ] = 1'b1;
	if (!r1.valid && i_mem_valid && r1.tag == i_mem_tag) tag_match_mem[1 ] = 1'b1;
	if (!r2.valid && i_mem_valid && r2.tag == i_mem_tag) tag_match_mem[2 ] = 1'b1;
	if (!r3.valid && i_mem_valid && r3.tag == i_mem_tag) tag_match_mem[3 ] = 1'b1;
	if (!r4.valid && i_mem_valid && r4.tag == i_mem_tag) tag_match_mem[4 ] = 1'b1;
	if (!r5.valid && i_mem_valid && r5.tag == i_mem_tag) tag_match_mem[5 ] = 1'b1;
	if (!r6.valid && i_mem_valid && r6.tag == i_mem_tag) tag_match_mem[6 ] = 1'b1;
	if (!r7.valid && i_mem_valid && r7.tag == i_mem_tag) tag_match_mem[7 ] = 1'b1;
	
	//r8-r12, depending on if we're in FIRQ mode or not
	if (i_wb_mode != FIRQ) begin
		if (!r8.valid && i_mem_valid && r8.tag == i_mem_tag) tag_match_mem[8 ] = 1'b1;
		if (!r9.valid && i_mem_valid && r9.tag == i_mem_tag) tag_match_mem[9 ] = 1'b1;
		if (!r10.valid && i_mem_valid && r10.tag == i_mem_tag) tag_match_mem[10] = 1'b1;
		if (!r11.valid && i_mem_valid && r11.tag == i_mem_tag) tag_match_mem[11] = 1'b1;
		if (!r12.valid && i_mem_valid && r12.tag == i_mem_tag) tag_match_mem[12] = 1'b1;
	end
	else begin
		if (!r8_firq.valid && i_mem_valid && r8_firq.tag == i_mem_tag) tag_match_mem[8 ] = 1'b1;
		if (!r9_firq.valid && i_mem_valid && r9_firq.tag == i_mem_tag) tag_match_mem[9 ] = 1'b1;
		if (!r10_firq.valid && i_mem_valid && r10_firq.tag == i_mem_tag) tag_match_mem[10] = 1'b1;
		if (!r11_firq.valid && i_mem_valid && r11_firq.tag == i_mem_tag) tag_match_mem[11] = 1'b1;
		if (!r12_firq.valid && i_mem_valid && r12_firq.tag == i_mem_tag) tag_match_mem[12] = 1'b1;
	end
	
	//r13-r14, based on our mode
	if (i_wb_mode == USR) begin
		if (!r13.valid && i_mem_valid && r13.tag == i_mem_tag) tag_match_mem[13] = 1'b1;
		if (!r14.valid && i_mem_valid && r14.tag == i_mem_tag) tag_match_mem[14] = 1'b1;
	end
	else if (i_wb_mode == SVC) begin
		if (!r13_svc.valid && i_mem_valid && r13_svc.tag == i_mem_tag) tag_match_mem[13] = 1'b1;
		if (!r14_svc.valid && i_mem_valid && r14_svc.tag == i_mem_tag) tag_match_mem[14] = 1'b1;
	end
	else if (i_wb_mode == IRQ) begin
		if (!r13_irq.valid && i_mem_valid && r13_irq.tag == i_mem_tag) tag_match_mem[13] = 1'b1;
		if (!r14_irq.valid && i_mem_valid && r14_irq.tag == i_mem_tag) tag_match_mem[14] = 1'b1;
	end
	else if (i_wb_mode == FIRQ) begin
		if (!r13_firq.valid && i_mem_valid && r13_firq.tag == i_mem_tag) tag_match_mem[13] = 1'b1;
		if (!r14_firq.valid && i_mem_valid && r14_firq.tag == i_mem_tag) tag_match_mem[14] = 1'b1;
	end
	
	//r15 (PC), handled the same in all modes
	if (!r15.valid && i_mem_valid && r15.tag == i_mem_tag) tag_match_mem[15] = 1'b1;
	
end

//per-register valid bit and tag update logic
logic [15:0] 		r_valid_nxt;
logic [15:0][5:0]	r_tag_nxt;
always_comb begin
	//4 cases per register:
	//1) nothing new,
	//2) it's a new destination register, so we need to update the tag and invalidate the reg,
	//3) it's being written back this cycle, so we update the stored data and set the reg to valid, or
	//4) both 2 and 3, so we update the stored data, leave it invalid, and update the tag
	
	//r0-r7: always
	//r0
	if (reg_bank_wen_c[0]) begin
		r_valid_nxt[0] = 1'b0;
		r_tag_nxt[0] = i_rd_tag;
	end
	else if (tag_match_alu[0] || tag_match_mult[0] || tag_match_mem[0]) begin
		r_valid_nxt[0] = 1'b1;
		r_tag_nxt[0] = r0.tag;
	end
	else begin
		r_valid_nxt[0] = r0.valid;
		r_tag_nxt[0] = r0.tag;
	end
	
	//r1
	if (reg_bank_wen_c[1]) begin
		r_valid_nxt[1] = 1'b0;
		r_tag_nxt[1] = i_rd_tag;
	end
	else if (tag_match_alu[1] || tag_match_mult[1] || tag_match_mem[1]) begin
		r_valid_nxt[1] = 1'b1;
		r_tag_nxt[1] = r1.tag;
	end
	else begin
		r_valid_nxt[1] = r1.valid;
		r_tag_nxt[1] = r1.tag;
	end
	
	//r2
	if (reg_bank_wen_c[2]) begin
		r_valid_nxt[2] = 1'b0;
		r_tag_nxt[2] = i_rd_tag;
	end
	else if (tag_match_alu[2] || tag_match_mult[2] || tag_match_mem[2]) begin
		r_valid_nxt[2] = 1'b1;
		r_tag_nxt[2] = r2.tag;
	end
	else begin
		r_valid_nxt[2] = r2.valid;
		r_tag_nxt[2] = r2.tag;
	end
	
	//r3
	if (reg_bank_wen_c[3]) begin
		r_valid_nxt[3] = 1'b0;
		r_tag_nxt[3] = i_rd_tag;
	end
	else if (tag_match_alu[3] || tag_match_mult[3] || tag_match_mem[3]) begin
		r_valid_nxt[3] = 1'b1;
		r_tag_nxt[3] = r3.tag;
	end
	else begin
		r_valid_nxt[0] = r0.valid;
		r_tag_nxt[0] = r0.tag;
	end
	
	//r4
	if (reg_bank_wen_c[4]) begin
		r_valid_nxt[4] = 1'b0;
		r_tag_nxt[4] = i_rd_tag;
	end
	else if (tag_match_alu[4] || tag_match_mult[4] || tag_match_mem[4]) begin
		r_valid_nxt[4] = 1'b1;
		r_tag_nxt[4] = r4.tag;
	end
	else begin
		r_valid_nxt[4] = r4.valid;
		r_tag_nxt[4] = r4.tag;
	end
	
	//r5
	if (reg_bank_wen_c[5]) begin
		r_valid_nxt[5] = 1'b0;
		r_tag_nxt[5] = i_rd_tag;
	end
	else if (tag_match_alu[5] || tag_match_mult[5] || tag_match_mem[5]) begin
		r_valid_nxt[5] = 1'b1;
		r_tag_nxt[5] = r5.tag;
	end
	else begin
		r_valid_nxt[5] = r5.valid;
		r_tag_nxt[5] = r5.tag;
	end
	
	//r6
	if (reg_bank_wen_c[6]) begin
		r_valid_nxt[6] = 1'b0;
		r_tag_nxt[6] = i_rd_tag;
	end
	else if (tag_match_alu[6] || tag_match_mult[6] || tag_match_mem[6]) begin
		r_valid_nxt[6] = 1'b1;
		r_tag_nxt[6] = r6.tag;
	end
	else begin
		r_valid_nxt[6] = r6.valid;
		r_tag_nxt[6] = r6.tag;
	end
	
	//r7
	if (reg_bank_wen_c[7]) begin
		r_valid_nxt[7] = 1'b0;
		r_tag_nxt[7] = i_rd_tag;
	end
	else if (tag_match_alu[7] || tag_match_mult[7] || tag_match_mem[7]) begin
		r_valid_nxt[7] = 1'b1;
		r_tag_nxt[7] = r7.tag;
	end
	else begin
		r_valid_nxt[7] = r7.valid;
		r_tag_nxt[7] = r7.tag;
	end
	
	
	//r8-r12, depending on irq/firq
	//r8
	if (reg_bank_wen_c[8]) begin
		r_valid_nxt[8] = 1'b0;
		r_tag_nxt[8] = i_rd_tag;
	end
	else if (tag_match_alu[8] || tag_match_mult[8] || tag_match_mem[8]) begin
		r_valid_nxt[8] = 1'b1;
		r_tag_nxt[8] = (i_wb_mode == FIRQ) ? r8_firq.tag : r8.tag;
	end
	else begin
		r_valid_nxt[8] = (i_wb_mode == FIRQ) ? r8_firq.valid : r8.valid;
		r_tag_nxt[8] = (i_wb_mode == FIRQ) ? r8_firq.tag : r8.tag;
	end
	
	//r9
	if (reg_bank_wen_c[9]) begin
		r_valid_nxt[9] = 1'b0;
		r_tag_nxt[9] = i_rd_tag;
	end
	else if (tag_match_alu[9] || tag_match_mult[9] || tag_match_mem[9]) begin
		r_valid_nxt[9] = 1'b1;
		r_tag_nxt[9] = (i_wb_mode == FIRQ) ? r9_firq.tag : r9.tag;
	end
	else begin
		r_valid_nxt[9] = (i_wb_mode == FIRQ) ? r9_firq.valid : r9.valid;
		r_tag_nxt[9] = (i_wb_mode == FIRQ) ? r9_firq.tag : r9.tag;
	end
	
	//r10
	if (reg_bank_wen_c[10]) begin
		r_valid_nxt[10] = 1'b0;
		r_tag_nxt[10] = i_rd_tag;
	end
	else if (tag_match_alu[10] || tag_match_mult[10] || tag_match_mem[10]) begin
		r_valid_nxt[10] = 1'b1;
		r_tag_nxt[10] = (i_wb_mode == FIRQ) ? r10_firq.tag : r10.tag;
	end
	else begin
		r_valid_nxt[10] = (i_wb_mode == FIRQ) ? r10_firq.valid : r10.valid;
		r_tag_nxt[10] = (i_wb_mode == FIRQ) ? r10_firq.tag : r10.tag;
	end
	
	//r11
	if (reg_bank_wen_c[11]) begin
		r_valid_nxt[11] = 1'b0;
		r_tag_nxt[11] = i_rd_tag;
	end
	else if (tag_match_alu[11] || tag_match_mult[11] || tag_match_mem[11]) begin
		r_valid_nxt[11] = 1'b1;
		r_tag_nxt[11] = (i_wb_mode == FIRQ) ? r11_firq.tag : r11.tag;
	end
	else begin
		r_valid_nxt[11] = (i_wb_mode == FIRQ) ? r11_firq.valid : r11.valid;
		r_tag_nxt[11] = (i_wb_mode == FIRQ) ? r11_firq.tag : r11.tag;
	end
	
	//r12
	if (reg_bank_wen_c[12]) begin
		r_valid_nxt[12] = 1'b0;
		r_tag_nxt[12] = i_rd_tag;
	end
	else if (tag_match_alu[12] || tag_match_mult[12] || tag_match_mem[12]) begin
		r_valid_nxt[12] = 1'b1;
		r_tag_nxt[12] = (i_wb_mode == FIRQ) ? r12_firq.tag : r12.tag;
	end
	else begin
		r_valid_nxt[12] = (i_wb_mode == FIRQ) ? r12_firq.valid : r12.valid;
		r_tag_nxt[12] = (i_wb_mode == FIRQ) ? r12_firq.tag : r12.tag;
	end
	
	
	//r13-r14, depending on usr/svc/irq/firq
	//r13
	if (reg_bank_wen_c[13]) begin
		r_valid_nxt[13] = 1'b0;
		r_tag_nxt[13] = i_rd_tag;
	end
	else if (tag_match_alu[13] || tag_match_mult[13] || tag_match_mem[13]) begin
		r_valid_nxt[13]= 1'b1;
		r_tag_nxt[13] = (i_wb_mode == USR) 	? r13.tag :
						(i_wb_mode == SVC) 	? r13_svc.tag :
						(i_wb_mode == IRQ) 	? r13_irq.tag :
						(i_wb_mode == FIRQ) ? r13_firq.tag;
	end
	else begin
		r_valid_nxt[13]=(i_wb_mode == USR) 	? r13.valid :
						(i_wb_mode == SVC) 	? r13_svc.valid :
						(i_wb_mode == IRQ) 	? r13_irq.valid :
						(i_wb_mode == FIRQ) ? r13_firq.valid;
		r_tag_nxt[13] =	(i_wb_mode == USR) 	? r13.tag :
						(i_wb_mode == SVC) 	? r13_svc.tag :
						(i_wb_mode == IRQ) 	? r13_irq.tag :
						(i_wb_mode == FIRQ) ? r13_firq.tag;
	end
	
	//r14
	if (reg_bank_wen_c[14]) begin
		r_valid_nxt[14] = 1'b0;
		r_tag_nxt[14] = i_rd_tag;
	end
	else if (tag_match_alu[14] || tag_match_mult[14] || tag_match_mem[14]) begin
		r_valid_nxt[14]= 1'b1;
		r_tag_nxt[14] =	(i_wb_mode == USR) 	? r14.tag :
						(i_wb_mode == SVC) 	? r14_svc.tag :
						(i_wb_mode == IRQ) 	? r14_irq.tag :
						(i_wb_mode == FIRQ) ? r14_firq.tag;
	end
	else begin
		r_valid_nxt[14]=(i_wb_mode == USR) 	? r14.valid :
						(i_wb_mode == SVC) 	? r14_svc.valid :
						(i_wb_mode == IRQ) 	? r14_irq.valid :
						(i_wb_mode == FIRQ) ? r14_firq.valid;
		r_tag_nxt[14] =	(i_wb_mode == USR) 	? r14.tag :
						(i_wb_mode == SVC) 	? r14_svc.tag :
						(i_wb_mode == IRQ) 	? r14_irq.tag :
						(i_wb_mode == FIRQ) ? r14_firq.tag;
	end
	
	
	//r15 (pc), always
	if (reg_bank_wen_c[15]) begin
		r_valid_nxt[15] = 1'b0;
		r_tag_nxt[15] = i_rd_tag;
	end
	else if (tag_match_alu[15] || tag_match_mult[15] || tag_match_mem[15]) begin
		r_valid_nxt[15] = 1'b1;
		r_tag_nxt[15] = r15.tag;
	end
	else begin
		r_valid_nxt[15] = r15.valid;
		r_tag_nxt[15] = r15.tag;
	end
	
end

always_ff @(posedge i_clk) begin
	//Write back to registers based on tag comparison result, above.
	//Note that this implicitly handles writebacks from memory, so we don't need read_data_wen or i_wb_read_data anymore (per se).
	
	//Valid bits
	r0.valid  <= r_valid_nxt[0 ];
	r1.valid  <= r_valid_nxt[1 ];
	r2.valid  <= r_valid_nxt[2 ];
	r3.valid  <= r_valid_nxt[3 ];
	r4.valid  <= r_valid_nxt[4 ];
	r5.valid  <= r_valid_nxt[5 ];
	r6.valid  <= r_valid_nxt[6 ];
	r7.valid  <= r_valid_nxt[7 ];
	
	r8.valid  <= i_wb_mode != FIRQ ? r_valid_nxt[8 ] : r8.valid;
	r9.valid  <= i_wb_mode != FIRQ ? r_valid_nxt[9 ] : r9.valid;
	r10.valid <= i_wb_mode != FIRQ ? r_valid_nxt[10] : r10.valid;
	r11.valid <= i_wb_mode != FIRQ ? r_valid_nxt[11] : r11.valid;
	r12.valid <= i_wb_mode != FIRQ ? r_valid_nxt[12] : r12.valid;
	
	r8_firq.valid  <= i_wb_mode == FIRQ ? r_valid_nxt[8 ] : r8_firq.valid;
	r9_firq.valid  <= i_wb_mode == FIRQ ? r_valid_nxt[9 ] : r9_firq.valid;
	r10_firq.valid <= i_wb_mode == FIRQ ? r_valid_nxt[10] : r10_firq.valid;
	r11_firq.valid <= i_wb_mode == FIRQ ? r_valid_nxt[11] : r11_firq.valid;
	r12_firq.valid <= i_wb_mode == FIRQ ? r_valid_nxt[12] : r12_firq.valid;
	
	r13.valid <= i_wb_mode == USR ? r_valid_nxt[13] : r13.valid;
	r14.valid <= i_wb_mode == USR ? r_valid_nxt[14] : r14.valid;
	r13_svc.valid <= i_wb_mode == SVC ? r_valid_nxt[13] : r13_svc.valid;
	r14_svc.valid <= i_wb_mode == SVC ? r_valid_nxt[14] : r14_svc.valid;
	r13_irq.valid <= i_wb_mode == IRQ ? r_valid_nxt[13] : r13_irq.valid;
	r14_irq.valid <= i_wb_mode == IRQ ? r_valid_nxt[14] : r14_irq.valid;
	r13_firq.valid <= i_wb_mode == FIRQ ? r_valid_nxt[13] : r13_firq.valid;
	r14_firq.valid <= i_wb_mode == FIRQ ? r_valid_nxt[14] : r14_firq.valid;
	
	
	//Tag bits
	r0.tag  <= r_tag_nxt[0 ];
	r1.tag  <= r_tag_nxt[1 ];
	r2.tag  <= r_tag_nxt[2 ];
	r3.tag  <= r_tag_nxt[3 ];
	r4.tag  <= r_tag_nxt[4 ];
	r5.tag  <= r_tag_nxt[5 ];
	r6.tag  <= r_tag_nxt[6 ];
	r7.tag  <= r_tag_nxt[7 ];
	
	r8.tag  <= i_wb_mode != FIRQ ? r_tag_nxt[8 ] : r8.tag;
	r9.tag  <= i_wb_mode != FIRQ ? r_tag_nxt[9 ] : r9.tag;
	r10.tag <= i_wb_mode != FIRQ ? r_tag_nxt[10] : r10.tag;
	r11.tag <= i_wb_mode != FIRQ ? r_tag_nxt[11] : r11.tag;
	r12.tag <= i_wb_mode != FIRQ ? r_tag_nxt[12] : r12.tag;
	
	r8_firq.tag  <= i_wb_mode == FIRQ ? r_tag_nxt[8 ] : r8_firq.tag;
	r9_firq.tag  <= i_wb_mode == FIRQ ? r_tag_nxt[9 ] : r9_firq.tag;
	r10_firq.tag <= i_wb_mode == FIRQ ? r_tag_nxt[10] : r10_firq.tag;
	r11_firq.tag <= i_wb_mode == FIRQ ? r_tag_nxt[11] : r11_firq.tag;
	r12_firq.tag <= i_wb_mode == FIRQ ? r_tag_nxt[12] : r12_firq.tag;
	
	r13.tag <= i_wb_mode == USR ? r_tag_nxt[13] : r13.tag;
	r14.tag <= i_wb_mode == USR ? r_tag_nxt[14] : r14.tag;
	r13_svc.tag <= i_wb_mode == SVC ? r_tag_nxt[13] : r13_svc.tag;
	r14_svc.tag <= i_wb_mode == SVC ? r_tag_nxt[14] : r14_svc.tag;
	r13_irq.tag <= i_wb_mode == IRQ ? r_tag_nxt[13] : r13_irq.tag;
	r14_irq.tag <= i_wb_mode == IRQ ? r_tag_nxt[14] : r14_irq.tag;
	r13_firq.tag <= i_wb_mode == FIRQ ? r_tag_nxt[13] : r13_firq.tag;
	r14_firq.tag <= i_wb_mode == FIRQ ? r_tag_nxt[14] : r14_firq.tag;
	
	
	//Data
	//TODO, based on tag bit input stuff
	
end

always @ ( posedge i_clk )
    begin
    // these registers are used in all modes
    r0       <= reg_bank_wen_c[0 ]               ? i_reg : read_data_wen[0 ]                      ? i_wb_read_data       : r0;  
    r1       <= reg_bank_wen_c[1 ]               ? i_reg : read_data_wen[1 ]                      ? i_wb_read_data       : r1;  
    r2       <= reg_bank_wen_c[2 ]               ? i_reg : read_data_wen[2 ]                      ? i_wb_read_data       : r2;  
    r3       <= reg_bank_wen_c[3 ]               ? i_reg : read_data_wen[3 ]                      ? i_wb_read_data       : r3;  
    r4       <= reg_bank_wen_c[4 ]               ? i_reg : read_data_wen[4 ]                      ? i_wb_read_data       : r4;  
    r5       <= reg_bank_wen_c[5 ]               ? i_reg : read_data_wen[5 ]                      ? i_wb_read_data       : r5;  
    r6       <= reg_bank_wen_c[6 ]               ? i_reg : read_data_wen[6 ]                      ? i_wb_read_data       : r6;  
    r7       <= reg_bank_wen_c[7 ]               ? i_reg : read_data_wen[7 ]                      ? i_wb_read_data       : r7;  
    
    // these registers are used in all modes, except fast irq
    r8       <= reg_bank_wen_c[8 ] && !firq_idec ? i_reg : read_data_wen[8 ] && i_wb_mode != FIRQ ? i_wb_read_data       : r8;  
    r9       <= reg_bank_wen_c[9 ] && !firq_idec ? i_reg : read_data_wen[9 ] && i_wb_mode != FIRQ ? i_wb_read_data       : r9;  
    r10      <= reg_bank_wen_c[10] && !firq_idec ? i_reg : read_data_wen[10] && i_wb_mode != FIRQ ? i_wb_read_data       : r10; 
    r11      <= reg_bank_wen_c[11] && !firq_idec ? i_reg : read_data_wen[11] && i_wb_mode != FIRQ ? i_wb_read_data       : r11; 
    r12      <= reg_bank_wen_c[12] && !firq_idec ? i_reg : read_data_wen[12] && i_wb_mode != FIRQ ? i_wb_read_data       : r12; 
    
    // these registers are used in fast irq mode
    r8_firq  <= reg_bank_wen_c[8 ] &&  firq_idec ? i_reg : read_data_wen[8 ] && i_wb_mode == FIRQ ? i_wb_read_data       : r8_firq;
    r9_firq  <= reg_bank_wen_c[9 ] &&  firq_idec ? i_reg : read_data_wen[9 ] && i_wb_mode == FIRQ ? i_wb_read_data       : r9_firq;
    r10_firq <= reg_bank_wen_c[10] &&  firq_idec ? i_reg : read_data_wen[10] && i_wb_mode == FIRQ ? i_wb_read_data       : r10_firq;
    r11_firq <= reg_bank_wen_c[11] &&  firq_idec ? i_reg : read_data_wen[11] && i_wb_mode == FIRQ ? i_wb_read_data       : r11_firq;
    r12_firq <= reg_bank_wen_c[12] &&  firq_idec ? i_reg : read_data_wen[12] && i_wb_mode == FIRQ ? i_wb_read_data       : r12_firq;

    // these registers are used in user mode
    r13      <= reg_bank_wen_c[13] &&  usr_idec  ? i_reg : read_data_wen[13] && i_wb_mode == USR ? i_wb_read_data        : r13;         
    r14      <= reg_bank_wen_c[14] &&  usr_idec  ? i_reg : read_data_wen[14] && i_wb_mode == USR ? i_wb_read_data        : r14;         
 
    // these registers are used in supervisor mode
    r13_svc  <= reg_bank_wen_c[13] &&  svc_idec  ? i_reg : read_data_wen[13] && i_wb_mode == SVC  ? i_wb_read_data       : r13_svc;     
    r14_svc  <= reg_bank_wen_c[14] &&  svc_idec  ? i_reg : read_data_wen[14] && i_wb_mode == SVC  ? i_wb_read_data       : r14_svc;     
   
    // these registers are used in irq mode
    r13_irq  <= reg_bank_wen_c[13] &&  irq_idec  ? i_reg : read_data_wen[13] && i_wb_mode == IRQ  ? i_wb_read_data       : r13_irq; 
    r14_irq  <= (reg_bank_wen_c[14] && irq_idec) ? i_reg : read_data_wen[14] && i_wb_mode == IRQ  ? i_wb_read_data       : r14_irq;      
  
    // these registers are used in fast irq mode
    r13_firq <= reg_bank_wen_c[13] &&  firq_idec ? i_reg : read_data_wen[13] && i_wb_mode == FIRQ ? i_wb_read_data       : r13_firq;
    r14_firq <= reg_bank_wen_c[14] &&  firq_idec ? i_reg : read_data_wen[14] && i_wb_mode == FIRQ ? i_wb_read_data       : r14_firq;  
    
    // these registers are used in all modes
    r15      <= pc_wen_c                         ?  i_pc : pc_dmem_wen                            ? i_wb_read_data[25:2] : r15;
    end
    
    
// ========================================================
// Register Read based on Mode
// ========================================================
assign r0_out = r0;
assign r1_out = r1;
assign r2_out = r2;
assign r3_out = r3;
assign r4_out = r4;
assign r5_out = r5;
assign r6_out = r6;
assign r7_out = r7;

assign r8_out  = firq_exec ? r8_firq  : r8;
assign r9_out  = firq_exec ? r9_firq  : r9;
assign r10_out = firq_exec ? r10_firq : r10;
assign r11_out = firq_exec ? r11_firq : r11;
assign r12_out = firq_exec ? r12_firq : r12;

assign r13_out = usr_exec ? r13      :
                 svc_exec ? r13_svc  :
                 irq_exec ? r13_irq  :
                          r13_firq ;
                       
assign r14_out = usr_exec ? r14      :
                 svc_exec ? r14_svc  :
                 irq_exec ? r14_irq  :
                          r14_firq ;
 

assign r15_out_rm     = { i_status_bits_flags, 
                          i_status_bits_irq_mask, 
                          i_status_bits_firq_mask, 
                          r15, 
                          i_mode_exec};

assign r15_out_rm_nxt = { i_status_bits_flags, 
                          i_status_bits_irq_mask, 
                          i_status_bits_firq_mask, 
                          i_pc, 
                          i_mode_exec};
                      
assign r15_out_rn     = {6'd0, r15, 2'd0};


// rds outputs
assign r8_rds  = i_mode_rds_exec[OH_FIRQ] ? r8_firq  : r8;
assign r9_rds  = i_mode_rds_exec[OH_FIRQ] ? r9_firq  : r9;
assign r10_rds = i_mode_rds_exec[OH_FIRQ] ? r10_firq : r10;
assign r11_rds = i_mode_rds_exec[OH_FIRQ] ? r11_firq : r11;
assign r12_rds = i_mode_rds_exec[OH_FIRQ] ? r12_firq : r12;

assign r13_rds = i_mode_rds_exec[OH_USR]  ? r13      :
                 i_mode_rds_exec[OH_SVC]  ? r13_svc  :
                 i_mode_rds_exec[OH_IRQ]  ? r13_irq  :
                                            r13_firq ;
                       
assign r14_rds = i_mode_rds_exec[OH_USR]  ? r14      :
                 i_mode_rds_exec[OH_SVC]  ? r14_svc  :
                 i_mode_rds_exec[OH_IRQ]  ? r14_irq  :
                                            r14_firq ;


// ========================================================
// Program Counter out
// ========================================================
assign o_pc = r15_out_rn;

// ========================================================
// Rm Selector
// ========================================================
assign o_rm = i_rm_sel == 4'd0  ? r0_out  :
              i_rm_sel == 4'd1  ? r1_out  : 
              i_rm_sel == 4'd2  ? r2_out  : 
              i_rm_sel == 4'd3  ? r3_out  : 
              i_rm_sel == 4'd4  ? r4_out  : 
              i_rm_sel == 4'd5  ? r5_out  : 
              i_rm_sel == 4'd6  ? r6_out  : 
              i_rm_sel == 4'd7  ? r7_out  : 
              i_rm_sel == 4'd8  ? r8_out  : 
              i_rm_sel == 4'd9  ? r9_out  : 
              i_rm_sel == 4'd10 ? r10_out : 
              i_rm_sel == 4'd11 ? r11_out : 
              i_rm_sel == 4'd12 ? r12_out : 
              i_rm_sel == 4'd13 ? r13_out : 
              i_rm_sel == 4'd14 ? r14_out : 
                                  r15_out_rm ; 


// ========================================================
// Rds Selector
// ========================================================
always @*
    case ( i_rs_sel )
       4'd0  :  o_rs = r0_out  ;
       4'd1  :  o_rs = r1_out  ; 
       4'd2  :  o_rs = r2_out  ; 
       4'd3  :  o_rs = r3_out  ; 
       4'd4  :  o_rs = r4_out  ; 
       4'd5  :  o_rs = r5_out  ; 
       4'd6  :  o_rs = r6_out  ; 
       4'd7  :  o_rs = r7_out  ; 
       4'd8  :  o_rs = r8_rds  ; 
       4'd9  :  o_rs = r9_rds  ; 
       4'd10 :  o_rs = r10_rds ; 
       4'd11 :  o_rs = r11_rds ; 
       4'd12 :  o_rs = r12_rds ; 
       4'd13 :  o_rs = r13_rds ; 
       4'd14 :  o_rs = r14_rds ; 
       default: o_rs = r15_out_rn ; 
    endcase

                                    
// ========================================================
// Rd Selector
// ========================================================
always @*
    case ( i_rs_sel )
       4'd0  :  o_rd = r0_out  ;
       4'd1  :  o_rd = r1_out  ; 
       4'd2  :  o_rd = r2_out  ; 
       4'd3  :  o_rd = r3_out  ; 
       4'd4  :  o_rd = r4_out  ; 
       4'd5  :  o_rd = r5_out  ; 
       4'd6  :  o_rd = r6_out  ; 
       4'd7  :  o_rd = r7_out  ; 
       4'd8  :  o_rd = r8_rds  ; 
       4'd9  :  o_rd = r9_rds  ; 
       4'd10 :  o_rd = r10_rds ; 
       4'd11 :  o_rd = r11_rds ; 
       4'd12 :  o_rd = r12_rds ; 
       4'd13 :  o_rd = r13_rds ; 
       4'd14 :  o_rd = r14_rds ; 
       default: o_rd = r15_out_rm_nxt ; 
    endcase

                                    
// ========================================================
// Rn Selector
// ========================================================
assign o_rn = i_rn_sel == 4'd0  ? r0_out  :
              i_rn_sel == 4'd1  ? r1_out  : 
              i_rn_sel == 4'd2  ? r2_out  : 
              i_rn_sel == 4'd3  ? r3_out  : 
              i_rn_sel == 4'd4  ? r4_out  : 
              i_rn_sel == 4'd5  ? r5_out  : 
              i_rn_sel == 4'd6  ? r6_out  : 
              i_rn_sel == 4'd7  ? r7_out  : 
              i_rn_sel == 4'd8  ? r8_out  : 
              i_rn_sel == 4'd9  ? r9_out  : 
              i_rn_sel == 4'd10 ? r10_out : 
              i_rn_sel == 4'd11 ? r11_out : 
              i_rn_sel == 4'd12 ? r12_out : 
              i_rn_sel == 4'd13 ? r13_out : 
              i_rn_sel == 4'd14 ? r14_out : 
                                  r15_out_rn ; 

    //logic [7:0] led_reg;
    //assign led = led_reg;
    always_comb begin
        case (sw[7:2])
            6'd0: begin
                case (sw[1:0])
                    2'd0: led = r0[7:0];
                    2'd1: led = r0[15:8];
                    2'd2: led = r0[23:16];
                    2'd3: led = r0[31:24];
                endcase
            end
            6'd1: begin
                case (sw[1:0])
                    2'd0: led = r1[7:0];
                    2'd1: led = r1[15:8];
                    2'd2: led = r1[23:16];
                    2'd3: led = r1[31:24];
                endcase
            end
            6'd2: begin
                case (sw[1:0])
                    2'd0: led = r2[7:0];
                    2'd1: led = r2[15:8];
                    2'd2: led = r2[23:16];
                    2'd3: led = r2[31:24];
                endcase
            end
            6'd3: begin
                case (sw[1:0])
                    2'd0: led = r3[7:0];
                    2'd1: led = r3[15:8];
                    2'd2: led = r3[23:16];
                    2'd3: led = r3[31:24];
                endcase
            end
            6'd15: begin
                case (sw[1:0])
                    2'd0: led = r15[7:0];
                    2'd1: led = r15[15:8];
                    2'd2: led = r15[23:16];
                    2'd3: led = 8'd0/*r15[31:24]*/;
                endcase
            end
            default: begin
                led = 8'hcc;
            end
        endcase
        /*if (a25_core.u_execute.u_register_bank.r0
        if (blkmem_wea_mask != 16'h0000) led_reg <= s_wb_adr[2][7:0];
        else led_reg <= 8'h0;*/
    end


endmodule


