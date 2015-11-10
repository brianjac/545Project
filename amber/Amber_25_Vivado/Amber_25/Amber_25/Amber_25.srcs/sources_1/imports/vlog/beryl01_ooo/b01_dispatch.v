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


//Define reservation station information being recorded for each given instruction
typedef struct alu_station_entry {
	//Operand 1 data
	logic rn_valid;
	logic [5:0] rn_tag;
	logic [31:0] rn;
	
	//Operand 2 data
	logic rs_valid;
	logic [5:0] rs_tag;
	logic [31:0] rs;
	
	//Operand 3 data (used if we write to a reg, one operand is a reg, and the other operand is a reg shifted by another reg)
	logic rm_valid;
	logic [5:0] rm_tag;
	logic [31:0] rm;
	
	//CCR data
	logic ccr_valid;
	logic [5:0] ccr_tag;
	logic [31:0] ccr;
	
	//Control signals for the ALU stage
	logic [31:0] imm32;
	logic [4:0] imm_shift_amount;
	logic shift_imm_zero;
	logic [8:0] exec_load_rd,
	logic [1:0] barrel_shift_amount_sel,
	logic [1:0] barrel_shift_data_sel,
	logic [1:0] barrel_shift_function,
	logic [8:0] alu_function,
	logic pc_wen,
	logic [14:0] reg_bank_wen,
	logic status_bits_flags_wen,
	logic status_bits_mode_wen,
	logic status_bits_irq_mask_wen,
	logic status_bits_firq_mask_wen
} alu_station_entry;

typedef struct mult_station_entry {
	//Operand 1 data
	logic op1_valid;
	logic [5:0] op1_tag;
	logic [31:0] op1;
	
	//Operand 2 data
	logic op2_valid;
	logic [5:0] op2_tag;
	logic [31:0] op2;
	
	//Operand 3 data (used if we write to a reg, one operand is a reg, and the other operand is a reg shifted by another reg)
	logic op3_valid;
	logic [5:0] op3_tag;
	logic [31:0] op3;
	
	//CCR data
	logic ccr_valid;
	logic [5:0] ccr_tag;
	logic [31:0] ccr;
	
	//Control signals for the multiply stage
	logic [3:0] rm_sel,
	logic [3:0] rs_sel,
	logic [3:0] rn_sel,
	logic [8:0] exec_load_rd,
	logic [1:0] multiply_function,
	logic use_carry_in,
	logic [14:0] reg_bank_wen,
	logic status_bits_flags_wen,
	logic status_bits_mode_wen,
	logic status_bits_irq_mask_wen,
	logic status_bits_firq_mask_wen
} mult_station_entry;

typedef struct mem_station_entry {
	//2, 4, 8.1, byte_enable_sel, daddress_sel, iaddress_sel
	//Operand 1 data
	logic op1_valid;
	logic [5:0] op1_tag;
	logic [31:0] op1;
	
	//Operand 2 data
	logic op2_valid;
	logic [5:0] op2_tag;
	logic [31:0] op2;
	
	//Operand 3 data (used if we write to a reg, one operand is a reg, and the other operand is a reg shifted by another reg)
	//TODO: determine if this is ever actually used in memory instructions
	logic op3_valid;
	logic [5:0] op3_tag;
	logic [31:0] op3;
	
	//CCR data
	logic ccr_valid;
	logic [5:0] ccr_tag;
	logic [31:0] ccr;
	
	//Control signals for the memory stage
	logic exclusive;
	logic decode_iaccess,
	logic decode_daccess,
	logic [7:0] decode_load_rd, //TODO not on the original list; is it really needed?
	logic [3:0] rm_sel,
	logic [3:0] rs_sel,
	logic [3:0] rn_sel,
	logic [8:0] exec_load_rd,
	logic [14:0] reg_bank_wen,
	logic status_bits_flags_wen,
	logic status_bits_mode_wen,
	logic status_bits_irq_mask_wen,
	logic status_bits_firq_mask_wen,
	logic [3:0] iaddress_sel,
	logic [3:0] daddress_sel,
	logic [1:0] byte_enable_sel
} mem_station_entry;



module b01_dispatch (

input                       i_clk,
input                       i_core_stall,               // stall all stages of the Amber core at the same time
input                       i_mem_stall,                // data memory access stalls
output                      o_exec_stall,               // stall the core pipeline

input       [31:0]          i_wb_read_data,             // data reads
input                       i_wb_read_data_valid,       // read data is valid
input       [10:0]          i_wb_load_rd,               // Rd for data reads

input       [31:0]          i_copro_read_data,          // From Co-Processor, to either Register
                                                        // or Memory
input                       i_decode_iaccess,           // Indicates an instruction access
input                       i_decode_daccess,           // Indicates a data access
input       [7:0]           i_decode_load_rd,           // The destination register for a load instruction

//TODO add coprocessor instruction handling
//output reg  [31:0]          o_copro_write_data = 'd0,
//output reg  [31:0]          o_write_data = 'd0,
output wire [31:0]          o_iaddress,
output      [31:0]          o_iaddress_nxt,             // un-registered version of address to the
                                                        // cache rams address ports
output reg                  o_iaddress_valid = 'd0,     // High when instruction address is valid

output reg                  o_adex = 'd0,               // Address Exception
output reg                  o_priviledged = 'd0,        // Priviledged access
output reg                  o_exclusive = 'd0,          // swap access
output reg                  o_write_enable = 'd0,
output reg  [3:0]           o_byte_enable = 'd0,

output      [31:0]          o_status_bits,              // Full PC will all status bits, but PC part zero'ed out


// --------------------------------------------------
// Control signals from Instruction Decode stage
// --------------------------------------------------
input      [1:0]            i_status_bits_mode,
input                       i_status_bits_irq_mask,
input                       i_status_bits_firq_mask,
input      [31:0]           i_imm32,
input      [4:0]            i_imm_shift_amount,
input                       i_shift_imm_zero,
input      [3:0]            i_condition,
input                       i_decode_exclusive,       // swap access

input      [3:0]            i_rm_sel,
input      [3:0]            i_rs_sel,
input      [3:0]            i_rn_sel,
input      [1:0]            i_barrel_shift_amount_sel,
input      [1:0]            i_barrel_shift_data_sel,
input      [1:0]            i_barrel_shift_function,
input      [8:0]            i_alu_function,
input      [1:0]            i_multiply_function,
input      [2:0]            i_interrupt_vector_sel,
input      [3:0]            i_iaddress_sel,
input      [3:0]            i_daddress_sel,
input      [2:0]            i_pc_sel,
input      [1:0]            i_byte_enable_sel,
input      [2:0]            i_status_bits_sel,
input      [2:0]            i_reg_write_sel,
input                       i_user_mode_regs_store_nxt,
input                       i_firq_not_user_mode,
input                       i_use_carry_in,         // e.g. add with carry instruction

input                       i_write_data_wen,
input                       i_base_address_wen,     // save LDM base address register,
                                                    // in case of data abort
input                       i_pc_wen,
input      [14:0]           i_reg_bank_wen,
input                       i_status_bits_flags_wen,
input                       i_status_bits_mode_wen,
input                       i_status_bits_irq_mask_wen,
input                       i_status_bits_firq_mask_wen,
input                       i_copro_write_data_wen,
input                       i_conflict,
input                       i_rn_use_read,
input                       i_rm_use_read,
input                       i_rs_use_read,
input                       i_rd_use_read,

//Stuff specifically added for OOO processing
output	alu_station_entry	o_alu_stage_command;
output						o_alu_stage_command_valid;
output	mult_station_entry	o_mult_stage_command;
output						o_mult_stage_command_valid;
output	mem_station_entry	o_mem_stage_command;
output						o_mem_stage_command_valid;
input						i_alu_valid,
							i_mult_valid,
							i_mem_valid;
input	[5:0]				i_alu_tag,
							i_mult_tag,
							i_mem_tag,
input	[31:0]				i_alu_data,
							i_mult_data,
							i_mem_data,
input	[3:0]				i_alu_flags,
							i_mult_flags,
							i_mem_flags;

output [7:0] led,
input [7:0] sw
);

`include "a25_localparams.vh"
`include "a25_functions.vh"

// ========================================================
// Internal signals
// ========================================================
wire [31:0]         write_data_nxt;
wire [3:0]          byte_enable_nxt;
wire [31:0]         pc_plus4;
wire [31:0]         pc_minus4;
wire [31:0]         daddress_plus4;
wire [31:0]         alu_plus4;
wire [31:0]         rn_plus4;
wire [31:0]         alu_out;
wire [3:0]          alu_flags;
wire [31:0]         rm;
wire [31:0]         rs;
wire [31:0]         rd;
wire [31:0]         rn;
wire [31:0]         reg_bank_rn;
wire [31:0]         reg_bank_rm;
wire [31:0]         reg_bank_rs;
wire [31:0]         reg_bank_rd;
wire [31:0]         pc;
wire [31:0]         pc_nxt;
wire [31:0]         interrupt_vector;
wire [7:0]          shift_amount;
wire [31:0]         barrel_shift_in;
wire [31:0]         barrel_shift_out;
wire                barrel_shift_carry;
wire                barrel_shift_stall;
wire                barrel_shift_carry_alu;

wire [3:0]          status_bits_flags_nxt;
reg  [3:0]          status_bits_flags = 'd0;
wire [1:0]          status_bits_mode_nxt;
reg  [1:0]          status_bits_mode = SVC;
                    // one-hot encoded rs select
wire [3:0]          status_bits_mode_rds_oh_nxt;
reg  [3:0]          status_bits_mode_rds_oh = 1'd1 << OH_SVC;
wire                status_bits_mode_rds_oh_update;
wire                status_bits_irq_mask_nxt;
reg                 status_bits_irq_mask = 1'd1;
wire                status_bits_firq_mask_nxt;
reg                 status_bits_firq_mask = 1'd1;
wire [8:0]          exec_load_rd_nxt;

wire                execute;                    // high when condition execution is true
wire [31:0]         reg_write_nxt;
wire                pc_wen;
wire [14:0]         reg_bank_wen;
wire [31:0]         multiply_out;
wire [1:0]          multiply_flags;
reg  [31:0]         base_address = 'd0;             // Saves base address during LDM instruction in
                                                    // case of data abort
wire [31:0]         read_data_filtered1;
wire [31:0]         read_data_filtered;
wire [31:0]         read_data_filtered_c;
reg  [31:0]         read_data_filtered_r = 'd0;
reg  [5:0]          load_rd_r = 'd0;
wire [5:0]          load_rd_c;

wire                write_enable_nxt;
wire                daddress_valid_nxt;
wire                iaddress_valid_nxt;
wire                priviledged_nxt;
wire                priviledged_update;
wire                iaddress_update;
wire                daddress_update;
wire                base_address_update;
wire                write_data_update;
wire                copro_write_data_update;
wire                byte_enable_update;
wire                exec_load_rd_update;
wire                write_enable_update;
wire                exclusive_update;
wire                status_bits_flags_update;
wire                status_bits_mode_update;
wire                status_bits_irq_mask_update;
wire                status_bits_firq_mask_update;

wire [31:0]         alu_out_pc_filtered;
wire                adex_nxt;
wire [31:0]         save_int_pc;
wire [31:0]         save_int_pc_m4;
wire                ldm_flags;
wire                ldm_status_bits;

wire                carry_in;

//reg   [31:0]        iaddress_r = 32'hdead_dead;
reg [31:0] iaddress_r = 32'h0000_0000;


// ========================================================
// Status Bits in PC register
// ========================================================
wire [1:0] status_bits_mode_out;
assign status_bits_mode_out = (i_status_bits_mode_wen && i_status_bits_sel == 3'd1 && !ldm_status_bits) ?
                                    alu_out[1:0] : status_bits_mode ;
									
logic [5:0] status_bits_flags_tag; //update on each newly decoded instruction based on i_status_bits_flags_wen
logic status_bits_flags_valid; //invalidate on each newly decoded instruction based on i_status_bits_flags_wen

//Should need no changes for OOO given how we structure things below
assign o_status_bits = {   status_bits_flags,           // 31:28
                           status_bits_irq_mask,        // 7
                           status_bits_firq_mask,       // 6
                           24'd0,
                           status_bits_mode_out };      // 1:0 = mode


// ========================================================
// Status Bits Select
// ========================================================

//TODO figure these two out
assign ldm_flags                 = i_wb_read_data_valid & ~i_mem_stall & i_wb_load_rd[8];
assign ldm_status_bits           = i_wb_read_data_valid & ~i_mem_stall & i_wb_load_rd[7];

//Note that here we're removing reference to the coprocessor, since it doesn't *actually* exist in the design.
//The rest of the time, i_xyz_flags will be set in the appropriate Execute substage based on the i_status_bits_sel we pass in.
//Additional note: the ldm flag-handling might require some special logic here.
assign status_bits_flags_nxt 	 = i_alu_valid && i_alu_tag == status_bits_flags_tag 	? i_alu_flags 	:
								   i_mult_valid && i_mult_tag == status_bits_flags_tag 	? i_mult_flags 	:
								   i_mem_valid && i_mem_tag == status_bits_flags_tag 	? i_mem_flags	:
																						  4'b1111;
/*assign status_bits_flags_nxt     = ldm_flags                 ? read_data_filtered[31:28]           :
                                   i_status_bits_sel == 3'd0 ? alu_flags                           :
                                   i_status_bits_sel == 3'd1 ? alu_out          [31:28]            :
                                   i_status_bits_sel == 3'd3 ? i_copro_read_data[31:28]            :
                                   // 4 = update flags after a multiply operation
                                   i_status_bits_sel == 3'd4 ? { multiply_flags, status_bits_flags[1:0] } :
                                   // regops that do not change the overflow flag
                                   i_status_bits_sel == 3'd5 ? { alu_flags[3:1], status_bits_flags[0] } :
                                                               4'b1111 ;*/

//Should need no changes for OOO
assign status_bits_mode_nxt      = ldm_status_bits           ? read_data_filtered [1:0] :
                                   i_status_bits_sel == 3'd0 ? i_status_bits_mode       :
                                   i_status_bits_sel == 3'd5 ? i_status_bits_mode       :
                                   i_status_bits_sel == 3'd1 ? alu_out            [1:0] :
                                                               i_copro_read_data  [1:0] ;


// Used for the Rds output of register_bank - this special version of
// status_bits_mode speeds up the critical path from status_bits_mode through the
// register_bank, barrel_shifter and alu. It moves a mux needed for the
// i_user_mode_regs_store_nxt signal back into the previous stage -
// so its really part of the decode stage even though the logic is right here
// In addition the signal is one-hot encoded to further speed up the logic

assign status_bits_mode_rds_oh_nxt    = i_user_mode_regs_store_nxt ? 1'd1 << OH_USR                            :
                                        status_bits_mode_update    ? oh_status_bits_mode(status_bits_mode_nxt) :
                                                                     oh_status_bits_mode(status_bits_mode)     ;


assign status_bits_irq_mask_nxt  = ldm_status_bits           ? read_data_filtered     [27] :
                                   i_status_bits_sel == 3'd0 ? i_status_bits_irq_mask      :
                                   i_status_bits_sel == 3'd5 ? i_status_bits_irq_mask      :
                                   i_status_bits_sel == 3'd1 ? alu_out                [27] :
                                                               i_copro_read_data      [27] ;

assign status_bits_firq_mask_nxt = ldm_status_bits           ? read_data_filtered     [26] :
                                   i_status_bits_sel == 3'd0 ? i_status_bits_firq_mask     :
                                   i_status_bits_sel == 3'd5 ? i_status_bits_firq_mask     :
                                   i_status_bits_sel == 3'd1 ? alu_out                [26] :
                                                               i_copro_read_data      [26] ;



// ========================================================
// Adders
// ========================================================
assign pc_plus4       = pc         + 32'd4;
assign pc_minus4      = pc         - 32'd4;
assign daddress_plus4 = o_daddress + 32'd4;
assign alu_plus4      = alu_out    + 32'd4;
assign rn_plus4       = rn         + 32'd4;

// ========================================================
// Barrel Shift Amount Select
// ========================================================
// An immediate shift value of 0 is translated into 32
assign shift_amount = i_barrel_shift_amount_sel == 2'd0 ? 8'd0                         :
                      i_barrel_shift_amount_sel == 2'd1 ? rs[7:0]                      :
                                                          {3'd0, i_imm_shift_amount  } ;


// ========================================================
// Barrel Shift Data Select
// ========================================================
assign barrel_shift_in = i_barrel_shift_data_sel == 2'd0 ? i_imm32 : rm ;


// ========================================================
// Interrupt vector Select
// ========================================================

assign interrupt_vector = // Reset vector
                          (i_interrupt_vector_sel == 3'd0) ? 32'h00000000 :
                          // Data abort interrupt vector
                          (i_interrupt_vector_sel == 3'd1) ? 32'h00000010 :
                          // Fast interrupt vector
                          (i_interrupt_vector_sel == 3'd2) ? 32'h0000001c :
                          // Regular interrupt vector
                          (i_interrupt_vector_sel == 3'd3) ? 32'h00000018 :
                          // Prefetch abort interrupt vector
                          (i_interrupt_vector_sel == 3'd5) ? 32'h0000000c :
                          // Undefined instruction interrupt vector
                          (i_interrupt_vector_sel == 3'd6) ? 32'h00000004 :
                          // Software (SWI) interrupt vector
                          (i_interrupt_vector_sel == 3'd7) ? 32'h00000008 :
                          // Default is the address exception interrupt
                                                             32'h00000014 ;


// ========================================================
// Address Select
// ========================================================
assign pc_dmem_wen    = i_wb_read_data_valid & ~i_mem_stall & i_wb_load_rd[3:0] == 4'd15;

// If rd is the pc, then seperate the address bits from the status bits for
// generating the next address to fetch
assign alu_out_pc_filtered = pc_wen && i_pc_sel == 3'd1 ? pcf(alu_out) : alu_out;

// if current instruction does not execute because it does not meet the condition
// then address advances to next instruction
assign o_iaddress_nxt = (pc_dmem_wen)            ? pcf(read_data_filtered) :
                        (!execute)               ? pc_plus4                :
                        (i_iaddress_sel == 4'd0) ? pc_plus4                :
                        (i_iaddress_sel == 4'd1) ? alu_out_pc_filtered     :
                        (i_iaddress_sel == 4'd2) ? interrupt_vector        :
                                                   pc                      ;



// if current instruction does not execute because it does not meet the condition
// then address advances to next instruction
assign o_daddress_nxt = (i_daddress_sel == 4'd1) ? alu_out_pc_filtered   :
                        (i_daddress_sel == 4'd2) ? interrupt_vector      :
                        (i_daddress_sel == 4'd4) ? rn                    :
                        (i_daddress_sel == 4'd5) ? daddress_plus4        :  // MTRANS address incrementer
                        (i_daddress_sel == 4'd6) ? alu_plus4             :  // MTRANS decrement after
                                                   rn_plus4              ;  // MTRANS increment before

// Data accesses use 32-bit address space, but instruction
// accesses are restricted to 26 bit space
//TODO modify based on what address space we *actually* get in the system?
assign adex_nxt      = |o_iaddress_nxt[31:26] && i_decode_iaccess;


// ========================================================
// Filter Read Data
// ========================================================
// mem_load_rd[10:9]-> shift ROR bytes
// mem_load_rd[8]   -> load flags with PC
// mem_load_rd[7]   -> load status bits with PC
// mem_load_rd[6:5] -> Write into this Mode registers
// mem_load_rd[4]   -> zero_extend byte
// mem_load_rd[3:0] -> Destination Register
/*assign read_data_filtered1 = i_wb_load_rd[10:9] == 2'd0 ? i_wb_read_data                                :
                             i_wb_load_rd[10:9] == 2'd1 ? {i_wb_read_data[7:0],  i_wb_read_data[31:8]}  :
                             i_wb_load_rd[10:9] == 2'd2 ? {i_wb_read_data[15:0], i_wb_read_data[31:16]} :
                                                          {i_wb_read_data[23:0], i_wb_read_data[31:24]} ;

assign read_data_filtered  = i_wb_load_rd[4] ? {24'd0, read_data_filtered1[7:0]} : read_data_filtered1 ;*/


// ========================================================
// Program Counter Select
// ========================================================
// If current instruction does not execute because it does not meet the condition
// then PC advances to next instruction
assign pc_nxt = (!execute)       ? pc_plus4                :
                i_pc_sel == 3'd0 ? pc_plus4                :
                i_pc_sel == 3'd1 ? alu_out                 :
                i_pc_sel == 3'd2 ? interrupt_vector        :
                i_pc_sel == 3'd3 ? pcf(read_data_filtered) :
                                   pc_minus4               ;


// ========================================================
// Register Write Select
// ========================================================

assign save_int_pc    = { status_bits_flags,
                          status_bits_irq_mask,
                          status_bits_firq_mask,
                          pc[25:2],
                          status_bits_mode      };


assign save_int_pc_m4 = { status_bits_flags,
                          status_bits_irq_mask,
                          status_bits_firq_mask,
                          pc_minus4[25:2],
                          status_bits_mode      };


assign reg_write_nxt = i_reg_write_sel == 3'd0 ? alu_out               :
                       // save pc to lr on an interrupt
                       i_reg_write_sel == 3'd1 ? save_int_pc_m4        :
                       // to update Rd at the end of Multiplication
                       i_reg_write_sel == 3'd2 ? multiply_out          :
                       i_reg_write_sel == 3'd3 ? o_status_bits         :
                       i_reg_write_sel == 3'd5 ? i_copro_read_data     :  // mrc
                       i_reg_write_sel == 3'd6 ? base_address          :
                                                 save_int_pc           ;


// ========================================================
// Byte Enable Select
// ========================================================

//TODO move to Mem stage
/*assign byte_enable_nxt = i_byte_enable_sel == 2'd0   ? 4'b1111 :  // word write
                         i_byte_enable_sel == 2'd2   ?            // halfword write
                         ( o_daddress_nxt[1] == 1'd0 ? 4'b0011 :
                                                       4'b1100  ) :

                         o_daddress_nxt[1:0] == 2'd0 ? 4'b0001 :  // byte write
                         o_daddress_nxt[1:0] == 2'd1 ? 4'b0010 :
                         o_daddress_nxt[1:0] == 2'd2 ? 4'b0100 :
                                                       4'b1000 ;*/


// ========================================================
// Write Data Select
// ========================================================

//TODO move to Mem stage
/*assign write_data_nxt = i_byte_enable_sel == 2'd0 ? rd            :
                                                    {4{rd[ 7:0]}} ;*/


// ========================================================
// Conditional Execution
// ========================================================

//TODO make dependent on status_bits_flags_valid
assign execute = conditional_execute ( i_condition, status_bits_flags );

// allow the PC to increment to the next instruction when current
// instruction does not execute
assign pc_wen       = (i_pc_wen || !execute) && !i_conflict;

// only update register bank if current instruction executes
assign reg_bank_wen = {{15{execute}} & i_reg_bank_wen};


// ========================================================
// Priviledged output flag
// ========================================================
// Need to look at status_bits_mode_nxt so switch to priviledged mode
// at the same time as assert interrupt vector address
assign priviledged_nxt  = ( i_status_bits_mode_wen ? status_bits_mode_nxt : status_bits_mode ) != USR ;


// ========================================================
// Write Enable
// ========================================================

//TODO as with "execute", make dependent on status_bits_flags_valid
// This must be de-asserted when execute is fault
assign write_enable_nxt = execute && i_write_data_wen;


// ========================================================
// Address Valid
// ========================================================
//assign daddress_valid_nxt = execute && i_decode_daccess && !i_core_stall;

// For some multi-cycle instructions, the stream of instrution
// reads can be paused. However if the instruction does not execute
// then the read stream must not be interrupted.
assign iaddress_valid_nxt = i_decode_iaccess || !execute;


// ========================================================
// Use read value from data memory instead of from register
// ========================================================
assign rn = /*i_rn_use_read && i_rn_sel == load_rd_c[3:0] && status_bits_mode == load_rd_c[5:4] ? read_data_filtered_c :*/ reg_bank_rn;
assign rm = /*i_rm_use_read && i_rm_sel == load_rd_c[3:0] && status_bits_mode == load_rd_c[5:4] ? read_data_filtered_c :*/ reg_bank_rm;
assign rs = /*i_rs_use_read && i_rs_sel == load_rd_c[3:0] && status_bits_mode == load_rd_c[5:4] ? read_data_filtered_c :*/ reg_bank_rs;
assign rd = /*i_rd_use_read && i_rs_sel == load_rd_c[3:0] && status_bits_mode == load_rd_c[5:4] ? read_data_filtered_c :*/ reg_bank_rd;


/*always@( posedge i_clk )
    if ( i_wb_read_data_valid )
        begin
        read_data_filtered_r <= read_data_filtered;
        load_rd_r            <= {i_wb_load_rd[6:5], i_wb_load_rd[3:0]};
        end

assign read_data_filtered_c = i_wb_read_data_valid ? read_data_filtered : read_data_filtered_r;

// the register number and the mode
assign load_rd_c            = i_wb_read_data_valid ? {i_wb_load_rd[6:5], i_wb_load_rd[3:0]}  : load_rd_r;*/


// ========================================================
// Set mode for the destination registers of a mem read
// ========================================================
// The mode is either user mode, or the current mode
assign  exec_load_rd_nxt   = { i_decode_load_rd[7:6],
                               i_decode_load_rd[5] ? USR : status_bits_mode,  // 1 bit -> 2 bits
                               i_decode_load_rd[4:0] };


// ========================================================
// Register Update
// ========================================================

//TODO o_exec_stall will need to be something different:
//Stall if:
//1) a reservation station is full and we need the space;
//2) we need to resolve the next PC and can't until we have more data;
//3) <... anything else? TODO decide.>
assign o_exec_stall                    = barrel_shift_stall;

//assign daddress_update                 = !i_core_stall;
//assign exec_load_rd_update             = !i_core_stall && execute;
assign priviledged_update              = !i_core_stall;
assign exclusive_update                = !i_core_stall && execute;
//assign write_enable_update             = !i_core_stall;
//assign write_data_update               = !i_core_stall && execute && i_write_data_wen;
//assign byte_enable_update              = !i_core_stall && execute && i_write_data_wen;

assign iaddress_update                 = pc_dmem_wen || (!i_core_stall && !i_conflict);
//assign copro_write_data_update         = !i_core_stall && execute && i_copro_write_data_wen;

//TODO what does base_address_update do exactly?
assign base_address_update             = !i_core_stall && execute && i_base_address_wen;
//TODO will need to change logic for status_bits_flags_update and/or eliminate this from here altogether.
//assign status_bits_flags_update        = ldm_flags       || (!i_core_stall && execute && i_status_bits_flags_wen);
assign status_bits_mode_update         = ldm_status_bits || (!i_core_stall && execute && i_status_bits_mode_wen);
assign status_bits_mode_rds_oh_update  = !i_core_stall;
assign status_bits_irq_mask_update     = ldm_status_bits || (!i_core_stall && execute && i_status_bits_irq_mask_wen);
assign status_bits_firq_mask_update    = ldm_status_bits || (!i_core_stall && execute && i_status_bits_firq_mask_wen);


always @( posedge i_clk )
    begin
    //o_daddress              <= daddress_update                ? o_daddress_nxt               : o_daddress;
    //o_daddress_valid        <= daddress_update                ? daddress_valid_nxt           : o_daddress_valid;
    //o_exec_load_rd          <= exec_load_rd_update            ? exec_load_rd_nxt             : o_exec_load_rd;
    o_priviledged           <= priviledged_update             ? priviledged_nxt              : o_priviledged;
    o_exclusive             <= exclusive_update               ? i_decode_exclusive           : o_exclusive;
    //o_write_enable          <= write_enable_update            ? write_enable_nxt             : o_write_enable;
    //o_write_data            <= write_data_update              ? write_data_nxt               : o_write_data;
    //o_byte_enable           <= byte_enable_update             ? byte_enable_nxt              : o_byte_enable;
    iaddress_r              <= iaddress_update                ? o_iaddress_nxt               : iaddress_r;
    o_iaddress_valid        <= iaddress_update                ? iaddress_valid_nxt           : o_iaddress_valid;
    o_adex                  <= iaddress_update                ? adex_nxt                     : o_adex;
    //o_copro_write_data      <= copro_write_data_update        ? write_data_nxt               : o_copro_write_data;

    base_address            <= base_address_update            ? rn                           : base_address;

    //status_bits_flags       <= status_bits_flags_update       ? status_bits_flags_nxt        : status_bits_flags;
    status_bits_mode        <= status_bits_mode_update        ? status_bits_mode_nxt         : status_bits_mode;
    status_bits_mode_rds_oh <= status_bits_mode_rds_oh_update ? status_bits_mode_rds_oh_nxt  : status_bits_mode_rds_oh;
    status_bits_irq_mask    <= status_bits_irq_mask_update    ? status_bits_irq_mask_nxt     : status_bits_irq_mask;
    status_bits_firq_mask   <= status_bits_firq_mask_update   ? status_bits_firq_mask_nxt    : status_bits_firq_mask;
    end

assign o_iaddress = iaddress_r;


// ========================================================
// Instantiate Barrel Shift
// ========================================================
assign carry_in = i_use_carry_in ? status_bits_flags[1] : 1'd0;


// ========================================================
// Instantiate ALU
// ========================================================
assign barrel_shift_carry_alu =  i_barrel_shift_data_sel == 2'd0 ?
                                  (i_imm_shift_amount[4:1] == 0 ? status_bits_flags[1] : i_imm32[31]) :
                                   barrel_shift_carry;


// ========================================================
// Instantiate Register Bank
// ========================================================

//TODO need to modify with tag data and valid-bit r/w
a25_register_bank u_register_bank(
    .i_clk                   ( i_clk                     ),
    .i_core_stall            ( i_core_stall              ),
    .i_mem_stall             ( i_mem_stall               ),
    .i_rm_sel                ( i_rm_sel                  ),
    .i_rs_sel                ( i_rs_sel                  ),
    .i_rn_sel                ( i_rn_sel                  ),
    .i_pc_wen                ( pc_wen                    ),
    .i_reg_bank_wen          ( reg_bank_wen              ),
    .i_pc                    ( pc_nxt[25:2]              ),
    .i_reg                   ( reg_write_nxt             ),
    .i_mode_idec             ( i_status_bits_mode        ),
    .i_mode_exec             ( status_bits_mode          ),

    .i_wb_read_data          ( read_data_filtered        ),
    .i_wb_read_data_valid    ( i_wb_read_data_valid      ),
    .i_wb_read_data_rd       ( i_wb_load_rd[3:0]         ),
    .i_wb_mode               ( i_wb_load_rd[6:5]         ),

    .i_status_bits_flags     ( status_bits_flags         ),
    .i_status_bits_irq_mask  ( status_bits_irq_mask      ),
    .i_status_bits_firq_mask ( status_bits_firq_mask     ),

    // pre-encoded in decode stage to speed up long path
    .i_firq_not_user_mode    ( i_firq_not_user_mode      ),

    // use one-hot version for speed, combine with i_user_mode_regs_store
    .i_mode_rds_exec         ( status_bits_mode_rds_oh   ),

    .o_rm                    ( reg_bank_rm               ),
    .o_rs                    ( reg_bank_rs               ),
    .o_rd                    ( reg_bank_rd               ),
    .o_rn                    ( reg_bank_rn               ),
    .o_pc                    ( pc                        ),
	
	//items for OOO: valid bits and tags
	//tie valid/tag/data/flags inputs directly to the dispatch module inputs
	.i_alu_valid	(i_alu_valid),
	.i_mult_valid	(i_mult_valid),
	.i_mem_valid	(i_mem_valid),
	.i_alu_tag		(i_alu_tag),
	.i_mult_tag		(i_mult_tag),
	.i_mem_tag		(i_mem_tag),
	.i_alu_data		(i_alu_data),
	.i_mult_data	(i_mult_data),
	.i_mem_data		(i_mem_data),
	.i_alu_flags	(i_alu_flags),
	.i_mult_flags	(i_mult_flags),
	.i_mem_flags	(i_mem_flags),
	//TODO need to handle writeback from alu, mult, and mem simultaneously!
	//TODO also, tag comparison for the writeback/valid bit updating should happen INSIDE the regfile module
	
	.o_rm_valid		(reg_bank_rm_valid),
	.o_rs_valid		(reg_bank_rs_valid),
	.o_rd_valid		(reg_bank_rd_valid),
	.o_rn_valid		(reg_bank_rn_valid),
	.o_pc_valid		(reg_bank_pc_valid),
	.o_rm_tag		(reg_bank_rm_tag),
	.o_rs_tag		(reg_bank_rs_tag),
	.o_rd_tag		(reg_bank_rd_tag),
	.o_rn_tag		(reg_bank_rn_tag),
	.o_pc_tag		(reg_bank_pc_tag),
    
    .led(led),
    .sw(sw)
);


// ================================================================================================
// NEW: Dispatch logic and structures =============================================================
// ================================================================================================

//Reservation station selection for the incoming instruction
enum logic [1:0] {STATION_ALU, STATION_MULT, STATION_MEM} reservation_sel;
assign reservation_sel = 	i_multiply_function[0] 	? 	STATION_MULT:
							i_decode_daccess		? 	STATION_MEM	:
														STATION_ALU;


//We skip the reorder buffer stuff for now. We still want to track instructions from least-recent to most-recent, though, so we can "prefer" in-order dispatch and retirement if multiple instructions are ready to do one of these in a single cycle.


//Track what reservation stations are currently occupied, plus which is most-recent and which is next to be filled
logic [15:0] alu_reserved;
//logic [3:0] alu_station_oldest;
logic [4:0] alu_station_next;
logic alu_station_full;
assign alu_station_full = alu_station_next[4];

logic [15:0] mult_reserved;
//logic [3:0] mult_station_oldest;
logic [4:0] mult_station_next;
logic mult_station_full;
assign mult_station_full = mult_station_next[4];

logic [15:0] mem_reserved;
//logic [3:0] mem_station_oldest;
logic [4:0] mem_station_next;
logic mem_station_full;
assign mem_station_full = mem_station_next[4];


//The reservation stations themselves
alu_station_entry [15:0] alu_station;
mult_station_entry [15:0] mult_station;
mem_station_entry [15:0] mem_station;


//The pending new reservation station entries
alu_station_entry alu_new_entry;
mult_station_entry mult_new_entry;
mem_station_entry mem_new_entry;


//Control signals
`define ALU_MATCH_IDX 0
`define MULT_MATCH_IDX 1
`define MEM_MATCH_IDX 2
logic 				alu_full,
					mult_full,
					mem_full; //combinationally set if there are no available entries for new instructions
/*logic [5:0] 		i_alu_tag,
					i_mult_tag,
					i_mem_tag; //TODO these should be *actual* inputs and not just logic pieces here*/
logic [15:0] 		alu_ready,
					mult_ready,
					mem_ready;
logic [15:0]		alu_shift_into, //define whether or not we need to shift up a given reservation station entry based on what we've just dispatched
					mult_shift_into,
					mem_shift_into;
logic [15:0][2:0]	alu_rn_tag_match, //of each entry, bit 0 says it matches the incoming ALU result, bit 1=mult, bit 2=mem
					alu_rs_tag_match,
					alu_rm_tag_match,
					alu_ccr_tag_match;
logic [15:0][2:0] 	mult_op1_tag_match,
					mult_op2_tag_match,
					mult_op3_tag_match,
					mult_ccr_tag_match;
logic [15:0][2:0]	mem_op1_tag_match,
					mem_op2_tag_match,
					mem_op3_tag_match,
					mem_ccr_tag_match;
logic [4:0]			alu_next_dispatch_idx,
					alu_next_dispatch_idx_minus1,
					mult_next_dispatch_idx,
					mult_next_dispatch_idx_minus1,
					mem_next_dispatch_idx,
					mem_next_dispatch_idx_minus1;
assign alu_next_dispatch_idx_minus1 = alu_next_dispatch_idx-1;
assign mult_next_dispatch_idx_minus1 = mult_next_dispatch_idx-1;
assign mem_next_dispatch_idx_minus1 = mem_next_dispatch_idx-1;


//Combinational control signal logic
always_comb begin
	//Determine if ALU reservation station tags match incoming data
	for (int i=0; i<16; i+=1) begin
		alu_rn_tag_match[i][ALU_MATCH_IDX] = (alu_station[i].rn_tag == i_alu_tag) & !alu_station[i].rn_valid;
		alu_rn_tag_match[i][MULT_MATCH_IDX] = (alu_station[i].rn_tag == i_mult_tag) & !alu_station[i].rn_valid;
		alu_rn_tag_match[i][MEM_MATCH_IDX] = (alu_station[i].rn_tag == i_mem_tag) & !alu_station[i].rn_valid;
		
		alu_op2_tag_match[i][ALU_MATCH_IDX] = (alu_station[i].op2_tag == i_alu_tag) & !alu_station[i].op2_valid;
		alu_op2_tag_match[i][MULT_MATCH_IDX] = (alu_station[i].op2_tag == i_mult_tag) & !alu_station[i].op2_valid;
		alu_op2_tag_match[i][MEM_MATCH_IDX] = (alu_station[i].op2_tag == i_mem_tag) & !alu_station[i].op2_valid;
		
		alu_op3_tag_match[i][ALU_MATCH_IDX] = (alu_station[i].op3_tag == i_alu_tag) & !alu_station[i].op3_valid;
		alu_op3_tag_match[i][MULT_MATCH_IDX] = (alu_station[i].op3_tag == i_mult_tag) & !alu_station[i].op3_valid;
		alu_op3_tag_match[i][MEM_MATCH_IDX] = (alu_station[i].op3_tag == i_mem_tag) & !alu_station[i].op3_valid;
		
		alu_ccr_tag_match[i][ALU_MATCH_IDX] = (alu_station[i].ccr_tag == i_alu_tag) & !alu_station[i].ccr_valid;
		alu_ccr_tag_match[i][MULT_MATCH_IDX] = (alu_station[i].ccr_tag == i_mult_tag) & !alu_station[i].ccr_valid;
		alu_ccr_tag_match[i][MEM_MATCH_IDX] = (alu_station[i].ccr_tag == i_mem_tag) & !alu_station[i].ccr_valid;
	end
	
	//Determine if MULT reservation station tags match incoming data
	for (int i=0; i<16; i+=1) begin
		mult_op1_tag_match[i][ALU_MATCH_IDX] = (mult_station[i].op1_tag == i_alu_tag) & !mult_station[i].op1_valid;
		mult_op1_tag_match[i][MULT_MATCH_IDX] = (mult_station[i].op1_tag == i_mult_tag) & !mult_station[i].op1_valid;
		mult_op1_tag_match[i][MEM_MATCH_IDX] = (mult_station[i].op1_tag == i_mem_tag) & !mult_station[i].op1_valid;
		
		mult_op2_tag_match[i][ALU_MATCH_IDX] = (mult_station[i].op2_tag == i_alu_tag) & !mult_station[i].op2_valid;
		mult_op2_tag_match[i][MULT_MATCH_IDX] = (mult_station[i].op2_tag == i_mult_tag) & !mult_station[i].op2_valid;
		mult_op2_tag_match[i][MEM_MATCH_IDX] = (mult_station[i].op2_tag == i_mem_tag) & !mult_station[i].op2_valid;
		
		mult_op3_tag_match[i][ALU_MATCH_IDX] = (mult_station[i].op3_tag == i_alu_tag) & !mult_station[i].op3_valid;
		mult_op3_tag_match[i][MULT_MATCH_IDX] = (mult_station[i].op3_tag == i_mult_tag) & !mult_station[i].op3_valid;
		mult_op3_tag_match[i][MEM_MATCH_IDX] = (mult_station[i].op3_tag == i_mem_tag) & !mult_station[i].op3_valid;
		
		mult_ccr_tag_match[i][ALU_MATCH_IDX] = (mult_station[i].ccr_tag == i_alu_tag) & !mult_station[i].ccr_valid;
		mult_ccr_tag_match[i][MULT_MATCH_IDX] = (mult_station[i].ccr_tag == i_mult_tag) & !mult_station[i].ccr_valid;
		mult_ccr_tag_match[i][MEM_MATCH_IDX] = (mult_station[i].ccr_tag == i_mem_tag) & !mult_station[i].ccr_valid;
	end
	
	//Determine if MEM reservation station tags match incoming data
	for (int i=0; i<16; i+=1) begin
		mem_op1_tag_match[i][ALU_MATCH_IDX] = (mem_station[i].op1_tag == i_alu_tag) & !mem_station[i].op1_valid;
		mem_op1_tag_match[i][MULT_MATCH_IDX] = (mem_station[i].op1_tag == i_mult_tag) & !mem_station[i].op1_valid;
		mem_op1_tag_match[i][MEM_MATCH_IDX] = (mem_station[i].op1_tag == i_mem_tag) & !mem_station[i].op1_valid;
		
		mem_op2_tag_match[i][ALU_MATCH_IDX] = (mem_station[i].op2_tag == i_alu_tag) & !mem_station[i].op2_valid;
		mem_op2_tag_match[i][MULT_MATCH_IDX] = (mem_station[i].op2_tag == i_mult_tag) & !mem_station[i].op2_valid;
		mem_op2_tag_match[i][MEM_MATCH_IDX] = (mem_station[i].op2_tag == i_mem_tag) & !mem_station[i].op2_valid;
		
		mem_op3_tag_match[i][ALU_MATCH_IDX] = (mem_station[i].op3_tag == i_alu_tag) & !mem_station[i].op3_valid;
		mem_op3_tag_match[i][MULT_MATCH_IDX] = (mem_station[i].op3_tag == i_mult_tag) & !mem_station[i].op3_valid;
		mem_op3_tag_match[i][MEM_MATCH_IDX] = (mem_station[i].op3_tag == i_mem_tag) & !mem_station[i].op3_valid;
		
		mem_ccr_tag_match[i][ALU_MATCH_IDX] = (mem_station[i].ccr_tag == i_alu_tag) & !mem_station[i].ccr_valid;
		mem_ccr_tag_match[i][MULT_MATCH_IDX] = (mem_station[i].ccr_tag == i_mult_tag) & !mem_station[i].ccr_valid;
		mem_ccr_tag_match[i][MEM_MATCH_IDX] = (mem_station[i].ccr_tag == i_mem_tag) & !mem_station[i].ccr_valid;
	end
	
	//Determine which instructions are ready for dispatch this cycle
	for (int i=0; i<16; i+=1) begin
		alu_ready[i] = (alu_station[i].rn_valid | (alu_rn_tag_match[i] != 3'b000))
						& (alu_station[i].op2_valid | (alu_op2_tag_match[i] != 3'b000))
						& (alu_station[i].op3_valid | (alu_op3_tag_match[i] != 3'b000))
						& (alu_station[i].ccr_valid | (alu_ccr_tag_match[i] != 3'b000));
		mult_ready[i] = (mult_station[i].op1_valid | (mult_op1_tag_match[i] != 3'b000))
						& (mult_station[i].op2_valid | (mult_op2_tag_match[i] != 3'b000))
						& (mult_station[i].op3_valid | (mult_op3_tag_match[i] != 3'b000))
						& (mult_station[i].ccr_valid | (mult_ccr_tag_match[i] != 3'b000));
		mem_ready[i] = (mem_station[i].op1_valid | (mem_op1_tag_match[i] != 3'b000))
						& (mem_station[i].op2_valid | (mem_op2_tag_match[i] != 3'b000))
						& (mem_station[i].op3_valid | (mem_op3_tag_match[i] != 3'b000))
						& (mem_station[i].ccr_valid | (mem_ccr_tag_match[i] != 3'b000));
	end
	
	//Determine index+1 of the next reservation station entry to dispatch (for each station). 0 means nothing is ready.
	//ALU
	if (alu_ready[0]) alu_next_dispatch_idx = 5'd1;
	else if (alu_ready[1]) alu_next_dispatch_idx = 5'd2;
	else if (alu_ready[2]) alu_next_dispatch_idx = 5'd3;
	else if (alu_ready[3]) alu_next_dispatch_idx = 5'd4;
	else if (alu_ready[4]) alu_next_dispatch_idx = 5'd5;
	else if (alu_ready[5]) alu_next_dispatch_idx = 5'd6;
	else if (alu_ready[6]) alu_next_dispatch_idx = 5'd7;
	else if (alu_ready[7]) alu_next_dispatch_idx = 5'd8;
	else if (alu_ready[8]) alu_next_dispatch_idx = 5'd9;
	else if (alu_ready[9]) alu_next_dispatch_idx = 5'd10;
	else if (alu_ready[10]) alu_next_dispatch_idx = 5'd11;
	else if (alu_ready[11]) alu_next_dispatch_idx = 5'd12;
	else if (alu_ready[12]) alu_next_dispatch_idx = 5'd13;
	else if (alu_ready[13]) alu_next_dispatch_idx = 5'd14;
	else if (alu_ready[14]) alu_next_dispatch_idx = 5'd15;
	else if (alu_ready[15]) alu_next_dispatch_idx = 5'd16;
	else alu_next_dispatch_idx = 5'd0;
	for (int i=0; i<16; i+=1) begin
		alu_shift_into[i] = alu_next_dispatch_idx_minus1 >= i;
	end
	
	//MULT
	if (mult_ready[0]) mult_next_dispatch_idx = 5'd1;
	else if (mult_ready[1]) mult_next_dispatch_idx = 5'd2;
	else if (mult_ready[2]) mult_next_dispatch_idx = 5'd3;
	else if (mult_ready[3]) mult_next_dispatch_idx = 5'd4;
	else if (mult_ready[4]) mult_next_dispatch_idx = 5'd5;
	else if (mult_ready[5]) mult_next_dispatch_idx = 5'd6;
	else if (mult_ready[6]) mult_next_dispatch_idx = 5'd7;
	else if (mult_ready[7]) mult_next_dispatch_idx = 5'd8;
	else if (mult_ready[8]) mult_next_dispatch_idx = 5'd9;
	else if (mult_ready[9]) mult_next_dispatch_idx = 5'd10;
	else if (mult_ready[10]) mult_next_dispatch_idx = 5'd11;
	else if (mult_ready[11]) mult_next_dispatch_idx = 5'd12;
	else if (mult_ready[12]) mult_next_dispatch_idx = 5'd13;
	else if (mult_ready[13]) mult_next_dispatch_idx = 5'd14;
	else if (mult_ready[14]) mult_next_dispatch_idx = 5'd15;
	else if (mult_ready[15]) mult_next_dispatch_idx = 5'd16;
	else mult_next_dispatch_idx = 5'd0;
	for (int i=0; i<16; i+=1) begin
		mult_shift_into[i] = mult_next_dispatch_idx_minus1 >= i;
	end
	
	//MEM
	if (mem_ready[0]) mem_next_dispatch_idx = 5'd1;
	else if (mem_ready[1]) mem_next_dispatch_idx = 5'd2;
	else if (mem_ready[2]) mem_next_dispatch_idx = 5'd3;
	else if (mem_ready[3]) mem_next_dispatch_idx = 5'd4;
	else if (mem_ready[4]) mem_next_dispatch_idx = 5'd5;
	else if (mem_ready[5]) mem_next_dispatch_idx = 5'd6;
	else if (mem_ready[6]) mem_next_dispatch_idx = 5'd7;
	else if (mem_ready[7]) mem_next_dispatch_idx = 5'd8;
	else if (mem_ready[8]) mem_next_dispatch_idx = 5'd9;
	else if (mem_ready[9]) mem_next_dispatch_idx = 5'd10;
	else if (mem_ready[10]) mem_next_dispatch_idx = 5'd11;
	else if (mem_ready[11]) mem_next_dispatch_idx = 5'd12;
	else if (mem_ready[12]) mem_next_dispatch_idx = 5'd13;
	else if (mem_ready[13]) mem_next_dispatch_idx = 5'd14;
	else if (mem_ready[14]) mem_next_dispatch_idx = 5'd15;
	else if (mem_ready[15]) mem_next_dispatch_idx = 5'd16;
	else mem_next_dispatch_idx = 5'd0;
	for (int i=0; i<16; i+=1) begin
		mem_shift_into[i] = mem_next_dispatch_idx_minus1 >= i;
	end
	
	//Determing where an incoming instruction will go is handled in the always_ff, below
end

//set up new reservation station entries based on incoming info from Decode
always_comb begin
	//defaults
	alu_new_entry = 0;
	mult_new_entry = 0;
	mem_new_entry = 0;
	
	case (reservation_sel)
		STATION_ALU: begin
			//TODO replace with rn
			//TODO also check if the tag is on the tag bus THIS cycle and insert info immediately if so. Note that per the current design, all instructions will have to spend at least one cycle in the reservation station before executing; this should be fixable in the future.
			alu_new_entry.rn_valid = 0;
			alu_new_entry.rn_tag = 0;
			alu_new_entry.rn = 0;
			
			//TODO replace with rm
			//TODO also check if the tag is on the tag bus THIS cycle and insert info immediately if so
			alu_new_entry.op2_valid = 0;
			alu_new_entry.op2_tag = 0;
			alu_new_entry.op2 = 0;
			
			//TODO replace with rs
			//TODO also check if the tag is on the tag bus THIS cycle and insert info immediately if so
			alu_new_entry.op3_valid = 0;
			alu_new_entry.op3_tag = 0;
			alu_new_entry.op3 = 0;
			
			//TODO replace with actual CCR stuff
			//TODO also check if the tag is on the tag bus THIS cycle and insert info immediately if so
			alu_new_entry.ccr_valid = 0;
			alu_new_entry.ccr_tag = 0;
			alu_new_entry.ccr = 0;
			
			alu_new_entry.imm32 = i_imm32;
			alu_new_entry.imm_shift_amount = i_imm_shift_amount;
			alu_new_entry.shift_imm_zero = i_shift_imm_zero;
			alu_new_entry.rm_sel = i_rm_sel;
			alu_new_entry.rs_sel = i_rs_sel;
			alu_new_entry.rn_sel = i_rn_sel;
			alu_new_entry.exec_load_rd = i_exec_load_rd;
			alu_new_entry.barrel_shift_amount_sel = i_barrel_shift_amount_sel;
			alu_new_entry.barrel_shift_data_sel = i_barrel_shift_data_sel;
			alu_new_entry.barrel_shift_function = i_barrel_shift_function;
			alu_new_entry.alu_function = i_alu_function;
			alu_new_entry.pc_wen = i_pc_wen;
			alu_new_entry.reg_bank_wen = i_reg_bank_wen;
			alu_new_entry.status_bits_flags_wen = i_status_bits_flags_wen;
			alu_new_entry.status_bits_mode_wen = i_status_bits_mode_wen;
			alu_new_entry.irq_mask_wen = i_irq_mask_wen;
			alu_new_entry.firq_mask_wen = i_firq_mask_wen;
		end
		STATION_MULT: begin
			//TODO replace with rn
			//TODO also check if the tag is on the tag bus THIS cycle and insert info immediately if so
			mult_new_entry.op1_valid = 0;
			mult_new_entry.op1_tag = 0;
			mult_new_entry.op1 = 0;
			
			//TODO replace with rm
			//TODO also check if the tag is on the tag bus THIS cycle and insert info immediately if so
			mult_new_entry.op2_valid = 0;
			mult_new_entry.op2_tag = 0;
			mult_new_entry.op2 = 0;
			
			//TODO replace with rs
			//TODO also check if the tag is on the tag bus THIS cycle and insert info immediately if so
			mult_new_entry.op3_valid = 0;
			mult_new_entry.op3_tag = 0;
			mult_new_entry.op3 = 0;
			
			//TODO replace with actual CCR stuff
			//TODO also check if the tag is on the tag bus THIS cycle and insert info immediately if so
			mult_new_entry.ccr_valid = 0;
			mult_new_entry.ccr_tag = 0;
			mult_new_entry.ccr = 0;
			
			mult_new_entry.rm_sel = i_rm_sel;
			mult_new_entry.rs_sel = i_rs_sel;
			mult_new_entry.rn_sel = i_rn_sel;
			mult_new_entry.exec_load_rd = i_exec_load_rd;
			mult_new_entry.multiply_function = i_multiply_function;
			mult_new_entry.use_carry_in = i_use_carry_in;
			mult_new_entry.reg_bank_wen = i_reg_bank_wen;
			mult_new_entry.status_bits_flags_wen = i_status_bits_flags_wen;
			mult_new_entry.status_bits_mode_wen = i_status_bits_mode_wen;
			mult_new_entry.irq_mask_wen = i_irq_mask_wen;
			mult_new_entry.firq_mask_wen = i_firq_mask_wen;
		end
		STATION_MEM: begin
			//TODO replace with rn
			//TODO also check if the tag is on the tag bus THIS cycle and insert info immediately if so
			mem_new_entry.op1_valid = 0;
			mem_new_entry.op1_tag = 0;
			mem_new_entry.op1 = 0;
			
			//TODO replace with rm
			//TODO also check if the tag is on the tag bus THIS cycle and insert info immediately if so
			mem_new_entry.op2_valid = 0;
			mem_new_entry.op2_tag = 0;
			mem_new_entry.op2 = 0;
			
			//TODO replace with rs
			//TODO also check if the tag is on the tag bus THIS cycle and insert info immediately if so
			mem_new_entry.op3_valid = 0;
			mem_new_entry.op3_tag = 0;
			mem_new_entry.op3 = 0;
			
			//TODO replace with actual CCR stuff
			//TODO also check if the tag is on the tag bus THIS cycle and insert info immediately if so
			mem_new_entry.ccr_valid = 0;
			mem_new_entry.ccr_tag = 0;
			mem_new_entry.ccr = 0;
			
			mem_new_entry.exclusive = i_exclusive;
			mem_new_entry.decode_iaccess = i_decode_iaccess;
			mem_new_entry.decode_daccess = i_decode_daccess;
			mem_new_entry.decode_load_rd = i_decode_load_rd;
			mult_new_entry.rm_sel = i_rm_sel;
			mult_new_entry.rs_sel = i_rs_sel;
			mult_new_entry.rn_sel = i_rn_sel;
			mult_new_entry.exec_load_rd = i_exec_load_rd;
			mult_new_entry.reg_bank_wen = i_reg_bank_wen;
			mult_new_entry.status_bits_flags_wen = i_status_bits_flags_wen;
			mult_new_entry.status_bits_mode_wen = i_status_bits_mode_wen;
			mult_new_entry.irq_mask_wen = i_irq_mask_wen;
			mult_new_entry.firq_mask_wen = i_firq_mask_wen;
			mem_new_entry.iaddress_sel = i_iaddress_sel;
			mem_new_entry.daddress_sel = i_daddress_sel;
			mem_new_entry.byte_enable_sel = i_byte_enable_sel;
		end
		DEFAULT: begin
			alu_new_entry = 0;
			mult_new_entry = 0;
			mem_new_entry = 0;
		end
	endcase
end


//ALU reservation station state machine
//TODO also hook up reset!!!
//TODO also need to add some case logic to handle interrupts
//TODO logic to handle core stall cases
always_ff @(posedge i_clk) begin
	if (i_clk) begin
		
		//Sniff and update tags in reservation station
		//check alu_opX_tag_match[i][ALU/MEM/MULT_MATCH_IDX] bit and grab data from the appropriate tag bus
		for (int i=0; i<16; i+=1) begin
			if (alu_rn_tag_match[i][ALU_MATCH_IDX]) begin
				alu_station[i].rn <= i_alu_data; //TODO note that this will mean a 1-cycle delay between writeback and dispatch; could be mitigated with add'l forwarding path
//==============Further URGENT TODO: it'll actually be "incorrect" as-is due to next-to-be-dispatched combinational logic; need to fix before the design will work at all!!!==============
				alu_station[i].rn_valid <= 1;
			end
			else if (alu_rn_tag_match[i][MULT_MATCH_IDX]) begin
				alu_station[i].rn <= i_mult_data;
				alu_station[i].rn_valid <= 1;
			end
			else if (alu_rn_tag_match[i][MEM_MATCH_IDX]) begin
				alu_station[i].rn <= i_mem_data;
				alu_station[i].rn_valid <= 1;
			end
			
			if (alu_op2_tag_match[i][ALU_MATCH_IDX]) begin
				alu_station[i].op2 <= i_alu_data; //TODO note that this will mean a 1-cycle delay between writeback and dispatch; could be mitigated with add'l forwarding path
				alu_station[i].op2_valid <= 1;
			end
			else if (alu_op2_tag_match[i][MULT_MATCH_IDX]) begin
				alu_station[i].op2 <= i_mult_data;
				alu_station[i].op2_valid <= 1;
			end
			else if (alu_op2_tag_match[i][MEM_MATCH_IDX]) begin
				alu_station[i].op2 <= i_mem_data;
				alu_station[i].op2_valid <= 1;
			end
			
			if (alu_op3_tag_match[i][ALU_MATCH_IDX]) begin
				alu_station[i].op3 <= i_alu_data; //TODO note that this will mean a 1-cycle delay between writeback and dispatch; could be mitigated with add'l forwarding path
				alu_station[i].op3_valid <= 1;
			end
			else if (alu_op3_tag_match[i][MULT_MATCH_IDX]) begin
				alu_station[i].op3 <= i_mult_data;
				alu_station[i].op3_valid <= 1;
			end
			else if (alu_op3_tag_match[i][MEM_MATCH_IDX]) begin
				alu_station[i].op3 <= i_mem_data;
				alu_station[i].op3_valid <= 1;
			end
			
			if (alu_ccr_tag_match[i][ALU_MATCH_IDX]) begin
				alu_station[i].ccr <= i_alu_data; //TODO note that this will mean a 1-cycle delay between writeback and dispatch; could be mitigated with add'l forwarding path
				alu_station[i].ccr_valid <= 1;
			end
			else if (alu_ccr_tag_match[i][MULT_MATCH_IDX]) begin
				alu_station[i].ccr <= i_mult_data;
				alu_station[i].ccr_valid <= 1;
			end
			else if (alu_ccr_tag_match[i][MEM_MATCH_IDX]) begin
				alu_station[i].ccr <= i_mem_data;
				alu_station[i].ccr_valid <= 1;
			end
		end
		
		//Dispatch from reservation station
		//based on alu_next_dispatch_idx-1 assuming alu_next_dispatch_idx != 0
		if (alu_next_dispatch_idx != 0) begin
			o_alu_stage_command <= alu_station[alu_next_dispatch_idx_minus1];
			o_alu_stage_command_valid <= 1;
			
			//Shift up the remaining commands. This is safe as far as tag assignment is concerned because the tag is really the index of a virtual reorder buffer [TODO], assigned as instructions are piped into this stage from Decode.
			for (int i=0; i<16; i+=1) begin
				if (alu_shift_into[i] && (i != 15)) alu_station[i] <= alu_station[i+1];
				else if (alu_shift_into[i] && (i==15)) alu_station[i] <= 0;
			end
		end
		else o_alu_stage_command_valid <= 0;
		
		//Insert new instruction data/control into reservation station
		/*
		
//Reservation station selection for the incoming instruction
enum logic [1:0] {STATION_ALU, STATION_MULT, STATION_MEM} reservation_sel;
assign reservation_sel = 	i_multiply_function[0] 	? 	STATION_MULT:
							i_decode_daccess		? 	STATION_MEM	:
														STATION_ALU;


//We skip the reorder buffer stuff for now. We still want to track instructions from least-recent to most-recent, though, so we can "prefer" in-order dispatch and retirement if multiple instructions are ready to do one of these in a single cycle.


//Track what reservation stations are currently occupied, plus which is most-recent and which is next to be filled
logic [15:0] alu_reserved;
//logic [3:0] alu_station_oldest;
logic [4:0] alu_station_next;
logic alu_station_full;
assign alu_station_full = alu_station_next[4];

*/
		
		//Update next-available-station-to-insert-into index
		
	end
end




// ========================================================
// Debug - non-synthesizable code
// ========================================================
//synopsys translate_off

wire    [(2*8)-1:0]    xCONDITION;
wire    [(4*8)-1:0]    xMODE;

assign  xCONDITION           = i_condition == EQ ? "EQ"  :
                               i_condition == NE ? "NE"  :
                               i_condition == CS ? "CS"  :
                               i_condition == CC ? "CC"  :
                               i_condition == MI ? "MI"  :
                               i_condition == PL ? "PL"  :
                               i_condition == VS ? "VS"  :
                               i_condition == VC ? "VC"  :
                               i_condition == HI ? "HI"  :
                               i_condition == LS ? "LS"  :
                               i_condition == GE ? "GE"  :
                               i_condition == LT ? "LT"  :
                               i_condition == GT ? "GT"  :
                               i_condition == LE ? "LE"  :
                               i_condition == AL ? "AL"  :
                                                   "NV " ;

assign  xMODE  =  status_bits_mode == SVC  ? "SVC"  :
                  status_bits_mode == IRQ  ? "IRQ"  :
                  status_bits_mode == FIRQ ? "FIRQ" :
                  status_bits_mode == USR  ? "USR"  :
                                             "XXX"  ;


//synopsys translate_on

/*assign led = (sw[1:0]==2'd0)?o_iaddress[7:0]:
            (sw[1:0]==2'd1)?o_iaddress[15:8]:
            (sw[1:0]==2'd2)?o_iaddress[23:16]:
                            o_iaddress[31:24];*/

endmodule


/*
======== Notes on branches and interrupts: ========
1) Branches must be resolved in this stage in a single cycle and without being dispatched to any station (unless there exist data dependencies).
2) Not just branches, but ALL program counter changes must happen in this stage and in a single cycle (unless there exist data dependencies).
3) Interrupts are simply signaled at the input and cause a jump to a specific address; no special handling is needed other than pre-existing PC modification logic.
*/