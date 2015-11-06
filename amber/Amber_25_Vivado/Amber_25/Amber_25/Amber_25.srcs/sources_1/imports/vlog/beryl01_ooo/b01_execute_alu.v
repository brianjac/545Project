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


module a25_execute (

input                       i_clk,
input                       i_core_stall,               // stall all stages of the Amber core at the same time
input                       i_mem_stall,                // data memory access stalls
//output                      o_exec_stall,               // stall the core pipeline //TODO not necessary for OOO since this stage should never ever cause a stall

output reg  [8:0]           o_exec_load_rd = 'd0,       // The destination register for a load instruction
output      [31:0]          o_status_bits,              // Full PC will all status bits, but PC part zero'ed out

// --------------------------------------------------
// Control signals from Dispatch stage
// --------------------------------------------------
input      [1:0]            i_status_bits_mode,
input                       i_status_bits_irq_mask,
input                       i_status_bits_firq_mask,
input      [31:0]           i_imm32,
input      [4:0]            i_imm_shift_amount,
input                       i_shift_imm_zero,
//input      [3:0]            i_condition, //TODO need to evaluate and handle after dispatch-time(?)

input      [3:0]            i_rm_sel,
input      [3:0]            i_rs_sel,
input      [3:0]            i_rn_sel,
input      [1:0]            i_barrel_shift_amount_sel,
input      [1:0]            i_barrel_shift_data_sel,
input      [1:0]            i_barrel_shift_function,
input      [8:0]            i_alu_function,
//input      [2:0]            i_interrupt_vector_sel, //TODO determine if handling here or separately
//input                       i_use_carry_in,         // e.g. add with carry instruction //TODO determine if needed

input      [14:0]           i_reg_bank_wen,
input                       i_status_bits_flags_wen,
input                       i_status_bits_mode_wen,
input                       i_status_bits_irq_mask_wen,
input                       i_status_bits_firq_mask_wen,
);

`include "a25_localparams.vh"
`include "a25_functions.vh"

// ========================================================
// Internal signals
// ========================================================
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

wire                ldm_flags; //TODO need to handle LDM instruction separately? TODO FIGURE THIS CRAP OUT
wire                ldm_status_bits;

wire                carry_in;


// ========================================================
// Status Bits in PC register
// ========================================================
wire [1:0] status_bits_mode_out;
assign status_bits_mode_out = (i_status_bits_mode_wen && i_status_bits_sel == 3'd1 && !ldm_status_bits) ?
                                    alu_out[1:0] : status_bits_mode ;

assign o_status_bits = {   status_bits_flags,           // 31:28
                           status_bits_irq_mask,        // 7
                           status_bits_firq_mask,       // 6
                           24'd0,
                           status_bits_mode_out };      // 1:0 = mode


// ========================================================
// Status Bits Select
// ========================================================
assign ldm_flags                 = i_wb_read_data_valid & ~i_mem_stall & i_wb_load_rd[8];
assign ldm_status_bits           = i_wb_read_data_valid & ~i_mem_stall & i_wb_load_rd[7];


assign status_bits_flags_nxt     = ldm_flags                 ? read_data_filtered[31:28]           :
                                   i_status_bits_sel == 3'd0 ? alu_flags                           :
                                   i_status_bits_sel == 3'd1 ? alu_out          [31:28]            :
                                   i_status_bits_sel == 3'd3 ? i_copro_read_data[31:28]            :
                                   // regops that do not change the overflow flag
                                   i_status_bits_sel == 3'd5 ? { alu_flags[3:1], status_bits_flags[0] } :
                                                               4'b1111 ;

assign status_bits_mode_nxt      = ldm_status_bits           ? read_data_filtered [1:0] :
                                   i_status_bits_sel == 3'd0 ? i_status_bits_mode       :
                                   i_status_bits_sel == 3'd5 ? i_status_bits_mode       :
                                   /*i_status_bits_sel == 3'd1 ?*/ alu_out            [1:0] ;


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
// Conditional Execution
// ========================================================
assign execute = conditional_execute ( i_condition, status_bits_flags );

// only update register bank if current instruction executes
assign reg_bank_wen = {{15{execute}} & i_reg_bank_wen};


// ========================================================
// Write Enable
// ========================================================
// This must be de-asserted when execute is fault
assign write_enable_nxt = execute && i_write_data_wen;


//TODO note: these probably aren't even needed to pass into Execute stage since we've already grabbed the specific operands (with the possible exception of rd)
assign rn = i_rn_sel;
assign rm = i_rm_sel;
assign rs = i_rs_sel;
assign rd = i_rd_sel;


always@( posedge i_clk )
    if ( i_wb_read_data_valid )
        begin
        read_data_filtered_r <= read_data_filtered;
        load_rd_r            <= {i_wb_load_rd[6:5], i_wb_load_rd[3:0]};
        end

assign read_data_filtered_c = i_wb_read_data_valid ? read_data_filtered : read_data_filtered_r;

// the register number and the mode
assign load_rd_c            = i_wb_read_data_valid ? {i_wb_load_rd[6:5], i_wb_load_rd[3:0]}  : load_rd_r;


// ========================================================
// Register Update
// ========================================================
assign o_exec_stall                    = barrel_shift_stall;

assign exec_load_rd_update             = !i_core_stall && execute;
assign write_enable_update             = !i_core_stall;
assign write_data_update               = !i_core_stall && execute && i_write_data_wen;

assign status_bits_flags_update        = ldm_flags       || (!i_core_stall && execute && i_status_bits_flags_wen);
assign status_bits_mode_update         = ldm_status_bits || (!i_core_stall && execute && i_status_bits_mode_wen);
assign status_bits_mode_rds_oh_update  = !i_core_stall;
assign status_bits_irq_mask_update     = ldm_status_bits || (!i_core_stall && execute && i_status_bits_irq_mask_wen);
assign status_bits_firq_mask_update    = ldm_status_bits || (!i_core_stall && execute && i_status_bits_firq_mask_wen);


always @( posedge i_clk )
    begin
    o_exec_load_rd          <= exec_load_rd_update            ? exec_load_rd_nxt             : o_exec_load_rd;
    o_write_enable          <= write_enable_update            ? write_enable_nxt             : o_write_enable;
    o_write_data            <= write_data_update              ? write_data_nxt               : o_write_data;
    
    status_bits_flags       <= status_bits_flags_update       ? status_bits_flags_nxt        : status_bits_flags;
    status_bits_mode        <= status_bits_mode_update        ? status_bits_mode_nxt         : status_bits_mode;
    status_bits_mode_rds_oh <= status_bits_mode_rds_oh_update ? status_bits_mode_rds_oh_nxt  : status_bits_mode_rds_oh;
    status_bits_irq_mask    <= status_bits_irq_mask_update    ? status_bits_irq_mask_nxt     : status_bits_irq_mask;
    status_bits_firq_mask   <= status_bits_firq_mask_update   ? status_bits_firq_mask_nxt    : status_bits_firq_mask;
    end


// ========================================================
// Instantiate Barrel Shift
// ========================================================
assign carry_in = i_use_carry_in ? status_bits_flags[1] : 1'd0;

a25_barrel_shift u_barrel_shift  (
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
                                  (i_imm_shift_amount[4:1] == 0 ? status_bits_flags[1] : i_imm32[31]) :
                                   barrel_shift_carry;

a25_alu u_alu (
    .i_a_in                 ( rn                      ),
    .i_b_in                 ( barrel_shift_out        ),
    .i_barrel_shift_carry   ( barrel_shift_carry_alu  ),
    .i_status_bits_carry    ( status_bits_flags[1]    ),
    .i_function             ( i_alu_function          ),

    .o_out                  ( alu_out                 ),
    .o_flags                ( alu_flags               ));


endmodule


