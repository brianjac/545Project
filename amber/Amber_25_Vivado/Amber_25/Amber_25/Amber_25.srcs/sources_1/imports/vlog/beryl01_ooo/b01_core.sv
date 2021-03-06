//////////////////////////////////////////////////////////////////
//                                                              //
//  Beryl 01 Core top-Level module                              //
//                                                              //
//  Adapted from the Amber project                              //
//  http://www.opencores.org/project,amber                      //
//                                                              //
//  Description                                                 //
//  Instantiates the core                                       //
//                                                              //
//////////////////////////////////////////////////////////////////
//                                                              //
// Copyright (C) 2015 Authors and OPENCORES.ORG                 //
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


module b01_core
(
input                       i_clk,
input						i_rst,				//TODO hook up

input                       i_irq,              // Interrupt request, active high
input                       i_firq,             // Fast Interrupt request, active high

input                       i_system_rdy,       // Amber is stalled when this is low

// Wishbone Master I/F
output      [31:0]          o_wb_adr,
output      [15:0]          o_wb_sel,
output                      o_wb_we,
input       [127:0]         i_wb_dat,
output      [127:0]         o_wb_dat,
output                      o_wb_cyc,
output                      o_wb_stb,
input                       i_wb_ack,
input                       i_wb_err,

output [7:0] led,
input [7:0] sw

);

wire      [31:0]          execute_iaddress;
wire                      execute_iaddress_valid;
wire      [31:0]          execute_iaddress_nxt;  // un-registered version of execute_address
                                                 // to the instruction cache rams
wire      [31:0]          write_data;
wire                      write_enable;
wire      [31:0]          fetch_instruction;
wire                      decode_exclusive;
wire                      decode_iaccess;
wire                      decode_daccess;
wire      [3:0]           byte_enable;
wire                      exclusive;            // swap access
wire                      cache_enable;         // Enabel the cache
wire                      cache_flush;          // Flush the cache
wire      [31:0]          cacheable_area;

wire                      fetch_stall; 
wire                      mem_stall; 
wire                      exec_stall; 
wire                      core_stall; 

wire     [1:0]            status_bits_mode;               
wire                      status_bits_irq_mask;           
wire                      status_bits_firq_mask;           
wire                      status_bits_flags_wen;          
wire                      status_bits_mode_wen;           
wire                      status_bits_irq_mask_wen;       
wire                      status_bits_firq_mask_wen;       
wire     [31:0]           execute_status_bits;
                 
wire     [31:0]           imm32;                   
wire     [4:0]            imm_shift_amount; 
wire                      shift_imm_zero;      
wire     [3:0]            condition;               

wire     [3:0]            rm_sel;                  
wire     [3:0]            rs_sel;                 
wire     [7:0]            decode_load_rd;                 
wire     [8:0]            exec_load_rd;                 
wire     [3:0]            rn_sel;                  
wire     [1:0]            barrel_shift_amount_sel; 
wire     [1:0]            barrel_shift_data_sel;   
wire     [1:0]            barrel_shift_function;   
wire     [8:0]            alu_function;            
wire     [1:0]            multiply_function;       
wire     [2:0]            interrupt_vector_sel;    
wire     [3:0]            iaddress_sel;             
wire     [3:0]            daddress_sel;             
wire     [2:0]            pc_sel;                  
wire     [1:0]            byte_enable_sel;         
wire     [2:0]            status_bits_sel;                
wire     [2:0]            reg_write_sel;           
wire                      user_mode_regs_store_nxt;    
wire                      firq_not_user_mode;
wire                      use_carry_in;
wire                      write_data_wen;          
wire                      copro_write_data_wen;          
wire                      base_address_wen;        
wire                      pc_wen;                  
wire     [14:0]           reg_bank_wen;            

wire                      multiply_done;

wire                      decode_fault;
wire                      iabt_trigger;
wire                      dabt_trigger;

wire     [7:0]            decode_fault_status;
wire     [7:0]            iabt_fault_status;
wire     [7:0]            dabt_fault_status;

wire     [31:0]           decode_fault_address;
wire     [31:0]           iabt_fault_address;
wire     [31:0]           dabt_fault_address;

wire                      adex;

wire     [31:0]           mem_read_data;
wire                      mem_read_data_valid;
wire     [10:0]           mem_load_rd;  

wire     [31:0]           wb_read_data;
wire                      wb_read_data_valid;
wire     [10:0]           wb_load_rd;  
               
wire                      dcache_wb_cached_req;
wire                      dcache_wb_uncached_req;
wire                      dcache_wb_write;
wire     [15:0]           dcache_wb_byte_enable;
wire     [31:0]           dcache_wb_address;
wire     [127:0]          dcache_wb_cached_rdata;
wire     [127:0]          dcache_wb_write_data;
wire                      dcache_wb_cached_ready;
wire                      dcache_wb_uncached_ready;
wire     [31:0]           icache_wb_address;
wire                      icache_wb_req;
wire     [31:0]           icache_wb_adr; 
wire     [127:0]          icache_wb_read_data; 
wire                      icache_wb_ready;

wire                      conflict;
wire                      rn_use_read;
wire                      rm_use_read;
wire                      rs_use_read;
wire                      rd_use_read;


assign core_stall           = fetch_stall /*|| mem_stall || exec_stall*/;

wire dispatch_branch_taken;

// ======================================
//  Fetch Stage
// ======================================
a25_fetch u_fetch (
    .i_clk                              ( i_clk                             ),
    .i_rst(i_rst),
    .i_mem_stall                        ( /*mem_stall*/1'b0                         ),
    .i_exec_stall                       ( exec_stall                        ),
    .i_conflict                         ( /*conflict*/1'b0                          ),
    .i_system_rdy                       ( i_system_rdy                      ),
    .o_fetch_stall                      ( fetch_stall                       ),

    .i_iaddress                         ( {execute_iaddress[31:2], 2'd0}    ),
    .i_iaddress_valid                   ( execute_iaddress_valid            ), 
    .i_iaddress_nxt                     ( execute_iaddress_nxt              ),
    .i_branch_taken(dispatch_branch_taken),
    .o_fetch_instruction                ( fetch_instruction                 ),
    .i_cache_enable                     ( /*cache_enable*/1'b1                      ),     
    .i_cache_flush                      ( /*cache_flush*/'d0                       ), 
    .i_cacheable_area                   ( /*cacheable_area*/32'hffff_ffff                    ),

    .o_wb_req                           ( icache_wb_req                     ),
    .o_wb_address                       ( icache_wb_address                 ),
    .i_wb_read_data                     ( icache_wb_read_data               ),
    .i_wb_ready                         ( icache_wb_ready                   )
);

/*assign led = (sw[1:0]==2'd0)?fetch_instruction[7:0]:
            (sw[1:0]==2'd1)?fetch_instruction[15:8]:
            (sw[1:0]==2'd2)?fetch_instruction[23:16]:
                            fetch_instruction[31:24];*/

wire use_sr;
wire [3:0] rd_sel;
wire [1:0] decode_op_type_mem;
wire [1:0] decode_address_mode;

// ======================================
//  Decode Stage
// ======================================
b01_decode u_decode (
    .i_clk                              ( i_clk                             ),
	.i_rst								(i_rst),
    .i_core_stall                       ( core_stall                        ),                                          
        
    // Instruction fetch or data read signals
    .i_fetch_instruction                ( fetch_instruction                 ),   
    .i_adex                             ( adex                              ),                                       
    
    .i_irq                              ( i_irq                             ),                                          
    .i_firq                             ( i_firq                            ),                                          
    .i_execute_status_bits              ( execute_status_bits               ),         
    .i_branch_taken(dispatch_branch_taken),                                          
    
    .o_status_bits_mode                 ( status_bits_mode                  ),  
    .o_status_bits_irq_mask             ( status_bits_irq_mask              ),  
    .o_status_bits_firq_mask            ( status_bits_firq_mask             ),  
    .o_imm32                            ( imm32                             ),
    .o_imm_shift_amount                 ( imm_shift_amount                  ),
    .o_shift_imm_zero                   ( shift_imm_zero                    ),
    .o_condition                        ( condition                         ),
    .o_is_swap                 ( decode_exclusive                  ), 
    .o_is_memop                   ( decode_daccess                    ),
    .o_rm_sel                           ( rm_sel                            ),
    .o_rs_sel                           ( rs_sel                            ),
    .o_rn_sel                           ( rn_sel                            ),
    .o_rd_sel(rd_sel),
    .o_barrel_shift_amount_sel          ( barrel_shift_amount_sel           ),
    .o_barrel_shift_data_sel            ( barrel_shift_data_sel             ),
    .o_barrel_shift_function            ( barrel_shift_function             ),
    .o_alu_function                     ( alu_function                      ),
    .o_multiply_function                ( multiply_function                 ),
    .o_interrupt_vector_sel             ( interrupt_vector_sel              ),
    .o_iaddress_sel                     ( iaddress_sel                      ),
    .o_daddress_sel                     ( daddress_sel                      ),
    .o_pc_sel                           ( pc_sel                            ),
    .o_byte_enable_sel                  ( byte_enable_sel                   ),
    .o_status_bits_sel                  ( status_bits_sel                   ),
    .o_use_carry_in                     ( use_carry_in                      ),
    
    .o_op_type_mem(decode_op_type_mem),
    .o_address_mode(decode_address_mode),
    
    .o_write_data_wen                   ( write_data_wen                    ),
    .o_pc_wen                           ( pc_wen                            ),
    .o_reg_bank_wen                     ( reg_bank_wen                      ),
    .o_status_bits_flags_wen            ( status_bits_flags_wen             ),
    .o_status_bits_mode_wen             ( status_bits_mode_wen              ),
    .o_status_bits_irq_mask_wen         ( status_bits_irq_mask_wen          ),
    .o_status_bits_firq_mask_wen        ( status_bits_firq_mask_wen         ),
    
    .o_use_rn                      ( rn_use_read                       ),
    .o_use_rm                      ( rm_use_read                       ),
    .o_use_rs                      ( rs_use_read                       ),
    .o_use_sr(use_sr)
);


// ======================================
//  Execute Stage
// ======================================
wire instr_valid_alu;
wire [31:0] rn_alu;
wire [31:0] rs_alu;
wire [31:0] rm_alu;
wire [3:0] status_bits_flags_alu;
wire use_carry_in_alu;
wire [31:0] imm32_alu;
wire [4:0] imm_shift_amount_alu;
wire shift_imm_zero_alu;
wire [1:0] barrel_shift_amount_sel_alu;
wire [1:0] barrel_shift_data_sel_alu;
wire [1:0] barrel_shift_function_alu;
wire [8:0] alu_function_alu;
wire pc_wen_alu;
wire status_bits_flags_wen_alu;
wire [5:0] rd_tag_alu;
wire alu_valid;
wire [5:0] alu_tag;
wire [31:0] alu_data;
wire [3:0] alu_flags;

wire instr_valid_mult;
wire [31:0] rn_mult;
wire [31:0] rs_mult;
wire [31:0] rm_mult;
wire multiply_function_mult;
wire [5:0] rd_tag_mult;
wire mult_valid;
wire [5:0] mult_tag;
wire [31:0] mult_data;
wire [1:0] mult_flags;


wire mem_ready;
wire instr_valid_mem;
wire [31:0] address_mem;
wire [31:0] write_data_mem;
wire [1:0] op_type_mem;
wire [1:0] byte_enable_sel_mem;
wire [5:0] rd_tag_mem;
wire mem_valid;
wire [5:0] mem_tag;
wire [31:0] mem_data;


b01_dispatch u_dispatch (
    .i_clk                              ( i_clk                             ),
	.i_rst								(i_rst),
    .i_core_stall                       ( core_stall                        ),
    .o_exec_stall                       ( exec_stall                        ),
    
    .o_iaddress                         ( execute_iaddress                  ),
    .o_iaddress_valid                   ( execute_iaddress_valid            ),
    .o_iaddress_nxt                     ( execute_iaddress_nxt              ),
    .o_byte_enable                      ( byte_enable                       ),
    .o_write_enable                     ( write_enable                      ),
    .o_exclusive                        ( exclusive                         ),
    .o_priviledged                      (                                   ),
    
    .o_adex                             ( adex                              ),
    .o_status_bits                      ( execute_status_bits               ),
    .o_branch_taken(dispatch_branch_taken),
    
    .i_status_bits_mode                 ( status_bits_mode                  ),   
    .i_status_bits_irq_mask             ( status_bits_irq_mask              ),   
    .i_status_bits_firq_mask            ( status_bits_firq_mask             ),   
    .i_imm32                            ( imm32                             ),   
    .i_imm_shift_amount                 ( imm_shift_amount                  ),   
    .i_shift_imm_zero                   ( shift_imm_zero                    ),   
    .i_condition                        ( condition                         ),   
    .i_decode_exclusive                 ( decode_exclusive                  ),   //TODO rename later
    .i_is_memop                   ( decode_daccess                    ),
    .i_rm_sel                           ( rm_sel                            ),   
    .i_rs_sel                           ( rs_sel                            ),
    .i_rn_sel                           ( rn_sel                            ),
    .i_rd_sel(rd_sel),   
    .i_barrel_shift_amount_sel          ( barrel_shift_amount_sel           ),   
    .i_barrel_shift_data_sel            ( barrel_shift_data_sel             ),   
    .i_barrel_shift_function            ( barrel_shift_function             ),   
    .i_alu_function                     ( alu_function                      ),   
    .i_multiply_function                ( multiply_function                 ),   
    .i_interrupt_vector_sel             ( interrupt_vector_sel              ),   
    .i_iaddress_sel                     ( iaddress_sel                      ),   
    .i_daddress_sel                     ( daddress_sel                      ),   
    .i_pc_sel                           ( pc_sel                            ),   
    .i_byte_enable_sel                  ( byte_enable_sel                   ),   
    .i_status_bits_sel                  ( status_bits_sel                   ),   
    .i_reg_write_sel                    ( reg_write_sel                     ),   
    .i_use_carry_in                     ( use_carry_in                      ),
    
    .i_op_type_mem(decode_op_type_mem),
    .i_address_mode(decode_address_mode),
    
    .i_write_data_wen                   ( write_data_wen                    ),   
    .i_pc_wen                           ( pc_wen                            ),   
    .i_reg_bank_wen                     ( reg_bank_wen                      ),   
    .i_status_bits_flags_wen            ( status_bits_flags_wen             ),   
    .i_status_bits_mode_wen             ( status_bits_mode_wen              ),   
    .i_status_bits_irq_mask_wen         ( status_bits_irq_mask_wen          ),   
    .i_status_bits_firq_mask_wen        ( status_bits_firq_mask_wen         ),   
    .i_use_rn                      ( rn_use_read                       ),
    .i_use_rm                      ( rm_use_read                       ),
    .i_use_rs                      ( rs_use_read                       ),
    .i_use_sr(use_sr),
    
	.o_instr_valid_alu(instr_valid_alu),
	.o_rn_alu(rn_alu),
	.o_rs_alu(rs_alu),
	.o_rm_alu(rm_alu),
	.o_status_bits_flags_alu(status_bits_flags_alu),
	.o_use_carry_in_alu(use_carry_in_alu),
	.o_imm32_alu(imm32_alu),
	.o_imm_shift_amount_alu(imm_shift_amount_alu),
	.o_shift_imm_zero_alu(shift_imm_zero_alu),
	.o_barrel_shift_amount_sel_alu(barrel_shift_amount_sel_alu),
	.o_barrel_shift_data_sel_alu(barrel_shift_data_sel_alu),
	.o_barrel_shift_function_alu(barrel_shift_function_alu),
	.o_alu_function_alu(alu_function_alu),
	.o_rd_tag_alu(rd_tag_alu),
	.i_alu_valid(alu_valid),
	.i_alu_tag(alu_tag),
	.i_alu_data(alu_data),
	.i_alu_flags(alu_flags),
	//.i_alu_pc_wen(), //TODO hook up eventually
	
	.o_instr_valid_mult(instr_valid_mult),
	.o_rn_mult(rn_mult),
	.o_rs_mult(rs_mult),
	.o_rm_mult(rm_mult),
	.o_multiply_function_mult(multiply_function_mult),
	.o_use_carry_in_mult(),
	.o_rd_tag_mult(rd_tag_mult),
	.i_mult_valid(mult_valid),
	.i_mult_tag(mult_tag),
	.i_mult_data(mult_data),
	.i_mult_flags(mult_flags),
	//.i_mult_pc_wen(/*i_mult_pc_wen*/1'd0),
	
	.i_mem_ready(mem_ready),
	.o_instr_valid_mem(instr_valid_mem),
	.o_address_mem(address_mem),
	.o_write_data_mem(write_data_mem),
	.o_op_type_mem(op_type_mem),
	.o_byte_enable_sel_mem(byte_enable_sel_mem),
	.o_rd_tag_mem(rd_tag_mem),
	.i_mem_valid(mem_valid),
	.i_mem_tag(mem_tag),
	.i_mem_data(mem_data),
    
    .led(led),
    .sw(sw)
);


b01_execute_alu u_execute_alu (
	.i_clk(i_clk),
	.i_rst(i_rst),
	.i_core_stall(core_stall),
	.i_instr_valid(instr_valid_alu),
	.i_rn(rn_alu),
	.i_rs(rs_alu),
	.i_rm(rm_alu),
	.i_status_bits_flags(status_bits_flags_alu),
	.i_use_carry_in(use_carry_in_alu),
	.i_imm32(imm32_alu),
	.i_imm_shift_amount(imm_shift_amount_alu),
	.i_shift_imm_zero(shift_imm_zero_alu),
	.i_barrel_shift_amount_sel(barrel_shift_amount_sel_alu),
	.i_barrel_shift_data_sel(barrel_shift_data_sel_alu),
	.i_barrel_shift_function(barrel_shift_function_alu),
	.i_alu_function(alu_function_alu),
	.i_rd_tag(rd_tag_alu),
	.o_rd_valid(alu_valid),
	.o_rd_tag(alu_tag),
	.o_rd_data(alu_data),
	.o_alu_flags(alu_flags)
);


b01_execute_multiply u_execute_multiply (
	.i_clk(i_clk),
	.i_rst(i_rst),
	.i_instr_valid(instr_valid_mult),
	.i_rs(rs_mult),
	.i_rm(rm_mult),
	.i_rn(rn_mult),
	.i_function(multiply_function_mult),
	.i_rd_tag(rd_tag_mult),
	.o_valid(mult_valid),
	.o_tag(mult_tag),
	.o_data(mult_data),
	.o_flags(mult_flags)
);


b01_execute_mem u_execute_mem (
    .i_clk                              ( i_clk                             ),
    .i_rst(i_rst),
    .o_ready(mem_ready),
    
    .i_instr_valid(instr_valid_mem),
    .i_address(address_mem),
    .i_write_data(write_data_mem),
    .i_op_type(op_type_mem),
    .i_byte_enable_sel(byte_enable_sel_mem),
    .i_rd_tag(rd_tag_mem),
    
    .o_valid(mem_valid),
    .o_tag(mem_tag),
    .o_data(mem_data),
    
    .o_wb_req                  ( dcache_wb_uncached_req            ),
    .o_wb_write                         ( dcache_wb_write                   ),
    .o_wb_byte_enable                   ( dcache_wb_byte_enable             ),
    .o_wb_write_data                    ( dcache_wb_write_data              ),
    .o_wb_address                       ( dcache_wb_address                 ),      
    .i_wb_ready                ( dcache_wb_uncached_ready          ),
    .i_wb_rdata                ( dcache_wb_cached_rdata            )
);
 
 
// ======================================
//  Wishbone Master I/F
// ======================================
a25_wishbone u_wishbone (
    // CPU Side
    .i_clk                              ( i_clk                             ),
    
    // Port 0 - dcache uncached
    .i_port0_req                        ( dcache_wb_uncached_req            ),
    .o_port0_ack                        ( dcache_wb_uncached_ready          ),
    .i_port0_write                      ( dcache_wb_write                   ),
    .i_port0_wdata                      ( dcache_wb_write_data              ),
    .i_port0_be                         ( dcache_wb_byte_enable             ),
    .i_port0_addr                       ( dcache_wb_address                 ),
    .o_port0_rdata                      ( dcache_wb_cached_rdata                                  ),

    // Port 1 - dcache cached
    .i_port1_req                        ( /*dcache_wb_cached_req  */'d0            ),
    .o_port1_ack                        ( /*dcache_wb_cached_ready*/            ),
    .i_port1_write                      ( /*dcache_wb_write       */'d0            ),
    .i_port1_wdata                      ( /*dcache_wb_write_data  */'d0            ),
    .i_port1_be                         ( /*dcache_wb_byte_enable */'d0            ),
    .i_port1_addr                       ( /*dcache_wb_address     */'d0            ),
    .o_port1_rdata                      ( /*dcache_wb_cached_rdata*/            ),

    // Port 2 - instruction cache accesses, read only
    .i_port2_req                        ( icache_wb_req                     ),
    .o_port2_ack                        ( icache_wb_ready                   ),
    .i_port2_write                      ( 1'd0                              ),
    .i_port2_wdata                      ( 128'd0                            ),
    .i_port2_be                         ( 16'd0                             ),
    .i_port2_addr                       ( icache_wb_address                 ),
    .o_port2_rdata                      ( icache_wb_read_data               ),

    // Wishbone
    .o_wb_adr                           ( o_wb_adr                          ),
    .o_wb_sel                           ( o_wb_sel                          ),
    .o_wb_we                            ( o_wb_we                           ),
    .i_wb_dat                           ( i_wb_dat                          ),
    .o_wb_dat                           ( o_wb_dat                          ),
    .o_wb_cyc                           ( o_wb_cyc                          ),
    .o_wb_stb                           ( o_wb_stb                          ),
    .i_wb_ack                           ( i_wb_ack                          ),
    .i_wb_err                           ( i_wb_err                          )
);


endmodule

