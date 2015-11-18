module b01_dispatch (

input                       i_clk,
input                       i_core_stall,               // stall all stages of the Amber core at the same time
input                       i_mem_stall,                // data memory access stalls
output                      o_exec_stall,               // stall the core pipeline

/*input       [31:0]          i_wb_read_data,             // data reads
input                       i_wb_read_data_valid,       // read data is valid
input       [10:0]          i_wb_load_rd,               // Rd for data reads

input       [31:0]          i_copro_read_data,          // From Co-Processor, to either Register
                                                        // or Memory*/
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
input                       i_decode_exclusive,       // swap access //for SWP/SWPB instruction that atomically writes one reg's value into mem at a given addr and loads the value at that addr into another register. Will need to handle "specially" with some sort of state machine in the mem stage.

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
input                       i_use_rn/*use_read*/, //TODO note: these *should* tell us if a given reg is needed for this instruction; TODO confirm in simulation of original-Amber!!!
input                       i_use_rm/*use_read*/,
input                       i_use_rs/*use_read*/,
input                       i_use_rd/*use_read*/,

//Stuff specifically added for OOO processing
//ALU station interface
//TODO replace o_alu_stage_command and o_alu_stage_command_valid with these. Ditto for mult and mem (when those stations done).
output logic o_instr_valid_alu,
output logic [31:0] o_rn_alu,
output logic [31:0] o_rs_alu,
output logic [31:0] o_rm_alu,
output logic [31:0] o_imm32_alu;
output logic [4:0] o_imm_shift_amount_alu,
output logic o_shift_imm_zero_alu,
output logic [1:0] o_barrel_shift_amount_sel_alu,
output logic [1:0] o_barrel_shift_data_sel,
output logic [1:0] o_barrel_shift_function_alu,
output logic [8:0] o_alu_function_alu,
output logic o_pc_wen_alu,
output logic o_status_bits_flags_wen_alu,
output logic [5:0] o_rd_tag_alu,
input logic i_alu_valid,
input logic [5:0] i_alu_tag,
input logic [31:0] i_alu_data,
input logic [3:0] i_alu_flags,

//Mult station interface
output logic o_instr_valid_mult,
output logic [31:0] o_rn_mult,
output logic [31:0] o_rs_mult,
output logic [31:0] o_rm_mult,
output logic [31:0] o_ccr_mult,
output logic [1:0] o_multiply_function_mult,
output logic o_use_carry_in_mult,
output logic o_pc_wen_mult,
output logic o_status_bits_flags_wen_mult,
output logic [5:0] o_rd_tag_mult,
input logic i_mult_valid,
input logic [5:0] i_mult_tag,
input logic [31:0] i_mult_data,
input logic [3:0] i_mult_flags,

//Mem station interface
output logic o_instr_valid_mem,
output logic [31:0] o_rn_mem,
output logic [31:0] o_rs_mem,
output logic [31:0] o_rm_mem, //TODO: determine if this is ever actually used in memory instructions
output logic o_exclusive_mem,
output logic o_pc_wen_mem,
output logic o_status_bits_flags_wen_mem,
output logic [1:0] o_byte_enable_sel_mem,
output logic [5:0] o_rd_tag_mem,
input logic i_mem_valid,
input logic [5:0] i_mem_tag,
input logic [31:0] i_mem_data,
input logic [3:0] i_mem_flags,

output [7:0] led,
input [7:0] sw
);

//Local vars
//TODO


//Register bank instantiation
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

    .o_rm                    ( rm           		     ),
    .o_rs                    ( rs            		     ),
    .o_rd                    ( rd           		     ),
    .o_rn                    ( rn           		     ),
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
	
	//TODO create the set of reg_bank_xyz_valid/tag signals and hook up correctly
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


logic [5:0] tag_nxt;

//Tag store instantiation
b01_tagstore u_tagstore(
	.i_clk(i_clk),
	.i_stall(/*TODO*/),
	.i_instr_type(/*TODO*/), //00=alu, 01=mult, 10=mem. TODO: `define these in a header file
	.i_alu_valid(i_alu_valid),
	.i_alu_tag(i_alu_tag),
	.i_mult_valid(i_mult_valid),
	.i_mult_tag(i_mult_tag),
	.i_mem_valid(i_mem_valid),
	.i_mem_tag(i_mem_tag),
	.o_station_full(/*TODO*/),
	.o_tag(tag_nxt)
);

//Reservation station instantiation
b01_reservation u_reservation(
	.i_clk(i_clk),
	.i_stall(/*TODO*/),
	.i_tag_nxt(tag_nxt),
	.i_imm32(i_imm32),
	.i_imm_shift_amount(i_imm_shift_amount),
	.i_shift_imm_zero(i_imm_shift_zero),
	.i_decode_exclusive(i_decode_exclusive),
	.i_barrel_shift_amount_sel(i_barrel_shift_amount_sel),
	.i_barrel_shift_data_sel(i_barrel_shift_data_sel),
	.i_barrel_shift_function(i_barrel_shift_function),
	.i_alu_function(i_alu_function),
	.i_multiply_function(i_multiply_function),
	.i_byte_enable_sel(i_byte_enable_sel),
	.i_use_carry_in(i_use_carry_in),
	.i_write_data_wen(i_write_data_wen),
	.i_base_address_wen(i_base_address_wen),
	.i_pc_wen(i_pc_wen),
	.i_reg_bank_wen(i_reg_bank_wen),
	.i_status_bits_flags_wen(i_status_bits_flags_wen),
	.i_rn_valid(reg_bank_rn_valid), //TODO revise to account for case where param unused
	.i_rn_tag(reg_bank_rn_tag),
	.i_rn(rn),
	.i_rs_valid(reg_bank_rs_valid), //TODO revise to account for case where param unused
	.i_rs_tag(reg_bank_rs_tag),
	.i_rs(rs),
	.i_rm_valid(reg_bank_rm_valid), //TODO revise to account for case where param unused
	.i_rm_tag(reg_bank_rm_tag),
	.i_rm(rm),
	
	.o_instr_valid_alu(o_instr_valid_alu),
	.o_rn_alu(o_rn_alu),
	.o_rs_alu(o_rs_alu),
	.o_rm_alu(o_rm_alu),
	.o_imm32_alu(o_imm32_alu),
	.o_imm_shift_amount_alu(o_imm_shift_amount_alu),
	.o_shift_imm_zero_alu(o_shift_imm_zero_alu),
	.o_barrel_shift_amount_sel_alu(o_barrel_shift_amount_sel_alu),
	.o_barrel_shift_data_sel_alu(o_barrel_shift_data_sel_alu),
	.o_barrel_shift_function_alu(o_barrel_shift_function_alu),
	.o_alu_function_alu(o_alu_function_alu),
	.o_pc_wen_alu(o_pc_wen_alu),
	.o_status_bits_flags_wen_alu(o_status_bits_flags_wen_alu),
	.o_rd_tag_alu(o_rd_tag_alu),
	.i_alu_valid(i_alu_valid),
	.i_alu_tag(i_alu_tag),
	.i_alu_data(i_alu_data),
	
	.o_instr_valid_mult(o_instr_valid_mult),
	.o_rn_mult(o_rn_mult),
	.o_rs_mult(o_rs_mult),
	.o_rm_mult(o_rm_mult),
	.o_multiply_function_mult(o_multiply_function_mult),
	.o_use_carry_in_mult(o_use_carry_in_mult),
	.o_pc_wen_mult(o_pc_wen_mult),
	.o_status_bits_flags_wen_mult(o_status_bits_flags_wen_mult),
	.o_rd_tag_mult(o_rd_tag_mult),
	.i_mult_valid(i_mult_valid),
	.i_mult_tag(i_mult_tag),
	.i_mult_data(i_mult_data),
	
	.o_instr_valid_mem(o_instr_valid_mem),
	.o_rn_mem(o_rn_mem),
	.o_rs_mem(o_rs_mem),
	.o_rm_mem(o_rm_mem),
	.o_exclusive_mem(o_exclusive_mem),
	.o_pc_wen_mem(o_pc_wen_mem),
	.o_status_bits_flags_wen_mem(o_status_bits_flags_wen_mem),
	.o_byte_enable_sel_mem(o_byte_enable_sel_mem),
	.o_rd_tag_mem(o_rd_tag_mem),
	.i_mem_valid(i_mem_valid),
	.i_mem_tag(i_mem_tag),
	.i_mem_data(i_mem_data)
);


//Logic tying it all together



endmodule