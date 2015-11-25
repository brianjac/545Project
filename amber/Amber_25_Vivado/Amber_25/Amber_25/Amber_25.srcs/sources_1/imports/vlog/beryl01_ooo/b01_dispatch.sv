//all-modules TODO: hook up reset logic and initial register values

module b01_dispatch (

input                       i_clk,
input						i_rst,
input                       i_core_stall,               // stall all stages of the Amber core at the same time
input                       i_mem_stall,                // data memory access stalls
output                      o_exec_stall,               // stall the core pipeline

//input                       i_decode_iaccess,           // Indicates an instruction access
input                       /*i_decode_daccess*/i_is_memop,           // Indicates a data access
//input       [7:0]           i_decode_load_rd,           // The destination register for a load instruction

output logic[31:0]          o_iaddress,
output      [31:0]          o_iaddress_nxt,             // un-registered version of address to the
                                                        // cache rams address ports
output logic                o_iaddress_valid,     // High when instruction address is valid

output logic                o_adex = 'd0,               // Address Exception
output logic                o_priviledged = 'd0,        // Priviledged access
output logic                o_exclusive = 'd0,          // swap access
output logic                o_write_enable = 'd0,
output logic[3:0]           o_byte_enable = 'd0,

output      [31:0]          o_status_bits,              // Full PC will all status bits, but PC part zero'ed out


// --------------------------------------------------
// Control signals from Instruction Decode stage
// --------------------------------------------------

//TODO revise based on revised Decode stage interface!
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
//input                       i_user_mode_regs_store_nxt,
//input                       i_firq_not_user_mode,
input                       i_use_carry_in,         // e.g. add with carry instruction

input                       i_write_data_wen,
//input                       i_base_address_wen,     // save LDM base address register,
                                                    // in case of data abort
input                       i_pc_wen,
input      [14:0]           i_reg_bank_wen,
input                       i_status_bits_flags_wen,
input                       i_status_bits_mode_wen,
input                       i_status_bits_irq_mask_wen,
input                       i_status_bits_firq_mask_wen,
//input                       i_copro_write_data_wen,
//input                       i_conflict,
input                       i_use_rn/*use_read*/, //TODO note: these *should* tell us if a given reg is needed for this instruction; TODO confirm in simulation of original-Amber!!!
input                       i_use_rm/*use_read*/,
input                       i_use_rs/*use_read*/,
//input                       i_use_rd/*use_read*/,

//Stuff specifically added for OOO processing
//ALU station interface
output logic o_instr_valid_alu,
output logic [31:0] o_rn_alu,
output logic [31:0] o_rs_alu,
output logic [31:0] o_rm_alu,
output logic [3:0] o_status_bits_flags_alu,
output logic o_use_carry_in_alu,
output logic [31:0] o_imm32_alu,
output logic [4:0] o_imm_shift_amount_alu,
output logic o_shift_imm_zero_alu,
output logic [1:0] o_barrel_shift_amount_sel_alu,
output logic [1:0] o_barrel_shift_data_sel_alu,
output logic [1:0] o_barrel_shift_function_alu,
output logic [8:0] o_alu_function_alu,
output logic o_pc_wen_alu,
output logic o_status_bits_flags_wen_alu,
output logic [5:0] o_rd_tag_alu,
input logic i_alu_valid,
input logic [5:0] i_alu_tag,
input logic [31:0] i_alu_data,
input logic [3:0] i_alu_flags,
input logic i_alu_pc_wen,

//Mult station interface
output logic o_instr_valid_mult,
output logic [31:0] o_rn_mult,
output logic [31:0] o_rs_mult,
output logic [31:0] o_rm_mult,
//output logic [31:0] o_ccr_mult,
output logic [1:0] o_multiply_function_mult,
output logic o_use_carry_in_mult,
output logic o_pc_wen_mult,
output logic o_status_bits_flags_wen_mult,
output logic [5:0] o_rd_tag_mult,
input logic i_mult_valid,
input logic [5:0] i_mult_tag,
input logic [31:0] i_mult_data,
input logic [3:0] i_mult_flags,
input logic i_mult_pc_wen,

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
input logic i_mem_pc_wen,
//TODO add mem_write_enable signal!!!

output [7:0] led,
input [7:0] sw
);

`include "a25_localparams.vh"

logic [5:0] 	tag_nxt; //tied from tag store output to register file input and reservation station input

logic [31:0]	pc_nxt;


//Status register logic
logic 			status_bits_flags_valid; //invalidate on each newly decoded instruction based on i_status_bits_flags_wen
logic [5:0] 	status_bits_flags_tag; //update on each newly decoded instruction based on i_status_bits_flags_wen
logic [3:0] 	status_bits_flags; //set by the EU outputs
logic			status_bits_irq_mask; //set by the decode stage
logic			status_bits_firq_mask; //set by the decode stage
logic [1:0] 	status_bits_mode; //set by the decode stage

//todo create a place into which to save the status register on irq
//"spsr" instead of the below which is "cpsr"
assign o_status_bits = {status_bits_flags,	   //31:28 = flags
						status_bits_irq_mask,  //27
						status_bits_firq_mask, //26
						24'd0,
						status_bits_mode}; //1:0 = mode
						
logic instr_conditional, condition_stall;
assign instr_conditional = i_condition != AL;
assign condition_stall = instr_conditional & ~status_bits_flags_valid; //TODO incorporate into o_exec_stall

//Note that here we're removing reference to the coprocessor, since it doesn't *actually* exist in the design.
//The rest of the time, i_xyz_flags will be set in the appropriate Execute substage based on the i_status_bits_sel we pass in.
//Additional note: the ldm flag-handling might require some special logic here.

//TODO: if rd is r15, i_status_bits_sel set to 3'd1 --> result of computation goes into status bits.
//TODO: thus, if ^^, we need to stall until resolved.
//WARNING: KNOWN ISSUE: The above is NOT implemented in the current design!

//TODO incorporate stall logic in the below as needed
always_ff @(posedge i_rst, posedge i_clk) begin //TODO confirm reset polarity
	if (i_rst) begin
		status_bits_flags_valid <= '1;
		status_bits_flags_tag <= '0;
		status_bits_flags <= 4'b1111;
		status_bits_mode <= SVC;
		status_bits_irq_mask <= '1;
		status_bits_firq_mask <= '1;
	end
	else if (i_clk) begin
		if (!i_core_stall) begin //TODO note: must be careful of logical loops that could cause this to get locked high forever
			status_bits_flags_valid <= 	i_status_bits_flags_wen								?	1'b0:
										i_alu_valid && i_alu_tag == status_bits_flags_tag	?	1'b1:
										i_mult_valid && i_mult_tag == status_bits_flags_tag	?	1'b1:
										i_mem_valid && i_mem_tag == status_bits_flags_tag	?	1'b1:
																								status_bits_flags_valid;
			
			status_bits_flags_tag	<=	i_status_bits_flags_wen								?	tag_nxt	:
																								status_bits_flags_tag;
			
			status_bits_flags 	 	<=	status_bits_flags_valid								? status_bits_flags	:
										i_alu_valid && i_alu_tag == status_bits_flags_tag 	? i_alu_flags 		:
										i_mult_valid && i_mult_tag == status_bits_flags_tag ? i_mult_flags 		:
										i_mem_valid && i_mem_tag == status_bits_flags_tag 	? i_mem_flags		:
																							  4'b1111;
				
			status_bits_mode 	 	<=	i_status_bits_mode_wen		?	i_status_bits_mode		:
																		status_bits_mode;
																
			status_bits_irq_mask 	<=	i_status_bits_irq_mask_wen	?	i_status_bits_irq_mask	:
																		status_bits_irq_mask;
			
			status_bits_firq_mask	<=	i_status_bits_firq_mask_wen?	i_status_bits_firq_mask	:
																		status_bits_firq_mask;
		end
	end
end


//Register bank instantiation and hookups
logic			reg_bank_rm_valid,
				reg_bank_rs_valid,
				reg_bank_rn_valid,
				reg_bank_rd_valid,
				reg_bank_pc_valid;
logic [5:0]		reg_bank_rm_tag,
				reg_bank_rs_tag,
				reg_bank_rn_tag,
				reg_bank_rd_tag,
				reg_bank_pc_tag;
logic [31:0]	rm,
				rs,
				rn,
				rd,
				pc;

b01_register_bank u_register_bank(
    .i_clk                   ( i_clk                     ),
	.i_rst(i_rst),
    .i_core_stall            ( i_core_stall              ),
    .i_mem_stall             ( i_mem_stall               ),
    .i_mode_idec             ( i_status_bits_mode        ), //TODO is this necessary?
    .i_mode_exec             ( status_bits_mode          ),
    // use one-hot version for speed, combine with i_user_mode_regs_store
    //.i_mode_rds_exec         ( oh_status_bits_mode(status_bits_mode)   ), //TODO this isn't strictly necessary for correctness if register bank is suitably modified...
    // pre-encoded in decode stage to speed up long path
    //.i_firq_not_user_mode    ( i_firq_not_user_mode      ), //TODO do we care about this?
/*input       [1:0]           i_mode_idec,            // user, supervisor, irq_idec, firq_idec etc.
                                                    // Used for register writes
input       [1:0]           i_mode_exec,            // 1 periods delayed from i_mode_idec
                                                    // Used for register reads
input       [3:0]           i_mode_rds_exec,        // Use one-hot version specifically for rds, 
                                                    // includes i_user_mode_regs_store
input                       i_firq_not_user_mode,*/
    .i_rm_sel                ( i_rm_sel                  ),
    .i_rs_sel                ( i_rs_sel                  ),
    .i_rn_sel                ( i_rn_sel                  ),
    .i_pc_wen                ( i_pc_wen                    ), //TODO confirm/fix
    .i_reg_bank_wen          ( i_reg_bank_wen              ), //TODO confirm/fix, esp. wrt stall logic
    .i_pc                    ( /*pc_nxt[25:2]*/pc[25:2]+24'd1              ), //TODO confirm/fix
	
    .i_status_bits_flags     ( status_bits_flags         ),
    .i_status_bits_irq_mask  ( status_bits_irq_mask      ),
    .i_status_bits_firq_mask ( status_bits_firq_mask     ),

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
	
	.i_rd_tag		(tag_nxt),
    
    .led(led),
    .sw(sw)
);


//PC and instruction address logic

//Interrupt vector definition
//TODO confirm the particular addresses with team
logic [31:0] interrupt_vector;
assign interrupt_vector = // Reset vector
                          (i_interrupt_vector_sel == 3'd0) ? 32'h00000000 :
                          /*// Data abort interrupt vector
                          (i_interrupt_vector_sel == 3'd1) ? 32'h00000010 :*/
                          // Fast interrupt vector
                          (i_interrupt_vector_sel == 3'd2) ? 32'h0000001c :
                          // Regular interrupt vector
                          (i_interrupt_vector_sel == 3'd3) ? 32'h00000018 :
                          /* // Prefetch abort interrupt vector
                          (i_interrupt_vector_sel == 3'd5) ? 32'h0000000c :
                          // Undefined instruction interrupt vector
                          (i_interrupt_vector_sel == 3'd6) ? 32'h00000004 :*/
                          // Software (SWI) interrupt vector
                          (i_interrupt_vector_sel == 3'd7) ? 32'h00000008 :
                          // Default is the address exception interrupt
                                                             32'h00000014 ;

//recall we have pc, reg_bank_pc_valid, and reg_bank_pc_tag. we also need to deal with i_core_stall.
logic pc_stall;
//we must stall if we're getting the PC from anything other than pc+4 or a predefined interrupt vector
assign pc_stall = (~reg_bank_pc_valid) | (i_iaddress_sel != 3'd0 && i_iaddress_sel != 3'd2); //note: TODO WARNING: changed from i_pc_sel to i_iaddress_sel; this *should* work but might not!
/*assign o_iaddress_nxt =	(~reg_bank_pc_valid)		? pc				:
						(i_iaddress_sel == 4'd0)	? pc+32'd4				:
						(i_iaddress_sel == 4'd2)	? interrupt_vector	:
													  pc+32'd4; //default, just in case*/
assign o_iaddress_nxt = pc;//tmp, for stupid-testing
assign o_iaddress_valid = 1'b1;//tmp, for stupid-testing
always_ff @(posedge i_rst, posedge i_clk) begin
	if (i_rst) begin
		//o_iaddress_valid <= 'd0;
		o_iaddress <= 32'd0; //note that on reset, at the next clock edge PC will have been initialized and set by the register file
	end
	else if (i_clk) begin
		if (!i_core_stall) begin
			//o_iaddress_valid 	<= 	!pc_stall;
			o_iaddress			<=	o_iaddress_nxt;
		end
	end
end

//TODO set up pc_nxt and pc_wen and default o_iaddress value
//TODO confirm/think through more about the specific o_iaddress_valid and o_iaddress logic, incl. interactions with and forwarding from register file/tag bus.
//TODO the above stuff particularly in light of the need to redesign much of b01_decode

/* OLD

// if current instruction does not execute because it does not meet the condition
// then address advances to next instruction
assign o_iaddress_nxt = (pc_dmem_wen)            ? pcf(read_data_filtered) :
                        (!execute)               ? pc_plus4                :
                        (i_iaddress_sel == 4'd0) ? pc_plus4                :
                        (i_iaddress_sel == 4'd1) ? alu_out_pc_filtered     :
                        (i_iaddress_sel == 4'd2) ? interrupt_vector        :
                                                   pc                      ;

// If current instruction does not execute because it does not meet the condition
// then PC advances to next instruction
assign pc_nxt = (!execute)       ? pc_plus4                :
                i_pc_sel == 3'd0 ? pc_plus4                :
                i_pc_sel == 3'd1 ? alu_out                 :
                i_pc_sel == 3'd2 ? interrupt_vector        :
                i_pc_sel == 3'd3 ? pcf(read_data_filtered) :
                                   pc_minus4               ;

// allow the PC to increment to the next instruction when current
// instruction does not execute
assign pc_wen       = (i_pc_wen || !execute) && !i_conflict;

// For some multi-cycle instructions, the stream of instrution
// reads can be paused. However if the instruction does not execute
// then the read stream must not be interrupted.
assign iaddress_valid_nxt = i_decode_iaccess || !execute;

assign iaddress_update                 = pc_dmem_wen || (!i_core_stall && !i_conflict);

always @(posedge i_clk)
	iaddress_r              <= iaddress_update                ? o_iaddress_nxt               : iaddress_r;
    o_iaddress_valid        <= iaddress_update                ? iaddress_valid_nxt           : o_iaddress_valid;
end    

*/

logic [1:0] instr_type;
assign instr_type = 	i_multiply_function[0] ? 2'b01:
						i_is_memop ? 2'b10:
									 2'b00;
logic tag_stall;
assign o_exec_stall = condition_stall | tag_stall | pc_stall; //TODO confirm/edit
									 
//Tag store instantiation
b01_tagstore u_tagstore(
	.i_clk(i_clk),
	.i_rst(i_rst),
	.i_stall(/*TODO fix*/i_core_stall),
	.i_instr_type(/*TODO*/instr_type), //00=alu, 01=mult, 10=mem. TODO: `define these in a header file
	.i_alu_valid(i_alu_valid),
	.i_alu_tag(i_alu_tag),
	.i_mult_valid(i_mult_valid),
	.i_mult_tag(i_mult_tag),
	.i_mem_valid(i_mem_valid),
	.i_mem_tag(i_mem_tag),
	.o_station_full(tag_stall),
	.o_tag(tag_nxt)
);

//Reservation station instantiation
b01_reservation u_reservation(
	.i_clk(i_clk),
	.i_rst(i_rst),
	.i_stall(/*TODO confirm proper operation*//*o_exec_stall*/i_core_stall),
	.i_tag_nxt(tag_nxt),
	.i_imm32(i_imm32),
	.i_imm_shift_amount(i_imm_shift_amount),
	.i_shift_imm_zero(i_shift_imm_zero),
	.i_decode_exclusive(i_decode_exclusive),
	.i_barrel_shift_amount_sel(i_barrel_shift_amount_sel),
	.i_barrel_shift_data_sel(i_barrel_shift_data_sel),
	.i_barrel_shift_function(i_barrel_shift_function),
	.i_alu_function(i_alu_function),
	.i_multiply_function(i_multiply_function),
	.i_byte_enable_sel(i_byte_enable_sel),
	.i_use_carry_in(i_use_carry_in),
	.i_write_data_wen(i_write_data_wen),
	.i_base_address_wen(/*i_base_address_wen*/), //TODO remove!!!
	.i_pc_wen(i_pc_wen),
	.i_reg_bank_wen(i_reg_bank_wen),
	.i_status_bits_flags_wen(i_status_bits_flags_wen),
	.i_rn_valid(reg_bank_rn_valid | !i_use_rn), //"or" accounts for case where operand is unused (ditto rs, rm)
	.i_rn_tag(reg_bank_rn_tag),
	.i_rn(rn),
	.i_rs_valid(reg_bank_rs_valid | !i_use_rs),
	.i_rs_tag(reg_bank_rs_tag),
	.i_rs(rs),
	.i_rm_valid(reg_bank_rm_valid | !i_use_rm),
	.i_rm_tag(reg_bank_rm_tag),
	.i_rm(rm),
	.i_status_bits_flags_valid(status_bits_flags_valid | i_use_carry_in),
	.i_status_bits_flags_tag(status_bits_flags_tag),
	.i_status_bits_flags(status_bits_flags),
	
	.o_instr_valid_alu(o_instr_valid_alu),
	.o_rn_alu(o_rn_alu),
	.o_rs_alu(o_rs_alu),
	.o_rm_alu(o_rm_alu),
	.o_status_bits_flags_alu(o_status_bits_flags_alu),
	.o_use_carry_in_alu(o_use_carry_in_alu),
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


endmodule