//all-modules TODO: hook up reset logic and initial register values

module b01_dispatch (

input                       i_clk,
input						i_rst,
input                       i_core_stall,               // stall all stages of the Amber core at the same time
output                      o_exec_stall,               // stall the core pipeline

input                       /*i_decode_daccess*/i_is_memop,           // Indicates a data access

output logic[31:0]          o_iaddress,
output      [31:0]          o_iaddress_nxt,             // un-registered version of address to the
                                                        // cache rams address ports
output logic                o_iaddress_valid,     // High when instruction address is valid

//output logic                o_adex = 'd0,               // Address Exception
//output logic                o_priviledged = 'd0,        // Priviledged access
output logic                o_write_enable = 'd0,
output logic[3:0]           o_byte_enable = 'd0,

output      [31:0]          o_status_bits,              // Full PC will all status bits, but PC part zero'ed out

//TODO ensure set properly and that combinational vs. registered works right
output logic				o_branch_taken, //asserted when branch instruction executes and needs to flush what's currently in Decode, or when *anything* is written back to r15


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

input      [3:0]            i_rm_sel,
input      [3:0]            i_rs_sel,
input      [3:0]            i_rn_sel,
input      [3:0]            i_rd_sel,
input      [1:0]            i_barrel_shift_amount_sel,
input      [1:0]            i_barrel_shift_data_sel,
input      [1:0]            i_barrel_shift_function,
input      [8:0]            i_alu_function,
input      [1:0]            i_multiply_function,
input      [2:0]            i_interrupt_vector_sel,
input      [3:0]            i_iaddress_sel,
//input      [3:0]            i_daddress_sel,
//input      [2:0]            i_pc_sel,
input      [1:0]            i_byte_enable_sel,
//input                       i_user_mode_regs_store_nxt,
//input                       i_firq_not_user_mode,
input                       i_use_carry_in,         // e.g. add with carry instruction

input [1:0] i_op_type_mem,
input [1:0] i_address_mode,

input                       i_write_data_wen,
input                       i_pc_wen,
input      [14:0]           i_reg_bank_wen,
input                       i_status_bits_flags_wen,
input                       i_status_bits_mode_wen,
input                       i_status_bits_irq_mask_wen,
input                       i_status_bits_firq_mask_wen,
input                       i_use_rn, //these tell us if a given reg is needed for this instruction
input                       i_use_rm,
input                       i_use_rs,
input						i_use_sr,

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
output logic [5:0] o_rd_tag_alu,
input logic i_alu_valid,
input logic [5:0] i_alu_tag,
input logic [31:0] i_alu_data,
input logic [3:0] i_alu_flags,
//input logic i_alu_pc_wen,

//Mult station interface
output logic o_instr_valid_mult,
output logic [31:0] o_rn_mult,
output logic [31:0] o_rs_mult,
output logic [31:0] o_rm_mult,
//output logic [31:0] o_ccr_mult,
output logic o_multiply_function_mult,
output logic o_use_carry_in_mult,
output logic [5:0] o_rd_tag_mult,
input logic i_mult_valid,
input logic [5:0] i_mult_tag,
input logic [31:0] i_mult_data,
input logic [1:0] i_mult_flags,
//input logic i_mult_pc_wen,

//Mem station interface
input logic i_mem_ready,
output logic o_instr_valid_mem,
output logic [31:0] o_address_mem,
output logic [31:0] o_write_data_mem,
output logic [1:0] o_op_type_mem,
output logic [1:0] o_byte_enable_sel_mem, //byte_enable vector derived from this, inside the b01_execute_mem module itself
output logic [5:0] o_rd_tag_mem,
input logic i_mem_valid,
input logic [5:0] i_mem_tag,
input logic [31:0] i_mem_data,
//input logic [3:0] i_mem_flags, //note that memory ops never set the flags, although they may be conditionally executed
//input logic i_mem_pc_wen,

input logic i_is_psr,
input logic i_mrs_msr,
input logic i_psr_sel,
input logic [3:0] i_psr_reg,

output [7:0] led,
input [7:0] sw
);

`include "a25_localparams.vh"
`include "a25_functions.vh"

logic [5:0] 	tag_nxt_alu_mult_1,
              tag_nxt_mem; //tied from tag store output to register file input and reservation station input
logic [31:0]	pc_nxt;
logic condition_execute;


assign o_branch_taken = (/*pc_sel_nxt*//*i_pc_sel == 3'd1*/i_iaddress_sel==4'd1) & condition_execute;


//Status register logic
logic 			status_bits_flags_valid; //invalidate on each newly decoded instruction based on i_status_bits_flags_wen
logic			status_bits_flags_valid_nxt;
logic			status_bits_flags_valid_nxt_eucompare;
logic [5:0] 	status_bits_flags_tag; //update on each newly decoded instruction based on i_status_bits_flags_wen
logic [5:0]		status_bits_flags_tag_nxt;
logic [3:0] 	status_bits_flags; //set by the EU outputs
logic [3:0]		status_bits_flags_nxt;

logic			status_bits_irq_mask; //set by the decode stage
logic			status_bits_firq_mask; //set by the decode stage
logic [1:0] 	status_bits_mode; //set by the decode stage

logic [31:0]    cpsr; //a dummy based on status_bits_whatever
assign cpsr = { status_bits_flags,	   //31:28 = flags
                status_bits_irq_mask,  //27
                status_bits_firq_mask, //26
                24'd0,
                status_bits_mode}; //1:0 = mode
                
//these actually used as registers rather than just as dummies
logic [31:0]    spsr_usr, //all bits of all of these are used. we save flags, irq mask, firq mask, and mode all, and can restore all of them. note that the restoration supercedes any mode-setting by Decode, use with care as this could cause strange behavior if an interrupt fires on the same clock cycle as an MSR is being executed!
                spsr_svc,
                spsr_irq,
                spsr_firq;

logic psr_stall;
assign psr_stall = i_is_psr && i_mrs_msr==1'b1 && !reg_bank_psr_reg_valid;

//update logic for SPSRs
always_ff @(posedge i_clk, negedge i_rst) begin
    if (i_rst) begin
        spsr_usr <= {4'hf, 2'b00, 24'd0, 2'd0};
        spsr_svc <= {4'hf, 2'b00, 24'd0, 2'd1};
        spsr_irq <= {4'hf, 2'b00, 24'd0, 2'd2};
        spsr_firq<= {4'hf, 2'b00, 24'd0, 2'd3};
    end
    else begin
        if (!(i_core_stall || condition_stall/* || pc_stall*/) && i_is_psr && i_mrs_msr==1'b1 && i_psr_sel==1'b1) begin //note that if this enters the stage, PC should NEVER be stalled
            if (!psr_stall) begin
                case (status_bits_mode)
                    2'd0: spsr_usr <= reg_bank_psr_reg_data;
                    2'd1: spsr_svc <= reg_bank_psr_reg_data;
                    2'd2: spsr_irq <= reg_bank_psr_reg_data;
                    2'd3: spsr_firq<= reg_bank_psr_reg_data;
                endcase
            end 
        end
        //else, leave as-is
    end
end

//TODO modify with cpsr, spsr, and saved info stuff (not here, but in the always_ff below)
assign o_status_bits = cpsr;
						
logic condition_stall;
assign condition_stall = i_use_sr & ~status_bits_flags_valid_nxt_eucompare;
//condition_execute: set to 1 if ccr valid and flags match OR not using ccr OR ccr invalid but eu writing back flags and forwarded condition match
assign condition_execute =	~i_use_sr								?	1'b1:
							status_bits_flags_valid					?	conditional_execute(i_condition, status_bits_flags):
							status_bits_flags_valid_nxt_eucompare	?	conditional_execute(i_condition, status_bits_flags_nxt):
																		1'b0;
//condition_execute hooked up to the tag store and the reservation station so they know not to generate a tag/add something to the reservation station if this instruction does not conditionally execute

//TODO: if rd is r15, i_status_bits_sel set to 3'd1 --> result of computation goes into status bits.
//TODO: thus, if ^^, we need to stall until resolved.
//WARNING: KNOWN ISSUE: The above is NOT implemented in the current design!

always_comb begin
	status_bits_flags_valid_nxt_eucompare = //i_is_psr && i_mrs_msr==1'b1 && i_psr_sel==1'b0 && reg_bank_psr_reg_valid ? 1'b1:
	                                        i_alu_valid && i_alu_tag == status_bits_flags_tag	?	1'b1:
											i_mult_valid && i_mult_tag == status_bits_flags_tag	?	1'b1:
											//i_mem_valid && i_mem_tag == status_bits_flags_tag	?	1'b1: //removed since mem stage cannot set status bits
																									status_bits_flags_valid;
	
	status_bits_flags_valid_nxt = 	i_status_bits_flags_wen								?	1'b0:
																							status_bits_flags_valid_nxt_eucompare;
	status_bits_flags_tag_nxt	=	i_status_bits_flags_wen								?	tag_nxt_alu_mult_1	:
																							status_bits_flags_tag;
	
	status_bits_flags_nxt 	 	=	//i_is_psr && i_mrs_msr==1'b1 && i_psr_sel==1'b0      ? reg_bank_psr_reg_data[31:28]          :
	                                status_bits_flags_valid								? status_bits_flags						:
									i_alu_valid && i_alu_tag == status_bits_flags_tag 	? i_alu_flags 							:
									i_mult_valid && i_mult_tag == status_bits_flags_tag ? {status_bits_flags[3:2], i_mult_flags}: //since multiply only sets the N and Z bits
									//i_mem_valid && i_mem_tag == status_bits_flags_tag 	? i_mem_flags							:
																						  4'b1111;
end

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
			status_bits_flags_valid <= 	i_is_psr && i_mrs_msr==1'b1 && i_psr_sel==1'b0 && reg_bank_psr_reg_valid ? 1'b1 : status_bits_flags_valid_nxt;
			status_bits_flags_tag	<=	status_bits_flags_tag_nxt;
			status_bits_flags 	 	<=	i_is_psr && i_mrs_msr==1'b1 && i_psr_sel==1'b0 && reg_bank_psr_reg_valid ? reg_bank_psr_reg_data[31:28] : status_bits_flags_nxt;
			status_bits_mode 	 	<=	 i_is_psr && i_mrs_msr==1'b1 && i_psr_sel==1'b0 && reg_bank_psr_reg_valid ? reg_bank_psr_reg_data[3:0] :
			                             i_status_bits_mode_wen		?	i_status_bits_mode		:
																		status_bits_mode;
																
			status_bits_irq_mask 	<=	 i_is_psr && i_mrs_msr==1'b1 && i_psr_sel==1'b0 && reg_bank_psr_reg_valid ? reg_bank_psr_reg_data[27]:
			                             i_status_bits_irq_mask_wen	?	i_status_bits_irq_mask	:
																		status_bits_irq_mask;
			
			status_bits_firq_mask	<=	 i_is_psr && i_mrs_msr==1'b1 && i_psr_sel==1'b0 && reg_bank_psr_reg_valid ? reg_bank_psr_reg_data[26]:
			                             i_status_bits_firq_mask_wen?	i_status_bits_firq_mask	:
																		status_bits_firq_mask;
		end
		else begin //note that if the instr is loading a PSR, we should still update the mode/irq/firq/flags from the loaded data even if fetch has stalled
            status_bits_mode         <=     i_is_psr && i_mrs_msr==1'b1 && i_psr_sel==1'b0 && reg_bank_psr_reg_valid ? reg_bank_psr_reg_data[3:0] :
                                                                        status_bits_mode;
            status_bits_irq_mask     <=     i_is_psr && i_mrs_msr==1'b1 && i_psr_sel==1'b0 && reg_bank_psr_reg_valid ? reg_bank_psr_reg_data[27]:
                                                                        status_bits_irq_mask;
            status_bits_firq_mask    <=     i_is_psr && i_mrs_msr==1'b1 && i_psr_sel==1'b0 && reg_bank_psr_reg_valid ? reg_bank_psr_reg_data[26]:
                                                                        status_bits_firq_mask;
            status_bits_flags_valid <= i_is_psr && i_mrs_msr==1'b1 && i_psr_sel==1'b0 && reg_bank_psr_reg_valid ? 1'b1 : status_bits_flags_valid_nxt_eucompare;
            status_bits_flags <= i_is_psr && i_mrs_msr==1'b1 && i_psr_sel==1'b0 && reg_bank_psr_reg_valid ? reg_bank_psr_reg_data[31:28] : status_bits_flags_nxt;
		end
	end
end


//Register bank instantiation and hookups
logic			reg_bank_rm_valid,
				reg_bank_rs_valid,
				reg_bank_rn_valid,
				reg_bank_rd_valid,
				reg_bank_pc_valid,
                reg_bank_psr_reg_valid;
logic [5:0]		reg_bank_rm_tag,
				reg_bank_rs_tag,
				reg_bank_rn_tag,
				reg_bank_rd_tag,
				reg_bank_pc_tag;
logic [31:0]	rm,
				rs,
				rn,
				rd,
				pc,
				reg_bank_psr_reg_data,
				reg_bank_psr_data_in;
assign reg_bank_psr_data_in =   !i_psr_sel ? cpsr :
                                status_bits_mode==2'd0 ? spsr_usr :
                                status_bits_mode==2'd1 ? spsr_svc :
                                status_bits_mode==2'd2 ? spsr_irq :
                                                         spsr_firq;

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
                                                             
assign pc_nxt = o_exec_stall                ? pc                      :
                (!condition_execute)        ? pc+'d4                  :
                /*i_pc_sel == 3'd0*/i_iaddress_sel==4'd0            ? pc+'d4                  :
                /*i_pc_sel == 3'd1*/i_iaddress_sel==4'd1            ? /*alu_out*/pc+i_imm32                 : //we could do this in the ALU instead, but it'd be hideously complex and inefficient to do so
                /*i_pc_sel == 3'd2*/i_iaddress_sel==4'd2            ? interrupt_vector        :
                                              pc-'d4                  ;
logic pc_wait_for_tag;
assign pc_wait_for_tag = (i_iaddress_sel==4'd3);
logic reg_bank_pc_valid_curr;

b01_register_bank u_register_bank( //TODO revise so it works with the proposed/in-progress memory instruction handling
    .i_clk                   ( i_clk                     ),
	.i_rst(i_rst),
    .i_core_stall            ( i_core_stall              ),
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
    .i_rd_sel                ( i_rd_sel                  ),
    .i_pc_wen                ( i_pc_wen & ~o_exec_stall                    ), //TODO confirm/fix
    .i_reg_bank_wen          ( {{15{condition_execute/* & ~o_exec_stall*/}} & i_reg_bank_wen}              ), //TODO confirm/fix, esp. wrt stall logic and predicated execution
    .i_pc                    ( pc_nxt[25:2]/*pc[25:2]+24'd1*/              ), //TODO confirm/fix
    .i_pc_wait_for_tag(pc_wait_for_tag),
	
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
	
	//TODO create the set of reg_bank_xyz_valid/tag signals and hook up correctly
	.o_rm_valid		(reg_bank_rm_valid),
	.o_rs_valid		(reg_bank_rs_valid),
	.o_rd_valid		(reg_bank_rd_valid),
	.o_rn_valid		(reg_bank_rn_valid),
	.o_pc_valid		(reg_bank_pc_valid),
	.o_pc_valid_curr (reg_bank_pc_valid_curr), //to the tag store
	.o_rm_tag		(reg_bank_rm_tag),
	.o_rs_tag		(reg_bank_rs_tag),
	.o_rd_tag		(reg_bank_rd_tag),
	.o_rn_tag		(reg_bank_rn_tag),
	.o_pc_tag		(reg_bank_pc_tag),
	
	.i_rd_tag		((i_is_memop & ~i_op_type_mem[0]) ? tag_nxt_mem : tag_nxt_alu_mult_1), //use load op's rd and tag if it's a memory ld or swap
	.i_mem_wb       (i_is_memop & i_address_mode[1]), //need to invalidate a 2nd register, with a different tag, if it's a pre- or post-indexed memop
	//note that the register we invalidate if it's a pre-/post-indexed memop is always rn
    .i_mem_wb_tag   (tag_nxt_alu_mult_1), //for pre-/post-indexed memops, this is the tag we use
    
    .i_is_psr(i_is_psr),
    .i_mrs_msr(i_mrs_msr),
    .i_psr_sel(i_psr_reg),
    .i_psr_reg_data(reg_bank_psr_data_in), //TODO set up this signal
    //note: stalling is irrelevant here; since we stall the whole core on psr with invalid condition, we'll just keep updating whatever this reg is with the input psr data until it becomes *actually* valid and the core unstalls
    .o_psr_reg_valid(reg_bank_psr_reg_valid),
    .o_psr_reg(reg_bank_psr_reg_data),
    
    .led(led),
    .sw(sw)
);


//PC and instruction address logic

//recall we have pc, reg_bank_pc_valid, and reg_bank_pc_tag. we also need to deal with i_core_stall.
logic pc_stall;
//we must stall if we're getting the PC from anything other than pc+4 or a predefined interrupt vector
assign pc_stall = (~reg_bank_pc_valid) ;//| (i_iaddress_sel != 3'd0 && i_iaddress_sel != 3'd2); //note: TODO WARNING: changed from i_pc_sel to i_iaddress_sel; this *should* work but might not!
/*assign o_iaddress_nxt =	(~reg_bank_pc_valid)		? pc				:
						(i_iaddress_sel == 4'd0)	? pc+32'd4				:
						(i_iaddress_sel == 4'd2)	? interrupt_vector	:
													  pc+32'd4; //default, just in case*/
assign o_iaddress_nxt = pc/*_nxt*/;//tmp, for stupid-testing //TODO TODO TODO added _nxt
assign o_iaddress_valid = /*1'b1*/~o_exec_stall && ~pc_wait_for_tag/* && ~o_branch_taken && i_iaddress_sel!=4'd2*/; //don't fetch new if we're stalled, we're branching (iaddress breaks) or we're interrupting (iaddress breaks, for the same reason)
always_ff @(posedge i_rst, posedge i_clk) begin
	if (i_rst) begin
		//o_iaddress_valid <= 'd0;
		o_iaddress <= 32'd0; //note that on reset, at the next clock edge PC will have been initialized and set by the register file
	end
	else if (i_clk) begin
		if (!i_core_stall && !o_exec_stall) begin
			//o_iaddress_valid 	<= 	!/*pc_stall*/o_exec_stall;
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
assign instr_type = 	i_iaddress_sel!=4'd0 && i_reg_bank_wen!=15'd0 ? 2'b11: //if control flow instruction, we don't want to do anything with the reservation stations unless we're also saving the link register
                        i_multiply_function[0] ? 2'b01:
						i_is_memop ? 2'b10:
									 2'b00;
logic tag_stall;
assign o_exec_stall = condition_stall | tag_stall | pc_stall; //TODO confirm/edit
									 
//Tag store instantiation
b01_tagstore u_tagstore( //TODO revise to generate sufficient tags simultaneously for an arbitrarily large LDM or STM (or else to say "tags are ready, here's the first one to use" or "here's a bit vector showing which tags are to be used for this instruction")
	.i_clk(i_clk),
	.i_rst(i_rst),
	.i_stall(i_core_stall | ~condition_execute /**/| pc_stall /**/| condition_stall), //TODO confirm functionality wrt basis on i_core_stall/fix if needed. Also ensure this works right with predicated execution.
	.i_pc_stall(/*pc_stall*/~reg_bank_pc_valid_curr),
	.i_pc_stall_nxt(pc_stall),
	.i_is_psr(i_is_psr),
	.i_instr_type(instr_type), //00=alu, 01=mult, 10=mem. TODO: `define these in a header file
	.i_branch_with_link(instr_type==2'b11 && i_reg_bank_wen!='d0),
	//.i_instr_mem_with_writeback(/*TODO*/),
	.i_alu_valid(i_alu_valid),
	.i_alu_tag(i_alu_tag),
	.i_mult_valid(i_mult_valid),
	.i_mult_tag(i_mult_tag),
	.i_mem_valid(i_mem_valid),
	.i_mem_tag(i_mem_tag),
	.o_station_full(tag_stall),
	.o_tag_alu_mult_1(tag_nxt_alu_mult_1),
	//.o_tag_alu_2(tag_nxt_alu_2),
	.o_tag_mem(tag_nxt_mem)
);

//Reservation station instantiation
b01_reservation u_reservation(//TODO revise to properly accept reserving multiple instructions at once (for superscalar)
	.i_clk(i_clk),
	.i_rst(i_rst),
	.i_stall(/*TODO confirm proper operation*//*o_exec_stall*/i_core_stall | ~condition_execute | pc_stall | condition_stall), //TODO confirm functionality wrt basis on i_core_stall/fix if needed. Also ensure this works right with predicated execution.
	.i_instr_type(instr_type),
	
	.i_alu_mult_tag1_nxt(tag_nxt_alu_mult_1),
	.i_mem_tag_nxt(tag_nxt_mem),
	.i_imm32(i_imm32),
	.i_imm_shift_amount(i_imm_shift_amount),
	.i_shift_imm_zero(i_shift_imm_zero),
	.i_barrel_shift_amount_sel(i_barrel_shift_amount_sel),
	.i_barrel_shift_data_sel(i_barrel_shift_data_sel),
	.i_barrel_shift_function(i_barrel_shift_function),
	.i_alu_function(i_alu_function),
	.i_multiply_function(i_multiply_function[1]), //at this point, we only care about saving the is-or-isn't-accumulate bit in the reservation station
	.i_byte_enable_sel(i_byte_enable_sel),
	.i_use_carry_in(i_use_carry_in),
	//.i_write_data_wen(i_write_data_wen),
	//.i_base_address_wen(/*i_base_address_wen*/), //TODO remove!!!
	//.i_reg_bank_wen(i_reg_bank_wen),
	.i_op_type_mem(i_op_type_mem),
	.i_address_mode(i_address_mode),
	.i_rn_valid(reg_bank_rn_valid | !i_use_rn), //"or" accounts for case where operand is unused (ditto rs, rm)
	.i_rn_tag(reg_bank_rn_tag),
	.i_rn(rn),
	.i_rs_valid(reg_bank_rs_valid | !i_use_rs),
	.i_rs_tag(reg_bank_rs_tag),
	.i_rs(rs),
	.i_rm_valid(reg_bank_rm_valid | !i_use_rm),
	.i_rm_tag(reg_bank_rm_tag),
	.i_rm(rm),
	.i_rd_valid(reg_bank_rd_valid), //no i_use_rd since the data in this reg is only ever used for mem swaps or writes, and we case on the V bit inside the reservation station instead
	.i_rd_tag(reg_bank_rd_tag),
	.i_rd(rd),
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
	.o_rd_tag_mult(o_rd_tag_mult),
	.i_mult_valid(i_mult_valid),
	.i_mult_tag(i_mult_tag),
	.i_mult_data(i_mult_data),
	
	.i_mem_ready(i_mem_ready),
	.o_instr_valid_mem(o_instr_valid_mem),
	.o_address_mem(o_address_mem),
	.o_write_data_mem(o_write_data_mem),
	.o_op_type_mem(o_op_type_mem),
	.o_byte_enable_sel_mem(o_byte_enable_sel_mem),
	.o_rd_tag_mem(o_rd_tag_mem),
	.i_mem_valid(i_mem_valid),
	.i_mem_tag(i_mem_tag),
	.i_mem_data(i_mem_data)
);


endmodule
