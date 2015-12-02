`include "global_defines.vh"

module b01_decode (

	input logic i_clk,
	input logic i_rst,
	
	input logic [31:0] i_fetch_instruction,
	input logic i_core_stall,
	input logic i_irq,
	input logic i_firq,
	input logic i_adex,
	input logic [31:0] i_execute_status_bits,
	input logic i_branch_taken,
	
	output logic [31:0] o_imm32,
	output logic [4:0] o_imm_shift_amount,
	output logic o_shift_imm_zero,
	output logic [3:0] o_condition, //TODO in original, this is a wire, not a reg... is it registered elsewhere?
	output logic o_is_swap, //TODO in original, is o_decode_exclusive
	output logic o_is_memop, //TODO in original, is o_decode_daccess
	output logic [1:0] o_status_bits_mode,
	output logic o_status_bits_irq_mask,
	output logic o_status_bits_firq_mask,
	
	output logic [3:0] o_rm_sel,
	output logic [3:0] o_rs_sel,
	output logic [3:0] o_rn_sel,
	//output logic [7:0] o_load_rd, //TODO this may matter later for LDM, but we don't care for now
	output logic [1:0] o_barrel_shift_amount_sel,
	output logic [1:0] o_barrel_shift_data_sel,
	output logic [1:0] o_barrel_shift_function,
	output logic [8:0] o_alu_function,
	output logic [1:0] o_multiply_function,
	output logic [2:0] o_interrupt_vector_sel,
	output logic [3:0] o_iaddress_sel,
	output logic [3:0] o_daddress_sel,
	output logic [2:0] o_pc_sel,
	output logic [1:0] o_byte_enable_sel,
	output logic [2:0] o_status_bits_sel, //this will tell the status register whether or not it should invalidate itself
	//output logic [2:0] o_reg_write_sel, //note that this must be replaced with something else for if we have an interrupt and need to write to the link register
	//output logic o_user_mode_regs_store_nxt, //only matters for old STM implementation
	//output logic o_firq_not_user_mode, //don't care now that we've redone how the register bank checks this stuff
	output logic o_use_carry_in, //critical for many arithmetic ops
	
	output logic o_write_data_wen, //define if this is a ld or st op if it's a memop
	//output logic o_base_address_wen, //only relevant for LDM/STM (MTRANS) instructions
	
	output logic o_pc_wen, //might matter in the future depending on how we implement LDM/STM, but for now we can just tie it to "1"
	output logic [14:0] o_reg_bank_wen, //critical. duh.
	output logic o_status_bits_flags_wen, //critical
	output logic o_status_bits_mode_wen, //critical for interrupt handling, doesn't matter as much for now
	output logic o_status_bits_irq_mask_wen, //ditto
	output logic o_status_bits_firq_mask_wen, //ditto
	
	//Removed coprocessor interface because we don't give a crap, except for o_rn_valid (now o_use_rn) etc.
	output logic o_use_rn,
	output logic o_use_rm,
	output logic o_use_rs,
	//output logic o_use_rd, //probably not needed
	output logic o_use_sr //whether or not this instruction must look at the status bits

);

`include "a25_localparams.vh"
`include "a25_functions.vh"

//removed localparam for the State Machine From Hell

logic [31:0] instruction;
logic [3:0] instr_type;
//logic [1:0] instruction_sel; //don't care about this. let's lock it to 2'b00.
logic [3:0] opcode;
logic [7:0] imm8;
logic [31:0] offset12;
logic [31:0] offset24;
logic [4:0] shift_imm;

logic opcode_compare;
logic mem_op;
logic store_op;
//logic write_pc; //TODO not needed if we flush instead of stalling on a pc-writing instruction
//logic current_write_pc; //TODO not needed if we flush instead of stalling on a pc-writing instruction
logic load_pc_nxt; //used to define if a memop will be used to load the pc; TODO revise
logic load_pc_r; //ditto
logic immediate_shift_op; //critical
logic rds_use_rs; //pretty sure this is needed, but need to think harder about exactly what it does
logic branch; //critical
logic mem_op_pre_indexed; //critical
logic mem_op_post_indexed; //critical

logic [31:0] imm32_nxt; //can probably remove and replace with latching directly to o_imm32
logic [4:0] imm_shift_amount_nxt; //ditto
logic shift_imm_zero_nxt; //ditto
logic [3:0] condition_nxt; //ditto
logic is_swap_nxt; //ditto; TODO note: changed from decode_exclusive_nxt!
logic is_memop_nxt; //ditto; TODO in original, is decode_daccess_nxt
logic shift_extend;

logic [1:0] barrel_shift_function_nxt; //ditto
logic [8:0] alu_function_nxt; //ditto
logic [1:0] multiply_function_nxt; //ditto
logic [1:0] status_bits_mode_nxt; //ditto
logic status_bits_irq_mask_nxt; //ditto
logic status_bits_firq_mask_nxt; //ditto

logic [3:0] rm_sel_nxt; //ditto
logic [3:0] rs_sel_nxt; //ditto
logic [3:0] rn_sel_nxt; //ditto

logic [1:0] barrel_shift_amount_sel_nxt; //ditto
logic [1:0] barrel_shift_data_sel_nxt; //ditto
logic [3:0] iaddress_sel_nxt; //ditto
logic [3:0] daddress_sel_nxt; //ditto
logic [2:0] pc_sel_nxt; //ditto
logic [1:0] byte_enable_sel_nxt; //ditto
logic [2:0] status_bits_sel_nxt; //ditto
//logic [2:0] reg_write_sel_nxt; //ditto; TODO also refer to the note above about interrupts and link register things
//logic firq_not_user_mode_nxt;
logic use_carry_in_nxt; //ditto*/

//Internal ALU-function signals
logic alu_swap_sel_nxt;
logic alu_not_sel_nxt;
logic [1:0] alu_cin_sel_nxt;
logic alu_cout_sel_nxt;
logic [3:0] alu_out_sel_nxt;

logic write_data_wen_nxt; //ditto
//removed copro_write_data_wen_nxt
//removed base_address_wen_nxt
logic pc_wen_nxt; //ditto
logic [14:0] reg_bank_wen_nxt; //ditto
logic status_bits_flags_wen_nxt; //ditto
logic status_bits_mode_wen_nxt; //ditto
logic status_bits_irq_mask_wen_nxt; //ditto
logic status_bits_firq_mask_wen_nxt; //ditto*/

//removed saved_current_instruction_wen
//removed pre_fetch_instruction_wen
//removed control_state
//removed control_state_nxt
//removed dabt, iabt, pre_fetch, hold_instruction and all related
//logic adex_reg; //might need this, but *probably* not
//logic [31:0] fetch_address_r; //probably safe to remove
//logic [31:0] fetch_instruction_r; //same
//logic [3:0] fetch_instruction_type_r; //same

//logic instruction_valid; //should be ok to remove; formerly for multicycle instructions/weird-state-machine stuff
//logic instruction_execute; //same
//logic instruction_execute_r; //same

//removed mtrans_reg1, mtrans_reg2, mtrans_instruction_nxt, mtrans_reg2_mask, mtrans_base_reg_change, mtrans_num_registers, etc.. will probably need to re-add some later.
//removed use_saved_current/hold/pre_fetch_instruction

logic interrupt;
logic interrupt_or_conflict; //may not truly be needed, but left in for now just in case
logic [1:0] interrupt_mode;
logic [2:0] next_interrupt;
logic irq;
logic firq;
logic firq_request;
logic irq_request;
logic swi_request;
//logic und_request; //undefined instruction interrupt request; formerly hooked up to the coprocessor crap
//logic dabt_request;
//removed copro_operation_nxt
//removed restore_base_address and restore_base_address_nxt (formerly for MTRANS (ldm/stm) ops)

logic regop_set_flags;

//logic load_rd_nxt; //not needed for immediate purposes; TODO revise later MTRANS implementation
//logic load_rd_byte; //not needed for immediate purposes; TODO revise later MTRANS and SWP implementation
//removed ldm_user_mode, ldm_status_bits, and ldm_flags (for now, at least)
//logic load_rd_d1_nxt; //for previous memory load implementation. no longer needed b/c of OOO stuff.
//logic laod_rd_d1; //same

//We can safely remove all register conflict detection signals now that we have *proper* OOO execution
//This includes xx_valid, xx_conflict1, xx_conflict2, conflict1, conflict2, conflict, and <xyz>_conflict_r

/*logic [3:0] condition_r; //should be ok to remove, just latch the value to o_condition instead
//logic decode_iaccess_r; //ditto (but we also don't really care about decode_iaccess at all anymore)
logic [1:0] status_bits_mode_r; //ditto ^^2x
logic status_bits_irq_mask_r; //ditto
logic status_bits_firq_mask_r; //ditto
logic [3:0] iaddress_sel_r; //ditto
logic [3:0] daddress_sel_r; //ditto
logic [2:0] pc_sel_r; //ditto
logic pc_wen_r; //ditto*/

//TODO don't forget to hook up o_use_rn/rs/rm in the following logic!

/*****************************************************************************/
/* THE ACTUAL SIGNALS ********************************************************/
/*****************************************************************************/

assign instruction = i_fetch_instruction; //TODO need to register this now inside the Fetch stage, AS IT FRIGGING SHOULD HAVE DONE BEFORE
assign instr_type = instruction_type(i_fetch_instruction);

//Fixed fields inside the instruction
assign opcode = instruction[24:21];
assign condition_nxt = instruction[31:28];
assign rm_sel_nxt = instruction[3:0];
assign rn_sel_nxt = branch ? 4'd15 : instruction[19:16];
assign rs_sel_nxt = branch 		? 4'd15 : 
					rds_use_rs 	? instruction[11:8]:
								  instruction[15:12];
assign shift_extend = ~instruction[25] & ~instruction[4] & ~(|instruction[11:7]) & (instruction[6:5]==2'b11);
assign shift_imm = instruction[11:7];
assign offset12 = {20'h0, instruction[11:0]};
assign offset24 = {{6{instruction[23]}}, instruction[23:0], 2'd0}; //sign-extend
assign imm8 = instruction[7:0];
assign immediate_shift_op = instruction[25];
assign rds_use_rs = (instr_type==REGOP && ~instruction[25] && instruction[4]) || (instr_type==MULT); //TODO confirm change will work
assign branch = (instr_type==BRANCH);
assign opcode_compare = (opcode==CMP || opcode==CMN || opcode==TEQ || opcode==TST);
assign mem_op = (instr_type==TRANS);
assign load_op = (mem_op && instruction[20]);
assign store_op = (mem_op && instruction[20]);
//skipping write_pc and current_write_pc
assign regop_set_flags = (instr_type==REGOP && instruction[20]);
assign mem_op_pre_indexed = instruction[24] && instruction[21];
assign mem_op_post_indexed = !instruction[24];
assign imm32_nxt = 	instr_type==MULT		   ? 32'd0:
					instr_type==BRANCH		   ? offset24:
					instr_type==TRANS		   ? offset12:
					instruction[11:8] == 4'h0  ? {            24'h0, imm8[7:0] } :
					instruction[11:8] == 4'h1  ? { imm8[1:0], 24'h0, imm8[7:2] } :
					instruction[11:8] == 4'h2  ? { imm8[3:0], 24'h0, imm8[7:4] } :
					instruction[11:8] == 4'h3  ? { imm8[5:0], 24'h0, imm8[7:6] } :
					instruction[11:8] == 4'h4  ? { imm8[7:0], 24'h0            } :
					instruction[11:8] == 4'h5  ? { 2'h0,  imm8[7:0], 22'h0     } :
					instruction[11:8] == 4'h6  ? { 4'h0,  imm8[7:0], 20'h0     } :
					instruction[11:8] == 4'h7  ? { 6'h0,  imm8[7:0], 18'h0     } :
					instruction[11:8] == 4'h8  ? { 8'h0,  imm8[7:0], 16'h0     } :
					instruction[11:8] == 4'h9  ? { 10'h0, imm8[7:0], 14'h0     } :
					instruction[11:8] == 4'ha  ? { 12'h0, imm8[7:0], 12'h0     } :
					instruction[11:8] == 4'hb  ? { 14'h0, imm8[7:0], 10'h0     } :
					instruction[11:8] == 4'hc  ? { 16'h0, imm8[7:0], 8'h0      } :
					instruction[11:8] == 4'hd  ? { 18'h0, imm8[7:0], 6'h0      } :
					instruction[11:8] == 4'he  ? { 20'h0, imm8[7:0], 4'h0      } :
												 { 22'h0, imm8[7:0], 2'h0      } ;

assign imm_shift_amount_nxt = shift_imm;
assign shift_imm_zero_nxt = imm_shift_amount_nxt == 5'd0 && barrel_shift_amount_sel_nxt == 2'd2;
assign alu_function_nxt = {	alu_swap_sel_nxt,
							alu_not_sel_nxt,
							alu_cin_sel_nxt,
							alu_cout_sel_nxt,
							alu_out_sel_nxt	};

//TODO change these to just latch the values into the outputs directly!
assign use_rn_nxt = instr_type==REGOP ||
					instr_type==MULT ||
					instr_type==SWAP ||
					instr_type==TRANS; //TODO add back MTRANS eventually
assign use_rm_nxt = instr_type==REGOP ||
					instr_type==MULT ||
					instr_type==SWAP ||
					(instr_type==TRANS && immediate_shift_op);
assign use_rs_nxt = rds_use_rs;
assign use_sr_nxt = (condition_nxt != AL);


//Interrupts
assign firq_request = firq && !i_execute_status_bits[26]; //TODO should stall fetch on i_firq or i_irq and then jump to the handler in the next cycle as the next instruction
assign irq_request  = irq  && !i_execute_status_bits[27];
assign swi_request  = instr_type == SWI;

// in order of priority !!
// Highest
// 1 Reset
// 2 Data Abort (including data TLB miss) //no longer used
// 3 FIRQ
// 4 IRQ
// 5 Prefetch Abort (including prefetch TLB miss) //no longer used
// 6 Undefined instruction /*(no longer used)*/, SWI
// Lowest
assign next_interrupt = /*dabt_request     ? 3'd1 :*/  // Data Abort
                        firq_request     ? 3'd2 :  // FIRQ
                        irq_request      ? 3'd3 :  // IRQ
                        /*instruction_adex ? 3'd4 :  // Address Exception
                        instruction_iabt ? 3'd5 :  // PreFetch Abort, only triggered
                                                   // if the instruction is used
                        und_request      ? 3'd6 : */ // Undefined Instruction
                        swi_request      ? 3'd7 :  // SWI
                                           3'd0 ;  // none


// SWI and undefined instructions do not cause an interrupt in the decode
// stage. They only trigger interrupts if they arfe executed, so the
// interrupt is triggered if the execute condition is met in the execute stage
assign interrupt      = next_interrupt != 3'd0 &&
                        next_interrupt != 3'd7 /*&&  // SWI
                        next_interrupt != 3'd6 &&  // undefined interrupt
                        !conflict*/               ;  // Wait for conflicts to resolve before
                                                   // triggering int

assign interrupt_mode = next_interrupt == 3'd2 ? FIRQ :
                        next_interrupt == 3'd3 ? IRQ  :
                        next_interrupt == 3'd4 ? SVC  :
                        next_interrupt == 3'd5 ? SVC  :
                        next_interrupt == 3'd6 ? SVC  :
                        next_interrupt == 3'd7 ? SVC  :
                        next_interrupt == 3'd1 ? SVC  :
                                                 USR  ;


//Control signals
always_comb
    begin
    // default mode
    status_bits_mode_nxt            = i_execute_status_bits[1:0];   // change to mode in execute stage get reflected
                                                                    // back to this stage automatically
    status_bits_irq_mask_nxt        = o_status_bits_irq_mask;
    status_bits_firq_mask_nxt       = o_status_bits_firq_mask;
    is_swap_nxt            = 1'd0;
    is_memop_nxt              = 1'd0;
    //decode_iaccess_nxt              = 1'd1;
    //copro_operation_nxt             = 'd0;

    // Save an instruction to use later
    /*saved_current_instruction_wen   = 1'd0;
    pre_fetch_instruction_wen       = 1'd0;
    restore_base_address_nxt        = restore_base_address;*/

    // default Mux Select values
    barrel_shift_amount_sel_nxt     = 'd0;  // don't shift the input
    barrel_shift_data_sel_nxt       = 'd0;  // immediate value
    barrel_shift_function_nxt       = 'd0;
    use_carry_in_nxt                = 'd0;
    multiply_function_nxt           = 'd0;
    iaddress_sel_nxt                = 'd0;
    daddress_sel_nxt                = 'd0;
    pc_sel_nxt                      = 'd0;
    load_pc_nxt                     = 'd0;
    byte_enable_sel_nxt             = 'd0;
    status_bits_sel_nxt             = 'd0;
    //reg_write_sel_nxt               = 'd0;
    //o_user_mode_regs_store_nxt      = 'd0;

    // ALU Muxes
    alu_swap_sel_nxt                = 'd0;
    alu_not_sel_nxt                 = 'd0;
    alu_cin_sel_nxt                 = 'd0;
    alu_cout_sel_nxt                = 'd0;
    alu_out_sel_nxt                 = 'd0;

    // default Flop Write Enable values
    write_data_wen_nxt              = 'd0;
    //copro_write_data_wen_nxt        = 'd0;
    //base_address_wen_nxt            = 'd0;
    pc_wen_nxt                      = 'd1;
    reg_bank_wen_nxt                = 'd0;  // Don't select any

    status_bits_flags_wen_nxt       = 'd0;
    status_bits_mode_wen_nxt        = 'd0;
    status_bits_irq_mask_wen_nxt    = 'd0;
    status_bits_firq_mask_wen_nxt   = 'd0;

    if ( /*instruction_valid &&*/ !interrupt /*&& !conflict*/ )
        begin
        if ( instr_type == REGOP )
            begin
            if ( !opcode_compare )
                begin
                // Check is the load destination is the PC
                if (instruction[15:12]  == 4'd15)
                    begin
                    pc_sel_nxt       = 3'd1; // alu_out
                    iaddress_sel_nxt = 4'd1; // alu_out
                    end
                else
                    reg_bank_wen_nxt = decode (instruction[15:12]);
                end

            if ( !immediate_shift_op )
                begin
                barrel_shift_function_nxt  = instruction[6:5];
                end

            if ( !immediate_shift_op )
                barrel_shift_data_sel_nxt = 2'd2; // Shift value from Rm register

            if ( !immediate_shift_op && instruction[4] )
                barrel_shift_amount_sel_nxt = 2'd1; // Shift amount from Rs registter

            if ( !immediate_shift_op && !instruction[4] )
                barrel_shift_amount_sel_nxt = 2'd2; // Shift immediate amount

            // regops that do not change the overflow flag
            if ( opcode == AND || opcode == EOR || opcode == TST || opcode == TEQ ||
                 opcode == ORR || opcode == MOV || opcode == BIC || opcode == MVN )
                status_bits_sel_nxt = 3'd5;

            if ( opcode == ADD || opcode == CMN )   // CMN is just like an ADD
                begin
                alu_out_sel_nxt  = 4'd1; // Add
                use_carry_in_nxt = shift_extend;
                end

            if ( opcode == ADC ) // Add with Carry
                begin
                alu_out_sel_nxt  = 4'd1; // Add
                alu_cin_sel_nxt  = 2'd2; // carry in from status_bits
                use_carry_in_nxt = shift_extend;
                end

            if ( opcode == SUB || opcode == CMP ) // Subtract
                begin
                alu_out_sel_nxt  = 4'd1; // Add
                alu_cin_sel_nxt  = 2'd1; // cin = 1
                alu_not_sel_nxt  = 1'd1; // invert B
                end

            // SBC (Subtract with Carry) subtracts the value of its
            // second operand and the value of NOT(Carry flag) from
            // the value of its first operand.
            //  Rd = Rn - shifter_operand - NOT(C Flag)
            if ( opcode == SBC ) // Subtract with Carry
                begin
                alu_out_sel_nxt  = 4'd1; // Add
                alu_cin_sel_nxt  = 2'd2; // carry in from status_bits
                alu_not_sel_nxt  = 1'd1; // invert B
                use_carry_in_nxt = 1'd1;
                end

            if ( opcode == RSB ) // Reverse Subtract
                begin
                alu_out_sel_nxt  = 4'd1; // Add
                alu_cin_sel_nxt  = 2'd1; // cin = 1
                alu_not_sel_nxt  = 1'd1; // invert B
                alu_swap_sel_nxt = 1'd1; // swap A and B
                use_carry_in_nxt = 1'd1;
                end

            if ( opcode == RSC ) // Reverse Subtract with carry
                begin
                alu_out_sel_nxt  = 4'd1; // Add
                alu_cin_sel_nxt  = 2'd2; // carry in from status_bits
                alu_not_sel_nxt  = 1'd1; // invert B
                alu_swap_sel_nxt = 1'd1; // swap A and B
                use_carry_in_nxt = 1'd1;
                end

            if ( opcode == AND || opcode == TST ) // Logical AND, Test  (using AND operator)
                begin
                alu_out_sel_nxt  = 4'd8;  // AND
                alu_cout_sel_nxt = 1'd1;  // i_barrel_shift_carry
                end

            if ( opcode == EOR || opcode == TEQ ) // Logical Exclusive OR, Test Equivalence (using EOR operator)
                begin
                alu_out_sel_nxt = 4'd6;  // XOR
                alu_cout_sel_nxt = 1'd1; // i_barrel_shift_carry
                use_carry_in_nxt = 1'd1;
                end

            if ( opcode == ORR )
                begin
                alu_out_sel_nxt  = 4'd7; // OR
                alu_cout_sel_nxt = 1'd1;  // i_barrel_shift_carry
                use_carry_in_nxt = 1'd1;
                end

            if ( opcode == BIC ) // Bit Clear (using AND & NOT operators)
                begin
                alu_out_sel_nxt  = 4'd8;  // AND
                alu_not_sel_nxt  = 1'd1;  // invert B
                alu_cout_sel_nxt = 1'd1;  // i_barrel_shift_carry
                use_carry_in_nxt = 1'd1;
               end

            if ( opcode == MOV ) // Move
                begin
                alu_cout_sel_nxt = 1'd1;  // i_barrel_shift_carry
                use_carry_in_nxt = 1'd1;
                end

            if ( opcode == MVN ) // Move NOT
                begin
                alu_not_sel_nxt  = 1'd1; // invert B
                alu_cout_sel_nxt = 1'd1;  // i_barrel_shift_carry
                use_carry_in_nxt = 1'd1;
               end
            end

        // Load & Store instructions
		//Removed for now because of crappy memory handling
        /*if ( mem_op )
            begin
            if ( load_op && instruction[15:12]  == 4'd15 ) // Write to PC
                begin
                saved_current_instruction_wen   = 1'd1; // Save the memory access instruction to refer back to later
                pc_wen_nxt                      = 1'd0; // hold current PC value rather than an instruction fetch
                load_pc_nxt                     = 1'd1;
                end

            decode_daccess_nxt              = 1'd1; // indicate a valid data access
            alu_out_sel_nxt                 = 4'd1; // Add

            if ( !instruction[23] )  // U: Subtract offset
                begin
                alu_cin_sel_nxt  = 2'd1; // cin = 1
                alu_not_sel_nxt  = 1'd1; // invert B
                end

            if ( store_op )
                begin
                write_data_wen_nxt = 1'd1;
                if ( type == TRANS && instruction[22] )
                    byte_enable_sel_nxt = 2'd1;         // Save byte
                end

                // need to update the register holding the address ?
                // This is Rn bits [19:16]
            if ( mem_op_pre_indexed || mem_op_post_indexed )
                begin
                // Check is the load destination is the PC
                if ( rn_sel_nxt  == 4'd15 )
                    pc_sel_nxt = 3'd1;
                else
                    reg_bank_wen_nxt = decode ( rn_sel_nxt );
                end

                // if post-indexed, then use Rn rather than ALU output, as address
            if ( mem_op_post_indexed )
               daddress_sel_nxt = 4'd4; // Rn
            else
               daddress_sel_nxt = 4'd1; // alu out

            if ( instruction[25] && type ==  TRANS )
                barrel_shift_data_sel_nxt = 2'd2; // Shift value from Rm register

            if ( type == TRANS && instruction[25] && shift_imm != 5'd0 )
                begin
                barrel_shift_function_nxt   = instruction[6:5];
                barrel_shift_amount_sel_nxt = 2'd2; // imm_shift_amount
                end
            end*/


        if ( instr_type == BRANCH )
            begin
            pc_sel_nxt            = 3'd1; // alu_out
            iaddress_sel_nxt      = 4'd1; // alu_out
            alu_out_sel_nxt       = 4'd1; // Add
			
			//branch is to *

            if ( instruction[24] ) // Link
                begin
				
				//TODO make this an ALU op with source reg == PC (i.e. *actually* the addr of the next instr after this one) and dest reg == r14 and op == nop
				
                reg_bank_wen_nxt  = decode (4'd14);  // Save PC to LR
                //reg_write_sel_nxt = 3'd1;            // pc - 32'd4 //TODO need to fix!!!
                end
            end


        /*if ( type == MTRANS ) //TODO re-insert with heavy revisions
            begin
            saved_current_instruction_wen   = 1'd1; // Save the memory access instruction to refer back to later
            decode_daccess_nxt              = 1'd1; // valid data access
            alu_out_sel_nxt                 = 4'd1; // Add
            base_address_wen_nxt            = 1'd1; // Save the value of the register used for the base address,
                                                    // in case of a data abort, and need to restore the value

            if ( mtrans_num_registers > 4'd1 )
                begin
                iaddress_sel_nxt        = 4'd3; // pc  (not pc + 4)
                pc_wen_nxt              = 1'd0; // hold current PC value rather than an instruction fetch
                end


            // The spec says -
            // If the instruction would have overwritten the base with data
            // (that is, it has the base in the transfer list), the overwriting is prevented.
            // This is true even when the abort occurs after the base word gets loaded
            restore_base_address_nxt        = instruction[20] &&
                                                (instruction[15:0] & (1'd1 << instruction[19:16]));

            // Increment
            if ( instruction[23] )
                begin
                if ( instruction[24] )    // increment before
                    daddress_sel_nxt = 4'd7; // Rn + 4
                else
                    daddress_sel_nxt = 4'd4; // Rn
                end
            else
            // Decrement
                begin
                alu_cin_sel_nxt  = 2'd1; // cin = 1
                alu_not_sel_nxt  = 1'd1; // invert B
                if ( !instruction[24] )    // decrement after
                    daddress_sel_nxt  = 4'd6; // alu out + 4
                else
                    daddress_sel_nxt  = 4'd1; // alu out
                end

            // Load or store ?
            if ( !instruction[20] )  // Store
                write_data_wen_nxt = 1'd1;

            // stm: store the user mode registers, when in priviledged mode
            if ( {instruction[22],instruction[20]} == 2'b10 )
                o_user_mode_regs_store_nxt = 1'd1;

            // update the base register ?
            if ( instruction[21] )  // the W bit
                reg_bank_wen_nxt  = decode (rn_sel_nxt);

            // write to the pc ?
            if ( instruction[20] && mtrans_reg1 == 4'd15 ) // Write to PC
                begin
                saved_current_instruction_wen   = 1'd1; // Save the memory access instruction to refer back to later
                pc_wen_nxt                      = 1'd0; // hold current PC value rather than an instruction fetch
                load_pc_nxt                     = 1'd1;
                end
            end*/


        if ( instr_type == MULT )
            begin
            multiply_function_nxt[0]        = 1'd1; // set enable
                                                    // some bits can be changed just below
            //saved_current_instruction_wen   = 1'd1; // Save the Multiply instruction to
                                                    // refer back to later
            pc_wen_nxt                      = 1'd0; // hold current PC value

            if ( instruction[21] )
                multiply_function_nxt[1]    = 1'd1; // accumulate
            end


        // swp - do read part first
        /*if ( type == SWAP ) //TODO add back with memory
            begin
            saved_current_instruction_wen   = 1'd1; // Save the memory access instruction to refer back to later
            pc_wen_nxt                      = 1'd0; // hold current PC value
            decode_iaccess_nxt              = 1'd0; // skip the instruction fetch
            decode_daccess_nxt              = 1'd1; // data access
            barrel_shift_data_sel_nxt       = 2'd2; // Shift value from Rm register
            daddress_sel_nxt                = 4'd4; // Rn
            decode_exclusive_nxt            = 1'd1; // signal an exclusive access
            end*/


        if ( instr_type == SWI /*|| und_request*/ ) //TODO rework link-register handling
            begin
            // save address of next instruction to Supervisor Mode LR
            //reg_write_sel_nxt               = 3'd1;            // pc -4
            reg_bank_wen_nxt                = decode (4'd14);  // LR

            iaddress_sel_nxt                = 4'd2;            // interrupt_vector
            pc_sel_nxt                      = 3'd2;            // interrupt_vector

            status_bits_mode_nxt            = interrupt_mode;  // e.g. Supervisor mode
            status_bits_mode_wen_nxt        = 1'd1;

            // disable normal interrupts
            status_bits_irq_mask_nxt        = 1'd1;
            status_bits_irq_mask_wen_nxt    = 1'd1;
            end


        if ( regop_set_flags )
            begin
            status_bits_flags_wen_nxt = 1'd1;

            // If <Rd> is r15, the ALU output is copied to the Status Bits.
            // Not allowed to use r15 for mul or lma instructions
            if ( instruction[15:12] == 4'd15 )
                begin
                status_bits_sel_nxt       = 3'd1; // alu out

                // Priviledged mode? Then also update the other status bits
                if ( i_execute_status_bits[1:0] != USR )
                    begin
                    status_bits_mode_wen_nxt      = 1'd1;
                    status_bits_irq_mask_wen_nxt  = 1'd1;
                    status_bits_firq_mask_wen_nxt = 1'd1;
                    end
                end
            end

        end

    // Handle asynchronous interrupts.
    // interrupts are processed only during execution states
    // multicycle instructions must complete before the interrupt starts
    // SWI, Address Exception and Undefined Instruction interrupts are only executed if the
    // instruction that causes the interrupt is conditionally executed so
    // its not handled here
    if ( /*instruction_valid &&*/ interrupt &&  next_interrupt != 3'd6 )
        begin
        // Save the interrupt causing instruction to refer back to later
        // This also saves the instruction abort vma and status, in the case of an
        // instruction abort interrupt
        //saved_current_instruction_wen   = 1'd1;

        // save address of next instruction to Supervisor Mode LR
        /* // Address Exception ?
        if ( next_interrupt == 3'd4 )
            reg_write_sel_nxt               = 3'd7;            // pc
        else
            reg_write_sel_nxt               = 3'd1;            // pc -4 //on interrupt, we always want to re-fetch the instruction currently entering Decode
*/
        reg_bank_wen_nxt                = decode (4'd14);  // LR //TODO r13 = stack pointer, r14 = link register

        iaddress_sel_nxt                = 4'd2;            // interrupt_vector
        pc_sel_nxt                      = 3'd2;            // interrupt_vector

        status_bits_mode_nxt            = interrupt_mode;  // e.g. Supervisor mode
        status_bits_mode_wen_nxt        = 1'd1;

        // disable normal interrupts
        status_bits_irq_mask_nxt        = 1'd1;
        status_bits_irq_mask_wen_nxt    = 1'd1;

        // disable fast interrupts
        if ( next_interrupt == 3'd2 ) // FIRQ
            begin
            status_bits_firq_mask_nxt        = 1'd1;
            status_bits_firq_mask_wen_nxt    = 1'd1;
            end
        end


    /* // previous instruction was ldr
    // if it is currently executing in the execute stage do the following
    if ( control_state == MEM_WAIT1 && !conflict )
        begin
        // Save the next instruction to execute later
        // Do this even if the ldr instruction does not execute because of Condition
        pre_fetch_instruction_wen   = 1'd1;

        if ( instruction_execute ) // conditional execution state
            begin
            iaddress_sel_nxt            = 4'd3; // pc  (not pc + 4)
            pc_wen_nxt                  = 1'd0; // hold current PC value
            load_pc_nxt                 = load_pc_r;
            end
        end


    // completion of ldr instruction
    if ( control_state == MEM_WAIT2 )
        begin
        if ( !dabt )  // dont load data there is an abort on the data read
            begin
            pc_wen_nxt                  = 1'd0; // hold current PC value

            // Check if the load destination is the PC
            if (( type == TRANS && instruction[15:12]  == 4'd15 ) ||
                ( type == MTRANS && instruction[20] && mtrans_reg1 == 4'd15 ))
                begin
                pc_sel_nxt       = 3'd3; // read_data_filtered
                iaddress_sel_nxt = 4'd3; // hold value after reading in from mem
                load_pc_nxt      = load_pc_r;
                end
            end
        end*/


    /* // second cycle of multiple load or store
    if ( control_state == MTRANS_EXEC1 && !conflict )
        begin
        // Save the next instruction to execute later
        pre_fetch_instruction_wen   = 1'd1;

        if ( instruction_execute ) // conditional execution state
            begin
            daddress_sel_nxt            = 4'd5;  // o_address
            decode_daccess_nxt          = 1'd1;  // data access

            if ( mtrans_num_registers > 4'd2 )
                decode_iaccess_nxt      = 1'd0;  // skip the instruction fetch


            if ( mtrans_num_registers != 4'd1 )
                begin
                pc_wen_nxt              = 1'd0;  // hold current PC value
                iaddress_sel_nxt        = 4'd3;  // pc  (not pc + 4)
                end


            if ( !instruction[20] ) // Store
                write_data_wen_nxt = 1'd1;

            // stm: store the user mode registers, when in priviledged mode
            if ( {instruction[22],instruction[20]} == 2'b10 )
                o_user_mode_regs_store_nxt = 1'd1;

            // write to the pc ?
            if ( instruction[20] && mtrans_reg1 == 4'd15 ) // Write to PC
                begin
                saved_current_instruction_wen   = 1'd1; // Save the memory access instruction to refer back to later
                pc_wen_nxt                      = 1'd0; // hold current PC value rather than an instruction fetch
                load_pc_nxt                     = 1'd1;
                end
            end
        end


    // third cycle of multiple load or store
    if ( control_state == MTRANS_EXEC2 )
        begin
        daddress_sel_nxt            = 4'd5;  // o_address
        decode_daccess_nxt          = 1'd1;  // data access

        if ( mtrans_num_registers > 4'd2 )
            begin
            decode_iaccess_nxt      = 1'd0;  // skip the instruction fetch
            end

        if ( mtrans_num_registers > 4'd1 )
            begin
            pc_wen_nxt              = 1'd0; // hold current PC value
            iaddress_sel_nxt        = 4'd3;  // pc  (not pc + 4)
            end

        // Store
        if ( !instruction[20] )
            write_data_wen_nxt = 1'd1;

        // stm: store the user mode registers, when in priviledged mode
        if ( {instruction[22],instruction[20]} == 2'b10 )
            o_user_mode_regs_store_nxt = 1'd1;

        // write to the pc ?
        if ( instruction[20] && mtrans_reg1 == 4'd15 ) // Write to PC
            begin
            saved_current_instruction_wen   = 1'd1; // Save the memory access instruction to refer back to later
            pc_wen_nxt                      = 1'd0; // hold current PC value rather than an instruction fetch
            load_pc_nxt                     = 1'd1;
            end
        end


    // state is for when a data abort interrupt is triggered during an ldm
    if ( control_state == MTRANS_ABORT )
        begin
        // Restore the Base Address, if the base register is included in the
        // list of registers being loaded
        if (restore_base_address) // ldm with base address in register list
            begin
            reg_write_sel_nxt = 3'd6;                        // write base_register
            reg_bank_wen_nxt  = decode ( instruction[19:16] ); // to Rn
            end
        end*/


        // Multiply or Multiply-Accumulate
    /*if ( control_state == MULT_PROC1 && instruction_execute && !conflict )
        begin
        // Save the next instruction to execute later
        // Do this even if this instruction does not execute because of Condition
        pre_fetch_instruction_wen   = 1'd1;
        pc_wen_nxt                  = 1'd0;  // hold current PC value
        multiply_function_nxt       = o_multiply_function;
        end


        // Multiply or Multiply-Accumulate
        // Do multiplication
        // Wait for done or accumulate signal
    if ( control_state == MULT_PROC2 )
        begin
        // Save the next instruction to execute later
        // Do this even if this instruction does not execute because of Condition
        pc_wen_nxt              = 1'd0;  // hold current PC value
        iaddress_sel_nxt        = 4'd3;  // pc  (not pc + 4)
        multiply_function_nxt   = o_multiply_function;
        end


    // Save RdLo
    // always last cycle of all multiply or multiply accumulate operations
    if ( control_state == MULT_STORE )
        begin
        reg_write_sel_nxt     = 3'd2; // multiply_out
        multiply_function_nxt = o_multiply_function;

        if ( type == MULT ) // 32-bit
            reg_bank_wen_nxt      = decode (instruction[19:16]); // Rd
        else  // 64-bit / Long
            reg_bank_wen_nxt      = decode (instruction[15:12]); // RdLo

        if ( instruction[20] )  // the 'S' bit
            begin
            status_bits_sel_nxt       = 3'd4; // { multiply_flags, status_bits_flags[1:0] }
            status_bits_flags_wen_nxt = 1'd1;
            end
        end


    // Add lower 32 bits to multiplication product
    if ( control_state == MULT_ACCUMU )
        begin
        multiply_function_nxt = o_multiply_function;
        pc_wen_nxt            = 1'd0;  // hold current PC value
        iaddress_sel_nxt      = 4'd3;  // pc  (not pc + 4)
        end*/


    // swp - do write request in 2nd cycle
    /*if ( control_state == SWAP_WRITE && instruction_execute && !conflict )
        begin
        barrel_shift_data_sel_nxt       = 2'd2; // Shift value from Rm register
        daddress_sel_nxt                = 4'd4; // Rn
        write_data_wen_nxt              = 1'd1;
        decode_iaccess_nxt              = 1'd0; // skip the instruction fetch
        decode_daccess_nxt              = 1'd1; // data access

        if ( instruction[22] )
            byte_enable_sel_nxt = 2'd1;         // Save byte

        if ( instruction_execute )              // conditional execution state
            pc_wen_nxt                  = 1'd0; // hold current PC value

        // Save the next instruction to execute later
        // Do this even if this instruction does not execute because of Condition
        pre_fetch_instruction_wen       = 1'd1;

        load_pc_nxt                     = load_pc_r;
        end


    // swp - receive read response in 3rd cycle
    if ( control_state == SWAP_WAIT1 )
        begin

        if ( instruction_execute ) // conditional execution state
            begin
            iaddress_sel_nxt            = 4'd3; // pc  (not pc + 4)
            pc_wen_nxt                  = 1'd0; // hold current PC value
            end

        if ( !dabt )
            begin
            // Check is the load destination is the PC
            if ( instruction[15:12]  == 4'd15 )
                begin
                pc_sel_nxt       = 3'd3; // read_data_filtered
                iaddress_sel_nxt = 4'd3; // hold value after reading in from mem
                load_pc_nxt      = load_pc_r;
                end
            end
        end*/

//TODO confirm the below thingie works/reimplement as needed for interrupt handling in the revised design
    // Have just changed the status_bits mode but this
    // creates a 1 cycle gap with the old mode
    // coming back from execute into instruction_decode
    // So squash that old mode value during this
    // cycle of the interrupt transition
    /*if ( control_state == INT_WAIT1 )
        status_bits_mode_nxt            = status_bits_mode_r;   // Supervisor mode
*/
    end

//Removed next-state logic section

//Register update/output-latching
//TODO also add reset state!
always_ff @(posedge i_rst, posedge i_clk) begin
	if (i_rst) begin
		o_status_bits_mode <= SVC;
		o_status_bits_irq_mask <= '1;
		o_status_bits_firq_mask <= '1;
		o_imm32 <= '0;
		o_imm_shift_amount <= '0;
		o_shift_imm_zero <= '0;
		o_condition <= AL;
		o_is_memop <= '0;
		o_rm_sel <= '0;
		o_rs_sel <= '0;
		o_rn_sel <= '0;
		o_use_rn <= '0;
		o_use_rs <= '0;
		o_use_rm <= '0;
		o_use_sr <= '0;
		o_barrel_shift_amount_sel <= '0;
		o_barrel_shift_data_sel <= '0;
		o_barrel_shift_function <= '0;
		o_alu_function <= '0;
		o_use_carry_in <= '0;
		o_multiply_function <= '0;
		o_interrupt_vector_sel <= '0;
		o_iaddress_sel <= 4'd2;
		//o_daddress_sel <= 4'd2;
		o_byte_enable_sel <= '0;
		o_status_bits_sel <= '0;
		//o_reg_write_sel <= '0;
		o_write_data_wen <= '0;
		o_pc_wen <= 1'b1;
		o_reg_bank_wen <= '0;
		o_status_bits_flags_wen <= '0;
		o_status_bits_mode_wen <= '0;
		o_status_bits_irq_mask_wen <= '0;
		o_status_bits_firq_mask_wen <= '0;
	end
	else begin
		if (!i_core_stall) begin //TODO ensure that i_core_stall is hooked up properly
			if (i_branch_taken) begin
				//this case of !i_core_stall & i_branch_taken will occur on the cycle when branch has just been resolved, so set outputs to *safe* i.e. effective-NOP state
				//exception is if there's also an interrupt this cycle, in which case we let the IRQ/FIRQ stuff proceed normally (but we don't return to this current instr's PC, we return to the newly branched-to PC
				//we also always want to keep setting status_bits_mode, irq_mask, and firq_mask in all cases.
				//TODO revise to allow for interrupts!
				o_status_bits_mode <= status_bits_mode_nxt;
				o_status_bits_irq_mask <= status_bits_irq_mask_nxt;
				o_status_bits_firq_mask <= status_bits_firq_mask_nxt;
				o_imm32 <= '0;
				o_imm_shift_amount <= '0;
				o_shift_imm_zero <= '0;
				o_condition <= AL;
				o_is_memop <= '0;
				o_rm_sel <= '0;
				o_rs_sel <= '0;
				o_rn_sel <= '0;
				o_use_rn <= '0;
				o_use_rs <= '0;
				o_use_rm <= '0;
				o_use_sr <= '0;
				o_barrel_shift_amount_sel <= '0;
				o_barrel_shift_data_sel <= '0;
				o_barrel_shift_function <= '0;
				o_alu_function <= '0;
				o_use_carry_in <= '0;
				o_multiply_function <= '0;
				o_interrupt_vector_sel <= '0;
				o_iaddress_sel <= 4'd2;
				//o_daddress_sel <= 4'd2;
				o_byte_enable_sel <= '0;
				o_status_bits_sel <= '0;
				//o_reg_write_sel <= '0;
				o_write_data_wen <= '0;
				o_pc_wen <= 1'b1;
				o_reg_bank_wen <= '0;
				o_status_bits_flags_wen <= '0;
				o_status_bits_mode_wen <= status_bits_mode_wen_nxt;
				o_status_bits_irq_mask_wen <= status_bits_irq_mask_wen_nxt;
				o_status_bits_firq_mask_wen <= status_bits_firq_mask_wen_nxt;
			end
			else begin
				o_status_bits_mode <= status_bits_mode_nxt;
				o_status_bits_irq_mask <= status_bits_irq_mask_nxt;
				o_status_bits_firq_mask <= status_bits_firq_mask_nxt;
				o_imm32 <= imm32_nxt;
				o_imm_shift_amount <= imm_shift_amount_nxt;
				o_shift_imm_zero <= shift_imm_zero_nxt;
				o_condition <= !interrupt ? condition_nxt : AL;
				o_is_memop <= is_memop_nxt;
				o_rm_sel <= rm_sel_nxt;
				o_rs_sel <= rs_sel_nxt;
				o_rn_sel <= rn_sel_nxt;
				o_use_rm <= use_rm_nxt;
				o_use_rs <= use_rs_nxt;
				o_use_rn <= use_rn_nxt;
				o_use_sr <= use_sr_nxt;
				o_barrel_shift_amount_sel <= barrel_shift_amount_sel_nxt;
				o_barrel_shift_data_sel <= barrel_shift_data_sel_nxt;
				o_barrel_shift_function <= barrel_shift_function_nxt;
				o_alu_function <= alu_function_nxt;
				o_use_carry_in <= use_carry_in_nxt;
				o_multiply_function <= multiply_function_nxt;
				o_interrupt_vector_sel <= next_interrupt;
				o_iaddress_sel <= iaddress_sel_nxt;
				//o_daddress_sel <= daddress_sel_nxt; //TODO uncomment later
				//o_pc_sel <= pc_sel_nxt; //TODO note: pc_sel and iaddress_sel appear to be redundant!!!
				o_byte_enable_sel <= byte_enable_sel_nxt;
				o_status_bits_sel <= status_bits_sel_nxt;
				//o_reg_write_sel <= reg_write_sel_nxt; //TODO review what exactly this does
				o_write_data_wen <= write_data_wen_nxt;
				o_pc_wen <= pc_wen_nxt;
				o_reg_bank_wen <= reg_bank_wen_nxt;
				o_status_bits_flags_wen <= status_bits_flags_wen_nxt;
				o_status_bits_mode_wen <= status_bits_mode_wen_nxt;
				o_status_bits_irq_mask_wen <= status_bits_irq_mask_wen_nxt;
				o_status_bits_firq_mask_wen <= status_bits_firq_mask_wen_nxt;
			end
		end
		else begin
			//if we're core-stalled, don't change anything at all
		end
	end
end


endmodule //b01_decode