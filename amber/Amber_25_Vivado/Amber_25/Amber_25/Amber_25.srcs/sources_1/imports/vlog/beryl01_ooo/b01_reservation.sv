//Define reservation station information being recorded for each given instruction
typedef struct packed {
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
	
	logic status_bits_flags_valid;
	logic [5:0] status_bits_flags_tag;
	logic [3:0] status_bits_flags;
	
	//Control signals for the ALU stage
	logic use_carry_in;
	logic [31:0] imm32;
	logic [4:0] imm_shift_amount;
	logic shift_imm_zero;
	logic [1:0] barrel_shift_amount_sel;
	logic [1:0] barrel_shift_data_sel;
	logic [1:0] barrel_shift_function;
	logic [8:0] alu_function;
	logic pc_wen;
	logic status_bits_flags_wen;
	logic [5:0] rd_tag;
} alu_station_entry;

typedef struct packed {
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
	
	//Control signals for the multiply stage
	logic [1:0] multiply_function;
	logic use_carry_in;
	logic pc_wen;
	logic status_bits_flags_wen;
	logic [5:0] rd_tag;
} mult_station_entry;

typedef struct packed {
	//2, 4, 8.1, byte_enable_sel, daddress_sel, iaddress_sel
	//Operand 1 data
	logic rn_valid;
	logic [5:0] rn_tag;
	logic [31:0] rn;
	
	//Operand 2 data
	logic rs_valid;
	logic [5:0] rs_tag;
	logic [31:0] rs;
	
	//Operand 3 data (used if we write to a reg, one operand is a reg, and the other operand is a reg shifted by another reg)
	//TODO: determine if this is ever actually used in memory instructions
	logic rm_valid;
	logic [5:0] rm_tag;
	logic [31:0] rm;
	
	//Control signals for the memory stage
	logic exclusive;
	logic pc_wen;
	logic status_bits_flags_wen;
	logic [1:0] byte_enable_sel;
	logic [5:0] rd_tag;
} mem_station_entry;


module b01_reservation (
	input logic			i_clk,
	input logic			i_rst,
	input logic			i_stall,
	
	input logic	 [5:0]	i_tag_nxt,
	
	input logic  [31:0]	i_imm32,
	input logic  [4:0]	i_imm_shift_amount,
	input logic 		i_shift_imm_zero,
	input logic			i_decode_exclusive,       // swap access //for SWP/SWPB instruction that atomically writes one reg's value into mem at a given addr and loads the value at that addr into another register. Will need to handle "specially" with some sort of state machine in the mem stage.
	input logic  [1:0]	i_barrel_shift_amount_sel,
	input logic  [1:0]	i_barrel_shift_data_sel,
	input logic  [1:0]	i_barrel_shift_function,
	input logic  [8:0]	i_alu_function,
	input logic  [1:0]	i_multiply_function,
	input logic  [1:0]	i_byte_enable_sel,
	input logic			i_use_carry_in,         // e.g. add with carry instruction

	input logic			i_write_data_wen,
	input logic			i_base_address_wen,     // save LDM base address register,
												// in case of data abort
	input logic			i_pc_wen,
	input logic  [14:0]	i_reg_bank_wen,
	input logic			i_status_bits_flags_wen,
	
	input logic			i_rn_valid,
	input logic  [5:0]	i_rn_tag,
	input logic  [31:0] i_rn,
	input logic			i_rs_valid,
	input logic  [5:0]	i_rs_tag,
	input logic  [31:0] i_rs,
	input logic			i_rm_valid,
	input logic  [5:0]	i_rm_tag,
	input logic  [31:0] i_rm,
	input logic			i_status_bits_flags_valid,
	input logic  [5:0]	i_status_bits_flags_tag,
	input logic  [3:0]	i_status_bits_flags,

	//ALU station interface
	output logic 		o_instr_valid_alu,
	output logic [31:0] o_rn_alu,
	output logic [31:0] o_rs_alu,
	output logic [31:0] o_rm_alu,
	output logic [3:0]	o_status_bits_flags_alu,
	output logic		o_use_carry_in_alu,
	//output logic [31:0] o_ccr_alu,
	output logic [31:0] o_imm32_alu,
	output logic [4:0] 	o_imm_shift_amount_alu,
	output logic 		o_shift_imm_zero_alu,
	output logic [1:0] 	o_barrel_shift_amount_sel_alu,
	output logic [1:0] 	o_barrel_shift_data_sel_alu,
	output logic [1:0] 	o_barrel_shift_function_alu,
	output logic [8:0] 	o_alu_function_alu,
	output logic		o_pc_wen_alu,
	output logic		o_status_bits_flags_wen_alu,
	output logic [5:0] 	o_rd_tag_alu,
	input logic			i_alu_valid,
	input logic  [5:0]	i_alu_tag,
	input logic  [31:0]	i_alu_data,

	//Mult station interface
	output logic 		o_instr_valid_mult,
	output logic [31:0] o_rn_mult,
	output logic [31:0] o_rs_mult,
	output logic [31:0] o_rm_mult,
	output logic [1:0] 	o_multiply_function_mult,
	output logic 		o_use_carry_in_mult,
	output logic		o_pc_wen_mult,
	output logic 		o_status_bits_flags_wen_mult,
	output logic [5:0] 	o_rd_tag_mult,
	input logic 		i_mult_valid,
	input logic  [5:0] 	i_mult_tag,
	input logic  [31:0] i_mult_data,

	//Mem station interface
	output logic 		o_instr_valid_mem,
	output logic [31:0] o_rn_mem,
	output logic [31:0] o_rs_mem,
	output logic [31:0] o_rm_mem,
	output logic 		o_exclusive_mem,
	output logic		o_pc_wen_mem,
	output logic 		o_status_bits_flags_wen_mem,
	output logic [1:0] 	o_byte_enable_sel_mem,
	output logic [5:0] 	o_rd_tag_mem,
	input logic 		i_mem_valid,
	input logic  [5:0] 	i_mem_tag,
	input logic  [31:0] i_mem_data
);


logic	new_instr_ready,
		new_instr_ready_alu,
		new_instr_ready_mult,
		new_instr_ready_mem;
assign new_instr_ready = i_rn_valid & i_rs_valid & i_rm_valid;
assign new_instr_ready_alu = new_instr_ready && i_tag_nxt[5:4] == 2'b00;
assign new_instr_ready_mult = new_instr_ready && i_tag_nxt[5:4] == 2'b01;
assign new_instr_ready_mem = new_instr_ready && i_tag_nxt[5:4] == 2'b10;


//The reservation stations themselves
alu_station_entry alu_station[15:0];
mult_station_entry mult_station[15:0];
mem_station_entry mem_station[15:0];


//The pending new reservation station entries
alu_station_entry alu_new_entry;
mult_station_entry mult_new_entry;
mem_station_entry mem_new_entry;


//Control signals
localparam ALU_MATCH_IDX = 0; //note that these aren't *really* parameters, they're more like `defines
localparam MULT_MATCH_IDX = 1;
localparam MEM_MATCH_IDX = 2;
logic [15:0] 		alu_ready,
					mult_ready,
					mem_ready;
logic [15:0]		alu_shift_into, //define whether or not we need to shift up a given reservation station entry based on what we've just dispatched
					mult_shift_into,
					mem_shift_into;
logic [15:0][2:0]	alu_rn_tag_match, //of each entry, bit 0 says it matches the incoming ALU result, bit 1=mult, bit 2=mem
					alu_rs_tag_match,
					alu_rm_tag_match,
					alu_status_bits_flags_tag_match;
					//alu_ccr_tag_match;
logic [15:0][2:0] 	mult_rn_tag_match,
					mult_rs_tag_match,
					mult_rm_tag_match;
					//mult_ccr_tag_match;
logic [15:0][2:0]	mem_rn_tag_match,
					mem_rs_tag_match,
					mem_rm_tag_match;
					//mem_ccr_tag_match;
logic [4:0]			alu_next_dispatch_idx,
					mult_next_dispatch_idx,
					mem_next_dispatch_idx;

//1-hot (register) vector for each station to say which stations are currently occupied
//TODO need to initialize!
logic [15:0]		alu_occupied,
					mult_occupied,
					mem_occupied;
//combinational index to select which station to insert into next based on which stations are unoccupied
logic [3:0]			alu_next_insert_idx,
					mult_next_insert_idx,
					mem_next_insert_idx;
assign alu_next_insert_idx	=	!alu_occupied[0 ]	?	4'd0	:
								!alu_occupied[1 ]	?	4'd1	:
								!alu_occupied[2 ]	?	4'd2	:
								!alu_occupied[3 ]	?	4'd3	:
								!alu_occupied[4 ]	?	4'd4	:
								!alu_occupied[5 ]	?	4'd5	:
								!alu_occupied[6 ]	?	4'd6	:
								!alu_occupied[7 ]	?	4'd7	:
								!alu_occupied[8 ]	?	4'd8	:
								!alu_occupied[9 ]	?	4'd9	:
								!alu_occupied[10]	?	4'd10	:
								!alu_occupied[11]	?	4'd11	:
								!alu_occupied[12]	?	4'd12	:
								!alu_occupied[13]	?	4'd13	:
								!alu_occupied[14]	?	4'd14	:
														4'd15;
assign mult_next_insert_idx	=	!mult_occupied[0 ]	?	4'd0	:
								!mult_occupied[1 ]	?	4'd1	:
								!mult_occupied[2 ]	?	4'd2	:
								!mult_occupied[3 ]	?	4'd3	:
								!mult_occupied[4 ]	?	4'd4	:
								!mult_occupied[5 ]	?	4'd5	:
								!mult_occupied[6 ]	?	4'd6	:
								!mult_occupied[7 ]	?	4'd7	:
								!mult_occupied[8 ]	?	4'd8	:
								!mult_occupied[9 ]	?	4'd9	:
								!mult_occupied[10]	?	4'd10	:
								!mult_occupied[11]	?	4'd11	:
								!mult_occupied[12]	?	4'd12	:
								!mult_occupied[13]	?	4'd13	:
								!mult_occupied[14]	?	4'd14	:
														4'd15;
assign mem_next_insert_idx	=	!mem_occupied[0 ]	?	4'd0	:
								!mem_occupied[1 ]	?	4'd1	:
								!mem_occupied[2 ]	?	4'd2	:
								!mem_occupied[3 ]	?	4'd3	:
								!mem_occupied[4 ]	?	4'd4	:
								!mem_occupied[5 ]	?	4'd5	:
								!mem_occupied[6 ]	?	4'd6	:
								!mem_occupied[7 ]	?	4'd7	:
								!mem_occupied[8 ]	?	4'd8	:
								!mem_occupied[9 ]	?	4'd9	:
								!mem_occupied[10]	?	4'd10	:
								!mem_occupied[11]	?	4'd11	:
								!mem_occupied[12]	?	4'd12	:
								!mem_occupied[13]	?	4'd13	:
								!mem_occupied[14]	?	4'd14	:
														4'd15;


//Combinational control signal logic
always_comb begin
	//Determine if ALU reservation station tags match incoming data
	for (int i=0; i<16; i+=1) begin
		alu_rn_tag_match[i][ALU_MATCH_IDX] = (alu_station[i].rn_tag == i_alu_tag) & !alu_station[i].rn_valid;
		alu_rn_tag_match[i][MULT_MATCH_IDX] = (alu_station[i].rn_tag == i_mult_tag) & !alu_station[i].rn_valid;
		alu_rn_tag_match[i][MEM_MATCH_IDX] = (alu_station[i].rn_tag == i_mem_tag) & !alu_station[i].rn_valid;
		
		alu_rs_tag_match[i][ALU_MATCH_IDX] = (alu_station[i].rs_tag == i_alu_tag) & !alu_station[i].rs_valid;
		alu_rs_tag_match[i][MULT_MATCH_IDX] = (alu_station[i].rs_tag == i_mult_tag) & !alu_station[i].rs_valid;
		alu_rs_tag_match[i][MEM_MATCH_IDX] = (alu_station[i].rs_tag == i_mem_tag) & !alu_station[i].rs_valid;
		
		alu_rm_tag_match[i][ALU_MATCH_IDX] = (alu_station[i].rm_tag == i_alu_tag) & !alu_station[i].rm_valid;
		alu_rm_tag_match[i][MULT_MATCH_IDX] = (alu_station[i].rm_tag == i_mult_tag) & !alu_station[i].rm_valid;
		alu_rm_tag_match[i][MEM_MATCH_IDX] = (alu_station[i].rm_tag == i_mem_tag) & !alu_station[i].rm_valid;
		
		alu_status_bits_flags_tag_match[i][ALU_MATCH_IDX] = (alu_station[i].status_bits_flags_tag == i_alu_tag) & !alu_station[i].status_bits_flags_valid;
		alu_status_bits_flags_tag_match[i][MULT_MATCH_IDX] = (alu_station[i].status_bits_flags_tag == i_mult_tag) & !alu_station[i].status_bits_flags_valid;
		alu_status_bits_flags_tag_match[i][MEM_MATCH_IDX] = (alu_station[i].status_bits_flags_tag == i_mem_tag) & !alu_station[i].status_bits_flags_valid;
		//alu_ccr_tag_match[i][ALU_MATCH_IDX] = (alu_station[i].ccr_tag == i_alu_tag) & !alu_station[i].ccr_valid;
		//alu_ccr_tag_match[i][MULT_MATCH_IDX] = (alu_station[i].ccr_tag == i_mult_tag) & !alu_station[i].ccr_valid;
		//alu_ccr_tag_match[i][MEM_MATCH_IDX] = (alu_station[i].ccr_tag == i_mem_tag) & !alu_station[i].ccr_valid;
	end
	
	//Determine if MULT reservation station tags match incoming data
	for (int i=0; i<16; i+=1) begin
		mult_rn_tag_match[i][ALU_MATCH_IDX] = (mult_station[i].rn_tag == i_alu_tag) & !mult_station[i].rn_valid;
		mult_rn_tag_match[i][MULT_MATCH_IDX] = (mult_station[i].rn_tag == i_mult_tag) & !mult_station[i].rn_valid;
		mult_rn_tag_match[i][MEM_MATCH_IDX] = (mult_station[i].rn_tag == i_mem_tag) & !mult_station[i].rn_valid;
		
		mult_rs_tag_match[i][ALU_MATCH_IDX] = (mult_station[i].rs_tag == i_alu_tag) & !mult_station[i].rs_valid;
		mult_rs_tag_match[i][MULT_MATCH_IDX] = (mult_station[i].rs_tag == i_mult_tag) & !mult_station[i].rs_valid;
		mult_rs_tag_match[i][MEM_MATCH_IDX] = (mult_station[i].rs_tag == i_mem_tag) & !mult_station[i].rs_valid;
		
		mult_rm_tag_match[i][ALU_MATCH_IDX] = (mult_station[i].rm_tag == i_alu_tag) & !mult_station[i].rm_valid;
		mult_rm_tag_match[i][MULT_MATCH_IDX] = (mult_station[i].rm_tag == i_mult_tag) & !mult_station[i].rm_valid;
		mult_rm_tag_match[i][MEM_MATCH_IDX] = (mult_station[i].rm_tag == i_mem_tag) & !mult_station[i].rm_valid;
		
		//mult_ccr_tag_match[i][ALU_MATCH_IDX] = (mult_station[i].ccr_tag == i_alu_tag) & !mult_station[i].ccr_valid;
		//mult_ccr_tag_match[i][MULT_MATCH_IDX] = (mult_station[i].ccr_tag == i_mult_tag) & !mult_station[i].ccr_valid;
		//mult_ccr_tag_match[i][MEM_MATCH_IDX] = (mult_station[i].ccr_tag == i_mem_tag) & !mult_station[i].ccr_valid;
	end
	
	//Determine if MEM reservation station tags match incoming data
	for (int i=0; i<16; i+=1) begin
		mem_rn_tag_match[i][ALU_MATCH_IDX] = (mem_station[i].rn_tag == i_alu_tag) & !mem_station[i].rn_valid;
		mem_rn_tag_match[i][MULT_MATCH_IDX] = (mem_station[i].rn_tag == i_mult_tag) & !mem_station[i].rn_valid;
		mem_rn_tag_match[i][MEM_MATCH_IDX] = (mem_station[i].rn_tag == i_mem_tag) & !mem_station[i].rn_valid;
		
		mem_rs_tag_match[i][ALU_MATCH_IDX] = (mem_station[i].rs_tag == i_alu_tag) & !mem_station[i].rs_valid;
		mem_rs_tag_match[i][MULT_MATCH_IDX] = (mem_station[i].rs_tag == i_mult_tag) & !mem_station[i].rs_valid;
		mem_rs_tag_match[i][MEM_MATCH_IDX] = (mem_station[i].rs_tag == i_mem_tag) & !mem_station[i].rs_valid;
		
		mem_rm_tag_match[i][ALU_MATCH_IDX] = (mem_station[i].rm_tag == i_alu_tag) & !mem_station[i].rm_valid;
		mem_rm_tag_match[i][MULT_MATCH_IDX] = (mem_station[i].rm_tag == i_mult_tag) & !mem_station[i].rm_valid;
		mem_rm_tag_match[i][MEM_MATCH_IDX] = (mem_station[i].rm_tag == i_mem_tag) & !mem_station[i].rm_valid;
		
		//mem_ccr_tag_match[i][ALU_MATCH_IDX] = (mem_station[i].ccr_tag == i_alu_tag) & !mem_station[i].ccr_valid;
		//mem_ccr_tag_match[i][MULT_MATCH_IDX] = (mem_station[i].ccr_tag == i_mult_tag) & !mem_station[i].ccr_valid;
		//mem_ccr_tag_match[i][MEM_MATCH_IDX] = (mem_station[i].ccr_tag == i_mem_tag) & !mem_station[i].ccr_valid;
	end
	
	//Determine which instructions are ready for dispatch this cycle
	//TODO make sure when inserting the instructions each element is set to "valid" if it's unused (up in the Dispatch stage in which this unit is instantiated)
	for (int i=0; i<16; i+=1) begin
		alu_ready[i] = (alu_station[i].rn_valid | (alu_rn_tag_match[i] != 3'b000))
						& (alu_station[i].rs_valid | (alu_rs_tag_match[i] != 3'b000))
						& (alu_station[i].rm_valid | (alu_rm_tag_match[i] != 3'b000))
						& (alu_station[i].status_bits_flags_valid | (alu_status_bits_flags_tag_match[i] != 3'b000))
						/*& (alu_station[i].ccr_valid | (alu_ccr_tag_match[i] != 3'b000))*/;
		mult_ready[i] = (mult_station[i].rn_valid | (mult_rn_tag_match[i] != 3'b000))
						& (mult_station[i].rs_valid | (mult_rs_tag_match[i] != 3'b000))
						& (mult_station[i].rm_valid | (mult_rm_tag_match[i] != 3'b000))
						/*& (mult_station[i].ccr_valid | (mult_ccr_tag_match[i] != 3'b000))*/;
		mem_ready[i] = (mem_station[i].rn_valid | (mem_rn_tag_match[i] != 3'b000))
						& (mem_station[i].rs_valid | (mem_rs_tag_match[i] != 3'b000))
						& (mem_station[i].rm_valid | (mem_rm_tag_match[i] != 3'b000))
						/*& (mem_station[i].ccr_valid | (mem_ccr_tag_match[i] != 3'b000))*/;
	end
	
	//Determine index of the next reservation station entry to dispatch (for each station).
	//16 means dispatch this cycle's incoming instruction.
	//17 means nothing is ready.
	//(And also change the output logic to account for forwarding paths with incoming data from the EUs, as applicable...)********//
	//ALU
	if (alu_ready[0]) alu_next_dispatch_idx = 5'd0;
	else if (alu_ready[1]) alu_next_dispatch_idx = 5'd1;
	else if (alu_ready[2]) alu_next_dispatch_idx = 5'd2;
	else if (alu_ready[3]) alu_next_dispatch_idx = 5'd3;
	else if (alu_ready[4]) alu_next_dispatch_idx = 5'd4;
	else if (alu_ready[5]) alu_next_dispatch_idx = 5'd5;
	else if (alu_ready[6]) alu_next_dispatch_idx = 5'd6;
	else if (alu_ready[7]) alu_next_dispatch_idx = 5'd7;
	else if (alu_ready[8]) alu_next_dispatch_idx = 5'd8;
	else if (alu_ready[9]) alu_next_dispatch_idx = 5'd9;
	else if (alu_ready[10]) alu_next_dispatch_idx = 5'd10;
	else if (alu_ready[11]) alu_next_dispatch_idx = 5'd11;
	else if (alu_ready[12]) alu_next_dispatch_idx = 5'd12;
	else if (alu_ready[13]) alu_next_dispatch_idx = 5'd13;
	else if (alu_ready[14]) alu_next_dispatch_idx = 5'd14;
	else if (alu_ready[15]) alu_next_dispatch_idx = 5'd15;
	else if (new_instr_ready_alu) alu_next_dispatch_idx = 5'd16; //16 denotes "forward the data from the input to the EU on this cycle"
	else alu_next_dispatch_idx = 5'd17; //17 denotes "nothing ready to dispatch"
	for (int i=0; i<16; i+=1) begin
		alu_shift_into[i] = (alu_next_dispatch_idx/*_minus1*/ >= i) & ~alu_next_dispatch_idx[4];
	end
	
	//MULT
	if (mult_ready[0]) mult_next_dispatch_idx = 5'd0;
	else if (mult_ready[1]) mult_next_dispatch_idx = 5'd1;
	else if (mult_ready[2]) mult_next_dispatch_idx = 5'd2;
	else if (mult_ready[3]) mult_next_dispatch_idx = 5'd3;
	else if (mult_ready[4]) mult_next_dispatch_idx = 5'd4;
	else if (mult_ready[5]) mult_next_dispatch_idx = 5'd5;
	else if (mult_ready[6]) mult_next_dispatch_idx = 5'd6;
	else if (mult_ready[7]) mult_next_dispatch_idx = 5'd7;
	else if (mult_ready[8]) mult_next_dispatch_idx = 5'd8;
	else if (mult_ready[9]) mult_next_dispatch_idx = 5'd9;
	else if (mult_ready[10]) mult_next_dispatch_idx = 5'd10;
	else if (mult_ready[11]) mult_next_dispatch_idx = 5'd11;
	else if (mult_ready[12]) mult_next_dispatch_idx = 5'd12;
	else if (mult_ready[13]) mult_next_dispatch_idx = 5'd13;
	else if (mult_ready[14]) mult_next_dispatch_idx = 5'd14;
	else if (mult_ready[15]) mult_next_dispatch_idx = 5'd15;
	else if (new_instr_ready_mult) mult_next_dispatch_idx = 5'd16; //16 denotes "forward the data from the input to the EU on this cycle"
	else mult_next_dispatch_idx = 5'd17; //17 denotes "nothing ready to dispatch"
	for (int i=0; i<16; i+=1) begin
		mult_shift_into[i] = (mult_next_dispatch_idx/*_minus1*/ >= i) & ~mult_next_dispatch_idx[4];
	end
	
	//MEM
	if (mem_ready[0]) mem_next_dispatch_idx = 5'd0;
	else if (mem_ready[1]) mem_next_dispatch_idx = 5'd1;
	else if (mem_ready[2]) mem_next_dispatch_idx = 5'd2;
	else if (mem_ready[3]) mem_next_dispatch_idx = 5'd3;
	else if (mem_ready[4]) mem_next_dispatch_idx = 5'd4;
	else if (mem_ready[5]) mem_next_dispatch_idx = 5'd5;
	else if (mem_ready[6]) mem_next_dispatch_idx = 5'd6;
	else if (mem_ready[7]) mem_next_dispatch_idx = 5'd7;
	else if (mem_ready[8]) mem_next_dispatch_idx = 5'd8;
	else if (mem_ready[9]) mem_next_dispatch_idx = 5'd9;
	else if (mem_ready[10]) mem_next_dispatch_idx = 5'd10;
	else if (mem_ready[11]) mem_next_dispatch_idx = 5'd11;
	else if (mem_ready[12]) mem_next_dispatch_idx = 5'd12;
	else if (mem_ready[13]) mem_next_dispatch_idx = 5'd13;
	else if (mem_ready[14]) mem_next_dispatch_idx = 5'd14;
	else if (mem_ready[15]) mem_next_dispatch_idx = 5'd15;
	else if (new_instr_ready_mem) mem_next_dispatch_idx = 5'd16; //16 denotes "forward the data from the input to the EU on this cycle"
	else mem_next_dispatch_idx = 5'd17; //17 denotes "nothing ready to dispatch"
	for (int i=0; i<16; i+=1) begin
		mem_shift_into[i] = (mem_next_dispatch_idx/*_minus1*/ >= i) & ~mem_next_dispatch_idx[4];
	end
	
	//Determing where an incoming instruction will go is handled in the always_ff, below
	//*** note: an incoming instruction will always go into the "back of the queue" or directly to the output.
end

//set up new reservation station entries based on incoming info from Decode
always_comb begin
	//defaults
	alu_new_entry = '0;
	mult_new_entry = '0;
	mem_new_entry = '0;
	
	case (i_tag_nxt[5:4])
		2'b00: begin
			//This will also check if the tag is on the tag bus THIS cycle and insert info immediately if so. Note that per the current design, all instructions will have to spend at least one cycle in the reservation station before executing; this should be fixable in the future.
			//Grab the *actual* register file data if it's available; this will also handle the first part of ^^.
			//ensure if an operand is unused for this instruction, we set "valid" to true immediately!
			alu_new_entry.rn_valid = i_rn_valid;
			alu_new_entry.rn_tag = i_rn_tag;
			alu_new_entry.rn = i_rn;
			
			alu_new_entry.rs_valid = i_rs_valid;
			alu_new_entry.rs_tag = i_rs_tag;
			alu_new_entry.rs = i_rs;
			
			alu_new_entry.rm_valid = i_rm_valid;
			alu_new_entry.rm_tag = i_rm_tag;
			alu_new_entry.rm = i_rm;
			
			alu_new_entry.status_bits_flags_valid = i_status_bits_flags_valid;
			alu_new_entry.status_bits_flags_tag = i_status_bits_flags_tag;
			alu_new_entry.status_bits_flags = i_status_bits_flags;
			
			alu_new_entry.use_carry_in = i_use_carry_in;
			alu_new_entry.imm32 = i_imm32;
			alu_new_entry.imm_shift_amount = i_imm_shift_amount;
			alu_new_entry.shift_imm_zero = i_shift_imm_zero;
			alu_new_entry.barrel_shift_amount_sel = i_barrel_shift_amount_sel;
			alu_new_entry.barrel_shift_data_sel = i_barrel_shift_data_sel;
			alu_new_entry.barrel_shift_function = i_barrel_shift_function;
			alu_new_entry.alu_function = i_alu_function;
			alu_new_entry.pc_wen = i_pc_wen;
			alu_new_entry.status_bits_flags_wen = i_status_bits_flags_wen;
			alu_new_entry.rd_tag = i_tag_nxt;
		end
		2'b01: begin
			//This will also check if the tag is on the tag bus THIS cycle and insert info immediately if so. Note that per the current design, all instructions will have to spend at least one cycle in the reservation station before executing; this should be fixable in the future.
			//Grab the *actual* register file data if it's available; this will also handle the first part of ^^.
			mult_new_entry.rn_valid = i_rn_valid;
			mult_new_entry.rn_tag = i_rn_tag;
			mult_new_entry.rn = i_rn;
			
			mult_new_entry.rs_valid = i_rs_valid;
			mult_new_entry.rs_tag = i_rs_tag;
			mult_new_entry.rs = i_rs;
			
			mult_new_entry.rm_valid = i_rm_valid;
			mult_new_entry.rm_tag = i_rm_tag;
			mult_new_entry.rm = i_rm;
			
			mult_new_entry.multiply_function = i_multiply_function;
			mult_new_entry.use_carry_in = i_use_carry_in;
			mult_new_entry.pc_wen = i_pc_wen;
			mult_new_entry.status_bits_flags_wen = i_status_bits_flags_wen;
			mult_new_entry.rd_tag = i_tag_nxt;
		end
		2'b10: begin
			//This will also check if the tag is on the tag bus THIS cycle and insert info immediately if so. Note that per the current design, all instructions will have to spend at least one cycle in the reservation station before executing; this should be fixable in the future.
			//Grab the *actual* register file data if it's available; this will also handle the first part of ^^.
			mem_new_entry.rn_valid = i_rn_valid;
			mem_new_entry.rn_tag = i_rn_tag;
			mem_new_entry.rn = i_rn;
			
			mem_new_entry.rs_valid = i_rs_valid;
			mem_new_entry.rs_tag = i_rs_tag;
			mem_new_entry.rs = i_rs;
			
			mem_new_entry.rm_valid = i_rm_valid;
			mem_new_entry.rm_tag = i_rm_tag;
			mem_new_entry.rm = i_rm;
			
			mem_new_entry.exclusive = i_decode_exclusive; //TODO confirm proper operation with this (as opposed to the previous Dispatch stage's "exclusive" signal, or whatever it was...)
			mem_new_entry.pc_wen = i_pc_wen;
			mem_new_entry.status_bits_flags_wen = i_status_bits_flags_wen;
			//mem_new_entry.iaddress_sel = i_iaddress_sel;
			//mem_new_entry.daddress_sel = i_daddress_sel; //TODO okay to not have this?
			mem_new_entry.byte_enable_sel = i_byte_enable_sel;
			mem_new_entry.rd_tag = i_tag_nxt;
		end
		default: begin
			alu_new_entry = 0;
			mult_new_entry = 0;
			mem_new_entry = 0;
		end
	endcase
end


/* TODO: What needs to happen from here on down:

1) Set up output logic based on if the incoming stuff is ready versus something earlier being ready
2) Update the reservation station entries based on the above as well
3) Set up forwarding path for incoming data directly to the output if that's the instruction that should be dispatched this cycle
4) Create new combinational vectors for whether/not any given instruction, or the newly input instruction, is currently ready based on current state OR incoming data from the EUs. And, remember that the EUs' outputs are already piped into any incoming instruction and its operands' valid bits.

*/

/*
Update logic:
- Update based on xyz_next_dispatch_idx and xyz_shift_into
- For each entry:
	- if xyz_next_dispatch_idx[16]: update only based on tag inputs from the EUs
	- if xyz_next_dispatch_idx[17]:
		- update based on tag inputs from the EUs
		- add xyz_new_entry to the station if not i_stall
	- If xyz_shift_into[this_station's_index]:
		- shift in the entry from this_station's_index+1 if this_station's_index != 15
		- update this station's reg data and V bits based on tag match from this_station's_index+1 if this_station's_index != 15
		update this entry based on xyz_shift_into
	- If !xyz_shift_into[this_station's_index] and some_tag_matches: update this entry's reg data and V bits
	- If this_station's_index == next_available_station_idx and not i_stall: latch xyz_new_entry at this index
- Next_available_station_idx logic: TODO handle update logic etc. for this with a separate state machine thingiemabobbie
*/

/*xyz_occupied
xyz_next_insert_idx*/

//ALU station
always_ff @(posedge i_rst, posedge i_clk) begin
	if (i_rst) begin
		for (int i=0; i<16; i+=1) alu_station[i] <= 'd0;
		alu_occupied <= 'd0;
	end
	else /*if (i_clk)*/ begin
		for (int i=0; i<16; i+=1) begin
			if (~i_stall && alu_next_insert_idx == i) begin //this also implies ~xyz_occupied[i] by definition
				//this is the destination into which we'll latch the new entry
				alu_occupied[i] <= 1'b1;
				alu_station[i] <= alu_new_entry;
			end
			else if (alu_occupied[i]) begin
				if (alu_shift_into[i]) begin
					/* Special case:
					if i==15 and the station is full this cycle, we must be stalling (due to the way tag retirement works), so all we need to do is set this entry to unoccupied and it will be latched into station 14 per the below logic.
					we'll never get here if i==15 and the station is not full.
					*/
					if (i==15) begin
						alu_occupied[i] <= 1'b0;
						alu_station[i] <= alu_station[i]; //doesn't really matter, but we'll leave it in for the sake of consistency in logical flow
					end
					else begin
						alu_occupied[i] <= alu_occupied[i+1];
						alu_station[i] <= alu_station[i+1];
						
						//Order is important here: data update based on incoming tags must be listed AFTER "alu_station[i] <= alu_station[i+1];"
						if (alu_occupied[i+1]) begin
							if (alu_rn_tag_match[i+1][ALU_MATCH_IDX]) begin
								alu_station[i].rn_valid <= 1'b1;
								alu_station[i].rn <= i_alu_data;
							end
							else if (alu_rn_tag_match[i+1][MULT_MATCH_IDX]) begin
								alu_station[i].rn_valid <= 1'b1;
								alu_station[i].rn <= i_mult_data;
							end
							else if (alu_rn_tag_match[i+1][MEM_MATCH_IDX]) begin
								alu_station[i].rn_valid <= 1'b1;
								alu_station[i].rn <= i_mem_data;
							end
							else if (alu_rs_tag_match[i+1][ALU_MATCH_IDX]) begin
								alu_station[i].rs_valid <= 1'b1;
								alu_station[i].rs <= i_alu_data;
							end
							else if (alu_rs_tag_match[i+1][MULT_MATCH_IDX]) begin
								alu_station[i].rs_valid <= 1'b1;
								alu_station[i].rs <= i_mult_data;
							end
							else if (alu_rs_tag_match[i+1][MEM_MATCH_IDX]) begin
								alu_station[i].rs_valid <= 1'b1;
								alu_station[i].rs <= i_mem_data;
							end
							else if (alu_rm_tag_match[i+1][ALU_MATCH_IDX]) begin
								alu_station[i].rm_valid <= 1'b1;
								alu_station[i].rm <= i_alu_data;
							end
							else if (alu_rm_tag_match[i+1][MULT_MATCH_IDX]) begin
								alu_station[i].rm_valid <= 1'b1;
								alu_station[i].rm <= i_mult_data;
							end
							else if (alu_rm_tag_match[i+1][MEM_MATCH_IDX]) begin
								alu_station[i].rm_valid <= 1'b1;
								alu_station[i].rm <= i_mem_data;
							end
						end
						else begin
							//do nothing; we don't care about the junk in there if it's unoccupied
						end
					end
				end
				else begin
					alu_occupied[i] <= alu_occupied[i];
					alu_station[i] <= alu_station[i];
					
					if (alu_rn_tag_match[i][ALU_MATCH_IDX]) begin
						alu_station[i].rn_valid <= 1'b1;
						alu_station[i].rn <= i_alu_data;
					end
					else if (alu_rn_tag_match[i][MULT_MATCH_IDX]) begin
						alu_station[i].rn_valid <= 1'b1;
						alu_station[i].rn <= i_mult_data;
					end
					else if (alu_rn_tag_match[i][MEM_MATCH_IDX]) begin
						alu_station[i].rn_valid <= 1'b1;
						alu_station[i].rn <= i_mem_data;
					end
					else if (alu_rs_tag_match[i][ALU_MATCH_IDX]) begin
						alu_station[i].rs_valid <= 1'b1;
						alu_station[i].rs <= i_alu_data;
					end
					else if (alu_rs_tag_match[i][MULT_MATCH_IDX]) begin
						alu_station[i].rs_valid <= 1'b1;
						alu_station[i].rs <= i_mult_data;
					end
					else if (alu_rs_tag_match[i][MEM_MATCH_IDX]) begin
						alu_station[i].rs_valid <= 1'b1;
						alu_station[i].rs <= i_mem_data;
					end
					else if (alu_rm_tag_match[i][ALU_MATCH_IDX]) begin
						alu_station[i].rm_valid <= 1'b1;
						alu_station[i].rm <= i_alu_data;
					end
					else if (alu_rm_tag_match[i][MULT_MATCH_IDX]) begin
						alu_station[i].rm_valid <= 1'b1;
						alu_station[i].rm <= i_mult_data;
					end
					else if (alu_rm_tag_match[i][MEM_MATCH_IDX]) begin
						alu_station[i].rm_valid <= 1'b1;
						alu_station[i].rm <= i_mem_data;
					end
				end
			end
			else begin //station is unoccupied and we're not inserting into it, so we really don't care
				alu_occupied[i] <= alu_occupied[i];
				alu_station[i] <= alu_station[i];
			end
		end
	end
end

//Mult station
always_ff @(posedge i_rst, posedge i_clk) begin
	if (i_rst) begin
		for (int i=0; i<16; i+=1) mult_station[i] <= 'd0;
		mult_occupied <= 'd0;
	end
	else /*if (i_clk)*/ begin
		for (int i=0; i<16; i+=1) begin
			if (~i_stall && mult_next_insert_idx == i) begin //this also implies ~xyz_occupied[i] by definition
				//this is the destination into which we'll latch the new entry
				mult_occupied[i] <= 1'b1;
				mult_station[i] <= mult_new_entry;
			end
			else if (mult_occupied[i]) begin
				if (mult_shift_into[i]) begin
					/* Special case:
					if i==15 and the station is full this cycle, we must be stalling (due to the way tag retirement works), so all we need to do is set this entry to unoccupied and it will be latched into station 14 per the below logic.
					we'll never get here if i==15 and the station is not full.
					*/
					if (i==15) begin
						mult_occupied[i] <= 1'b0;
						mult_station[i] <= mult_station[i]; //doesn't really matter, but we'll leave it in for the sake of consistency in logical flow
					end
					else begin
						mult_occupied[i] <= mult_occupied[i+1];
						mult_station[i] <= mult_station[i+1];
						
						//Order is important here: data update based on incoming tags must be listed AFTER "mult_station[i] <= mult_station[i+1];"
						if (mult_occupied[i+1]) begin
							if (mult_rn_tag_match[i+1][ALU_MATCH_IDX]) begin
								mult_station[i].rn_valid <= 1'b1;
								mult_station[i].rn <= i_alu_data;
							end
							else if (mult_rn_tag_match[i+1][MULT_MATCH_IDX]) begin
								mult_station[i].rn_valid <= 1'b1;
								mult_station[i].rn <= i_mult_data;
							end
							else if (mult_rn_tag_match[i+1][MEM_MATCH_IDX]) begin
								mult_station[i].rn_valid <= 1'b1;
								mult_station[i].rn <= i_mem_data;
							end
							else if (mult_rs_tag_match[i+1][ALU_MATCH_IDX]) begin
								mult_station[i].rs_valid <= 1'b1;
								mult_station[i].rs <= i_alu_data;
							end
							else if (mult_rs_tag_match[i+1][MULT_MATCH_IDX]) begin
								mult_station[i].rs_valid <= 1'b1;
								mult_station[i].rs <= i_mult_data;
							end
							else if (mult_rs_tag_match[i+1][MEM_MATCH_IDX]) begin
								mult_station[i].rs_valid <= 1'b1;
								mult_station[i].rs <= i_mem_data;
							end
							else if (mult_rm_tag_match[i+1][ALU_MATCH_IDX]) begin
								mult_station[i].rm_valid <= 1'b1;
								mult_station[i].rm <= i_alu_data;
							end
							else if (mult_rm_tag_match[i+1][MULT_MATCH_IDX]) begin
								mult_station[i].rm_valid <= 1'b1;
								mult_station[i].rm <= i_mult_data;
							end
							else if (mult_rm_tag_match[i+1][MEM_MATCH_IDX]) begin
								mult_station[i].rm_valid <= 1'b1;
								mult_station[i].rm <= i_mem_data;
							end
						end
						else begin
							//do nothing; we don't care about the junk in there if it's unoccupied
						end
					end
				end
				else begin
					mult_occupied[i] <= mult_occupied[i];
					mult_station[i] <= mult_station[i];
					
					if (mult_rn_tag_match[i][ALU_MATCH_IDX]) begin
						mult_station[i].rn_valid <= 1'b1;
						mult_station[i].rn <= i_alu_data;
					end
					else if (mult_rn_tag_match[i][MULT_MATCH_IDX]) begin
						mult_station[i].rn_valid <= 1'b1;
						mult_station[i].rn <= i_mult_data;
					end
					else if (mult_rn_tag_match[i][MEM_MATCH_IDX]) begin
						mult_station[i].rn_valid <= 1'b1;
						mult_station[i].rn <= i_mem_data;
					end
					else if (mult_rs_tag_match[i][ALU_MATCH_IDX]) begin
						mult_station[i].rs_valid <= 1'b1;
						mult_station[i].rs <= i_alu_data;
					end
					else if (mult_rs_tag_match[i][MULT_MATCH_IDX]) begin
						mult_station[i].rs_valid <= 1'b1;
						mult_station[i].rs <= i_mult_data;
					end
					else if (mult_rs_tag_match[i][MEM_MATCH_IDX]) begin
						mult_station[i].rs_valid <= 1'b1;
						mult_station[i].rs <= i_mem_data;
					end
					else if (mult_rm_tag_match[i][ALU_MATCH_IDX]) begin
						mult_station[i].rm_valid <= 1'b1;
						mult_station[i].rm <= i_alu_data;
					end
					else if (mult_rm_tag_match[i][MULT_MATCH_IDX]) begin
						mult_station[i].rm_valid <= 1'b1;
						mult_station[i].rm <= i_mult_data;
					end
					else if (mult_rm_tag_match[i][MEM_MATCH_IDX]) begin
						mult_station[i].rm_valid <= 1'b1;
						mult_station[i].rm <= i_mem_data;
					end
				end
			end
			else begin //station is unoccupied and we're not inserting into it, so we really don't care
				mult_occupied[i] <= mult_occupied[i];
				mult_station[i] <= mult_station[i];
			end
		end
	end
end

//Mem station
always_ff @(posedge i_rst, posedge i_clk) begin
	if (i_rst) begin
		for (int i=0; i<16; i+=1) mem_station[i] <= 'd0;
		mem_occupied <= 'd0;
	end
	else /*if (i_clk)*/ begin
		for (int i=0; i<16; i+=1) begin
			if (~i_stall && mem_next_insert_idx == i) begin //this also implies ~xyz_occupied[i] by definition
				//this is the destination into which we'll latch the new entry
				mem_occupied[i] <= 1'b1;
				mem_station[i] <= mem_new_entry;
			end
			else if (mem_occupied[i]) begin
				if (mem_shift_into[i]) begin
					/* Special case:
					if i==15 and the station is full this cycle, we must be stalling (due to the way tag retirement works), so all we need to do is set this entry to unoccupied and it will be latched into station 14 per the below logic.
					we'll never get here if i==15 and the station is not full.
					*/
					if (i==15) begin
						mem_occupied[i] <= 1'b0;
						mem_station[i] <= mem_station[i]; //doesn't really matter, but we'll leave it in for the sake of consistency in logical flow
					end
					else begin
						mem_occupied[i] <= mem_occupied[i+1];
						mem_station[i] <= mem_station[i+1];
						
						//Order is important here: data update based on incoming tags must be listed AFTER "mem_station[i] <= mem_station[i+1];"
						if (mem_occupied[i+1]) begin
							if (mem_rn_tag_match[i+1][ALU_MATCH_IDX]) begin
								mem_station[i].rn_valid <= 1'b1;
								mem_station[i].rn <= i_alu_data;
							end
							else if (mem_rn_tag_match[i+1][MULT_MATCH_IDX]) begin
								mem_station[i].rn_valid <= 1'b1;
								mem_station[i].rn <= i_mult_data;
							end
							else if (mem_rn_tag_match[i+1][MEM_MATCH_IDX]) begin
								mem_station[i].rn_valid <= 1'b1;
								mem_station[i].rn <= i_mem_data;
							end
							else if (mem_rs_tag_match[i+1][ALU_MATCH_IDX]) begin
								mem_station[i].rs_valid <= 1'b1;
								mem_station[i].rs <= i_alu_data;
							end
							else if (mem_rs_tag_match[i+1][MULT_MATCH_IDX]) begin
								mem_station[i].rs_valid <= 1'b1;
								mem_station[i].rs <= i_mult_data;
							end
							else if (mem_rs_tag_match[i+1][MEM_MATCH_IDX]) begin
								mem_station[i].rs_valid <= 1'b1;
								mem_station[i].rs <= i_mem_data;
							end
							else if (mem_rm_tag_match[i+1][ALU_MATCH_IDX]) begin
								mem_station[i].rm_valid <= 1'b1;
								mem_station[i].rm <= i_alu_data;
							end
							else if (mem_rm_tag_match[i+1][MULT_MATCH_IDX]) begin
								mem_station[i].rm_valid <= 1'b1;
								mem_station[i].rm <= i_mult_data;
							end
							else if (mem_rm_tag_match[i+1][MEM_MATCH_IDX]) begin
								mem_station[i].rm_valid <= 1'b1;
								mem_station[i].rm <= i_mem_data;
							end
						end
						else begin
							//do nothing; we don't care about the junk in there if it's unoccupied
						end
					end
				end
				else begin
					mem_occupied[i] <= mem_occupied[i];
					mem_station[i] <= mem_station[i];
					
					if (mem_rn_tag_match[i][ALU_MATCH_IDX]) begin
						mem_station[i].rn_valid <= 1'b1;
						mem_station[i].rn <= i_alu_data;
					end
					else if (mem_rn_tag_match[i][MULT_MATCH_IDX]) begin
						mem_station[i].rn_valid <= 1'b1;
						mem_station[i].rn <= i_mult_data;
					end
					else if (mem_rn_tag_match[i][MEM_MATCH_IDX]) begin
						mem_station[i].rn_valid <= 1'b1;
						mem_station[i].rn <= i_mem_data;
					end
					else if (mem_rs_tag_match[i][ALU_MATCH_IDX]) begin
						mem_station[i].rs_valid <= 1'b1;
						mem_station[i].rs <= i_alu_data;
					end
					else if (mem_rs_tag_match[i][MULT_MATCH_IDX]) begin
						mem_station[i].rs_valid <= 1'b1;
						mem_station[i].rs <= i_mult_data;
					end
					else if (mem_rs_tag_match[i][MEM_MATCH_IDX]) begin
						mem_station[i].rs_valid <= 1'b1;
						mem_station[i].rs <= i_mem_data;
					end
					else if (mem_rm_tag_match[i][ALU_MATCH_IDX]) begin
						mem_station[i].rm_valid <= 1'b1;
						mem_station[i].rm <= i_alu_data;
					end
					else if (mem_rm_tag_match[i][MULT_MATCH_IDX]) begin
						mem_station[i].rm_valid <= 1'b1;
						mem_station[i].rm <= i_mult_data;
					end
					else if (mem_rm_tag_match[i][MEM_MATCH_IDX]) begin
						mem_station[i].rm_valid <= 1'b1;
						mem_station[i].rm <= i_mem_data;
					end
				end
			end
			else begin //station is unoccupied and we're not inserting into it, so we really don't care
				mem_occupied[i] <= mem_occupied[i];
				mem_station[i] <= mem_station[i];
			end
		end
	end
end



//Output logic for ALU station
//Base it on alu_next_dispatch_idx, mult_next_dispatch_idx, and mem_next_dispatch_idx
//Note that the register file automatically forwards operands from the EUs for a new instruction if the tags match
always_ff @(posedge i_rst, posedge i_clk) begin
	if (i_rst) begin
		o_instr_valid_alu <= '0;
		//don't care about other outputs since o_instr_valid is false
	end
	else /*if (i_clk)*/ begin
		if (~alu_next_dispatch_idx[4]) begin
			o_instr_valid_alu <= 1'b1;
			o_rn_alu <= alu_station[alu_next_dispatch_idx].rn_valid				?	alu_station[alu_next_dispatch_idx].rn:
						alu_rn_tag_match[alu_next_dispatch_idx][ALU_MATCH_IDX]	? 	i_alu_data:
						alu_rn_tag_match[alu_next_dispatch_idx][MULT_MATCH_IDX]	? 	i_mult_data:
						/*alu_rn_tag_match[alu_next_dispatch_idx][MEM_MATCH_IDX]	?*/ 	i_mem_data; //based on the above IF statement, this will always be true if the other 3 cases are false
			o_rs_alu <= alu_station[alu_next_dispatch_idx].rs_valid				?	alu_station[alu_next_dispatch_idx].rs:
						alu_rs_tag_match[alu_next_dispatch_idx][ALU_MATCH_IDX]	? 	i_alu_data:
						alu_rs_tag_match[alu_next_dispatch_idx][MULT_MATCH_IDX]	? 	i_mult_data:
																				i_mem_data;
			o_rm_alu <= alu_station[alu_next_dispatch_idx].rm_valid				?	alu_station[alu_next_dispatch_idx].rm:
						alu_rm_tag_match[alu_next_dispatch_idx][ALU_MATCH_IDX]	? 	i_alu_data:
						alu_rm_tag_match[alu_next_dispatch_idx][MULT_MATCH_IDX]	? 	i_mult_data:
																					i_mem_data;
			o_status_bits_flags_alu <= alu_station[alu_next_dispatch_idx].status_bits_flags;
			o_use_carry_in_alu <= alu_station[alu_next_dispatch_idx].use_carry_in;
			o_imm32_alu <= alu_station[alu_next_dispatch_idx].imm32;
			o_imm_shift_amount_alu <= alu_station[alu_next_dispatch_idx].imm_shift_amount;
			o_shift_imm_zero_alu <= alu_station[alu_next_dispatch_idx].shift_imm_zero;
			o_barrel_shift_amount_sel_alu <= alu_station[alu_next_dispatch_idx].barrel_shift_amount_sel;
			o_barrel_shift_data_sel_alu <= alu_station[alu_next_dispatch_idx].barrel_shift_data_sel;
			o_barrel_shift_function_alu <= alu_station[alu_next_dispatch_idx].barrel_shift_function;
			o_alu_function_alu <= alu_station[alu_next_dispatch_idx].alu_function;
			o_pc_wen_alu <= alu_station[alu_next_dispatch_idx].pc_wen;
			o_status_bits_flags_wen_alu <= alu_station[alu_next_dispatch_idx].status_bits_flags_wen;
			o_rd_tag_alu <= alu_station[alu_next_dispatch_idx].rd_tag;
		end
		else if (alu_next_dispatch_idx == 5'd16) begin
			o_instr_valid_alu <= 1'b1;
			o_rn_alu <= i_rn;
			o_rs_alu <= i_rs;
			o_rm_alu <= i_rm;
			o_imm32_alu <= i_imm32;
			o_imm_shift_amount_alu <= i_imm_shift_amount;
			o_shift_imm_zero_alu <= i_shift_imm_zero;
			o_barrel_shift_amount_sel_alu <= i_barrel_shift_amount_sel;
			o_barrel_shift_data_sel_alu <= i_barrel_shift_data_sel;
			o_barrel_shift_function_alu <= i_barrel_shift_function;
			o_alu_function_alu <= i_alu_function;
			o_pc_wen_alu <= i_pc_wen;
			o_status_bits_flags_wen_alu <= i_status_bits_flags_wen;
			o_rd_tag_alu <= i_tag_nxt;
		end
		else begin //nothing's ready; set outputs to "safe" state
			o_instr_valid_alu <= 1'b0;
			//We don't care about the rest of the outputs if valid is held low
		end
	end
end


//Output logic for mult station
always_ff @(posedge i_rst, posedge i_clk) begin
	if (i_rst) begin
		o_instr_valid_mult <= '0;
		//don't care about other outputs since o_instr_valid is false
	end
	else /*if (i_clk)*/ begin
		if (~mult_next_dispatch_idx[4]) begin
			o_instr_valid_mult <= 1'b1;
			o_rn_mult <=	mult_station[mult_next_dispatch_idx].rn_valid				?	mult_station[mult_next_dispatch_idx].rn:
							mult_rn_tag_match[mult_next_dispatch_idx][ALU_MATCH_IDX]	? 	i_alu_data:
							mult_rn_tag_match[mult_next_dispatch_idx][MULT_MATCH_IDX]	? 	i_mult_data:
							/*mult_rn_tag_match[mult_next_dispatch_idx][MEM_MATCH_IDX]	?*/ 	i_mem_data; //based on the above IF statement, this will always be true if the other 3 cases are false
			o_rs_mult <= 	mult_station[mult_next_dispatch_idx].rs_valid				?	mult_station[mult_next_dispatch_idx].rs:
							mult_rs_tag_match[mult_next_dispatch_idx][ALU_MATCH_IDX]	? 	i_alu_data:
							mult_rs_tag_match[mult_next_dispatch_idx][MULT_MATCH_IDX]	? 	i_mult_data:
																							i_mem_data;
			o_rm_mult <= 	mult_station[mult_next_dispatch_idx].rm_valid				?	mult_station[mult_next_dispatch_idx].rm:
							mult_rm_tag_match[mult_next_dispatch_idx][ALU_MATCH_IDX]	? 	i_alu_data:
							mult_rm_tag_match[mult_next_dispatch_idx][MULT_MATCH_IDX]	? 	i_mult_data:
																							i_mem_data;
			o_multiply_function_mult <= mult_station[mult_next_dispatch_idx].multiply_function;
			o_use_carry_in_mult <= mult_station[mult_next_dispatch_idx].use_carry_in;
			o_pc_wen_mult <= mult_station[mult_next_dispatch_idx].pc_wen;
			o_status_bits_flags_wen_mult <= mult_station[mult_next_dispatch_idx].status_bits_flags_wen;
			o_rd_tag_mult <= mult_station[mult_next_dispatch_idx].rd_tag;
		end
		else if (mult_next_dispatch_idx == 5'd16) begin
			o_instr_valid_mult <= 1'b1;
			o_rn_mult <= i_rn;
			o_rs_mult <= i_rs;
			o_rm_mult <= i_rm;
			o_multiply_function_mult <= i_multiply_function;
			o_use_carry_in_mult <= i_use_carry_in;
			o_pc_wen_mult <= i_pc_wen;
			o_status_bits_flags_wen_mult <= i_status_bits_flags_wen;
			o_rd_tag_mult <= i_tag_nxt;
		end
		else begin //nothing's ready; set outputs to "safe" state
			o_instr_valid_mult <= 1'b0;
			//We don't care about the rest of the outputs if valid is held low
		end
	end
end


//Output logic for mem station
always_ff @(posedge i_rst, posedge i_clk) begin
	if (i_rst) begin
		o_instr_valid_mem <= '0;
		//don't care about other outputs since o_instr_valid is false
	end
	else /*if (i_clk)*/ begin
		if (~mem_next_dispatch_idx[4]) begin
			o_instr_valid_mem <= 1'b1;
			o_rn_mem <= mem_station[mem_next_dispatch_idx].rn_valid				?	mem_station[mem_next_dispatch_idx].rn:
						mem_rn_tag_match[mem_next_dispatch_idx][ALU_MATCH_IDX]	? 	i_alu_data:
						mem_rn_tag_match[mem_next_dispatch_idx][MULT_MATCH_IDX]	? 	i_mult_data:
						/*mem_rn_tag_match[mem_next_dispatch_idx][MEM_MATCH_IDX]	?*/ 	i_mem_data; //based on the above IF statement, this will always be true if the other 3 cases are false
			o_rs_mem <= mem_station[mem_next_dispatch_idx].rs_valid				?	mem_station[mem_next_dispatch_idx].rs:
						mem_rs_tag_match[mem_next_dispatch_idx][ALU_MATCH_IDX]	? 	i_alu_data:
						mem_rs_tag_match[mem_next_dispatch_idx][MULT_MATCH_IDX]	? 	i_mult_data:
																i_mem_data;
			o_rm_mem <= mem_station[mem_next_dispatch_idx].rm_valid				?	mem_station[mem_next_dispatch_idx].rm:
						mem_rm_tag_match[mem_next_dispatch_idx][ALU_MATCH_IDX]	? 	i_alu_data:
						mem_rm_tag_match[mem_next_dispatch_idx][MULT_MATCH_IDX]	? 	i_mult_data:
																i_mem_data;
			o_exclusive_mem <= mem_station[mem_next_dispatch_idx].exclusive;
			o_pc_wen_mem <= mem_station[mem_next_dispatch_idx].pc_wen;
			o_status_bits_flags_wen_mem <= mem_station[mem_next_dispatch_idx].status_bits_flags_wen;
			/*o_iaddress_sel_mem <= mem_station[mem_next_dispatch_idx].iaddress_sel;
			o_daddress_sel_mem <= mem_station[mem_next_dispatch_idx].daddress_sel;*/
			o_byte_enable_sel_mem <= mem_station[mem_next_dispatch_idx].byte_enable_sel;
			o_rd_tag_mem <= mem_station[mem_next_dispatch_idx].rd_tag;
		end
		else if (mem_next_dispatch_idx == 5'd16) begin
			o_instr_valid_mem <= 1'b1;
			o_rn_mem <= i_rn;
			o_rs_mem <= i_rs;
			o_rm_mem <= i_rm;
			o_exclusive_mem <= i_decode_exclusive; //TODO confirm proper operation with this as opposed to the doohicky the previous Dispatch/Execute stage had in it
			o_pc_wen_mem <= i_pc_wen;
			o_status_bits_flags_wen_mem <= i_status_bits_flags_wen;
			/*o_iaddress_sel_mem <= i_iaddress_sel;
			o_daddress_sel_mem <= i_daddress_sel;*/
			o_byte_enable_sel_mem <= i_byte_enable_sel;
			o_rd_tag_mem <= i_tag_nxt;
		end
		else begin //nothing's ready; set outputs to "safe" state
			o_instr_valid_mem <= 1'b0;
			//We don't care about the rest of the outputs if valid is held low
		end
	end
end

endmodule