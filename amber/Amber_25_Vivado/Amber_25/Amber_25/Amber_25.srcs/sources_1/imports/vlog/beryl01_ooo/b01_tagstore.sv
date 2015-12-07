/*

How this works:
- Instantiated from inside the main Dispatch module
- Inputs:
	- Instruction type (ALU, mult, or mem)
	- Whether/not the instruction is ready for dispatch this cycle
	- Whether/not another instruction currently in the reservation station is ready for dispatch this cycle
	- V bits and tags from ALU, mult, and mem execution units
- Outputs:
	- 6-bit vector of what this new instruction's tag will be
	- Bit of whether/not the reservation station is full and we therefore need to stall

*/

module b01_tagstore (
	input logic			i_clk,
	input logic			i_rst,
	input logic			i_stall,
	
	input logic [1:0]	i_instr_type, //00=alu, 01=mult, 10=mem. TODO: `define these in a header file
	//we actually don't care if the instruction is being dispatched this cycle, it still needs a tag
	//we only care what's being RETIRED this cycle to make a tag available again
	//input logic i_instr_mem_with_writeback,
	
	input logic			i_alu_valid,
	input logic [5:0]	i_alu_tag,
	input logic 		i_mult_valid,
	input logic [5:0]	i_mult_tag,
	input logic 		i_mem_valid,
	input logic [5:0]	i_mem_tag,
	
	output logic		o_station_full,
	output logic [5:0] o_tag_alu_mult_1,
	//output logic [5:0] o_tag_alu_2,
	output logic [5:0] o_tag_mem
);

/*

Tag store logic:
- 64 available tags. We allocate 16 per reservation station.
- Top 2 bits define which reservation station: 00=ALU, 01=multiplier, 10=memory.
- The last 16 tags (top 2 bits = 11) are reserved for eventual implementation of LDM/STM in a 4th "station".

For each station:
- Each station will operate like a queue, s.t. newer now-ready instructions are always preferred for dispatch (though not for retirement) than newer instructions
- Each incoming instruction will receive the "first available" unused tag.
- A 1-hot bit vector will denote which tags are currently in use. (Yes, it's a long combinational path, and one that'd be good to rectify at some point...)
- A tag will be considered available if it is either designated as available in the current store or it is being retired on one of the tag buses this cycle.

*/

logic [31:0] 	alu_tag_available;
logic [15:0]	mult_tag_available,
				      mem_tag_available;
				
logic [6:0]		alu_tag_nxt_1,
              //alu_tag_nxt_2,
				      mult_tag_nxt,
				      mem_tag_nxt;

//Define which will be the next tag supplied for each station
assign alu_tag_nxt_1 	= 	alu_tag_available[0 ] ? {2'b00,4'd0}:
						alu_tag_available[1 ] ? {2'b00,4'd1}:
						alu_tag_available[2 ] ? {2'b00,4'd2}:
						alu_tag_available[3 ] ? {2'b00,4'd3}:
						alu_tag_available[4 ] ? {2'b00,4'd4}:
						alu_tag_available[5 ] ? {2'b00,4'd5}:
						alu_tag_available[6 ] ? {2'b00,4'd6}:
						alu_tag_available[7 ] ? {2'b00,4'd7}:
						alu_tag_available[8 ] ? {2'b00,4'd8}:
						alu_tag_available[9 ] ? {2'b00,4'd9}:
						alu_tag_available[10] ? {2'b00,4'd10}:
						alu_tag_available[11] ? {2'b00,4'd11}:
						alu_tag_available[12] ? {2'b00,4'd12}:
						alu_tag_available[13] ? {2'b00,4'd13}:
						alu_tag_available[14] ? {2'b00,4'd14}:
						alu_tag_available[15] ? {2'b00,4'd15}:
						alu_tag_available[16] ? {2'b11,4'd0}:
						alu_tag_available[17] ? {2'b11,4'd1}:
						alu_tag_available[18] ? {2'b11,4'd2}:
						alu_tag_available[19] ? {2'b11,4'd3}:
						alu_tag_available[20] ? {2'b11,4'd4}:
						alu_tag_available[21] ? {2'b11,4'd5}:
						alu_tag_available[22] ? {2'b11,4'd6}:
						alu_tag_available[23] ? {2'b11,4'd7}:
						alu_tag_available[24] ? {2'b11,4'd8}:
						alu_tag_available[25] ? {2'b11,4'd9}:
						alu_tag_available[26] ? {2'b11,4'd10}:
						alu_tag_available[27] ? {2'b11,4'd11}:
						alu_tag_available[28] ? {2'b11,4'd12}:
						alu_tag_available[29] ? {2'b11,4'd13}:
						alu_tag_available[30] ? {2'b11,4'd14}:
						alu_tag_available[31] ? {2'b11,4'd15}:
						i_alu_valid			  ? i_alu_tag:
												6'd0; //default case; if we get here we'll assert full anyway

/*assign alu_tag_nxt_2 	= 	(alu_tag_nxt_1[4:0] != 5'd0) && alu_tag_available[0 ] ? {2'b00,4'd0}:
						(alu_tag_nxt_1[4:0] != 5'd1) && alu_tag_available[1 ] ? {2'b00,4'd1}:
						(alu_tag_nxt_1[4:0] != 5'd2) && alu_tag_available[2 ] ? {2'b00,4'd2}:
						(alu_tag_nxt_1[4:0] != 5'd3) && alu_tag_available[3 ] ? {2'b00,4'd3}:
						(alu_tag_nxt_1[4:0] != 5'd4) && alu_tag_available[4 ] ? {2'b00,4'd4}:
						(alu_tag_nxt_1[4:0] != 5'd5) && alu_tag_available[5 ] ? {2'b00,4'd5}:
						(alu_tag_nxt_1[4:0] != 5'd6) && alu_tag_available[6 ] ? {2'b00,4'd6}:
						(alu_tag_nxt_1[4:0] != 5'd7) && alu_tag_available[7 ] ? {2'b00,4'd7}:
						(alu_tag_nxt_1[4:0] != 5'd8) && alu_tag_available[8 ] ? {2'b00,4'd8}:
						(alu_tag_nxt_1[4:0] != 5'd9) && alu_tag_available[9 ] ? {2'b00,4'd9}:
						(alu_tag_nxt_1[4:0] != 5'd10) && alu_tag_available[10] ? {2'b00,4'd10}:
						(alu_tag_nxt_1[4:0] != 5'd11) && alu_tag_available[11] ? {2'b00,4'd11}:
						(alu_tag_nxt_1[4:0] != 5'd12) && alu_tag_available[12] ? {2'b00,4'd12}:
						(alu_tag_nxt_1[4:0] != 5'd13) && alu_tag_available[13] ? {2'b00,4'd13}:
						(alu_tag_nxt_1[4:0] != 5'd14) && alu_tag_available[14] ? {2'b00,4'd14}:
						(alu_tag_nxt_1[4:0] != 5'd15) && alu_tag_available[15] ? {2'b00,4'd15}:
						(alu_tag_nxt_1[4:0] != 5'd16) && alu_tag_available[16] ? {2'b11,4'd0}:
						(alu_tag_nxt_1[4:0] != 5'd17) && alu_tag_available[17] ? {2'b11,4'd1}:
						(alu_tag_nxt_1[4:0] != 5'd18) && alu_tag_available[18] ? {2'b11,4'd2}:
						(alu_tag_nxt_1[4:0] != 5'd19) && alu_tag_available[19] ? {2'b11,4'd3}:
						(alu_tag_nxt_1[4:0] != 5'd20) && alu_tag_available[20] ? {2'b11,4'd4}:
						(alu_tag_nxt_1[4:0] != 5'd21) && alu_tag_available[21] ? {2'b11,4'd5}:
						(alu_tag_nxt_1[4:0] != 5'd22) && alu_tag_available[22] ? {2'b11,4'd6}:
						(alu_tag_nxt_1[4:0] != 5'd23) && alu_tag_available[23] ? {2'b11,4'd7}:
						(alu_tag_nxt_1[4:0] != 5'd24) && alu_tag_available[24] ? {2'b11,4'd8}:
						(alu_tag_nxt_1[4:0] != 5'd25) && alu_tag_available[25] ? {2'b11,4'd9}:
						(alu_tag_nxt_1[4:0] != 5'd26) && alu_tag_available[26] ? {2'b11,4'd10}:
						(alu_tag_nxt_1[4:0] != 5'd27) && alu_tag_available[27] ? {2'b11,4'd11}:
						(alu_tag_nxt_1[4:0] != 5'd28) && alu_tag_available[28] ? {2'b11,4'd12}:
						(alu_tag_nxt_1[4:0] != 5'd29) && alu_tag_available[29] ? {2'b11,4'd13}:
						(alu_tag_nxt_1[4:0] != 5'd30) && alu_tag_available[30] ? {2'b11,4'd14}:
						(alu_tag_nxt_1[4:0] != 5'd31) && alu_tag_available[31] ? {2'b11,4'd15}:
						(alu_tag_nxt_1 != i_alu_tag) && i_alu_valid			  ? i_alu_tag:
												6'd0; //default case; if we get here we'll assert full anyway*/
						
assign mult_tag_nxt = 	mult_tag_available[0 ] ? {2'b01,4'd0}:
						mult_tag_available[1 ] ? {2'b01,4'd1}:
						mult_tag_available[2 ] ? {2'b01,4'd2}:
						mult_tag_available[3 ] ? {2'b01,4'd3}:
						mult_tag_available[4 ] ? {2'b01,4'd4}:
						mult_tag_available[5 ] ? {2'b01,4'd5}:
						mult_tag_available[6 ] ? {2'b01,4'd6}:
						mult_tag_available[7 ] ? {2'b01,4'd7}:
						mult_tag_available[8 ] ? {2'b01,4'd8}:
						mult_tag_available[9 ] ? {2'b01,4'd9}:
						mult_tag_available[10] ? {2'b01,4'd10}:
						mult_tag_available[11] ? {2'b01,4'd11}:
						mult_tag_available[12] ? {2'b01,4'd12}:
						mult_tag_available[13] ? {2'b01,4'd13}:
						mult_tag_available[14] ? {2'b01,4'd14}:
						mult_tag_available[15] ? {2'b01,4'd15}:
						i_mult_valid		   ? i_mult_tag:
												6'd0; //default case; if we get here we'll assert full anyway
						
assign mem_tag_nxt 	= 	mem_tag_available[0 ] ? {2'b10,4'd0}:
						mem_tag_available[1 ] ? {2'b10,4'd1}:
						mem_tag_available[2 ] ? {2'b10,4'd2}:
						mem_tag_available[3 ] ? {2'b10,4'd3}:
						mem_tag_available[4 ] ? {2'b10,4'd4}:
						mem_tag_available[5 ] ? {2'b10,4'd5}:
						mem_tag_available[6 ] ? {2'b10,4'd6}:
						mem_tag_available[7 ] ? {2'b10,4'd7}:
						mem_tag_available[8 ] ? {2'b10,4'd8}:
						mem_tag_available[9 ] ? {2'b10,4'd9}:
						mem_tag_available[10] ? {2'b10,4'd10}:
						mem_tag_available[11] ? {2'b10,4'd11}:
						mem_tag_available[12] ? {2'b10,4'd12}:
						mem_tag_available[13] ? {2'b10,4'd13}:
						mem_tag_available[14] ? {2'b10,4'd14}:
						mem_tag_available[15] ? {2'b10,4'd15}:
						i_mem_valid			  ? i_mem_tag:
												6'd0; //default case; if we get here we'll assert full anyway

//Tag-available update logic
always_ff @(posedge i_rst, posedge i_clk) begin
	if (i_rst) begin
		alu_tag_available <= 32'hffff_ffff;
		mult_tag_available <= 16'hffff;
		mem_tag_available <= 16'hffff;
	end
	else begin
		if (i_stall/*_all*/ || i_instr_type==2'b11) begin //if stalled or instruction is control flow
			//make tags available if stalled, but don't make any unavailable
			for (int i=0; i<15; i+=1) begin
          if (alu_tag_available[i] || (i_alu_tag[3:0]==i && i_alu_valid && i_alu_tag[5:4]==2'b00)) alu_tag_available[i] <= 1'b1;
          else alu_tag_available[i] <= alu_tag_available[i];
          if (alu_tag_available[i+16] || (i_alu_tag[3:0]==i && i_alu_valid && i_alu_tag[5:4]==2'b11)) alu_tag_available[i+16] <= 1'b1;
          else alu_tag_available[i+16] <= alu_tag_available[i+16];
          
          if (mult_tag_available[i] || (i_mult_tag[3:0]==i && i_mult_valid)) mult_tag_available[i] <= 1'b1;
          else mult_tag_available[i] <= mult_tag_available[i];
          
          if (mem_tag_available[i] || (i_mem_tag[3:0]==i && i_mem_valid)) mem_tag_available[i] <= 1'b1;
          else mem_tag_available[i] <= mem_tag_available[i];
      end
		end
		/*else if (i_stall_new) begin
			//don't update available based on what tag we say is available this cycle, but continue retiring instructions.
			//this will be the case if the reservation station is full but we still need to retire so we can make forward progress.
		end*/
		else begin
			for (int i=0; i<15; i+=1) begin
				if ((alu_tag_available[i] || (i_alu_tag[3:0]==i && i_alu_valid && i_alu_tag[5:4]==2'b00)) && (i_instr_type == 2'b01 || (alu_tag_nxt_1[3:0]!=i/* && (!i_instr_mem_with_writeback || alu_tag_nxt_2!=i)*/))) alu_tag_available[i] <= 1'b1;
				else alu_tag_available[i] <= 1'b0;
        if ((alu_tag_available[i+16] || (i_alu_tag[3:0]==i && i_alu_valid && i_alu_tag[5:4]==2'b11)) && (i_instr_type == 2'b01 || (alu_tag_nxt_1[3:0]!=(i+16)/* && (!i_instr_mem_with_writeback || alu_tag_nxt_2!=(i+16))*/))) alu_tag_available[i+16] <= 1'b1;
        else alu_tag_available[i+16] <= 1'b0;
				
				if ((mult_tag_available[i] || (i_mult_tag[3:0]==i && i_mult_valid)) && (i_instr_type != 2'b01 || mult_tag_nxt[3:0]!=i)) mult_tag_available[i] <= 1'b1;
				else mult_tag_available[i] <= 1'b0;
				
				if ((mem_tag_available[i] || (i_mem_tag[3:0]==i && i_mem_valid)) && (i_instr_type != 2'b10 || mem_tag_nxt[3:0]!=i)) mem_tag_available[i] <= 1'b1;
				else mem_tag_available[i] <= 1'b0;
			end
		end
	end
	
end


//Output logic
always_comb begin
  //o_tag_alu_2 = 6'b111111; //default; only relevant if we have a pre- or post-indexed writeback to a register
  o_tag_mem = 6'b111111; //default; irrelevant for *actual* memops
	case (i_instr_type)
		2'b00: begin //alu station
			o_station_full	= (alu_tag_available == 32'h0000_0000/*ffff*/) & ~i_alu_valid;
			o_tag_alu_mult_1			= alu_tag_nxt_1;
		end
		2'b01: begin //multiplier station
			o_station_full	= (mult_tag_available == 16'h0000/*ffff*/) & ~i_mult_valid;
			o_tag_alu_mult_1			= mult_tag_nxt;
		end
		2'b10: begin //memory station; this means we need both ALU-related logic and mem-related logic
			o_station_full	= ((mem_tag_available == 16'h0000/*ffff*/) & ~i_mem_valid) & ((alu_tag_available == 32'h0000_0000) & ~i_alu_valid);
			o_tag_mem			= mem_tag_nxt;
			o_tag_alu_mult_1 = alu_tag_nxt_1;
			//o_tag_alu_2 = alu_tag_nxt_2;
		end
		2'b11: begin
			//it's a "pure" control flow instruction, so we don't care about reservation station availability
			o_station_full	= 1'b0;
			o_tag_alu_mult_1			= 6'b111111;
			//o_tag_alu_2 = 6'b111111;
			o_tag_mem = 6'b111111;
		end
	endcase
end


endmodule
