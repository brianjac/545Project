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
	
	input logic			i_alu_valid,
	input logic [5:0]	i_alu_tag,
	input logic 		i_mult_valid,
	input logic [5:0]	i_mult_tag,
	input logic 		i_mem_valid,
	input logic [5:0]	i_mem_tag,
	
	output logic		o_station_full,
	output logic [5:0]	o_tag
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

logic [15:0] 	alu_tag_available,
				mult_tag_available,
				mem_tag_available;
				
logic [3:0]		alu_tag_nxt,
				mult_tag_nxt,
				mem_tag_nxt;

//Define which will be the next tag supplied for each station
assign alu_tag_nxt 	= 	alu_tag_available[0 ] ? 4'd0:
						alu_tag_available[1 ] ? 4'd1:
						alu_tag_available[2 ] ? 4'd2:
						alu_tag_available[3 ] ? 4'd3:
						alu_tag_available[4 ] ? 4'd4:
						alu_tag_available[5 ] ? 4'd5:
						alu_tag_available[6 ] ? 4'd6:
						alu_tag_available[7 ] ? 4'd7:
						alu_tag_available[8 ] ? 4'd8:
						alu_tag_available[9 ] ? 4'd9:
						alu_tag_available[10] ? 4'd10:
						alu_tag_available[11] ? 4'd11:
						alu_tag_available[12] ? 4'd12:
						alu_tag_available[13] ? 4'd13:
						alu_tag_available[14] ? 4'd14:
						alu_tag_available[15] ? 4'd15:
						i_alu_valid			  ? i_alu_tag[3:0]:
												4'd0; //default case; if we get here we'll assert full anyway
						
assign mult_tag_nxt = 	mult_tag_available[0 ] ? 4'd0:
						mult_tag_available[1 ] ? 4'd1:
						mult_tag_available[2 ] ? 4'd2:
						mult_tag_available[3 ] ? 4'd3:
						mult_tag_available[4 ] ? 4'd4:
						mult_tag_available[5 ] ? 4'd5:
						mult_tag_available[6 ] ? 4'd6:
						mult_tag_available[7 ] ? 4'd7:
						mult_tag_available[8 ] ? 4'd8:
						mult_tag_available[9 ] ? 4'd9:
						mult_tag_available[10] ? 4'd10:
						mult_tag_available[11] ? 4'd11:
						mult_tag_available[12] ? 4'd12:
						mult_tag_available[13] ? 4'd13:
						mult_tag_available[14] ? 4'd14:
						mult_tag_available[15] ? 4'd15:
						i_mult_valid		   ? i_mult_tag[3:0]:
												4'd0; //default case; if we get here we'll assert full anyway
						
assign mem_tag_nxt 	= 	mem_tag_available[0 ] ? 4'd0:
						mem_tag_available[1 ] ? 4'd1:
						mem_tag_available[2 ] ? 4'd2:
						mem_tag_available[3 ] ? 4'd3:
						mem_tag_available[4 ] ? 4'd4:
						mem_tag_available[5 ] ? 4'd5:
						mem_tag_available[6 ] ? 4'd6:
						mem_tag_available[7 ] ? 4'd7:
						mem_tag_available[8 ] ? 4'd8:
						mem_tag_available[9 ] ? 4'd9:
						mem_tag_available[10] ? 4'd10:
						mem_tag_available[11] ? 4'd11:
						mem_tag_available[12] ? 4'd12:
						mem_tag_available[13] ? 4'd13:
						mem_tag_available[14] ? 4'd14:
						mem_tag_available[15] ? 4'd15:
						i_mem_valid			  ? i_mem_tag[3:0]:
												4'd0; //default case; if we get here we'll assert full anyway

//Tag-available update logic
always_ff @(posedge i_rst, posedge i_clk) begin
	if (i_rst) begin
		alu_tag_available <= 16'hffff;
		mult_tag_available <= 16'hffff;
		mem_tag_available <= 16'hffff;
	end
	else begin
		if (i_stall/*_all*/) begin
			//Make no changes to current state if we're stalled
			alu_tag_available <= alu_tag_available;
		end
		/*else if (i_stall_new) begin
			//don't update available based on what tag we say is available this cycle, but continue retiring instructions.
			//this will be the case if the reservation station is full but we still need to retire so we can make forward progress.
		end*/
		else begin
			for (int i=0; i<15; i+=1) begin
				if ((alu_tag_available[i] || i_alu_tag[3:0]==i) && alu_tag_nxt!=i) alu_tag_available[i] <= 1'b1;
				else alu_tag_available[i] <= 1'b0;
				
				if ((mult_tag_available[i] || i_mult_tag[3:0]==i) && mult_tag_nxt!=i) mult_tag_available[i] <= 1'b1;
				else mult_tag_available[i] <= 1'b0;
				
				if ((mem_tag_available[i] || i_mem_tag[3:0]==i) && mem_tag_nxt!=i) mem_tag_available[i] <= 1'b1;
				else mem_tag_available[i] <= 1'b0;
			end
		end
	end
	
end


//Output logic
always_comb begin
	case (i_instr_type)
		2'b00: begin //alu station
			o_station_full	= (alu_tag_available == 16'hffff) & ~i_alu_valid;
			o_tag			= {2'b00, alu_tag_nxt};
		end
		2'b01: begin //multiplier station
			o_station_full	= (mult_tag_available == 16'hffff) & ~i_mult_valid;
			o_tag			= {2'b01, mult_tag_nxt};
		end
		2'b10: begin //memory station
			o_station_full	= (mem_tag_available == 16'hffff) & ~i_mem_valid;
			o_tag			= {2'b10, mem_tag_nxt};
		end
		default: begin
			//protocol error if not one of the above; assert o_station_full and set tag to 'b111111
			o_station_full	= 1'b1;
			o_tag			= 6'b111111;
		end
	endcase
end


endmodule