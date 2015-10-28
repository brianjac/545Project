module top
  (output logic [7:0] led,
   input logic data,
   input logic clk
   );

   logic [10:0] packet;
   logic [3:0] 	index;

   always_ff @(posedge clk) begin
      if (index == 4'd11) begin
	 index <= 4'd0;
	 led <= packet[8:1];
      end
      else begin
	 index <= index + 1;
      end
	 
      packet[index] <= data;
   end


endmodule : top