`default_nettype none

/* Outputs all signals necessary to run a VGA display */
module VGA(clk, rst, R, G, B, h_sync, v_sync);
   input bit clk, rst;
   output bit [1:0] R, G, B;
   output bit 	    h_sync, v_sync;

   bit [8:0] 	    x;
   bit [9:0] 	    y;
   bit [1:0] 	    R_t, G_t, B_t;
   bit 		    blank;

   VGA_Timer vga_t(.clk(clk), .rst(rst),
		   .VS(v_sync), .HS(h_sync),
		   .x(x), .y(y), .blank(blank));

   VGA_Emitter vga_e(.x(x), .y(y),
		     .R(R_t), .G(G_t), .B(B_t));

   assign R = blank ? 0 : R_t;
   assign G = blank ? 0 : G_t;
   assign B = blank ? 0 : B_t;

endmodule : VGA

/* Outputs VGA timing signals */
/*   Assumes that the clk is at the pixel clock speed
 * of the monitor. Probably 27.125Hz */
module VGA_Timer(clk, rst, VS, HS, blank, x, y);

   /* DESIGN CONSTANTS */
   localparam H_PIXELS = 640; /* Horizontal width in pixels          */
   localparam V_PIXELS = 480; /* Vertical height in pixels           */
   localparam H_FPORCH = 128; /* Horizontal front porch in pixels    */
   localparam V_FPORCH =   1; /* Vertical front porch in rows        */
   localparam H_BPORCH = 336; /* Horizontal back porch in pixels     */
   localparam V_BPORCH =  38; /* Vertical back porch in rows         */
   localparam H_PULSE  = 208; /* Horizontal pulse in pixels          */
   localparam V_PULSE  =   3; /* Vertical pulse in rows              */
   localparam H_POLAR  =   1; /* Polarity of h_sync; 1 => assert low */
   localparam V_POLAR  =   1; /* Polarity of v_sync; 1 => assert low */
   // Pixel clocks in a row
   localparam H_PERIOD = H_PULSE + H_BPORCH + H_PIXELS + H_FPORCH;
   // Rows in a column
   localparam V_PERIOD = V_PULSE + V_BPORCH + V_PIXELS + V_FPORCH;
   /* END DESIGN CONSTANTS */
   
   input bit 	    clk;
   input bit 	    rst;
   output bit 	    VS, HS, blank;
   output bit [$clog2(H_PIXELS) - 1:0] x;
   output bit [$clog2(V_PIXELS) - 1:0] y;

   // Counters
   bit [$clog2(H_PERIOD) - 1:0] h_count;
   bit [$clog2(V_PERIOD) - 1:0] v_count;
   bit 				h_zero, v_zero, v_enable;
   counter #(H_PERIOD - 1) H_count(.clk(clk), .rst(h_zero),
					      .count(1'b1),
					      .counter(h_count));
   counter #(V_PERIOD - 1) V_count(.clk(clk), .rst(v_zero),
					      .count(v_enable),
					      .counter(v_count));

   // Vertical count enable
   assign v_enable = (h_count == H_PERIOD - 1);
   
   always_ff @(posedge clk, posedge rst) begin
      if (rst) begin
	 x <= 0; y <= 0;
	 h_zero <= 1; v_zero <= 1;
	 HS <= 1; VS <= 1;
	 blank <= 0;
      end
      else begin

	 // Reset
	 h_zero <= 0;
	 h_zero <= 0;

	 // Coordinates
	 x <= h_count < H_PIXELS ? h_count : 0;
	 y <= v_count < V_PIXELS ? v_count : 0;

	 // Horizontal Sync Signal
	 HS <= H_POLAR ^ (h_count > H_PIXELS + H_FPORCH | 
			  h_count < H_PIXELS + H_FPORCH + H_PULSE);

	 // Vertical Sync Signal
	 VS <= V_POLAR ^ (v_count > V_PIXELS + V_FPORCH |
			  v_count < V_PIXELS + V_FPORCH + V_PULSE);

	 // Blank line
	 blank <= ~(h_count < H_PIXELS & v_count < V_PIXELS);
	 
      end
   end
 
endmodule : VGA_Timer

/* Takes in an x and y and outputs a pixel. */
module VGA_Emitter(x, y, R, G, B);
   input bit [9:0] x, y;
   output bit [1:0] R, G, B;

   // Test Values
   assign R = 2'b11;
   assign G = 2'b10;
   assign B = 2'b01;

endmodule : VGA_Emitter

/* Counts to a number */
/*   When count is asserted, counter increments
 * by one on every clock cycle.
 */
module counter(clk, rst, count, counter);
   parameter COUNT_TO = 64;
   input bit clk, rst, count;
   output bit [$clog2(COUNT_TO) - 1:0] counter;

   always_ff @(posedge clk, rst) begin
      if (rst) begin
	 counter <= 0;
      end
      else begin
	 if (counter == COUNT_TO)
	   counter <= 0;
	 else
	   counter <= counter + count;
      end
   end

endmodule : counter