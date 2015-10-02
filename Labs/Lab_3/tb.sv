`default_nettype none

module tb();
   bit clk, rst, h_sync, v_sync;
   bit [1:0] R, G, B;

   VGA vga(.*);

   // Clock and reset
   initial begin
      clk = 0;
      rst = 0; #10 rst = 1; #10 rst = 0;
      forever #10 clk = ~clk;
   end

   // Test
   initial begin
      #100000 $finish;
   end

endmodule : tb