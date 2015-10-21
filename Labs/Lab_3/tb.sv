`default_nettype none

module tb();
   bit clk_p, clk_n, rst, h_sync, v_sync;
   bit R, G, B;

   VGA vga(.*);
   
   assign clk_p = ~clk_n;

   // Clock and reset
   initial begin
      clk_n = 0;
      rst = 0; #10 rst = 1; #10 rst = 0;
      forever #10 clk_n = ~clk_n;
   end

   // Test
   initial begin
      #100000 $finish;
   end

endmodule : tb