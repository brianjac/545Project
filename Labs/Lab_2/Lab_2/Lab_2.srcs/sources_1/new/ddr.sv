`timescale 1ns / 1ps
`default_nettype none

module ddr(
    input bit left, right, reset,
    input bit SYSCLK_P, SYSCLK_N,
    output bit [7:0] leds
    );
    
    // Clock
    bit clk;
    clk_wiz_0 doc_brown(
        // Inputs
        .clk_in1_p(SYSCLK_P), .clk_in1_n(SYSCLK_N), .reset(reset),
        // Outputs
        .clk_out1(clk), .locked());
        
    // Perform State Transitions
    always_ff @(posedge clk, posedge reset) begin
        if (reset)
            leds <= 8'b00000001;
        else
            if (leds == 8'b10000000) leds <= 8'b00000001;
            else leds <= leds<<1;
    end
    
endmodule : ddr
