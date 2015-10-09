`timescale 1ns / 1ps


module Top(
    input bit reset,
    input bit SYSCLK_P, SYSCLK_N,
    output bit [7:0] leds
    //output bit GPIO_LED_7, GPIO_LED_6, GPIO_LED_5, GPIO_LED_4, GPIO_LED_3, GPIO_LED_2, GPIO_LED_1, GPIO_LED_0
    );
    
    bit [31:0] counter;
    
    /*bit [7:0] leds;
    assign {GPIO_LED_7, GPIO_LED_6, GPIO_LED_5, GPIO_LED_4, GPIO_LED_3, GPIO_LED_2, GPIO_LED_1, GPIO_LED_0} = leds;*/
    
    // Clock
    bit clock;
    clk_wiz_0 anakin_skywalker(
        // Inputs
        .clk_in1_p(SYSCLK_P), .clk_in1_n(SYSCLK_N), .reset(reset),
        // Outputs
        .clk_out1(clock), .locked());
    
    // Counter
    always_ff @(posedge clock, posedge reset) begin
        if (reset) begin
            counter <= 32'd0;
            leds <= 8'b00000000;
        end
        else begin
            if (counter == 32'd20000000) begin
                leds <= ~leds;
                counter <= 32'd0;
            end
            else counter <= counter+32'd1;
        end
    end
    
endmodule : Top