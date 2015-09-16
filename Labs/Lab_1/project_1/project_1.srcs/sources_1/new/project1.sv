module top (input logic [7:0] switches, output logic [7:0] leds);
    
    always_comb begin
        leds[7:4] = switches[7:4]^switches[3:0];
        leds[3:0] = switches[7:4]&switches[3:0];
    end
    
endmodule: top