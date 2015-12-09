// Brian

module ps2_controller(
    input logic i_clk, i_data,
    output logic o_ps2_available,
    output logic [7:0] o_ps2_data
    );
    
    logic [10:0] packet;
    logic [3:0]  index;
    
    always_ff @(posedge i_clk) begin
        if (index == 4'd11) begin
            index <= 4'd0;
            o_ps2_data <= packet[8:1];
            o_ps2_data <= 1'b1;
        end
        else begin
            o_ps2_data <= 1'b0;
            index <= index + 1;
        end
        
        packet[index] <= i_data;
    end
    
endmodule