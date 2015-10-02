library verilog;
use verilog.vl_types.all;
entity VGA_Emitter is
    port(
        x               : in     vl_logic_vector(9 downto 0);
        y               : in     vl_logic_vector(9 downto 0);
        R               : out    vl_logic_vector(1 downto 0);
        G               : out    vl_logic_vector(1 downto 0);
        B               : out    vl_logic_vector(1 downto 0)
    );
end VGA_Emitter;
