library verilog;
use verilog.vl_types.all;
entity VGA is
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        R               : out    vl_logic_vector(1 downto 0);
        G               : out    vl_logic_vector(1 downto 0);
        B               : out    vl_logic_vector(1 downto 0);
        h_sync          : out    vl_logic;
        v_sync          : out    vl_logic
    );
end VGA;
