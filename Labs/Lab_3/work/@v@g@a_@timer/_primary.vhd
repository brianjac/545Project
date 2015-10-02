library verilog;
use verilog.vl_types.all;
entity VGA_Timer is
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        VS              : out    vl_logic;
        HS              : out    vl_logic;
        blank           : out    vl_logic;
        x               : out    vl_logic_vector(9 downto 0);
        y               : out    vl_logic_vector(8 downto 0)
    );
end VGA_Timer;
