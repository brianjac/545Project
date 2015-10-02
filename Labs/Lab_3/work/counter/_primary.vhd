library verilog;
use verilog.vl_types.all;
entity counter is
    generic(
        COUNT_TO        : integer := 64
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        count           : in     vl_logic;
        counter         : out    vl_logic_vector
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of COUNT_TO : constant is 1;
end counter;
