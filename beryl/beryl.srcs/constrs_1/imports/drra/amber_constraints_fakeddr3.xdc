# Board Reset Line - Center Button GPIO_SW_C
set_property PACKAGE_PIN AV39 [get_ports brd_rst]
set_property IOSTANDARD LVCMOS18 [get_ports brd_rst]

# Clock Lines
set_property IOSTANDARD LVDS [get_ports brd_clk_p]
set_property PACKAGE_PIN E19 [get_ports brd_clk_p]
set_property PACKAGE_PIN E18 [get_ports brd_clk_n]
set_property IOSTANDARD LVDS [get_ports brd_clk_n]

# UART
set_property PACKAGE_PIN AU36 [get_ports o_uart0_rx]
set_property IOSTANDARD LVCMOS18 [get_ports o_uart0_rx]
set_property PACKAGE_PIN AT32 [get_ports i_uart0_rts]
set_property IOSTANDARD LVCMOS18 [get_ports i_uart0_rts]
set_property PACKAGE_PIN AU33 [get_ports i_uart0_tx]
set_property IOSTANDARD LVCMOS18 [get_ports i_uart0_tx]
set_property PACKAGE_PIN AR34 [get_ports o_uart0_cts]
set_property IOSTANDARD LVCMOS18 [get_ports o_uart0_cts]

# LEDs
set_property PACKAGE_PIN AM39 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {led[0]}]
set_property PACKAGE_PIN AN39 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {led[1]}]
set_property PACKAGE_PIN AR37 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {led[2]}]
set_property PACKAGE_PIN AT37 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {led[3]}]
set_property PACKAGE_PIN AR35 [get_ports {led[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {led[4]}]
set_property PACKAGE_PIN AP41 [get_ports {led[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {led[5]}]
set_property PACKAGE_PIN AP42 [get_ports {led[6]}]
set_property IOSTANDARD LVCMOS18 [get_ports {led[6]}]
set_property PACKAGE_PIN AU39 [get_ports {led[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {led[7]}]

# SWs
set_property PACKAGE_PIN AY33 [get_ports {sw[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {sw[1]}]
set_property PACKAGE_PIN BA31 [get_ports {sw[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {sw[2]}]
set_property PACKAGE_PIN BA32 [get_ports {sw[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {sw[3]}]
set_property PACKAGE_PIN AW30 [get_ports {sw[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {sw[4]}]
set_property PACKAGE_PIN AY30 [get_ports {sw[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {sw[5]}]
set_property PACKAGE_PIN BA30 [get_ports {sw[6]}]
set_property IOSTANDARD LVCMOS18 [get_ports {sw[6]}]
set_property PACKAGE_PIN BB31 [get_ports {sw[7]}]
set_property IOSTANDARD LVCMOS18 [get_ports {sw[7]}]
set_property PACKAGE_PIN AV30 [get_ports {sw[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {sw[0]}]

set_property PACKAGE_PIN AP40 [get_ports GPIO_SW_S]
set_property IOSTANDARD LVCMOS18 [get_ports GPIO_SW_S]

#set_property LOAD 0 [get_ports {led[0]}]
#set_property LOAD 0 [get_ports {led[1]}]
#set_property LOAD 0 [get_ports {led[2]}]
#set_property LOAD 0 [get_ports {led[3]}]
#set_property LOAD 0 [get_ports {led[4]}]
#set_property LOAD 0 [get_ports {led[5]}]
#set_property LOAD 0 [get_ports {led[6]}]
#set_property LOAD 0 [get_ports {led[7]}]
#set_property LOAD 0 [get_ports o_uart0_cts]
#set_property LOAD 0 [get_ports o_uart0_rx]
#set_load 0.000 [all_outputs]
