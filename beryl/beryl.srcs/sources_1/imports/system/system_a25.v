//////////////////////////////////////////////////////////////////
//                                                              //
//  Top-level module instantiating the entire Amber 2 system.   //
//                                                              //
//  This file is part of the Amber project                      //
//  http://www.opencores.org/project,amber                      //
//                                                              //
//  Description                                                 //
//  This is the highest level synthesizable module in the       //
//  project. The ports in this module represent pins on the     //
//  FPGA.                                                       //
//                                                              //
//  Author(s):                                                  //
//      - Conor Santifort, csantifort.amber@gmail.com           //
//                                                              //
//////////////////////////////////////////////////////////////////
//                                                              //
// Copyright (C) 2010 Authors and OPENCORES.ORG                 //
//                                                              //
// This source file may be used and distributed without         //
// restriction provided that this copyright statement is not    //
// removed from the file and that any derivative work contains  //
// the original copyright notice and the associated disclaimer. //
//                                                              //
// This source file is free software; you can redistribute it   //
// and/or modify it under the terms of the GNU Lesser General   //
// Public License as published by the Free Software Foundation; //
// either version 2.1 of the License, or (at your option) any   //
// later version.                                               //
//                                                              //
// This source is distributed in the hope that it will be       //
// useful, but WITHOUT ANY WARRANTY; without even the implied   //
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU Lesser General Public License for more //
// details.                                                     //
//                                                              //
// You should have received a copy of the GNU Lesser General    //
// Public License along with this source; if not, download it   //
// from http://www.opencores.org/lgpl.shtml                     //
//                                                              //
//////////////////////////////////////////////////////////////////


module system_a25
(
 input 	       brd_rst,
 input 	       brd_clk_n,
 input 	       brd_clk_p,
 
// PS/2
 input         ps2_clk,
 input  [7:0]  ps2_data,


// UART 0 Interface
 input 	       i_uart0_rts,
 output        o_uart0_rx,
 output        o_uart0_cts,
 input 	       i_uart0_tx,

// Xilinx Virtex 7 MIG DDR3 Interface
 /*inout [63:0]  ddr3_dq,
 output [13:0] ddr3_addr,
 output [2:0]  ddr3_ba,
 output [7:0]  ddr3_ras_n,
 output [7:0]  ddr3_cas_n,
 output        ddr3_we_n,
 output        ddr3_odt,
 output        ddr3_reset_n,
 output        ddr3_cke,
 output        ddr3_cs_n,
 output [7:0]  ddr3_dm,
 inout [1:0]   ddr3_dqs_p,
 inout [1:0]   ddr3_dqs_n,
 output        ddr3_ck_p,
 output        ddr3_ck_n,
 input         clk_ref_p,
 input         clk_ref_n,*/

//`ifdef XILINX_SPARTAN6_FPGA
//inout 	       mcb3_rzq,
//`endif


// Ethmac B100 MAC to PHY Interface
 /*input 	       mtx_clk_pad_i,
 output [3:0]  mtxd_pad_o,
 output        mtxen_pad_o,
 output        mtxerr_pad_o,
 input 	       mrx_clk_pad_i,
 input [3:0]   mrxd_pad_i,
 input 	       mrxdv_pad_i,
 input 	       mrxerr_pad_i,
 input 	       mcoll_pad_i,
 input 	       mcrs_pad_i,
 inout 	       md_pad_io,
 output        mdc_pad_o,
 output        phy_reset_n,*/

 output [7:0]  led,
 input [7:0] sw,
 
 input GPIO_SW_S
);


logic            sys_clk;    // System clock
wire            sys_rst;    // Active high reset, synchronous to sys_clk
wire            clk_200;    // 200MHz from board

wire            ps2_available;
wire [7:0]      ps2_data;


// ======================================
// Xilinx MCB DDR3 Controller connections
// ======================================
//`ifdef XILINX_SPARTAN6_FPGA
wire            c3_p0_cmd_en;        //app_en
wire  [2:0]     c3_p0_cmd_instr;     //app_cmd
wire  [27:0]    c3_p0_cmd_byte_addr; //app_addr
wire            c3_p0_wr_en;         //app_wdf_wren
wire  [31:0]    c3_p0_wr_mask;       //app_wdf_mask
wire  [255:0]   c3_p0_wr_data;       //app_wdf_data
wire  [255:0]   c3_p0_rd_data;       //app_rd_data
wire            c3_p0_rd_empty;      //
wire            c3_p0_cmd_full;      //
wire            c3_p0_wr_full;       //
//`endif

wire            phy_init_done;      //init_calib_complete
wire            test_mem_ctrl;      //
wire            system_rdy;         //app_rdy

wire 	        ddr3_app_wdf_rdy;
wire 	        ddr3_app_rd_data_end;
wire 	        ddr3_app_rd_data_valid;
	


// ======================================
// Ethmac MII
// ======================================
/*wire            md_pad_i;
wire            md_pad_o;
wire            md_padoe_o;*/

// ======================================
// Wishbone Buses
// ======================================

localparam WB_MASTERS = 2;
localparam WB_SLAVES  = 9;
`define AMBER_A25_CORE //TODO temp; fix for real sometime
//`define XILINX_FPGA //TODO temp: fix for real sometime
`ifdef AMBER_A25_CORE
localparam WB_DWIDTH  = 128;
localparam WB_SWIDTH  = 16;
`else
localparam WB_DWIDTH  = 32;
localparam WB_SWIDTH  = 4;
`endif


// Wishbone Master Buses
wire      [31:0]            m_wb_adr      [WB_MASTERS-1:0];
wire      [WB_SWIDTH-1:0]   m_wb_sel      [WB_MASTERS-1:0];
wire      [WB_MASTERS-1:0]  m_wb_we                       ;
wire      [WB_DWIDTH-1:0]   m_wb_dat_w    [WB_MASTERS-1:0];
wire      [WB_DWIDTH-1:0]   m_wb_dat_r    [WB_MASTERS-1:0];
wire      [WB_MASTERS-1:0]  m_wb_cyc                      ;
wire      [WB_MASTERS-1:0]  m_wb_stb                      ;
wire      [WB_MASTERS-1:0]  m_wb_ack                      ;
wire      [WB_MASTERS-1:0]  m_wb_err                      ;


// Wishbone Slave Buses
wire      [31:0]            s_wb_adr      [WB_SLAVES-1:0];
wire      [WB_SWIDTH-1:0]   s_wb_sel      [WB_SLAVES-1:0];
wire      [WB_SLAVES-1:0]   s_wb_we                      ;
wire      [WB_DWIDTH-1:0]   s_wb_dat_w    [WB_SLAVES-1:0];
wire      [WB_DWIDTH-1:0]   s_wb_dat_r    [WB_SLAVES-1:0];
wire      [WB_SLAVES-1:0]   s_wb_cyc                     ;
wire      [WB_SLAVES-1:0]   s_wb_stb                     ;
wire      [WB_SLAVES-1:0]   s_wb_ack                     ;
wire      [WB_SLAVES-1:0]   s_wb_err                     ;

wire      [31:0]            emm_wb_adr;
wire      [3:0]             emm_wb_sel;
wire                        emm_wb_we;
wire      [31:0]            emm_wb_rdat;
wire      [31:0]            emm_wb_wdat;
wire                        emm_wb_cyc;
wire                        emm_wb_stb;
wire                        emm_wb_ack;
wire                        emm_wb_err;

wire      [31:0]            ems_wb_adr;
wire      [3:0]             ems_wb_sel;
wire                        ems_wb_we;
wire      [31:0]            ems_wb_rdat;
wire      [31:0]            ems_wb_wdat;
wire                        ems_wb_cyc;
wire                        ems_wb_stb;
wire                        ems_wb_ack;
wire                        ems_wb_err;


// ======================================
// Interrupts
// ======================================
wire                        amber_irq;
wire                        amber_firq;
wire                        ethmac_int;
wire                        test_reg_irq;
wire                        test_reg_firq;
wire                        uart0_int;
wire                        uart1_int;
wire      [2:0]             timer_int;


// ======================================
// Clocks and Resets Module
// ======================================
/*clocks_resets u_clocks_resets (
    .i_brd_rst          ( brd_rst           ),
    .i_brd_clk_n        ( brd_clk_n         ),
    .i_brd_clk_p        ( brd_clk_p         ),
    .i_ddr_calib_done   ( phy_init_done     ),
    .o_sys_rst          ( sys_rst           ),
    .o_sys_clk          ( sys_clk           ),
    .o_clk_200          ( clk_200           )
);*/
/*reg sys_rst_reg;
always_ff @(posedge sys_clk) begin
    sys_rst_reg <= brd_rst;
end*/
assign sys_rst = brd_rst/*sys_rst_reg*//* || phy_init_done*/;

logic clk_wiz_clk_out;
clk_wiz_0 merlin 
 (
 // Clock in ports
  .clk_in1_p(brd_clk_p),
  .clk_in1_n(brd_clk_n),
  // Clock out ports
  .clk_out1(clk_wiz_clk_out),
  //.clk_out2(/*clk_200*/), //not currently used; may need to reinsert eventually, though
  // Status and control signals
  .reset(sys_rst),
  .locked()
 );
 always_comb begin
    if (sw[7]) sys_clk = GPIO_SW_S;
    else sys_clk = clk_wiz_clk_out;
 end

// -------------------------------------------------------------
// Instantiate Amber Processor Core
// -------------------------------------------------------------
//`ifdef AMBER_A25_CORE
a25_core u_amber (
//`else
//a23_core u_amber (
//`endif
    .i_clk          ( sys_clk         ),
    //.i_rst          ( brd_rst         ), //TODO change to sys_rst if want sync reset

    .i_irq          ( amber_irq       ),
    .i_firq         ( amber_firq      ),

    .i_system_rdy   ( system_rdy      ),

    .o_wb_adr       ( m_wb_adr  [1]   ),
    .o_wb_sel       ( m_wb_sel  [1]   ),
    .o_wb_we        ( m_wb_we   [1]   ),
    .i_wb_dat       ( m_wb_dat_r[1]   ),
    .o_wb_dat       ( m_wb_dat_w[1]   ),
    .o_wb_cyc       ( m_wb_cyc  [1]   ),
    .o_wb_stb       ( m_wb_stb  [1]   ),
    .i_wb_ack       ( m_wb_ack  [1]   ),
    .i_wb_err       ( m_wb_err  [1]   ),
    
    .led(led),
    .sw({1'b0,sw[6:0]})
);

// Ethernet MII PHY reset
// Halt core until system is ready
//assign system_rdy = phy_init_done && !sys_rst;
assign system_rdy = !sys_rst; //TODO revise/edit/fix

// -------------------------------------------------------------
// Instantiate Boot Memory - 8KBytes of Embedded SRAM
// -------------------------------------------------------------

/*generate
if (WB_DWIDTH == 32) begin : boot_mem32
    boot_mem32 u_boot_mem (
        .i_wb_clk               ( sys_clk         ),
        .i_wb_adr               ( s_wb_adr  [1]   ),
        .i_wb_sel               ( s_wb_sel  [1]   ),
        .i_wb_we                ( s_wb_we   [1]   ),
        .o_wb_dat               ( s_wb_dat_r[1]   ),
        .i_wb_dat               ( s_wb_dat_w[1]   ),
        .i_wb_cyc               ( s_wb_cyc  [1]   ),
        .i_wb_stb               ( s_wb_stb  [1]   ),
        .o_wb_ack               ( s_wb_ack  [1]   ),
        .o_wb_err               ( s_wb_err  [1]   )
    );
end
else begin : boot_mem128*/

    reg s_wb_ack_1_reg = 1'b0;
    always_ff @(posedge sys_clk) begin
        s_wb_ack_1_reg <= s_wb_stb[1];
    end
    assign s_wb_ack[1] = s_wb_ack_1_reg;
    assign s_wb_err[1] = 1'b0;
    
    blk_mem_gen_bootrom u_boot_mem  (   .clka(sys_clk),
                                        .rsta(sys_rst),
                                        .ena(1'd1),
                                        .addra({16'h0000, s_wb_adr[1][15:0]}), //force to only ever enable bottom 16 bits
                                        .douta(s_wb_dat_r[1])
                                    );

    /*boot_mem128 u_boot_mem (
        .i_wb_clk               ( sys_clk         ),
        .i_wb_adr               ( s_wb_adr  [1]   ),
        .i_wb_sel               ( s_wb_sel  [1]   ),
        .i_wb_we                ( s_wb_we   [1]   ),
        .o_wb_dat               ( s_wb_dat_r[1]   ),
        .i_wb_dat               ( s_wb_dat_w[1]   ),
        .i_wb_cyc               ( s_wb_cyc  [1]   ),
        .i_wb_stb               ( s_wb_stb  [1]   ),
        .o_wb_ack               ( s_wb_ack  [1]   ),
        .o_wb_err               ( s_wb_err  [1]   )
    );*/
/*end
endgenerate*/


// -------------------------------------------------------------
// Instantiate UART0
// -------------------------------------------------------------
uart  #(
    .WB_DWIDTH              ( WB_DWIDTH       ),
    .WB_SWIDTH              ( WB_SWIDTH       )
    )
u_uart0 (
    .i_clk                  ( sys_clk        ),

    .o_uart_int             ( uart0_int      ),

    .i_uart_cts_n           ( i_uart0_rts    ),
    .o_uart_txd             ( o_uart0_rx     ),
    .o_uart_rts_n           ( o_uart0_cts    ),
    .i_uart_rxd             ( i_uart0_tx     ),

    .i_wb_adr               ( s_wb_adr  [3]  ),
    .i_wb_sel               ( s_wb_sel  [3]  ),
    .i_wb_we                ( s_wb_we   [3]  ),
    .o_wb_dat               ( s_wb_dat_r[3]  ),
    .i_wb_dat               ( s_wb_dat_w[3]  ),
    .i_wb_cyc               ( s_wb_cyc  [3]  ),
    .i_wb_stb               ( s_wb_stb  [3]  ),
    .o_wb_ack               ( s_wb_ack  [3]  ),
    .o_wb_err               ( s_wb_err  [3]  )
);


// -------------------------------------------------------------
// Instantiate UART1
// -------------------------------------------------------------
uart  #(
    .WB_DWIDTH              ( WB_DWIDTH       ),
    .WB_SWIDTH              ( WB_SWIDTH       )
    )
u_uart1 (
    .i_clk                  ( sys_clk        ),

    .o_uart_int             ( uart1_int      ),

    // These are not connected. ONly pins for 1 UART
    // on my development board
    .i_uart_cts_n           ( 1'd1           ),
    .o_uart_txd             (                ),
    .o_uart_rts_n           (                ),
    .i_uart_rxd             ( 1'd1           ),

    .i_wb_adr               ( s_wb_adr  [4]  ),
    .i_wb_sel               ( s_wb_sel  [4]  ),
    .i_wb_we                ( s_wb_we   [4]  ),
    .o_wb_dat               ( s_wb_dat_r[4]  ),
    .i_wb_dat               ( s_wb_dat_w[4]  ),
    .i_wb_cyc               ( s_wb_cyc  [4]  ),
    .i_wb_stb               ( s_wb_stb  [4]  ),
    .o_wb_ack               ( s_wb_ack  [4]  ),
    .o_wb_err               ( s_wb_err  [4]  )
);


// -------------------------------------------------------------
// Instantiate Test Module
//   - includes register used to terminate tests
// -------------------------------------------------------------
test_module #(
    .WB_DWIDTH              ( WB_DWIDTH      ),
    .WB_SWIDTH              ( WB_SWIDTH      )
    )
u_test_module (
    .i_clk                  ( sys_clk        ),

    .o_irq                  ( test_reg_irq   ),
    .o_firq                 ( test_reg_firq  ),
    .o_mem_ctrl             ( test_mem_ctrl  ),
    .i_wb_adr               ( s_wb_adr  [5]  ),
    .i_wb_sel               ( s_wb_sel  [5]  ),
    .i_wb_we                ( s_wb_we   [5]  ),
    .o_wb_dat               ( s_wb_dat_r[5]  ),
    .i_wb_dat               ( s_wb_dat_w[5]  ),
    .i_wb_cyc               ( s_wb_cyc  [5]  ),
    .i_wb_stb               ( s_wb_stb  [5]  ),
    .o_wb_ack               ( s_wb_ack  [5]  ),
    .o_wb_err               ( s_wb_err  [5]  ),
    .o_led                  ( /*led*/            ),
    .o_phy_rst_n            ( phy_reset_n    )
);


// -------------------------------------------------------------
// Instantiate Timer Module
// -------------------------------------------------------------
timer_module  #(
    .WB_DWIDTH              ( WB_DWIDTH      ),
    .WB_SWIDTH              ( WB_SWIDTH      )
    )
u_timer_module (
    .i_clk                  ( sys_clk        ),

    // Interrupt outputs
    .o_timer_int            ( timer_int      ),

    // Wishbone interface
    .i_wb_adr               ( s_wb_adr  [6]  ),
    .i_wb_sel               ( s_wb_sel  [6]  ),
    .i_wb_we                ( s_wb_we   [6]  ),
    .o_wb_dat               ( s_wb_dat_r[6]  ),
    .i_wb_dat               ( s_wb_dat_w[6]  ),
    .i_wb_cyc               ( s_wb_cyc  [6]  ),
    .i_wb_stb               ( s_wb_stb  [6]  ),
    .o_wb_ack               ( s_wb_ack  [6]  ),
    .o_wb_err               ( s_wb_err  [6]  )
);


// -------------------------------------------------------------
// Instantiate Interrupt Controller Module
// -------------------------------------------------------------
interrupt_controller  #(
    .WB_DWIDTH              ( WB_DWIDTH      ),
    .WB_SWIDTH              ( WB_SWIDTH      )
    )
u_interrupt_controller (
    .i_clk                  ( sys_clk        ),

    // Interrupt outputs
    .o_irq                  ( amber_irq      ),
    .o_firq                 ( amber_firq     ),

    // Interrupt inputs
    .i_uart0_int            ( uart0_int      ),
    .i_uart1_int            ( uart1_int      ),
    .i_ethmac_int           ( /*ethmac_int*/1'b0     ),
    .i_test_reg_irq         ( test_reg_irq   ),
    .i_test_reg_firq        ( test_reg_firq  ),
    .i_tm_timer_int         ( timer_int      ),

    // Wishbone interface
    .i_wb_adr               ( s_wb_adr  [7]  ),
    .i_wb_sel               ( s_wb_sel  [7]  ),
    .i_wb_we                ( s_wb_we   [7]  ),
    .o_wb_dat               ( s_wb_dat_r[7]  ),
    .i_wb_dat               ( s_wb_dat_w[7]  ),
    .i_wb_cyc               ( s_wb_cyc  [7]  ),
    .i_wb_stb               ( s_wb_stb  [7]  ),
    .o_wb_ack               ( s_wb_ack  [7]  ),
    .o_wb_err               ( s_wb_err  [7]  ),
    
    // PS/2 inputs
    .i_ps2_available(ps2_available),
    .i_ps2_data(ps2_data)
);

ps2_controller u_ps2_controller (
    .i_clk(ps2_clk),
    .i_data(ps2_data),
    .o_ps2_available(ps2_available),
    .o_ps2_data(ps2_data)
);




/*`ifndef XILINX_FPGA
    // ======================================
    // Instantiate non-synthesizable main memory model
    // ======================================

    assign phy_init_done = 1'd1;

    main_mem #(
                .WB_DWIDTH             ( WB_DWIDTH             ),
                .WB_SWIDTH             ( WB_SWIDTH             )
                )
    u_main_mem (
               .i_clk                  ( sys_clk               ),
               .i_mem_ctrl             ( test_mem_ctrl         ),
               .i_wb_adr               ( s_wb_adr  [2]         ),
               .i_wb_sel               ( s_wb_sel  [2]         ),
               .i_wb_we                ( s_wb_we   [2]         ),
               .o_wb_dat               ( s_wb_dat_r[2]         ),
               .i_wb_dat               ( s_wb_dat_w[2]         ),
               .i_wb_cyc               ( s_wb_cyc  [2]         ),
               .i_wb_stb               ( s_wb_stb  [2]         ),
               .o_wb_ack               ( s_wb_ack  [2]         ),
               .o_wb_err               ( s_wb_err  [2]         )
            );

`endif*/
    
    
    logic [15:0] blkmem_wea_mask;
    always_comb begin
        if (~s_wb_we) blkmem_wea_mask = 16'h0000;
        else begin
            case (s_wb_adr[2][3:2])
                2'b00: blkmem_wea_mask = {12'h000, s_wb_sel[2]};
                2'b01: blkmem_wea_mask = {8'h00, s_wb_sel[2], 4'h0};
                2'b10: blkmem_wea_mask = {4'h0, s_wb_sel[2], 8'h00};
                2'b11: blkmem_wea_mask = {s_wb_sel[2], 12'h000};
            endcase
        end
    end
    reg s_wb_ack_2_reg = 1'b0;
    always_ff @(posedge sys_clk) begin
        s_wb_ack_2_reg <= s_wb_stb[2];
    end
    assign s_wb_ack[2] = s_wb_ack_2_reg;
    assign s_wb_err[2] = 1'b0;
    blk_mem_gen_0 fake_ddr3 (   .clka(sys_clk),
                                .rsta(sys_rst),
                                .wea(blkmem_wea_mask),
                                .addra({16'h0000, s_wb_adr[2][15:0]}), //force to only ever enable bottom 16 bits
                                .dina(s_wb_dat_w[2]),
                                .douta(s_wb_dat_r[2])
                            );


// -------------------------------------------------------------
// Instantiate Wishbone Arbiter
// -------------------------------------------------------------
wishbone_arbiter #(
    .WB_DWIDTH              ( WB_DWIDTH         ),
    .WB_SWIDTH              ( WB_SWIDTH         )
    )
u_wishbone_arbiter (
    .i_wb_clk               ( sys_clk           ),

    // WISHBONE master 0 - Ethmac
    .i_m0_wb_adr            ( /*m_wb_adr   [0]*/32'd0    ),
    .i_m0_wb_sel            ( /*m_wb_sel   [0]*/16'd0    ),
    .i_m0_wb_we             ( /*m_wb_we    [0]*/1'd0    ),
    .o_m0_wb_dat            ( /*m_wb_dat_r [0]*/    ),
    .i_m0_wb_dat            ( /*m_wb_dat_w [0]*/128'd0    ),
    .i_m0_wb_cyc            ( /*m_wb_cyc   [0]*/1'd0    ),
    .i_m0_wb_stb            ( /*m_wb_stb   [0]*/1'd0    ),
    .o_m0_wb_ack            ( /*m_wb_ack   [0]*/    ),
    .o_m0_wb_err            ( /*m_wb_err   [0]*/    ),


    // WISHBONE master 1 - Amber Process or
    .i_m1_wb_adr            ( m_wb_adr   [1]    ),
    .i_m1_wb_sel            ( m_wb_sel   [1]    ),
    .i_m1_wb_we             ( m_wb_we    [1]    ),
    .o_m1_wb_dat            ( m_wb_dat_r [1]    ),
    .i_m1_wb_dat            ( m_wb_dat_w [1]    ),
    .i_m1_wb_cyc            ( m_wb_cyc   [1]    ),
    .i_m1_wb_stb            ( m_wb_stb   [1]    ),
    .o_m1_wb_ack            ( m_wb_ack   [1]    ),
    .o_m1_wb_err            ( m_wb_err   [1]    ),


    // WISHBONE slave 0 - Ethmac
    .o_s0_wb_adr            ( /*s_wb_adr   [0]*/    ),
    .o_s0_wb_sel            ( /*s_wb_sel   [0]*/    ),
    .o_s0_wb_we             ( /*s_wb_we    [0]*/    ),
    .i_s0_wb_dat            ( /*s_wb_dat_r [0]*/128'd0    ),
    .o_s0_wb_dat            ( /*s_wb_dat_w [0]*/    ),
    .o_s0_wb_cyc            ( /*s_wb_cyc   [0]*/    ),
    .o_s0_wb_stb            ( /*s_wb_stb   [0]*/    ),
    .i_s0_wb_ack            ( /*s_wb_ack   [0]*/1'd0    ),
    .i_s0_wb_err            ( /*s_wb_err   [0]*/1'd0    ),


    // WISHBONE slave 1 - Boot Memory
    .o_s1_wb_adr            ( s_wb_adr   [1]    ),
    .o_s1_wb_sel            ( s_wb_sel   [1]    ),
    .o_s1_wb_we             ( s_wb_we    [1]    ),
    .i_s1_wb_dat            ( s_wb_dat_r [1]    ),
    .o_s1_wb_dat            ( s_wb_dat_w [1]    ),
    .o_s1_wb_cyc            ( s_wb_cyc   [1]    ),
    .o_s1_wb_stb            ( s_wb_stb   [1]    ),
    .i_s1_wb_ack            ( s_wb_ack   [1]    ),
    .i_s1_wb_err            ( s_wb_err   [1]    ),


    // WISHBONE slave 2 - Main Memory
    .o_s2_wb_adr            ( s_wb_adr   [2]    ),
    .o_s2_wb_sel            ( s_wb_sel   [2]    ),
    .o_s2_wb_we             ( s_wb_we    [2]    ),
    .i_s2_wb_dat            ( s_wb_dat_r [2]    ),
    .o_s2_wb_dat            ( s_wb_dat_w [2]    ),
    .o_s2_wb_cyc            ( s_wb_cyc   [2]    ),
    .o_s2_wb_stb            ( s_wb_stb   [2]    ),
    .i_s2_wb_ack            ( s_wb_ack   [2]    ),
    .i_s2_wb_err            ( s_wb_err   [2]    ),


    // WISHBONE slave 3 - UART 0
    .o_s3_wb_adr            ( s_wb_adr   [3]    ),
    .o_s3_wb_sel            ( s_wb_sel   [3]    ),
    .o_s3_wb_we             ( s_wb_we    [3]    ),
    .i_s3_wb_dat            ( s_wb_dat_r [3]    ),
    .o_s3_wb_dat            ( s_wb_dat_w [3]    ),
    .o_s3_wb_cyc            ( s_wb_cyc   [3]    ),
    .o_s3_wb_stb            ( s_wb_stb   [3]    ),
    .i_s3_wb_ack            ( s_wb_ack   [3]    ),
    .i_s3_wb_err            ( s_wb_err   [3]    ),


    // WISHBONE slave 4 - UART 1
    .o_s4_wb_adr            ( s_wb_adr   [4]    ),
    .o_s4_wb_sel            ( s_wb_sel   [4]    ),
    .o_s4_wb_we             ( s_wb_we    [4]    ),
    .i_s4_wb_dat            ( s_wb_dat_r [4]    ),
    .o_s4_wb_dat            ( s_wb_dat_w [4]    ),
    .o_s4_wb_cyc            ( s_wb_cyc   [4]    ),
    .o_s4_wb_stb            ( s_wb_stb   [4]    ),
    .i_s4_wb_ack            ( s_wb_ack   [4]    ),
    .i_s4_wb_err            ( s_wb_err   [4]    ),


    // WISHBONE slave 5 - Test Module
    .o_s5_wb_adr            ( s_wb_adr   [5]    ),
    .o_s5_wb_sel            ( s_wb_sel   [5]    ),
    .o_s5_wb_we             ( s_wb_we    [5]    ),
    .i_s5_wb_dat            ( s_wb_dat_r [5]    ),
    .o_s5_wb_dat            ( s_wb_dat_w [5]    ),
    .o_s5_wb_cyc            ( s_wb_cyc   [5]    ),
    .o_s5_wb_stb            ( s_wb_stb   [5]    ),
    .i_s5_wb_ack            ( s_wb_ack   [5]    ),
    .i_s5_wb_err            ( s_wb_err   [5]    ),


    // WISHBONE slave 6 - Timer Module
    .o_s6_wb_adr            ( s_wb_adr   [6]    ),
    .o_s6_wb_sel            ( s_wb_sel   [6]    ),
    .o_s6_wb_we             ( s_wb_we    [6]    ),
    .i_s6_wb_dat            ( s_wb_dat_r [6]    ),
    .o_s6_wb_dat            ( s_wb_dat_w [6]    ),
    .o_s6_wb_cyc            ( s_wb_cyc   [6]    ),
    .o_s6_wb_stb            ( s_wb_stb   [6]    ),
    .i_s6_wb_ack            ( s_wb_ack   [6]    ),
    .i_s6_wb_err            ( s_wb_err   [6]    ),


    // WISHBONE slave 7 - Interrupt Controller
    .o_s7_wb_adr            ( s_wb_adr   [7]    ),
    .o_s7_wb_sel            ( s_wb_sel   [7]    ),
    .o_s7_wb_we             ( s_wb_we    [7]    ),
    .i_s7_wb_dat            ( s_wb_dat_r [7]    ),
    .o_s7_wb_dat            ( s_wb_dat_w [7]    ),
    .o_s7_wb_cyc            ( s_wb_cyc   [7]    ),
    .o_s7_wb_stb            ( s_wb_stb   [7]    ),
    .i_s7_wb_ack            ( s_wb_ack   [7]    ),
    .i_s7_wb_err            ( s_wb_err   [7]    )
    );


endmodule


