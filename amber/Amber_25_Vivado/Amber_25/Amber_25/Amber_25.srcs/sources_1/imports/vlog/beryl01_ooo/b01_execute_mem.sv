//////////////////////////////////////////////////////////////////
//                                                              //
//  Memory Access - Instantiates the memory access stage        //
//  sub-modules of the Amber 25 Core                            //
//                                                              //
//  This file is part of the Amber project                      //
//  http://www.opencores.org/project,amber                      //
//                                                              //
//  Description                                                 //
//  Instantiates the Data Cache                                 //
//  Also contains a little bit of logic to decode memory        //
//  accesses to decide if they are cached or not                //
//                                                              //
//  Author(s):                                                  //
//      - Conor Santifort, csantifort.amber@gmail.com           //
//                                                              //
//////////////////////////////////////////////////////////////////
//                                                              //
// Copyright (C) 2011 Authors and OPENCORES.ORG                 //
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
/*

	.o_instr_valid_mem(o_instr_valid_mem),
	.o_rn_mem(o_rn_mem), //TODO make this just i_address_mem
	.o_rs_mem(o_rs_mem), //TODO remove
	.o_rm_mem(o_rm_mem), //TODO remove
	.o_exclusive_mem(o_exclusive_mem), //TODO remove
	.o_pc_wen_mem(o_pc_wen_mem),
	.o_status_bits_flags_wen_mem(o_status_bits_flags_wen_mem),
	.o_byte_enable_sel_mem(o_byte_enable_sel_mem),
	.o_rd_tag_mem(o_rd_tag_mem),
	.i_mem_valid(i_mem_valid),
	.i_mem_tag(i_mem_tag),
	.i_mem_data(i_mem_data)
	*/
	
	/*
	New interface:
	o_instr_valid_mem
	o_address_mem [31:0]
	o_write_data_mem [31:0]
	o_op_type_mem [1:0] //00=read, 01=write, 10=swap
	o_byte_enable_sel_mem [3:0]
	o_rd_tag_mem [5:0]
	i_mem_ready
	i_mem_valid
	i_mem_tag [5:0]
	i_mem_data [31:0]
	i_mem_flags [3:0]
	*/
	
	/*
	Supported:
	ldm
	stm
	ldr
	ldrb
	str
	strb
	*/

module b01_execute_mem
(
input logic 			i_clk,
input logic				i_rst,
output logic 			o_ready,            // asserted high on the cycle when data_out_valid goes high and held high until another memop is requested

input logic				i_instr_valid,
input logic	 [31:0]		i_address,
input logic	 [31:0]		i_write_data,
input logic  [1:0]		i_op_type,
input logic  [3:0]		i_byte_enable,
input logic  [5:0]		i_rd_tag,

output logic			o_valid,
output logic [5:0]		o_tag,
output logic [31:0]		o_data,
output logic [3:0]		o_flags,

/*input       [31:0]          i_daddress,
input                       i_daddress_valid,
input       [31:0]          i_daddress_nxt,         // un-registered version of address to the cache rams
input       [31:0]          i_write_data,
input                       i_write_enable,
//input                       i_exclusive,            // high for read part of swap access
input       [3:0]           i_byte_enable,
input       [8:0]           i_exec_load_rd,         // The destination register for a load instruction
input                       i_cache_enable,         // cache enable
input                       i_cache_flush,          // cache flush
input       [31:0]          i_cacheable_area,       // each bit corresponds to 2MB address space

output      [31:0]          o_mem_read_data,
output                      o_mem_read_data_valid,
output      [10:0]          o_mem_load_rd,          // The destination register for a load instruction*/

// Wishbone accesses                                                         
//output                      o_wb_cached_req,        // Cached Request
output                      o_wb_uncached_req,      // Unached Request
output                      o_wb_write,             // Read=0, Write=1
output     [15:0]           o_wb_byte_enable,       // byte eable
output     [127:0]          o_wb_write_data,
output     [31:0]           o_wb_address,           // wb bus                                 
input      [127:0]          i_wb_uncached_rdata,    // wb bus                              
input      [127:0]          i_wb_cached_rdata,      // wb bus                              
//input                       i_wb_cached_ready,      // wishbone access complete and read data valid
input                       i_wb_uncached_ready     // wishbone access complete and read data valid
);

`include "memory_configuration.vh"

/*wire                        uncached_data_access;
wire                        uncached_data_access_p;
wire                        cache_stall;
wire                        uncached_wb_wait;
reg                         uncached_wb_req_r = 'd0;
reg                         uncached_wb_stop_r = 'd0;
wire                        daddress_valid_p;  // pulse
reg      [31:0]             mem_read_data_r = 'd0;
reg                         mem_read_data_valid_r = 'd0;
reg      [10:0]             mem_load_rd_r = 'd0;
wire     [10:0]             mem_load_rd_c;
wire     [31:0]             mem_read_data_c;
wire                        mem_read_data_valid_c;
reg                         mem_stall_r = 'd0;
wire                        use_mem_reg;
reg                         fetch_only_stall_r = 'd0;
wire                        fetch_only_stall;
wire                        void_output;
wire                        wb_stop;
reg                         daddress_valid_stop_r = 'd0;
wire     [31:0]             wb_rdata32;*/

always_ff @(posedge i_clk, posedge i_rst) begin
	if (i_rst) begin
		/*
			All outputs and internal registers to "safe" state
		*/
	end
	else begin
		/*
			Operation:
			- if no op in progress and no input ready: do nothing
			- if no op in progress and input ready:
				- latch operands
				- forward appropriate data directly to mem
			- if op in progress:
				- if op incomplete:
					- wait
				- if op complete:
					- latch outputs: ready, data, tag, valid, etc.
			
			- if op is typical read or write: nothing special
				- read: need to latch ready, data, tag, valid, and flags at end
				- write: only need to latch ready at end
			- if op is swap:
				- latch all data and begin read op
				- when read op done, put data on module output and start write op
				- when write op done, set "ready" output
		*/
	end
end

wire mem_stall;

// Return read data either from the wishbone bus or the cache
assign wb_rdata32               = i_daddress[3:2] == 2'd0 ? i_wb_uncached_rdata[ 31: 0] :
                                  i_daddress[3:2] == 2'd1 ? i_wb_uncached_rdata[ 63:32] :
                                  i_daddress[3:2] == 2'd2 ? i_wb_uncached_rdata[ 95:64] :
                                                            i_wb_uncached_rdata[127:96] ;
                                                            
assign mem_read_data_c          = sel_cache             ? cache_read_data : 
                                  uncached_data_access  ? wb_rdata32      :
                                                          32'h76543210    ;
                                                          
assign mem_load_rd_c            = {i_address[1:0], i_exec_load_rd}; //TODO edit
assign mem_read_data_valid_c    = i_instr_valid && !i_write_enable && !(uncached_wb_wait || cache_stall); //TODO edit

assign mem_stall              = uncached_wb_wait || cache_stall;

// Request wishbone access
assign o_wb_byte_enable         = i_address[3:2] == 2'd0 ? {12'd0, i_byte_enable       } :
                                  i_address[3:2] == 2'd1 ? { 8'd0, i_byte_enable,  4'd0} :
                                  i_address[3:2] == 2'd2 ? { 4'd0, i_byte_enable,  8'd0} :
                                                           {       i_byte_enable, 12'd0} ;

assign o_wb_write               = i_write_enable; //TODO edit
assign o_wb_address             = {i_address[31:2], 2'd0};
assign o_wb_write_data          = {4{i_write_data}};
assign o_wb_cached_req          = !cached_wb_stop_r && cached_wb_req;
assign o_wb_uncached_req        = !uncached_wb_stop_r && uncached_data_access_p;

assign uncached_wb_wait         = (o_wb_uncached_req || uncached_wb_req_r) && !i_wb_uncached_ready;

always @( posedge i_clk )
    begin
    uncached_wb_req_r <=  (o_wb_uncached_req || uncached_wb_req_r) && !i_wb_uncached_ready;
    end

/*assign fetch_only_stall     = i_fetch_stall && !o_mem_stall;

always @( posedge i_clk )
    fetch_only_stall_r <= fetch_only_stall;

assign void_output = (fetch_only_stall_r && fetch_only_stall) || (fetch_only_stall_r && mem_read_data_valid_r);*/


// pulse this signal
assign daddress_valid_p = i_daddress_valid && !daddress_valid_stop_r;

always @( posedge i_clk )
    begin
    uncached_wb_stop_r      <= (uncached_wb_stop_r || (uncached_data_access_p&&!cache_stall)) && (/*i_fetch_stall || o_*/mem_stall);
    cached_wb_stop_r        <= (cached_wb_stop_r   || cached_wb_req)          && (/*i_fetch_stall || o_*/mem_stall);
    daddress_valid_stop_r   <= (daddress_valid_stop_r || daddress_valid_p)    && (/*i_fetch_stall || o_*/mem_stall);
    // hold this until the mem access completes
    mem_stall_r <= o_mem_stall;
    end


assign wb_stop = uncached_wb_stop_r || cached_wb_stop_r;

always @( posedge i_clk )
    if ( !wb_stop || o_mem_stall )
        begin
        mem_read_data_r         <= mem_read_data_c;
        mem_load_rd_r           <= mem_load_rd_c;
        mem_read_data_valid_r   <= mem_read_data_valid_c;
        end


// ======================================
// L1 Data Cache
// ======================================
/*a25_dcache u_dcache (
    .i_clk                      ( i_clk                 ),
    .i_fetch_stall              ( /*i_fetch_stall1'b0         ),
    .i_exec_stall               ( /*i_exec_stall1'b0          ),
    .o_stall                    ( cache_stall           ),
     
    .i_request                  ( sel_cache_p           ),
    .i_exclusive                ( /*i_exclusive1'b0           ), //TODO confirm operation
    .i_write_data               ( i_write_data          ),
    .i_write_enable             ( i_write_enable        ), //TODO revise
    .i_address                  ( i_address            ),
    .i_address_nxt              ( i_daddress_nxt        ), //TODO revise
    .i_byte_enable              ( i_byte_enable         ),

    .i_cache_enable             ( i_cache_enable        ),
    .i_cache_flush              ( i_cache_flush         ),
    .o_read_data                ( cache_read_data       ),
    
    .o_wb_cached_req            ( cached_wb_req         ),
    .i_wb_cached_rdata          ( i_wb_cached_rdata     ),
    .i_wb_cached_ready          ( i_wb_cached_ready     )
);*/



endmodule

