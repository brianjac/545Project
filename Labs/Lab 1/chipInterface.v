module lcd_top(USER_CLK, 
			   GPIO_SW_C,			   
			   GPIO_SW_E,			   
			   GPIO_SW_S,			   
			   GPIO_SW_W,			   
			   LCD_FPGA_RS, LCD_FPGA_RW, LCD_FPGA_E,
			   LCD_FPGA_DB7, LCD_FPGA_DB6, LCD_FPGA_DB5, LCD_FPGA_DB4);
		
	input	   USER_CLK;
	/* switch C is reset, E is clear, S is resetFSM, W is nextString */
	input	   GPIO_SW_C, GPIO_SW_E, GPIO_SW_S, GPIO_SW_W;	
	output	   LCD_FPGA_RW, LCD_FPGA_RS, LCD_FPGA_E, LCD_FPGA_DB7, LCD_FPGA_DB6, LCD_FPGA_DB5, LCD_FPGA_DB4;
	
	wire		[2:0]	control_out; //rs, rw, en
	wire		[3:0]   out;
	wire				reset;
	
	wire	writeStart;
	wire	writeDone;
	wire	initDone;
	wire[7:0]	data;
	wire	clearAll;
	wire	resetFSM;
	wire	nextString;
	
	assign reset = GPIO_SW_C;
	assign resetFSM = GPIO_SW_S;
	assign clearAll = GPIO_SW_E;
	assign nextString = GPIO_SW_W;
	
	assign LCD_FPGA_DB7 = out[3];
	assign LCD_FPGA_DB6 = out[2];
	assign LCD_FPGA_DB5 = out[1];
	assign LCD_FPGA_DB4 = out[0];	
	
	assign LCD_FPGA_RS = control_out[2];
	assign LCD_FPGA_RW = control_out[1];
	assign LCD_FPGA_E  = control_out[0];
	
	lcd_control		lcd(.rst(reset), .clk(USER_CLK), .control(control_out), .sf_d(out),
							 .writeStart(writeStart), .initDone(initDone), .writeDone(writeDone), 
							 .dataIn(data), 
							 .clearAll(clearAll));
								
	testFSM			myTestFsm(.clkFSM(USER_CLK), .resetFSM(resetFSM), .data(data),
									 .initDone(initDone), .writeStart(writeStart),
									 .nextString(nextString),
									 .writeDone(writeDone));

endmodule