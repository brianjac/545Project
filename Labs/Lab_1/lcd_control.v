/*

 The LCD module could serve as a handy debug tool.  This module provides
 an alternate interface, with most of the tricky timing hidden.  A simple
 FSM can send data values into this module, which will take care of sending
 them to the LCD module.  

 Here is how this module works:

 After reset, the LCD module requires a fairly complicated initialization sequence
 of commands.  Those signals are generated by this module.  After initialization, 
 initDone is asserted and this module waits for writeStart. Once it detect writeStart, it
 reads the 8-bit dataIn.  Those values are written to the LCD screen (which requires several
 commands, thus several clocks to accomplish). When writing is finished, writeDone is asserted 
 and this module waits for the new coming character.  Characters are written in order, from 
 the top left of the module, top line first.

 If the clearAll signal is received, this module will send a clear command to the LCD, which
 clears any data on the LCD module.

 The sf_d and control signals need to be connected to the LCD module by whatever module instantiates 
 lcd_control.  This module assumes the Vertex5 board clock frequency.

 */

module lcd_control(rst,
                   clk,
                   control,
                   sf_d, 
                   initDone, 
                   writeStart, 
                   writeDone, 
                   dataIn, 
                   clearAll); 
   input        rst; 
   input        clk;
   input        writeStart;
   input        clearAll;
   input [7:0] 	dataIn;
   
   
   output [2:0] control; // LCD_RS, LCD_RW, LCD_E
   output [3:0] sf_d;    //LCD data bus, 4 bit interface
   output 	writeDone;
   output 	initDone;

   reg [5:0] 	state,next_state;
`define waiting              6'b000000
`define init1            6'b000001
`define init2            6'b000010
`define init3            6'b000011
`define init4            6'b000100
`define init51           6'b000101
`define init52           6'b100101
`define init61           6'b000110
`define init62           6'b100110
`define init71           6'b000111
`define init72           6'b100111
`define init81           6'b001000
`define init82           6'b101000
   
`define waiting2         6'b010000
   
`define setDDRAM1        6'b001110
`define setDDRAM2        6'b101110
   
`define word1           6'b001011
`define word2           6'b101011
`define waiting3               6'b001001
`define stringEnd              6'b101001

   
   
   reg [2:0] 	control;
   reg [3:0] 	sf_d_temp, sf_d;
   reg [24:0] 	count, count_temp;
   reg 		state_flag;
   reg 		initDone;
   reg 		writeDone;
   reg [3:0] 	temp1, temp2;
   
`define TIME1 25'd200000        //2ms   it's under 100MHz clock
`define TIME2 25'd1600000       //16ms
`define TIME3 25'd800000    //8ms
`define TIME4 25'd500000        //5ms
`define TIME5 25'd500       //5us
`define TIME6 25'd50        //500ns
`define TIME7 25'd6                 //60ns
`define TIME8 25'd15000             //150us
`define TIME9 25'd6000              //60us

   always @ (clk or state or count or dataIn or writeStart 
             or clearAll 
             or temp1 or temp2)
     begin
        next_state <= `waiting;
        state_flag <= 1'b0;
        control = 3'b000;
        sf_d_temp = 4'b0000;
        writeDone = 1'b0;
        initDone = 1'b0;
        temp1 = dataIn[7:4];
        temp2 = dataIn[3:0];
        case (state)     
          // Initialization Starts --------------------------------
          `waiting : begin
             sf_d_temp = 4'b0000;
             control = 3'b000;                                                               // RS, RW, E             
             if (count >= `TIME2) begin                      // wait 16ms
                next_state <= `init1;  state_flag <= 1'b1;  end
             else    begin next_state <= `waiting; state_flag <= 1'b0; end
          end // case
          `init1 : begin 
             sf_d_temp = 4'b0011;    // Function set DL = 8bit
             if      (count == `TIME4) begin         //wait 5ms
                next_state <= `init2;   control = 3'b000; state_flag <= 1'b1; end
             else if ((count > `TIME6) & (count <= `TIME4)) begin
                next_state <= `init1; control = 3'b000; state_flag <= 1'b0; end    //E=0 for more than 4.1ms
             else if ((count > `TIME7) & (count <= `TIME6)) begin
                next_state <= `init1; control = 3'b001; state_flag <= 1'b0; end    //E=1 for 440ns(500-60)
             else begin next_state <= `init1; control = 3'b000; state_flag <= 1'b0; end //E=0 for 60ns 
          end // case
          `init2 : begin 
             sf_d_temp = 4'b0011;    //Function set DL = 8bit
             if      (count == `TIME8) begin
                next_state <= `init3;   control = 3'b000; state_flag <= 1'b1; end
             else if ((count > `TIME6) & (count <= `TIME8)) begin
                next_state <= `init2; control = 3'b000; state_flag <= 1'b0; end    //E=0 for more than 100us
             else if ((count > `TIME7) & (count <= `TIME6)) begin
                next_state <= `init2; control = 3'b001; state_flag <= 1'b0; end    //E=1 for 440ns(500-60)
             else begin next_state <= `init2; control = 3'b000; state_flag <= 1'b0; end //E=0 for 60ns 
          end // case
          `init3 : begin
             sf_d_temp = 4'b0011;     //Function set DL = 8bit
             if      (count == `TIME9) begin
                next_state <= `init4;   control = 3'b000; state_flag <= 1'b1; end
             else if ((count > `TIME6) & (count <= `TIME9)) begin
                next_state <= `init3; control = 3'b000; state_flag <= 1'b0; end    //E=0 for more than 40us
             else if ((count > `TIME7) & (count <= `TIME6)) begin
                next_state <= `init3; control = 3'b001; state_flag <= 1'b0; end    //E=1 for 440ns(500-60)
             else begin next_state <= `init3; control = 3'b000; state_flag <= 1'b0; end //E=0 for 60ns 
          end // case
          `init4 : begin
             sf_d_temp = 4'b0010;     //Function set DL = 4bit
             if      (count == `TIME8) begin
                next_state <= `init51;  control = 3'b000; state_flag <= 1'b1; end
             else if ((count > `TIME6) & (count <= `TIME8)) begin
                next_state <= `init4; control = 3'b000; state_flag <= 1'b0; end     //E=0 for more than 100us
             else if ((count > `TIME7) & (count <= `TIME6)) begin
                next_state <= `init4; control = 3'b001; state_flag <= 1'b0;  end    //E=1 for 440ns(500-60)
             else begin next_state <= `init4; control = 3'b000; state_flag <= 1'b0;  end //E=0 for 60ns
          end // case
          `init51 : begin
             sf_d_temp = 4'b0010;     //Function set (1) 0x28, DL = 4bit
             if      (count == `TIME5) begin
                next_state <= `init52; control = 3'b000; state_flag <= 1'b1; end
             else if ((count > `TIME6) & (count <= `TIME5)) begin
                next_state <= `init51; control = 3'b000; state_flag <= 1'b0; end     //E=0 for more than 1us
             else if ((count > `TIME7) & (count <= `TIME6)) begin
                next_state <= `init51; control = 3'b001; state_flag <= 1'b0; end  
             else begin next_state <= `init51; control = 3'b000; state_flag <= 1'b0; end
          end //case                      
          `init52 : begin
             sf_d_temp = 4'b1000;     //Function set (2) NL = 2, Font = 5x8
             if      (count == `TIME9) begin
                next_state <= `init61; control = 3'b000; state_flag <= 1'b1; end
             else if ((count > `TIME6) & (count <= `TIME9)) begin
                next_state <= `init52; control = 3'b000; state_flag <= 1'b0; end     //E=0 for more than 40us
             else if ((count > `TIME7) & (count <= `TIME6)) begin
                next_state <= `init52; control = 3'b001; state_flag <= 1'b0; end  
             else begin next_state <= `init52; control = 3'b000; state_flag <= 1'b0; end
          end //case
          `init61 : begin
             sf_d_temp = 4'b0000;     // Entry Mode set (1)
             if      (count == `TIME5) begin
                next_state <= `init62; control = 3'b000; state_flag <= 1'b1; end
             else if ((count > `TIME6) & (count <= `TIME5)) begin
                next_state <= `init61; control = 3'b000; state_flag <= 1'b0; end     //E=0 for more than 1us
             else if ((count > `TIME7) & (count <= `TIME6)) begin
                next_state <= `init61; control = 3'b001; state_flag <= 1'b0; end  
             else begin next_state <= `init61; control = 3'b000; state_flag <= 1'b0; end
          end // case
          `init62 : begin
             sf_d_temp = 4'b0110;     //Entry Mode set (2) I/D=1, S=0
             if      (count == `TIME9) begin
                next_state <= `init71; control = 3'b000; state_flag <= 1'b1; end
             else if ((count > `TIME6) & (count <= `TIME9)) begin
                next_state <= `init62; control = 3'b000; state_flag <= 1'b0; end     //E=0 for more than 40us
             else if ((count > `TIME7) & (count <= `TIME6)) begin
                next_state <= `init62; control = 3'b001; state_flag <= 1'b0; end  
             else begin next_state <= `init62; control = 3'b000; state_flag <= 1'b0; end
          end //case
          `init71 : begin
             sf_d_temp = 4'b0000;     //Display on/off control
             if      (count == `TIME5) begin
                next_state <= `init72; control = 3'b000; state_flag <= 1'b1; end
             else if ((count > `TIME6) & (count <= `TIME5)) begin
                next_state <= `init71; control = 3'b000; state_flag <= 1'b0; end     //E=0 for more than 1us
             else if ((count > `TIME7) & (count <= `TIME6)) begin
                next_state <= `init71; control = 3'b001; state_flag <= 1'b0; end  
             else begin next_state <= `init71; control = 3'b000; state_flag <= 1'b0; end
          end // case
          `init72 : begin
             sf_d_temp = 4'b1100;     //Display=off, Cursor=on, Blinking=on 
             if      (count == `TIME9) begin
                next_state <= `init81; control = 3'b000; state_flag <= 1'b1; end
             else if ((count > `TIME6) & (count <= `TIME9)) begin
                next_state <= `init72; control = 3'b000; state_flag <= 1'b0; end     //E=0 for more than 40us
             else if ((count > `TIME7) & (count <= `TIME6)) begin
                next_state <= `init72; control = 3'b001; state_flag <= 1'b0; end  
             else begin next_state <= `init72; control = 3'b000; state_flag <= 1'b0; end
          end //case
          `init81 : begin
             sf_d_temp = 4'b0000;     // Display clear phase 1
             if      (count == `TIME5) begin
                next_state <= `init82; control = 3'b000; state_flag <= 1'b1; end
             else if ((count > `TIME6) & (count <= `TIME5)) begin
                next_state <= `init81; control = 3'b000; state_flag <= 1'b0; end     //E=0 for more than 1us
             else if ((count > `TIME7) & (count <= `TIME6)) begin
                next_state <= `init81; control = 3'b001; state_flag <= 1'b0; end  
             else begin next_state <= `init81; control = 3'b000; state_flag <= 1'b0; end
          end // case
          `init82 : begin
             sf_d_temp = 4'b0001;     // Display clear phase 2
             if      (count == `TIME9) begin
                next_state <= `waiting2; control = 3'b000; state_flag <= 1'b1; end
             else if ((count > `TIME6) & (count <= `TIME9)) begin
                next_state <= `init82; control = 3'b000; state_flag <= 1'b0; end     //E=0 for more than 40us
             else if ((count > `TIME7) & (count <= `TIME6)) begin
                next_state <= `init82; control = 3'b001; state_flag <= 1'b0; end  
             else begin next_state <= `init82; control = 3'b000; state_flag <= 1'b0; end
          end //case
          // --------------------------------Initialization Ends -----------------------------------

          //-----------------------wait more than 1.64ms and then set DDRAM address as hx80-----------------------
          `waiting2 : begin
             sf_d_temp = 4'b0001;
             control = 3'b000;                                                               // RS, RW, E             
             if (count >= `TIME1) begin
                next_state <= `setDDRAM1;  state_flag <= 1'b1;  end
             else    begin next_state <= `waiting2; state_flag <= 1'b0; end
          end // case
          
          //-----------------------set DDRAM address--------------------                  
          `setDDRAM1 : begin
             sf_d_temp = 4'b1000; // Set Address hx80         
             if      (count == `TIME5) begin
                next_state <= `setDDRAM2; control = 3'b000; state_flag <= 1'b1; end
             else if ((count > `TIME6) & (count <= `TIME5)) begin
                next_state <= `setDDRAM1; control = 3'b000; state_flag <= 1'b0; end  //E=0 for more than 1us
             else if ((count > `TIME7) && (count <= `TIME6)) begin
                next_state <= `setDDRAM1; control = 3'b001; state_flag <= 1'b0; end
             else begin next_state <= `setDDRAM1; control = 3'b000; state_flag <= 1'b0; end
          end // case     
          `setDDRAM2 : begin
             sf_d_temp = 4'b0000; // Set Address hx00         
             if      (count == `TIME9) 
               begin
                  next_state <= `waiting3; control = 3'b000; state_flag <= 1'b1; 
               end
             else if ((count > `TIME6) & (count <= `TIME9)) begin
                next_state <= `setDDRAM2; control = 3'b000; state_flag <= 1'b0; end  //E=0 for more than 40us
             else if ((count > `TIME7) && (count <= `TIME6)) begin
                next_state <= `setDDRAM2; control = 3'b001; state_flag <= 1'b0; end
             else begin next_state <= `setDDRAM2; control = 3'b000; state_flag <= 1'b0; end
          end // case
          `waiting3 : begin
             initDone = 1'b1;
             if      (writeStart == 1'b1) 
               begin
                  state_flag <= 1'b1;
                  next_state <= `word1;
               end
             else if (clearAll == 'b1)
               begin 
                  next_state <= `init81; 
                  state_flag <= 1'b1; 
               end
             else 
               begin 
                  next_state <= `waiting3; 
                  state_flag <= 1'b0; 
               end
          end // case
	  //-----------------------Write Data sent from testFSM-----------------------
          `word1 : begin
             sf_d_temp = temp1; // higher 4 bits      
             if      (count == `TIME5) begin
                next_state <= `word2; control = 3'b100; state_flag <= 1'b1; end                 
             else if ((count > `TIME6) && (count <= `TIME5)) begin
                next_state <= `word1; control = 3'b100; state_flag <= 1'b0; end    //E=0 for more than 1us
             else if ((count > `TIME7) && (count <= `TIME6)) begin
                next_state <= `word1; control = 3'b101; state_flag <= 1'b0; end
             else begin next_state <= `word1; control = 3'b100; state_flag <= 1'b0; end
          end // case
          `word2 : begin
             sf_d_temp = temp2; // lower 4 bits       
             if      (count == `TIME9) begin
                next_state <= `waiting3; control = 3'b100; state_flag <= 1'b1; writeDone = 1'b1; end
             else if ((count > `TIME6) && (count <= `TIME9)) begin
                next_state <= `word2; control = 3'b100; state_flag <= 1'b0; end    //E=0 for more than 40us
             else if ((count > `TIME7) && (count <= `TIME6)) begin
                next_state <= `word2; control = 3'b101; state_flag <= 1'b0; end
             else begin next_state <= `word2; control = 3'b100; state_flag <= 1'b0; end
          end // case     

        endcase
     end // always

   //registers state variables
   always @ (posedge clk)
     begin
        sf_d <= sf_d_temp;
        count <= count_temp;
        if (rst) begin
           state <= `waiting;                      
           count_temp <= 0; end
        else if (state_flag) begin
           state <= next_state;                    
           count_temp <= 0; end
        else begin
           state <= next_state;                    
           count_temp <= count_temp + 1; end
     end // always 

endmodule //lcd_control 