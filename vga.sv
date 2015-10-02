module vga
  (input  logic CLOCK_50, reset, //50MHz clock
   output logic HS, VS, blank, //HS and VS active low, blank active high
   //during front porch, back porch or pulse timing, blank should be active
   output logic [8:0] row,
   output logic [9:0] col);

   logic 	  clrHS, clrVS, clrPixelCount,  pixelCountEn, rowEn, clrRowCount;
   logic [10:0]   count_HS, pixelCount;
   
   logic [19:0]    count_VS;
   logic 	   cHSn, cHS1, cHS2, cVSn, cVS1, cVS2, between_HS, between_VS, pixelCountEn1,
		   between2_HS, between2_VS;
   

   counter #(11) countHS(.en(1'b1), .clr(clrHS), .load(1'b0), .up(1'b1), .clk(CLOCK_50),
                         .D(11'd0), .Q(count_HS));
   counter #(20) countVS(.en(1'b1), .clr(clrVS), .load(1'b0), .up(1'b1), .clk(CLOCK_50),
		         .D(20'd0), .Q(count_VS));

   offset_check #(11) OCR(.is_between(between_HS), .low(11'd288), .delta(11'd1279),
                          .val(count_HS));
   offset_check #(20) OCS(.is_between(between_VS), .low(20'd49600), .delta(20'd767999),
			  .val(count_VS));
   range_check #(11) OCR2(.is_between(between2_HS), .low(11'd192), .high(11'd1599),
			  .val(count_HS));
   range_check #(20) OCS2(.is_between(between2_VS), .low(20'd3200), .high(20'd833599),
			  .val(count_VS));
   
			  
   assign HS = between2_HS;
   assign VS = between2_VS;
   assign blank = ~(between_HS | ~between_VS);
   assign pixelCountEn = between_HS & between_VS;
   assign clrPixelCount = (col == 10'd640 | reset);
   assign rowEn = clrHS & between_VS;
   assign clrRowCount = (row == 9'd480 | reset);
   assign pixelCountEn1 = pixelCount[0];
   
	
	comparator #(11) clearHS(.A(count_HS), .B(11'd1599), .AltB(cHSn), .AeqB(cHS1), .AgtB(cHS2));
	comparator #(20) clearVS(.A(count_VS), .B(20'd833599), .AltB(cVSn), .AeqB(cVS1), .AgtB(cVS2));
	
	always_comb begin
	if (cHS1 | cHS2 | reset)
	   clrHS = 1;
	else
	   clrHS = 0;
	if (cVS1 | cVS2 | reset)
	   clrVS = 1;
	else
	  clrVS = 0;			 
	end

   counter #(11) countPixelEnable(.en(pixelCountEn), .clr(clrPixelCount), .load(1'b0), .up(1'b1),
				  .clk(CLOCK_50), .D(11'd0), .Q(pixelCount));
   
   
   counter #(10) countPixel(.en(pixelCountEn1), .clr(clrPixelCount), .load(1'b0), .up(1'b1),
			    .clk(CLOCK_50), .D(10'd0), .Q(col));
   
   counter #(9) countRow(.en(rowEn), .clr(clrRowCount), .load(1'b0), .up(1'b1),
			 .clk(CLOCK_50), .D(9'd0), .Q(row));
      
endmodule: vga

module vga_test;
   logic    CLOCK_50, reset;
   logic    HS, VS, blank;
   logic [8:0] row;
   logic [9:0] col;
   int       i;
   
   vga dut(.*);

  initial begin
     CLOCK_50 = 1;
     forever #1 CLOCK_50 = ~CLOCK_50;
end 

  initial begin

     reset = 1;
     @ (posedge CLOCK_50);
     reset = 0;
     @ (posedge CLOCK_50);
      for (i = 0; i < 1000000; i++)
	@(posedge CLOCK_50);
      $finish;
   end
   
endmodule: vga_test

