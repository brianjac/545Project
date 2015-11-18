/* Converts an ASCII letter into a bitmap.
 * It's basically just a huge combinational
 * lookup table. */

module ASCII2Bitmap
  (
   input logic [7:0] 	 char,
   output logic [12][22] bitmap
   );


   always_comb begin
      case (char)
	8'h : bitmap = 12*22'd0;
      endcase
   end
   
endmodule : ASCII2Bitmap