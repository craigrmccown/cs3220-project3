module SevenSeg(output [6:0] OUT, input [3:0] IN, input OFF);
	reg [6 : 0] digit_code;
	
	always @(IN or OFF) begin
		if (OFF)
			digit_code = 7'b1111111;
		else
			case (IN)
				4'h0: digit_code = 7'b1000000;
				4'h1: digit_code = 7'b1111001;
				4'h2: digit_code = 7'b0100100;
				4'h3: digit_code = 7'b0110000;
				4'h4: digit_code = 7'b0011001;
				4'h5: digit_code = 7'b0010010;
				4'h6: digit_code = 7'b0000010;
				4'h7: digit_code = 7'b1111000;
				4'h8: digit_code = 7'b0000000;
				4'h9: digit_code = 7'b0010000;
				4'hA: digit_code = 7'b0001000;
				4'hb: digit_code = 7'b0000011;
				4'hc: digit_code = 7'b1000110;
				4'hd: digit_code = 7'b0100001;
				4'he: digit_code = 7'b0000110;
				default: digit_code = 7'b0001110;
			endcase
	end
	
	assign OUT = digit_code;
endmodule
