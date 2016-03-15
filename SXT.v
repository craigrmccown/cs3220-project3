module SXT(IN,OUT);
	parameter IBITS;
	parameter OBITS;
	input  [(IBITS-1):0] IN;
	output [(OBITS-1):0] OUT;
	assign OUT = {{(OBITS-IBITS){IN[IBITS-1]}},IN};
endmodule