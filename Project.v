module Project(
	input CLOCK_50,
	input RESET_N,
	input	[3 : 0] KEY,
	input	[9 : 0] SW,
	output [6 : 0] HEX0,
	output [6 : 0] HEX1,
	output [6 : 0] HEX2,
	output [6 : 0] HEX3,
	output [6 : 0] HEX4,
	output [6 : 0] HEX5,
	output [9 : 0] LEDR
);
	parameter DBITS = 32;
	parameter INSTSIZE = 32'd4;
	parameter INSTBITS = 32;
	parameter REGNOBITS = 6;
	parameter REGWORDS = (1 << REGNOBITS);
	parameter IMMBITS = 14;
	parameter STARTPC = 32'h100;
	parameter ADDRHEX = 32'hFFFFF000;
	parameter ADDRLEDR = 32'hFFFFF020;
	parameter ADDRKEY = 32'hFFFFF080;
	parameter ADDRSW = 32'hFFFFF090;
	parameter IMEMINITFILE = "Sorter3.mif";
	parameter IMEMADDRBITS = 16;
	parameter IMEMWORDBITS = 2;
	parameter IMEMWORDS = (1 << (IMEMADDRBITS - IMEMWORDBITS));
	parameter DMEMADDRBITS = 16;
	parameter DMEMWORDBITS = 2;
	parameter DMEMWORDS = (1 << (DMEMADDRBITS - DMEMWORDBITS));
	parameter BRANCHPREDBITS = 4;
	
	parameter OP1BITS = 6;
	parameter OP1_ALUR = 6'b000000;
	parameter OP1_BEQ = 6'b001000;
	parameter OP1_BLT = 6'b001001;
	parameter OP1_BLE = 6'b001010;
	parameter OP1_BNE = 6'b001011;
	parameter OP1_JAL = 6'b001100;
	parameter OP1_LW = 6'b010010;
	parameter OP1_SW = OP1_LW + 6'b001000;
	parameter OP1_ADDI = 6'b100000;
	parameter OP1_ANDI = 6'b100100;
	parameter OP1_ORI = 6'b100101;
	parameter OP1_XORI = 6'b100110;
	
	parameter OP2BITS = 6;
	parameter OP2_EQ = OP1_BEQ;
	parameter OP2_LT = OP1_BLT;
	parameter OP2_LE = OP1_BLE;
	parameter OP2_NE = OP1_BNE;
	parameter OP2_ADD = OP1_ADDI;
	parameter OP2_AND = OP1_ANDI;
	parameter OP2_OR = OP1_ORI;
	parameter OP2_XOR = OP1_XORI;
	parameter OP2_SUB = OP2_ADD | 6'b001000;
	parameter OP2_NAND = OP2_AND | 6'b001000;
	parameter OP2_NOR = OP2_OR | 6'b001000;
	parameter OP2_NXOR = OP2_XOR | 6'b001000;
	
	parameter FUNCSEL_EQ = 2'b00;
	parameter FUNCSEL_LT = 2'b01;
	parameter FUNCSEL_LE = 2'b10;
	parameter FUNCSEL_NE = 2'b11;
	parameter FUNCSEL_ADD = 3'b000;
	parameter FUNCSEL_AND = 3'b100;
	parameter FUNCSEL_OR = 3'b101;
	parameter FUNCSEL_XOR = 3'b110;
	
	// The reset signal comes from the reset button on the DE0 - CV board
	// RESET_N is active - low, so we flip its value ("reset" is active - high)
	wire clk, locked;
	
	// The PLL is wired to produce clk and locked signals for our logic
	Pll myPll(
		.refclk (CLOCK_50),
		.rst (!RESET_N),
		.outclk_0 (clk),
		.locked (locked)
	);
	
	wire reset = !locked;

	/*
	reg clk;
	reg reset;
	reg hasticked;
	reg [31 : 0] clk_cnt;
	
	initial begin
		reset <= 1'b1;
		hasticked <= 1'b0;
		clk_cnt <= 32'b0;
		clk <= 1'b0;
	end
	
	always @(posedge _clk) begin
		if (clk_cnt == 32'd30000000) begin
			clk_cnt <= 32'b0;
			clk <= 1'b1;
			hasticked <= 1'b1;
		end else begin
			if (hasticked)
				reset <= 1'b0;

			clk_cnt <= clk_cnt + 1;
			clk <= 1'b0;
		end
	end
	*/

	// The PC register and update logic
	reg [(DBITS - 1) : 0] PC;
	
	always @(posedge clk) begin
		if (reset)
			PC <= STARTPC;
		else if (stall_F)
			PC <= PC;
		else if (mispred_B)
			PC <= pcgood_B;
		else
			PC <= pcpred_F;
	end
		
	/*
	 * ----------------------------- FETCH ----------------------------- 
	 */
	 
	// This is the value of "incremented PC", computed in stage 1
	wire [(DBITS - 1) : 0] pcplus_F = PC + INSTSIZE;
	
	// Read branch prediction value
	reg [(DBITS - 1) : 0] branchpred[7 : 0];
	wire [7 : 0] predidx_F = pcplus_F[9 : 2];
	wire [(DBITS - 1) : 0] branchpred_F = branchpred[predidx_F];
	wire [(DBITS - 1) : 0] pcpred_F = isbranch_D ? branchpred_F : pcplus_F;

	// Instruction - fetch
	(* ram_init_file = IMEMINITFILE *)
	reg [(DBITS - 1) : 0] imem[(IMEMWORDS - 1) : 0];
	wire [(DBITS - 1) : 0] inst_F = imem[PC[(IMEMADDRBITS - 1) : IMEMWORDBITS]];

	/*
	 * ----------------------------- DECODE ----------------------------- 
	 */
	 
	// Create pipeline buffer for stage D
	wire stall_D = stall_F;
	wire [(DBITS - 1) : 0] inst_D = inst_F;
	wire [(DBITS - 1) : 0] pcplus_D = pcplus_F;
	wire [(DBITS - 1) : 0] pcpred_D = pcpred_F;
	
	// Instruction decoding
	// These have zero delay from inst_D
	// because they are just new names for those signals
	wire [(OP1BITS - 1) : 0] op1_D = inst_D[(DBITS - 1) : (DBITS - OP1BITS)];
	wire [(REGNOBITS - 1) : 0] rs_D, rt_D, rd_D;
	assign {rs_D, rt_D, rd_D} = inst_D[(DBITS - OP1BITS - 1) : (DBITS - OP1BITS - 3 * REGNOBITS)];
	wire [(OP2BITS - 1) : 0] op2_D = inst_D[(OP2BITS - 1) : 0];
	wire [(IMMBITS - 1) : 0] rawimm_D = inst_D[(IMMBITS - 1) : 0];
	wire [(DBITS - 1) : 0] sxtimm_D;
	SXT #(.IBITS(IMMBITS), .OBITS(DBITS)) sxt(.IN(rawimm_D), .OUT(sxtimm_D));
	
	// Register - read
	reg [(DBITS - 1) : 0] regs[(REGWORDS - 1) : 0];
	wire [(DBITS - 1) : 0] rsval_D = regs[rs_D];
	wire [(DBITS - 1) : 0] rtval_D = regs[rt_D];
	
	// Create decode registers
	reg aluimm_D, isbranch_D, isjump_D, isnop_D, wrmem_D, selaluout_D, selmemout_D, selpcplus_D, wrreg_D;
	reg [(OP2BITS - 1) : 0] alufunc_D;
	reg [(REGNOBITS - 1) : 0] wregno_D;
	
	// Decoding logic
	always @ * begin
		{aluimm_D, alufunc_D} = {1'bX,{OP2BITS{1'bX}}};
		{isbranch_D, isjump_D, isnop_D, wrmem_D} = {1'b0, 1'b0, 1'b0, 1'b0};
		{selaluout_D, selmemout_D, selpcplus_D, wregno_D, wrreg_D} = {1'bX, 1'bX, 1'bX, {REGNOBITS{1'bX}}, 1'b0};
		
		if(reset | flush_D)
			isnop_D = 1'b1;
		else case (op1_D)
			OP1_ALUR:
				{aluimm_D, alufunc_D, selaluout_D, selmemout_D, selpcplus_D, wregno_D, wrreg_D} =
				{1'b0, op2_D, 1'b1, 1'b0, 1'b0, rd_D, 1'b1};
			default:
				case (op1_D)
					OP1_BEQ, OP1_BNE, OP1_BLT, OP1_BLE:
						{aluimm_D, alufunc_D, isbranch_D} = {1'b0, op1_D, 1'b1};
					OP1_SW:
						{aluimm_D, alufunc_D, wrmem_D} = {1'b1, OP1_ADDI, 1'b1};
					OP1_JAL:
						{aluimm_D, alufunc_D, wrreg_D, wregno_D, isjump_D, selaluout_D, selmemout_D, selpcplus_D} =
						{1'b1, OP1_ADDI, 1'b1, rt_D, 1'b1, 1'b0, 1'b0, 1'b1};
					OP1_LW:
						{aluimm_D, alufunc_D, wrreg_D, wregno_D, selaluout_D, selmemout_D, selpcplus_D} =
						{1'b1, OP1_ADDI, 1'b1, rt_D, 1'b0, 1'b1, 1'b0};
					OP1_ADDI, OP1_ANDI, OP1_ORI, OP1_XORI:
						{aluimm_D, alufunc_D, wrreg_D, wregno_D, selaluout_D, selmemout_D, selpcplus_D} =
						{1'b1, op1_D, 1'b1, rt_D, 1'b1, 1'b0, 1'b0};
					default:  isnop_D = 1'b1;
				endcase
		endcase
	end
	
	// Hazard detection
	wire reading_t_D = ~aluimm_D | wrmem_D;
	wire hazard_s_A_D = wrreg_A & (wregno_A == rs_D);
	wire hazard_t_A_D = wrreg_A & reading_t_D & (wregno_A == rt_D);
	wire hazard_s_M_D = wrreg_M & (wregno_M == rs_D);
	wire hazard_t_M_D = wrreg_M & reading_t_D & (wregno_M == rt_D);
	
	// Data forwarding
	wire [(DBITS - 1) : 0] forwarded_A_D = selaluout_A ? aluout_A : pcplus_A;
	wire [(DBITS - 1) : 0] forwarded_M_D = selaluout_M ? aluout_M : pcplus_M;
	
	// Generate stall signals (only for LW instructions)
	wire load_hazard_A = hazard_s_A_D | hazard_t_A_D & selmemout_A;
	wire load_hazard_M = hazard_s_M_D | hazard_t_M_D & selmemout_M;
	wire stall_F = load_hazard_A | load_hazard_M;

	/*
	 * ----------------------------- ALU ----------------------------- 
	 */

	// Create pipeline buffer for stage A
	
	reg aluimm_A,
		isbranch_A,
		isjump_A,
		isnop_A,
		wrmem_A,
		selaluout_A,
		selmemout_A,
		selpcplus_A,
		wrreg_A;
		
	reg [(OP2BITS - 1) : 0] alufunc_A;
	reg [(REGNOBITS - 1) : 0] wregno_A;
	reg [(DBITS - 1) : 0] pcplus_A, pcpred_A, sxtimm_A;
	
	reg hazard_s_A_A,
		hazard_t_A_A,
		hazard_s_M_A,
		hazard_t_M_A;
		
	reg [(DBITS - 1) : 0] rsval_A, rtval_A, forwarded_A_A, forwarded_M_A;
	
	always @(posedge clk) begin
		if (!stall_D) begin
			{aluimm_A, isbranch_A, isjump_A, isnop_A, wrmem_A, selaluout_A, selmemout_A, selpcplus_A, wrreg_A} <=
			{aluimm_D, isbranch_D, isjump_D, isnop_D, wrmem_D, selaluout_D, selmemout_D, selpcplus_D, wrreg_D};
			
			alufunc_A <= alufunc_D;
			wregno_A <= wregno_D;
			{pcplus_A, pcpred_A, sxtimm_A} <= {pcplus_D, pcpred_D, sxtimm_D};
			
			{hazard_s_A_A, hazard_t_A_A, hazard_s_M_A, hazard_t_M_A} <= {hazard_s_A_D, hazard_t_A_D, hazard_s_M_D, hazard_t_M_D};
			{rsval_A, rtval_A, forwarded_A_A, forwarded_M_A} <= {rsval_D, rtval_D, forwarded_A_D, forwarded_M_D};
		end else begin
			{aluimm_A, isbranch_A, isjump_A, isnop_A, wrmem_A, selaluout_A, selmemout_A, selpcplus_A, wrreg_A} <=
			{1'bX, 1'b0, 1'b0, 1'b1, 1'b0, 1'bX, 1'bX, 1'bX, 1'b0};
			
			alufunc_A <= {OP2BITS{1'bX}};
			wregno_A <= {REGNOBITS{1'bX}};
			{pcplus_A, pcpred_A, sxtimm_A} <= {(DBITS * 3){1'bX}};
			
			{hazard_s_A_A, hazard_t_A_A, hazard_s_M_A, hazard_t_M_A} <= {4{1'bX}};
			{rsval_A, rtval_A, forwarded_A_A, forwarded_M_A} <= {(DBITS * 4){1'bX}};
		end
	end
	
	// Decide whether to use forwarded values
	wire [(DBITS - 1) : 0] regval1_A = hazard_s_A_A ? forwarded_A_A : (hazard_s_M_A ? forwarded_M_A : rsval_A);
	wire [(DBITS - 1) : 0] regval2_A = hazard_t_A_A ? forwarded_A_A : (hazard_t_M_A ? forwarded_M_A : rtval_A);
	
	// Instruction type selection
	wire iscomparison = ~alufunc_A[5];
	wire [1 : 0] comparisonfunc = alufunc_A[1 : 0];
	wire isbasecalc = ~alufunc_A[3];
	wire [2 : 0] calcfunc = alufunc_A[2 : 0];
	
	// Calculate inputs
	wire signed [(DBITS - 1) : 0] aluin1_A = regval1_A;
	wire signed [(DBITS - 1) : 0] aluin2_A = aluimm_A ? sxtimm_A : regval2_A;
	
	// Calculate outputs
	reg [(DBITS - 1) : 0] aluout_A;
	wire [(DBITS - 1) : 0] sub_out_A = aluin1_A - aluin2_A;
	wire[(DBITS - 1) : 0] add_out = aluin1_A + aluin2_A;
	wire[(DBITS - 1) : 0] and_out = aluin1_A & aluin2_A;
	wire[(DBITS - 1) : 0] or_out = aluin1_A | aluin2_A;
	wire[(DBITS - 1) : 0] xor_out = aluin1_A ^ aluin2_A;
	wire eq_out = aluin1_A == aluin2_A;
	wire lt_out = sub_out_A[(DBITS - 1)];
	
	always @ * begin
		if (iscomparison) begin
			case (comparisonfunc)
				FUNCSEL_EQ: aluout_A = {31'b0, eq_out};
				FUNCSEL_LT: aluout_A = {31'b0, lt_out};
				FUNCSEL_LE: aluout_A = {31'b0, (eq_out | lt_out)};
				FUNCSEL_NE: aluout_A = {31'b0, ~eq_out};
				default: aluout_A = {DBITS{1'bX}};
			endcase
		end else begin
			if (isbasecalc) begin
				case (calcfunc)
					FUNCSEL_ADD: aluout_A = add_out;
					FUNCSEL_AND: aluout_A = and_out;
					FUNCSEL_OR: aluout_A = or_out;
					FUNCSEL_XOR: aluout_A = xor_out;
					default: aluout_A = {DBITS{1'bX}};
				endcase
			end else begin
				case (calcfunc)
					FUNCSEL_ADD: aluout_A = sub_out_A;
					FUNCSEL_AND: aluout_A = ~and_out;
					FUNCSEL_OR: aluout_A = ~or_out;
					FUNCSEL_XOR: aluout_A = ~xor_out;
					default: aluout_A = {DBITS{1'bX}};
				endcase
			end
		end
	end

	// Generate branch and jump signals
	wire dobranch_A = isbranch_A & aluout_A[0];
	wire [(DBITS - 1) : 0] immx4_A = sxtimm_A << 2;
	wire [(DBITS - 1) : 0] brtarg_A = immx4_A + pcplus_A;
	wire [(DBITS - 1) : 0] jmptarg_A = immx4_A + regval1_A;
	
	// Decide what to do based off of signals and branch prediction
	wire [(DBITS - 1) : 0] pcgood_A = dobranch_A ? brtarg_A : (isjump_A ? jmptarg_A : pcplus_A);
	wire mispred_A = (pcgood_A != pcpred_A);
	wire mispred_B = mispred_A && !isnop_A;
	wire [(DBITS - 1) : 0] pcgood_B = pcgood_A;
	
	// Generate the flush signals
	wire flush_D = !isnop_A & mispred_B;
	
	// Set branch prediction values
	wire [7 : 0] predidx_A = pcplus_A[9 : 2];
	
	always @(posedge clk)
		if (!reset && isbranch_A)
			branchpred[predidx_A] <= pcgood_B;

	wire flush_D = 1'bX;
	wire pcgood_B = 1'bX;
	wire [(DBITS - 1) : 0] pcgood_B = {DBITS{1'bX}};

	/*
	 * ----------------------------- MEM ----------------------------- 
	 */
	 
	// Create pipeline buffer for M stage
	reg wrmem_M, selaluout_M, selmemout_M, selpcplus_M, wrreg_M;
	reg [(DBITS - 1) : 0] aluout_M, pcplus_M, regval2_M;
	reg [(REGNOBITS - 1) : 0] wregno_M;
	
	always @(posedge clk) begin
		{wrmem_M, selaluout_M, selmemout_M, selpcplus_M, wrreg_M} <= {wrmem_A, selaluout_A, selmemout_A, selpcplus_A, wrreg_A};
		{aluout_M, pcplus_M, regval2_M} <= {aluout_A, pcplus_A, regval2_A};
		wregno_M <= wregno_A;
	end
		
	// Create memory signals
	wire [(DBITS - 1) : 0] memaddr_M, wmemval_M;
	assign {memaddr_M, wmemval_M} = {aluout_M, regval2_M};
	
	// Create and connect HEX register
	reg [23 : 0] HexOut;
	SevenSeg ss5(.OUT(HEX5),.IN(HexOut[23 : 20]));
	SevenSeg ss4(.OUT(HEX4),.IN(HexOut[19 : 16]));
	SevenSeg ss3(.OUT(HEX3),.IN(HexOut[15 : 12]));
	SevenSeg ss2(.OUT(HEX2),.IN(HexOut[11 : 8]));
	SevenSeg ss1(.OUT(HEX1),.IN(HexOut[7 : 4]));
	SevenSeg ss0(.OUT(HEX0),.IN(HexOut[3 : 0]));
	
	always @(posedge clk or posedge reset)
		if(reset)
			HexOut <= 24'hFEDEAD;
		else if(wrmem_M && (memaddr_M == ADDRHEX))
			HexOut <= wmemval_M[23 : 0];

	// Create and connect LEDR register
	reg [9: 0] LEDRout;
	assign LEDR = LEDRout;
	
	always @(posedge clk or posedge reset)
		if(reset)
			LEDRout <= 10'd0;
		else if(wrmem_M && (memaddr_M == ADDRLEDR))
			LEDRout <= wmemval_M[9 : 0];
	
	// Now the real data memory
	wire MemEnable = !(memaddr_M[(DBITS - 1) : DMEMADDRBITS]);
	wire MemWE = (!reset) & wrmem_M & MemEnable;
	
	(* ram_init_file = IMEMINITFILE, ramstyle = "no_rw_check" *)
	reg [(DBITS - 1) : 0] dmem[(DMEMWORDS - 1) : 0];
	
	always @(posedge clk)
		if(MemWE)
			dmem[memaddr_M[(DMEMADDRBITS - 1) : DMEMWORDBITS]] <= wmemval_M;

	wire [(DBITS - 1) : 0] MemVal = MemWE ? {DBITS{1'bX}} : dmem[memaddr_M[(DMEMADDRBITS - 1) : DMEMWORDBITS]];
	
	// Connect memory and input devices to the bus
	wire [(DBITS - 1) : 0] memout_M = MemEnable ? MemVal : (
		(memaddr_M == ADDRKEY) ? {12'b0, ~KEY} : (
			(memaddr_M == ADDRSW) ? {6'b0, SW} : 32'hDEADDEAD)
		);

	// Decide what gets written into the destination register (wregval_M),
	// when it gets written (wrreg_M) and to which register it gets written (wregno_M)
	wire [(DBITS - 1) : 0] wregval_M = selpcplus_M ? pcplus_M : (
		selaluout_M ? aluout_M : (
			selmemout_M ? memout_M : {(DBITS){1'bX}}
		)
	);

	always @(posedge clk)
		if(wrreg_M && !reset)
			regs[wregno_M] <= wregval_M;
endmodule
