
module Mul_adder
#(N=8)
(input [N-1:0] A, B,
output [2*N -1: 0] P,
output [N-1:0][N-1:0]path,
output [N-1:0]carry);

	wire [N-1:0]cry;
	wire  [N*N-1:0] Apath, Bpath;
	wire [N-1:0] concate, ext1, ext0;
	wire [N-2:0] lowcase;
	assign path[N-1:0] = Apath[N*N-1:0];
	assign carry[N-1:0] = cry[N-1:0];
	assign ext0 = {N{B[0]}};
	assign ext1 = {N{B[1]}};		
	assign Apath[N-1:0] = A & ext0;
	assign lowcase[0] = Apath[0] * Bpath[0];
	assign cry[0] = 0;
	assign Bpath[N-1: 0] = A & ext1;
	assign concate = {cry[0], Apath[N-1:1]};
	NbitAdder #(.N(N))adder(.A(concate),.B(Bpath[N-1:0]),.Sw(Apath[2*N-1:N]),.carry(cry[1]));

	genvar j;
	generate
	 for (j=1; j<N-1; j=j+1) begin : generate_level_multiplier
	  assign Bpath[j*N+N-1: j*N] = A & {N{B[j+1]}};
	  assign lowcase[j] = Apath[j*N];
	  NbitAdder #(.N(N))adder2
				(.A({cry[j],Apath[N*j+N-1:N*j+1]}),
				.B(Bpath[N*j+N-1:N*j]),
				.Sw(Apath[j*N+N+N-1:j*N+N]),
				.carry(cry[j+1]));
	 end
	endgenerate
	
	assign P = {cry[N-1], Apath[N*N-1:N*N-N], lowcase};
endmodule
