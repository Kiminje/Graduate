module Mul
#(N=8)
(input [N-1:0] A, B,
output [2*N -1: 0] P,
output [N-1:0][N-1:0]path);

	wire [N*N-1:0]cry;
	wire  [N*N-1:0] Apath, Bpath;
	wire [N-2:0] lowcase;	
	assign Apath[N-1:0] = A & {N{B[0]}};
	assign lowcase[0] = A[0] & B[0];
	assign cry[N*N-1] = 0;
	assign path[N-1:0] = Apath[N*N-1:0];
	assign cry[0] = 0;
	assign Bpath[N-1:0] = A & {N{B[1]}};
	genvar i, j;
	generate
	 for (i=0; i<N-1; i=i+1) begin : generate_adder // <-- block name	
  	   FA Addergen (
     	    .A(Apath[i+1]),
     	    .B(Bpath[i]),
	    .Cin(cry[i]),
   	      .S(Apath[N+i]),
	      .Cout(cry[i+1])
  	   );end
	   FA Addergen (
     	    .A(cry[N*N-1]),
     	    .B(Bpath[N-1]),
	    .Cin(cry[N-1]),
   	      .S(Apath[N+N-1]),
	      .Cout(cry[N])
  	   );
	 for (j=1; j<N-1; j=j+1) begin : generate_level_multiplier
	  assign Bpath[j*N+N-1: j*N] = A & {N{B[j+1]}};
	  assign cry[j*N+j] = 0;
	  assign lowcase[j] = Apath[j*N];
   	  for (i=0; i<N-1; i=i+1) begin : generate_adder // <-- block name	   
  	   FA Addergen (
     	    .A(Apath[j*N+i+1]),
     	    .B(Bpath[j*N+i]),
	    .Cin(cry[j*N+j+i]),
   	      .S(Apath[j*N+N+i]),
	      .Cout(cry[j*N+j+i+1])
  	   );
	  end
	  FA Addergen (
     	    .A(cry[j*N+j-N-1+N]),
     	    .B(Bpath[j*N+N-1]),
	    .Cin(cry[j*N+j+N-1]),
   	      .S(Apath[j*N+N+N-1]),
	      .Cout(cry[j*N+j+N]));
	  
	 end
	endgenerate
	
	assign P = {cry[N*N-2], Apath[N*N-1:N*N-N], lowcase};
endmodule
