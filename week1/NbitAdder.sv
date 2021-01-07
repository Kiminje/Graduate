module NbitAdder #(N =4)
(input [N-1:0] A, B,
output [N-1:0] Sw,
output carry);
	
	wire [N: 0] cry;
	assign cry[0] = 0;
	assign carry = cry[N];


	genvar i;
	generate
   	 for (i=0; i< N; i=i+1) begin : generate_adder_block_identifier // <-- block name
  	  FA Addergen (
     	   .A(A[i]),
     	   .B(B[i]),
	   .Cin(cry[i]),
   	     .S(Sw[i]),
	     .Cout(cry[i+1])
  	  );
	 end 
	endgenerate
endmodule
