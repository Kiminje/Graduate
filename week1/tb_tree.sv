`timescale 1ns/100ps
module tb_tree
#(parameter N=3)();
	logic [2**N-1:0] A, B;
	logic [2*2**N-1:0] Result;
	logic clk;
	always begin
	 #5 clk = 1'b1;
	 #5 clk = 1'b0;
	 end
	 
	 always @(posedge clk)begin
	   A = 8'd12;
	   B = 8'd10;
	   #20;
	   A = 8'd255;
	   B = 8'd255;
	   #20;
	   A = 8'd127;
	   B = 8'd2;;
	   #20;
	   
	   end
	AdderTree #(.N(N))dut
		(.clk(clk), 
		.A(A), .B(B),
		.P(Result));

	
endmodule

