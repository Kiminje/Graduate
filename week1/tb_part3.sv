`timescale 1ns/100ps
module tb_part3
#(parameter N=8)();
	 
	logic [N-1:0] A, B, carry;
	logic [N-1:0][N-1:0] path, path1;
	logic [2*N-1:0] Result, Result1;
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
	   
	Lab6_part3 #(.N(N))dut1 
		(.clk(clk), 
		.A(A), .B(B),
		.P(Result1), .path(path1));
	Lab6_part4 #(.N(N))dut 
		(.clk(clk), 
		.A(A), .B(B),
		.P(Result), .path(path), .carry(carry));

	
endmodule

