`timescale 1ns/100ps
module tb_part2
#(parameter N=4)();
	 
	logic [N-1:0] A, B;
	logic ov, ca, sub;
	logic clk;
	initial begin
		A = 4'b0;
		B = 4'd0;
		sub = 0;
		ov = 0;
		ca = 0;
	end
	always begin
	 #5 clk = 1'b1;
	 #5 clk = 1'b0;
	 end
	 
	 always @(posedge clk)begin
	   sub = 0;
	   A = 4'd1;
	   B = 4'd13;
	   #50;
	   sub = 1;
	   A = 4'd1;
	   B = 4'd3;
	   #50;
	   
	   end
	   
	Lab6_part2 #(.N(N))dut
		(.clk(clk), 
		.A(A),
		.S(B), .overflow(ov),
		.carry(ca), .add_sub(sub));

	
endmodule

