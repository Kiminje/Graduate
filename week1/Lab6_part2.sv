module Lab6_part2 
#(N= 4)
(A, S, clk, overflow, carry, add_sub);
	input [N-1:0]A;
	input clk, add_sub;
	output overflow, carry;
	output [N-1:0]S;

	reg ov, ca, ctl;
	reg [N-1:0] sum, in;
	
	wire ovw, caw;
	wire [N-1:0] in1, sumw;
	initial begin
		ov = 0; ca = 0; ctl = 0;
		sum = 0; in = 0;
	end
	assign in1 = ctl? -in: in;
	assign ovw = (sum[N-1] ^ A[N-1])? 0 : (sumw[N-1] ^ A[N-1]);
	assign {caw, sumw[N-1:0]} = in1 + sum;
	always @(posedge clk)begin
		in <= A;
		ctl <= add_sub;
		ov <= ovw;
		ca <= caw;
		sum <= sumw;
	end

	assign overflow = ov;
	assign carry = ca;
	assign S = sum;

endmodule
 