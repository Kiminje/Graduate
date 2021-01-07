


module Lab6_part4
#(parameter N=8)
(input [N-1:0] A, B,
input clk,
output [2*N -1: 0] P,
output [N-1:0][N-1:0]path,
output [N-1:0] carry);
	reg [N-1:0] Ain, Bin;
	reg [2*N-1:0] Pout;
	wire [2*N-1:0] sum;
	Mul_adder #(.N(N)) multiplier(.A(Ain), .B(Bin), .P(sum), .path(path),
			.carry(carry));
	always @(posedge clk)begin
		Pout <= sum;
		Ain <= A;
		Bin <= B;
	end
	assign P = Pout;
endmodule