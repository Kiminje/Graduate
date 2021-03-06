

module Lab6_part3
#(parameter N=8)
(input [N-1:0] A, B,
input clk,
output [2*N -1: 0] P,
output [N-1:0][N-1:0]path);
	reg [N-1:0] Ain, Bin;
	reg [2*N-1:0] Pout;
	wire [2*N-1:0] sum;
	Mul #(.N(N)) multiplier2(.A(Ain), .B(Bin), .P(sum), .path(path));
	always @(posedge clk)begin
		Pout <= sum;
		Ain <= A;
		Bin <= B;
	end
	assign P = Pout;
endmodule
