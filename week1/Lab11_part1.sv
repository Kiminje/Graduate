
module Lab11_part1
#(N=8)
(input [N-1:0]data,
input s,
input clk,
output [1:0] st,
output [N-1:0] Result);
	

	reg [1:0] State;
	reg [N-1:0] A, result;
	reg sin;
	assign st = State;
	assign Result = result;

	initial begin
		State = 2'b00;
		result = 0;
		A = data;
	end

	always @(posedge clk)begin
	sin <= s;
	case(State)
	2'b00:	begin
		if(sin) State <= 2'b01;
		else A <= data;
		result <= 0;
		end
	2'b01:	begin
		if(A == 0) State <= 2'b10;
		else begin
			if(A[0]) result = result +1;
			A = A >> 1;
		end
		end
	2'b10: 	begin
		if(sin);
		else State <= 2'b00;
		end
	endcase
	end
endmodule
