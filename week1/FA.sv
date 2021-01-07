module FA(input A, B, Cin,
	output Cout, S);
	assign {Cout, S} = A + B + Cin;
endmodule

