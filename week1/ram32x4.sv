
module ram32x4 
(input [4:0] address, 
input clock, 
input [3:0] data, 
input wren, 
output [3:0] q);

	reg [3:0] RAM [31:0];
	reg [4:0] addrin;
	reg [3:0] datain;
	reg 	  we;
	assign q = RAM[addrin];
	
	always @(posedge clock)begin
		addrin <= address;
		datain <= data;
		we     <= wren;
		if (we)
			RAM[addrin] <= datain;
	end
endmodule
		