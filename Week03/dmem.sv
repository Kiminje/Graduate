/* ********************************************
 *	RISC-V RV32I single-cycle processor design
 *
 *	Module: data memory (dmem.sv)
 *	- 1 address input port
 *	- 32-bit 1 data output port
 *	- This data memory supports byte-address
 *	- RISC-V does not restrict aligned word
 *
 *	Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps
`define FF 1	// FF delay for just better waveform figures

module dmem
#(	parameter	DMEM_DEPTH = 1024,		// dmem depth in a word (4 bytes, default: 1024 entries = 4 KB)
				DMEM_ADDR_WIDTH = 12 )	// dmem address width in a byte
(
	input			clk,
	input	[DMEM_ADDR_WIDTH-1:0]	addr,
	input			rd_en,		// read enable
	input			wr_en,		// write enable
	input	[2:0]	funct3,			// data size (LB, LH, LW)
	input	[63:0]	din,
	output	[63:0]	dout
);
    logic [1:0] sz;
    assign sz = funct3[1:0];
    
	// dmem does not receive the clock signal in the textbook.
	// however, it requires clocked write operation for better operation and synthesis

	// memory entries. dmem is split into 4 banks to support various data granularity
	logic	[7:0]	d0[0:DMEM_DEPTH-1];
	logic	[7:0]	d1[0:DMEM_DEPTH-1];
	logic	[7:0]	d2[0:DMEM_DEPTH-1];
	logic	[7:0]	d3[0:DMEM_DEPTH-1];
	logic	[7:0]	d4[0:DMEM_DEPTH-1];
	logic	[7:0]	d5[0:DMEM_DEPTH-1];
	logic	[7:0]	d6[0:DMEM_DEPTH-1];
	logic	[7:0]	d7[0:DMEM_DEPTH-1];
	logic   [63:0]  data[0:DMEM_DEPTH-1];
	
	// address for each bank
	logic	[DMEM_ADDR_WIDTH-4:0]	addr0;	// address for bank 0
	logic	[DMEM_ADDR_WIDTH-4:0]	addr1;	// address for bank 1
	logic	[DMEM_ADDR_WIDTH-4:0]	addr2;	// address for bank 2
	logic	[DMEM_ADDR_WIDTH-4:0]	addr3;	// address for bank 3
	logic	[DMEM_ADDR_WIDTH-4:0]	addr4;	// address for bank 4
	logic	[DMEM_ADDR_WIDTH-4:0]	addr5;	// address for bank 5
	logic	[DMEM_ADDR_WIDTH-4:0]	addr6;	// address for bank 6
	logic	[DMEM_ADDR_WIDTH-4:0]	addr7;	// address for bank 7
	
	assign addr0 = addr[DMEM_ADDR_WIDTH-1:3] + |addr[2:0];
	assign addr1 = addr[DMEM_ADDR_WIDTH-1:3] + |addr[2:1];
	assign addr2 = addr[DMEM_ADDR_WIDTH-1:3] + (addr[2] | &addr[1:0]);
	assign addr3 = addr[DMEM_ADDR_WIDTH-1:3] + addr[2];
	assign addr4 = addr[DMEM_ADDR_WIDTH-1:3] + (addr[2] & |addr[1:0]);
	assign addr5 = addr[DMEM_ADDR_WIDTH-1:3] + &addr[2:1];
	assign addr6 = addr[DMEM_ADDR_WIDTH-1:3] + &addr[2:0];
	assign addr7 = addr[DMEM_ADDR_WIDTH-1:3];
	
	// data out from each bank
	wire	[7:0]	dout0, dout1, dout2, dout3;
	
	assign dout0 = d0[addr0];
	assign dout1 = d1[addr1];
	assign dout2 = d2[addr2];
	assign dout3 = d3[addr3];
	assign dout4 = d4[addr4];
	assign dout5 = d5[addr5];
	assign dout6 = d6[addr6];
	assign dout7 = d7[addr7];
	
	// read operation with rd_en
	logic	[63:0]	dout_tmp;	// need to be aligned by an address offset
	
	always_comb begin
		case (addr[2:0])	// synopsys full_case parallel_case
			3'b000: dout_tmp = {dout7, dout6, dout5, dout4, dout3, dout2, dout1, dout0};
			3'b001: dout_tmp = {dout0, dout7, dout6, dout5, dout4, dout3, dout2, dout1};
			3'b010: dout_tmp = {dout1, dout0, dout7, dout6, dout5, dout4, dout3, dout2};
			3'b011: dout_tmp = {dout2, dout1, dout0, dout7, dout6, dout5, dout4, dout3};
			3'b100: dout_tmp = {dout3, dout2, dout1, dout0, dout7, dout6, dout5, dout4};
			3'b101: dout_tmp = {dout4, dout3, dout2, dout1, dout0, dout7, dout6, dout5};
			3'b110: dout_tmp = {dout5, dout4, dout3, dout2, dout1, dout0, dout7, dout6};
			3'b111: dout_tmp = {dout6, dout5, dout4, dout3, dout2, dout1, dout0, dout7};
		endcase
	end
	logic [63:0] _dout;
	assign dout = (rd_en) ? _dout : 'b0;
	
	// write operation with wr_en
	logic	[7:0]	we;		// write enable for each bank
	
	always_comb begin
		if (sz==2'b00) begin
			case (addr[2:0])	// synopsys full_case
				3'b000: we = {7'b0000000, wr_en};
				3'b001: we = {6'b000000, wr_en, 1'b0};
				3'b010: we = {5'b00000, wr_en, 2'b00};
				3'b011: we = {4'b0000, wr_en, 3'b000};
				3'b100: we = {3'b000, wr_en, 4'b0000};
				3'b101: we = {2'b00, wr_en, 5'b00000};
				3'b110: we = {1'b0, wr_en, 6'b000000};
				3'b111: we = {wr_en, 7'b0000000};
			endcase
			_dout = funct3[2]? {{56'b0}, dout_tmp[7:0]}: {{56{dout_tmp[7]}}, dout_tmp[7:0]};
		end
		else if (sz==2'b01) begin
			case (addr[2:0])	// synopsys full_case
			    3'b000: we = {6'b0000000, {2{wr_en}}};
				3'b001: we = {5'b000000, {2{wr_en}}, 1'b0};
				3'b010: we = {4'b00000, {2{wr_en}}, 2'b00};
				3'b011: we = {3'b0000, {2{wr_en}}, 3'b000};
				3'b100: we = {2'b000, {2{wr_en}}, 4'b0000};
				3'b101: we = {1'b00, {2{wr_en}}, 5'b00000};
				3'b110: we = {{2{wr_en}}, 6'b000000};
				3'b111: we = {wr_en, 7'b0000000, wr_en};
			endcase
			_dout = funct3[2]? {{48'b0}, dout_tmp[15:0]}: {{48{dout_tmp[7]}}, dout_tmp[15:0]};
		end
		else begin
		    case (addr[2:0])	// synopsys full_case
			    3'b000: we = {4'b0000000, {4{wr_en}}};
				3'b001: we = {3'b000000, {4{wr_en}}, 1'b0};
				3'b010: we = {2'b00000, {4{wr_en}}, 2'b00};
				3'b011: we = {1'b0000, {4{wr_en}}, 3'b000};
				3'b100: we = {{4{wr_en}}, 4'b0000};
				3'b101: we = {{3{wr_en}}, 4'b0000, wr_en};
				3'b110: we = {{2{wr_en}}, 4'b0000, {2{wr_en}}};
				3'b111: we = {wr_en, 4'b0000, {3{wr_en}}};
			endcase
			_dout = funct3[2]? {{32'b0}, dout_tmp[31:0]}: {{32{dout_tmp[7]}}, dout_tmp[31:0]};
        end
	end
	
	// write operation that supports unaligned words
	logic	[63:0]	din_tmp;
	
	always_comb begin
		case (addr[2:0])	// synopsys full_case parallel_case
			3'b000: din_tmp = din[63:0];
			3'b001: din_tmp = {din[55:0], din[63:56]};
			3'b010: din_tmp = {din[47:0], din[63:48]};
			3'b011: din_tmp = {din[39:0], din[63:40]};
			3'b100: din_tmp = {din[31:0], din[63:32]};
			3'b101: din_tmp = {din[23:0], din[63:24]};
			3'b110: din_tmp = {din[15:0], din[63:16]};
			3'b111: din_tmp = {din[7:0], din[63:8]};
		endcase
	end
	
	// in the textbook, dmem does not receive the clock signal
	// but clocked write operation is required for better operation and synthesis
	// we must avoid latch for the normal cases
	always_ff @ (posedge clk) begin
		if (we[0]) d0[addr0] <= din_tmp[7:0];
		if (we[1]) d1[addr1] <= din_tmp[15:8];
		if (we[2]) d2[addr2] <= din_tmp[23:16];
		if (we[3]) d3[addr3] <= din_tmp[31:24];
		if (we[4]) d4[addr4] <= din_tmp[39:32];
		if (we[5]) d5[addr5] <= din_tmp[47:40];
		if (we[6]) d6[addr6] <= din_tmp[55:48];
		if (we[7]) d7[addr7] <= din_tmp[63:56];
	end
	
	// synthesis translate_off
	initial begin
        for (int i = 0; i < DMEM_DEPTH; i++)
            data[i] = 'b0;
        $readmemh("dmem.mem", data);
        for (int i = 0; i < DMEM_DEPTH; i++)
            {d7[i], d6[i], d5[i], d4[i], d3[i], d2[i], d1[i], d0[i]} = data[i];
        
    end
// synthesis translate_on
	
endmodule