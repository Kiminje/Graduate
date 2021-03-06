/* ********************************************
 *	COSE222 Lab #2
 *
 *	Module: data memory (dmem.sv)
 *	- 1 address input port
 *	- 32-bit 1 data input and output ports
 *	- A single entry size is 64-bit
 *
 *	Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps
`define FF 1    // Flip-flop delay for just better waveform view

module dmem
#(  parameter DMEM_DEPTH = 1024,    // dmem depth (default: 1024 entries = 8 KB)
              DMEM_ADDR_WIDTH = 10 )
(
    input           clk,
    input   [DMEM_ADDR_WIDTH-1:0]   addr,
    input   [63:0]  din,
    input   [2:0]   func3,
    input           mem_read,
    input           mem_write,
    output  [63:0]  dout
);

    /* Data memory does not receive the clock signal in the textbook.
     * Without clock we need to implement the data memory with latches.
     * However, you must avoid generating latches in real RTL design.
     * If latches are generated after synthesis, then it means your design includes critical bugs.
     * Hence, in this design you are requested to design the data memory with the clock signal.
     * That means the written data is updated at the rising edge of the clock signal.
     */

    // Actually RISC-V supports misaligned data accesses to memory, however it this design the data memory will only support
    // the aligned memory accesses.

    logic   [63:0]  data[0:DMEM_DEPTH-1];

    // Write operation:
    always_ff @ (posedge clk) begin
        if (mem_write)begin
            if(func3[1])
                data[addr][31:0] <= din[31:0];
            else begin
                if (func3[0])
                    data[addr][15:0] <= din[15:0];
                else data[addr][7:0] <= din[7:0];
            end
        end
    end

    // Read operation:
    // - dout = 0 if (mem_read==0) 
    assign dout = (mem_read) ? (func3[2]? (func3[0]?{48'b0, data[addr][15:0]}: 
        {56'b0, data[addr][7:0]}):(func3[1]?{{32{data[addr][63]}},data[addr]}:
        (func3[0]?{{48{data[addr][63]}}, data[addr][15:0]}:{{56{data[addr][63]}}, data[addr][7:0]}))): 'b0;

// synthesis translate_off
    initial begin
        for (int i = 0; i < DMEM_DEPTH; i++)
            data[i] = 'b0;
        $readmemh("dmem.mem", data);
    end
// synthesis translate_on

endmodule