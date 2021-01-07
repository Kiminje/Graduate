/* ********************************************
 *	COSE222 Lab #2
 *
 *	Module: ALU (alu.sv)
 *  - 64-bit 2 input and 1 output ports
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps
`define FF 1    // Flip-flop delay for just better waveform view

module alu
#(  parameter REG_WIDTH = 64 )  // ALU input data width is equal to the width of register file
(
    input   [REG_WIDTH-1:0] in1,    // Operand 1
    input   [REG_WIDTH-1:0] in2,    // Operand 2
    input   [3:0]   alu_control,    // ALU control signal
    output  logic [REG_WIDTH-1:0] result,mid, // ALU output
    output  logic        zero            // Zero detection
);

    always_comb begin
        case (alu_control)
            4'b0000: result = in1 & in2;
            4'b0001: result = in1 | in2;
            4'b0010: result = $signed(in1) + $signed(in2);
            4'b0011: result = in1 << in2;
            4'b0100: result = in1 >> in2;
            4'b0101: result = in1 >>> in2;
            4'b0110: result = $signed(in1) - $signed(in2);
            4'b0111: result = in1 ^ in2;
            4'b1x0x: begin 
                mid = $signed(in1)-$signed(in2);
                result = result[REG_WIDTH-1];
                end
            4'b1x1x: begin
                mid = in1 - in2;   
                result = mid[REG_WIDTH-1];  
                end       
            default: result = in1 + in2;    // default = add
		endcase
    end

    always_comb begin
        if (alu_control[3])begin
            case(alu_control[2:0])
                3'b000: zero = ~|result;
                3'b001: zero = |result;
                3'b1x0: zero = result[0];
                3'b1x1: zero = ~result[0];
                default: zero = ~|result;
            endcase
            end
        else zero = ~|result;
        end

endmodule