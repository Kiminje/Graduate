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
    output  logic [REG_WIDTH-1:0] result, // ALU output
    output  logic        zero            // Zero detection
);
    logic signed  [REG_WIDTH:0]   mid;
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
            4'b1100: begin 
                mid = $signed(in1)-$signed(in2);   
                result ={{63{1'b0}}, mid[REG_WIDTH]}; 
                end
            4'b1101: begin 
                result = $signed(in1)-$signed(in2);
                end
            4'b1000: begin
                result = $signed(in1) - $signed(in2);  
                end
            4'b1001: begin
                result = $signed(in1) - $signed(in2);   
                end
            4'b1110: begin
                mid = in1 - in2;   
                result = {{63{1'b0}}, mid[REG_WIDTH]};  
                end
            4'b1111: begin
                mid = in1 - in2;   
                result = {{63{1'b0}}, mid[REG_WIDTH]};
                end       
            default: result = in1 + in2;    // default = add
		endcase
    end

    always_comb begin
        if (alu_control[3])begin
            case(alu_control[2:0])
                3'b000: zero = ~|result;
                3'b001: zero = |result;
                3'b100: zero = result[0];
                3'b110: zero = result[0];
                3'b101: zero = ~result[REG_WIDTH-1];
                3'b111: zero = ~result[0];
                default: zero = ~|result;
            endcase
            end
        else zero = ~|result;
        end

endmodule