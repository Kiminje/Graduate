`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/06/2021 04:26:00 PM
// Design Name: 
// Module Name: decoder
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module decoder(
    input [6:0] opcode, func7,
    input [2:0] func3,
    output logic [6:0] signal,
    output logic [3:0] ctrl);


    logic [1:0] op;
    assign op = {opcode[6], opcode[2]};
// signal[7:0] = {jump, branch, mem_read, mem2reg,mem_write, alu_src, reg_write}
    always_comb begin
        case (opcode[4:2])
            'b000: begin
                if (opcode[6]) begin
                    ctrl = {1'b1, func3};
                    signal = 7'b0100000; //branch
                end
                else begin // store & load
                    ctrl = 4'b0010;
                    if (opcode[5]) 
                        signal = 7'b0000110; //store
                    else signal = 7'b0011011;//load
                end
                end // L(I)
            'b001: begin // JALR
                signal = 7'b1000011;
                ctrl = 4'b0010;
                end
            'b011: begin
                signal = 7'b1000011; //JAL
                ctrl = 4'b0010;
            end
            'b100: begin // S
                signal = opcode[5] ? 7'b0000001 : 7'b0000011;
                case (func3)
                    3'b000: begin
                        if (func7[5])
                            ctrl = 4'b0110; //sub
                        else ctrl = 4'b0010;//add
                     end
                    3'b001: ctrl = 4'b0011; //shift left
                    3'b010: ctrl = 4'b1100; // slt
                    3'b011: ctrl = 4'b1110; // slt unsigned
                    3'b100: ctrl = 4'b0111; // XOR
                    3'b101: ctrl = func7[5]? 4'b0101 : 4'b0100; //shift righta: shift right
                    3'b110: ctrl = 4'b0001; // OR
                    3'b111: ctrl = 4'b0000; // AND
                    default: ctrl = 4'b0010;                    
                endcase
                end
            'b101: begin // U {imm, 12'b0}
                if ( opcode[5]) signal = 7'b1100001;
                else signal = 7'b0000001;
                ctrl = 4'b0010;
                end
            default: begin
                ctrl = 4'b0010;    // default = add
		        signal = 7'b0000000;
		    end
		endcase                
        end
endmodule
