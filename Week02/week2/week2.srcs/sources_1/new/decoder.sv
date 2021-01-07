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
    output logic [7:0] signal,
    output logic [3:0] ctrl);


    logic [1:0] op;
    assign op = {opcode[6], opcode[2]};
// signal[7:0] = {jump, branch, mem_read, mem2reg,mem_write, alu_src, reg_write}
    always_comb begin
        case (opcode[4:2])
            'b000: begin
                if (opcode[6]) begin
                    ctrl = {1'b1, func3};
                    signal = 'b0100010; //branch
                end
                else begin // store & load
                    ctrl = 'b0010;
                    if (opcode[5]) 
                        signal = 'b0000110; //store
                    else signal = 'b0011011;//load
                end
                end // L(I)
            'b001: begin // JALR
                signal = 'b1000011;
                ctrl = 'b0010;
                end
            'b011: begin
                signal = 'b0100011; //JAL
                ctrl = 'b0010;
            end
            'b100: begin // S
                signal = opcode[5] ? 'b0000001 : 'b0000011;
                case (func3)
                    'b000: begin
                        if (func7[5])
                            ctrl = 'b0110; //sub
                        else ctrl = 'b0010;//add
                     end
                    'b001: ctrl = 'b0011; //shift left
                    'b010: ctrl = 'b1100; // slt
                    'b011: ctrl = 'b1110; // slt unsigned
                    'b100: ctrl = 'b0111; // XOR
                    'b101: ctrl = func7[5]? 'b0101 : 'b0100; //shift right
                    'b110: ctrl = 'b0001; // OR
                    'b111: ctrl = 'b0000; // AND
                    default: ctrl = 'b0010;                    
                endcase
                end
            'b101: begin // U {imm, 12'b0}
                if ( opcode[5]) signal = 'b1100001;
                else signal = 'b0000001;
                ctrl = 'b0010;
                end
            default: ctrl = 'b0010;    // default = add
		endcase                
        end
endmodule
