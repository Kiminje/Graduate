/* ********************************************
 *	COSE222 Lab #3
 *
 *	Module: top design of the single-cycle CPU (single_cycle_cpu.sv)
 *  - Top design of the single-cycle CPU
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps
`define FF 1    // Flip-flop delay for just better waveform view

module single_cycle_cpu
#(  parameter IMEM_DEPTH = 1024,    // imem depth (default: 1024 entries = 4 KB)
              IMEM_ADDR_WIDTH = 10,
              REG_WIDTH = 64,
              DMEM_DEPTH = 1024,    // dmem depth (default: 1024 entries = 8 KB)
              DMEM_ADDR_WIDTH = 10 )
(
    input           clk,            // System clock
    input           reset_b         // Asychronous negative reset
);

    // Wires for datapath elements
    logic   [IMEM_ADDR_WIDTH-1:0]   imem_addr;
    logic   [31:0]  inst;   // instructions = an output of ????

    logic   [4:0]   rs1, rs2, rd;    // register numbers
    logic   [REG_WIDTH-1:0] rd_din;
    logic           reg_write;
    logic   [REG_WIDTH-1:0] rs1_dout, rs2_dout;

    logic   [REG_WIDTH-1:0] alu_in1, alu_in2;
    logic   [3:0]   alu_control;    // ALU control signal
    logic   [REG_WIDTH-1:0] alu_result;
    logic           alu_zero;

    logic   [DMEM_ADDR_WIDTH-1:0]    dmem_addr;
    logic   [63:0]  dmem_din, dmem_dout;
    logic           mem_read, mem_write, mem2reg, alu_src;

    // -------------------------------------------------------------------
    /* Main control unit:
     * Main control unit generates control signals for datapath elements
     * The control signals are determined by decoding instructions
     * Generating control signals using opcode = inst[6:0]
     */
    logic   [6:0]   opcode, func7;
    logic   [2:0]   func3;
    logic   [1:0]   alu_op;
    logic           branch, alu_src, mem_to_reg, jump;
    //logic         mem_read, mem_write, reg_write; // declared above
    logic   [31:0]  imm32;
    // COMPLETE THE MAIN CONTROL UNIT HERE
    decoder decoder(opcode, func3, func7, {jump, branch, mem_read, mem2reg, 
     mem_write, alu_src, reg_write}, alu_control, alu_op);
     
 /*
ALU control
0000: AND
0001: OR
0010: +
0011: << 
0100: >>
0101: >>>
0110: -
0111: XOR
1000: 
1001: 
1010: ==
1011: !=
1100: set less than <
1101: >=
1110: set less than < unsigned
1111: >= unsigned
*/    


    // --------------------------------------------------------------------

    // --------------------------------------------------------------------
    /* ALU control unit:
     * ALU control unit generate alu_control signal which selects ALU operations
     * Generating control signals using alu_op, funct7, and funct3 fileds
     */

    // COMPLETE THE ALU CONTROL UNIT HERE



    // ---------------------------------------------------------------------


    // ---------------------------------------------------------------------
    /* Immediate generator:
     * Generating immediate value from inst[31:0]
     */
    logic   [63:0]  imm64;
    logic   [63:0]  imm64_branch,imm64_U, imm64_PC;  // imm64 left shifted by 1
//    logic   [31:0]  imm32;  // 12-bit immediate value extracted from inst
    // COMPLETE IMMEDIATE GENERATOR HERE
    function [63:0] imm_gen;
        input op6, op5, op3, op2;
        input [31:0] inst;
        if(op6)begin
            if(op2)begin
                if(op3) imm_gen = {{45{inst[31]}},inst[31], inst[19:12],inst[20], inst[30:21]}; //JAL
                else    imm_gen = {{52{inst[31]}},inst[31:20]} ; //JALR
                end
           else imm_gen = {{52{inst[31]}},inst[31], inst[7], inst[30:25], inst[11:8]} ;;//Branch
           end
       else begin
           if(op2) imm_gen = {{32{inst[31]}}, inst[31:12]} << 12;
           else begin
                if(op5) imm_gen = {{52{inst[31]}}, inst[31:25], inst[11:7]}; // S-type
                else    imm_gen = (func3 == 3'b011)? {{52{1'b0}}, inst[31:20]}:{{52{inst[31]}}, inst[31:20]}; // I-type
           end
        end                
        endfunction
    assign imm64 = imm_gen(opcode[6], opcode[5], opcode[3], opcode[2], inst);
    assign imm64_branch = imm64 << 1;
    


    // ----------------------------------------------------------------------

    // Program counter
    logic   [63:0]  pc_curr, pc_next;
    logic   [63:0]  pc_next_plus4, pc_next_branch, pc_next_jump;
    logic           exclusive;
    
    assign exclusive = jump ^ branch;
    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            pc_curr <= 'b0;
        end else begin
            pc_curr <=  pc_next;       // FILL THIS
        end
    end


    // MUXes:
    // COMPLETE MUXES HERE
    // PC_NEXT
    assign pc_next_branch = imm64_branch + pc_curr;
    assign pc_next_jump = rs1_dout + imm64;
    assign pc_next_plus4 = pc_curr + 'd4;    // FILL THIS
    assign pc_next = (jump & exclusive)? pc_next_jump: ((alu_zero & branch & exclusive)? pc_next_branch : pc_next_plus4);
    // ALU inputs
    assign alu_in1 = rs1_dout;
    assign alu_in2 = alu_src? imm64 : rs2_dout;

    // RF din
    assign rs2 = inst[24:20];
    assign rs1 = inst[19:15];
    assign rd = inst[11:7]; 
    assign rd_din = reg_write? (opcode[2]? imm64_U : alu_result): 'bz;
    // COMPLETE CONNECTIONS HERE
    // imem
    assign imem_addr = pc_curr;
    // regfile
    assign imm64_U = opcode[5] ? imm64 : (imm64 + pc_curr);
    // dmem
    assign dmem_in = rs2_dout;
//    func3[1]? {32'b0, rs2_dout[31:0]}: (func3[0]? {48'b0, rs2_dout[15:0]}: {56'b0, rs2_dout[7:0]});
    assign dmem_addr = alu_result;

    // -----------------------------------------------------------------------
    /* Instantiation of datapath elements
     * All input/output ports should be connected
     */
    
    // IMEM
    imem #(
        .IMEM_DEPTH         (IMEM_DEPTH),
        .IMEM_ADDR_WIDTH    (IMEM_ADDR_WIDTH)
    ) u_imem_0 (
        .addr               ( imem_addr     ),
        .dout               ( inst          )
    );

    // REGFILE
    regfile #(
        .REG_WIDTH       (REG_WIDTH)
    ) u_regfile_0 (
        .clk        (clk),
        .rs1        (rs1),
        .rs2        (rs2),
        .rd         (rd),
        .rd_din     (rd_din),
        .reg_write  (reg_write),
        .rs1_dout   (rs1_dout),
        .rs2_dout   (rs2_dout));
    // ALU
    alu #(
        .REG_WIDTH      (REG_WIDTH)
    ) u_alu_0   (
        .in1        (alu_in1),
        .in2        (alu_in2),
        .alu_control(alu_control),
        .result     (alu_result),
        .zero       (alu_zero));
    // DMEM
    dmem #(
        .DMEM_DEPTH     (DMEM_DEPTH),
        .DMEM_ADDR_WIDTH(DMEM_ADDR_WIDTH)
    ) u_dmem_0 (
        .clk    (clk),
        .addr   (alu_result),
        .din    (dmem_in),
        .func3  (func3),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .dout   (dmem_out));
endmodule