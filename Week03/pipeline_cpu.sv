/* ********************************************
 *	COSE222 Lab #4
 *
 *	Module: pipelined_cpu.sv
 *  - Top design of the 5-stage pipelined RISC-V processor
 *  - Processor supports instructions described in Chapter 4 of COD book
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps
`define FF 1    // Flip-flop delay for just better waveform view

// Packed structures for pipeline registers
// Pipe reg: IF/ID
typedef struct packed {
    logic   [63:0]  pc;
    logic   [31:0]  inst;
} pipe_if_id;

// Pipe reg: ID/EX
typedef struct packed {
    logic   [63:0]  pc;
    logic   [63:0]  rs1_dout;
    logic   [63:0]  rs2_dout;
    logic   [63:0]  imm64;
    logic   [2:0]   funct3;
    logic   [6:0]   funct7;
    logic           branch;
    logic           jump;
    logic           alu_src;
    logic   [1:0]   alu_op;
    logic           mem_read;
    logic           mem_write;
    logic   [4:0]   rs1;
    logic   [4:0]   rs2;
    logic   [4:0]   rd;         // rd for regfile
    logic           reg_write;
    logic           mem_to_reg;
    logic   [1:0]   pctrl;
} pipe_id_ex;

// Pipe reg: EX/MEM
typedef struct packed {
    logic   [63:0]  pc;
    logic   [63:0]  alu_result; // for address
    logic   [63:0]  rs2_dout;   // for store
    logic           mem_read;
    logic           mem_write;
    logic   [4:0]   rd;
    logic           reg_write;
    logic           mem_to_reg;
    logic   [2:0]   size;
} pipe_ex_mem;

// Pipe reg: MEM/WB
typedef struct packed {
    logic   [63:0]  pc;
    logic   [63:0]  alu_result;
    logic   [63:0]  dmem_dout;
    logic   [4:0]   rd;
    logic           reg_write;
    logic           mem_to_reg;
} pipe_mem_wb;

function [63:0] pc_decision;
//    (branch_hit) ? pc_next_branch : (ex.jump? pc_next_jump : pc_next_plus4);
    input branch_hit, jump;
    input [63:0] pc_next_branch, pc_next_jump, pc_next_plus4;
    casex({branch_hit, jump})
        2'b1x: pc_decision = pc_next_branch;
        2'bx1: pc_decision = pc_next_jump;
        default: pc_decision = pc_next_plus4;
    endcase
endfunction
    
function [10:0] decoder;        
    //{jump, branch, mem_read, mem_to_reg, mem_write, alu_src, reg_write}
    input [6:0] opcode;
    casex (opcode[4:2])
        'b000: begin
            if(opcode[6]) decoder = {7'b0100010, 2'b01, 1'b0}; // branch                
            else begin
                if(opcode[5]) decoder = {7'b0000110, 2'b00, 1'b0}; // store
                else          decoder = {7'b0011011, 2'b00, 1'b0}; // load
            end
        end 
        'b0x1: decoder = {7'b1000011, 2'b00, 1'b0}; // UJ : JAL & JALR
        'b100: decoder = opcode[5]? {7'b0000001, 2'b10, 1'b0} : {7'b0000011, 2'b11, 1'b0}; // R-type : I-type
        'b101: decoder = {7'b0000011, 2'b00, 1'b1}; // U-type 
        default: decoder = 10'b0000000000;
    endcase
endfunction

// I-type arithmatic, branch, S-type, load, 
function [11:0] imm12_gen;
    input [6:0] opcode;
    input [1:0] alu_op;
    input [31:0] inst;
    case(alu_op)
        2'b01: imm12_gen = {inst[31], inst[7], inst[30:25], inst[11:8]}; // SB type
        2'b11: imm12_gen = (inst[13:12] == 2'b01) ? {{7'b0}, inst[24:20]}:inst[31:20]; // seperate shift operation, I-type
        2'b00: begin
            case(^opcode[6:5])
                'b0:imm12_gen = inst[31:20]; // JALR, load
                'b1: imm12_gen = {inst[31:25], inst[11:7]}; // S-type
                default: imm12_gen = inst[31:20];
            endcase
        end
        default: imm12_gen = 'b0;
    endcase
endfunction

function [63:0] imm64_gen;
    input [6:0] opcode;
    input [31:0] inst;
    input [11:0] imm12;
    logic [2:0] encoder;
    assign encoder = {opcode[6:5], opcode[3:2]};
    casex(encoder)
        'b0x01: imm64_gen = {{44{inst[31]}}, inst[31:12]} << 12; // U-type
        'b1111: imm64_gen = {{44{inst[31]}},{inst[31], inst[19:12], inst[20], inst[30:21]}}; // JAL
        'b1100: begin
            if (&inst[14:13]) imm64_gen = {{52'b0}, imm12};
            else imm64_gen = {{52{imm12[11]}},imm12}; // SB-type
        end
        'b0000: begin
            if(&inst[13:12] & ~inst[14]) imm64_gen = {{52'b0}, imm12}; // I-type
            else imm64_gen = {{52{imm12[11]}},imm12};
        end
        default: imm64_gen = {{52{imm12[11]}},imm12}; // S-type
    endcase
endfunction

function [3:0] ctrl;
    input [1:0] alu_op;
    input [2:0] funct3;
    input [6:0] funct7;
    input       pctrl_1; //pctrl_1
    if (pctrl_1) ctrl = 4'b1000; //LUI -> imm + 0
    casex(alu_op)
        'b01: ctrl = 4'b0110;
        'b1x: begin
            case (funct3)
            3'b000: begin
                if (funct7[5])
                    ctrl = 4'b0110; //sub
                else ctrl = 4'b0010;//add
             end
            3'b001: ctrl = 4'b0011; //shift left
            3'b010: ctrl = 4'b1100; // slt
            3'b011: ctrl = 4'b1110; // slt unsigned
            3'b100: ctrl = 4'b0111; // XOR
            3'b101: ctrl = funct7[5]? 4'b0101 : 4'b0100; //shift righta: shift right
            3'b110: ctrl = 4'b0001; // OR
            3'b111: ctrl = 4'b0000; // AND
            default: ctrl = 4'b0010;                    
        endcase
        end
        'b00: ctrl = 4'b0010;
        default: ctrl = 4'b0010;
    endcase          
endfunction

//function [1:0] forward;
//        input [4:0]ex_rs;
//        input [4:0]mem_rd;
//        input [4:0]wb_rd;
//        input mem_reg_write;
//        input wb_reg_write;
//        logic [1:0] check;
//        check = {(mem_reg_write & (|mem_rd) & (~|(ex_rs ^ mem_rd))), (wb_reg_write & (|wb_rd) & (~|(ex_rs ^ wb_rd)))};
//        $monitor("[$monitor] check: %2b, [|wb_rd:%1b], [|mem_rd:%1b], [~|(ex_rs ^ mem_rd):%1b], [~|(ex_rs ^ wb_rd):%1b]", check[1:0], |wb.rd, |mem.rd, ~|(ex.rs ^ mem_rd), ~|(ex_rs ^ wb_rd) );
//        casex(check)
//             2'b00: forward = 2'b00;
//             2'b1x: forward = 2'b10;
//             2'b01: forward = 2'b01;
//            default: forward = 2'b00;
//        endcase
//    endfunction

function [63:0] alu_select;
//     assign alu_fwd_in1 = forward_a[1]? (mem.mem_to_reg? dmem_dout: mem.alu_result): (wb.mem_to_reg? wb.dmem_dout: wb.alu_result);
    input [1:0] forward;
    input   mem_mem2reg, wb_mem2reg;
    input [63:0] mem_alu_result, wb_alu_result, mem_dmem_dout, wb_dmem_dout, ex_dout;
    casex({forward, mem_mem2reg, wb_mem2reg})
        4'b00xx: alu_select = ex_dout;
        4'b01x0: alu_select = wb_alu_result;
        4'b01x1: alu_select = wb_dmem_dout;
        4'b100x: alu_select = mem_alu_result;
        4'b101x: alu_select = mem_dmem_dout;
        default: alu_select = ex_dout;
    endcase            
endfunction

function branch_chk;
    input [2:0] funct3;
    input [63:0] in1, in2;
    casex(funct3)
        'b000: branch_chk = (in1 == in2)? 'b1: 'b0;
        'b001: branch_chk = (in1 != in2)? 'b1: 'b0;
        'b100: branch_chk = (in1 < in2)? 'b1: 'b0;
        'b101: branch_chk = (in1 >= in2)? 'b1: 'b0;
        'b110: branch_chk = ($unsigned(in1) < $unsigned(in2))? 'b1: 'b0;
        'b111: branch_chk = ($unsigned(in1) >= $unsigned(in2))? 'b1: 'b0;
        default: branch_chk = 'b0;
    endcase
endfunction
    
module pipeline_cpu
#(  parameter IMEM_DEPTH = 1024,    // imem depth (default: 1024 entries = 4 KB)
              IMEM_ADDR_WIDTH = 10,
              REG_WIDTH = 64,
              DMEM_DEPTH = 1024,    // dmem depth (default: 1024 entries = 8 KB)
              DMEM_ADDR_WIDTH = 10 )
(
    input           clk,            // System clock
    input           reset_b         // Asychronous negative reset
);

    // -------------------------------------------------------------------
    /* Instruction fetch stage:
     * - Accessing the instruction memory with PC
     * - Control PC udpates for pipeline stalls
     */

    // Program counter
    logic           pc_write;   // enable PC updates
    logic   [63:0]  pc_curr, pc_next;
    logic   [63:0]  pc_next_plus4, pc_next_branch, pc_next_jump;
    logic           branch, jump;
    logic                   branch_hit;
//    logic           zero;   // zero detection from regfile
    logic           alu_zero;   // ALU
    assign pc_next_plus4 = pc_curr + 3'd4;
    

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            pc_curr <= 'b0;
        end else begin
            if(pc_write) pc_curr <= pc_next;// FILL THIS
        end
    end

    // imem
    logic   [IMEM_ADDR_WIDTH-1:0]   imem_addr;
    logic   [31:0]  inst;   // instructions = an output of ????
    
    assign imem_addr = pc_curr[11:2];// FILL THIS

    // instantiation: instruction memory
    imem #(
        .IMEM_DEPTH         (IMEM_DEPTH),
        .IMEM_ADDR_WIDTH    (IMEM_ADDR_WIDTH)
    ) u_imem_0 (
        .addr               ( imem_addr     ),
        .dout               ( inst          )
    );
    // -------------------------------------------------------------------

    // -------------------------------------------------------------------
    /* IF/ID pipeline register
     * - Supporting pipeline stalls and flush
     */
    pipe_if_id      id;         // THINK WHY THIS IS ID...
    logic           if_flush, if_stall;

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            id <= 'b0;
        end else begin
            if ( if_flush ) begin // flush
                id <= #(`FF) 'b0;
            end else if ( if_stall ) begin // stall
                id.pc <= #(`FF) id.pc; 
                id.inst <= #(`FF) id.inst;
            end else begin // normal operation
                id.pc <= #(`FF) pc_curr;
                id.inst <= #(`FF) inst;
            end            
        end
        $display("[%0t-pc]-------------------------------------------------------------------------------", $time);
         $display("[%0t-pc] IF.pc=[0x%4h], ID.pc=[0x%4h], EX.pc= [0x%4h], ME.pc=[0x%4h], WB.pc=[0x%4h]", $time, pc_curr[15:0], id.pc[15:0], ex.pc[15:0], mem.pc[15:0], wb.pc[15:0]);
        $display("[%0t-id] if.inst=[%32b], id=%p", $time, inst[31:0],  id);
    end
    // -------------------------------------------------------------------

    // ------------------------------------------------------------------
    /* Instruction decoder stage:
     * - Generating control signals
     * - Register file
     * - Immediate generator
     * - Hazard detection unit
     */
    
    // -------------------------------------------------------------------
    /* Main control unit:
     * Main control unit generates control signals for datapath elements
     * The control signals are determined by decoding instructions
     * Generating control signals using opcode = inst[6:0]
     */
    logic   [6:0]   opcode;
    logic   [6:0]   signal;
    //logic           branch;
    logic           alu_src, mem_to_reg;
    logic   [1:0]   alu_op, pctrl;
    logic           mem_read, mem_write, reg_write; // declared above
    
    // COMPLETE THE MAIN CONTROL UNIT HERE
    assign {jump, branch, mem_read, mem_to_reg, mem_write, alu_src, reg_write} = signal;
    assign opcode = id.inst[6:0];
    
    
    // --------------------------------------------------------------------

    // ---------------------------------------------------------------------
    /* Immediate generator:
     * Generating immediate value from inst[31:0]
     */
    logic   [63:0]  _imm64, imm64;
    logic   [63:0]  imm64_branch;  // imm64 left shifted by 1

    // COMPLETE IMMEDIATE GENERATOR HERE
    logic   [11:0]  imm12;

    assign imm12 = imm12_gen(opcode, alu_op, id.inst);// FILL THIS
    assign _imm64 = imm64_gen(opcode, id.inst, imm12);// FILL THIS
    assign imm64_branch = _imm64 <<1; // FILL THIS
    assign imm64 = (branch | (jump & opcode[3]))? imm64_branch: _imm64;
    

    // ----------------------------------------------------------------------

    // ----------------------------------------------------------------------
    /* Hazard detection unit
     * - Detecting data hazards from load instrcutions
     * - Detecting control hazards from taken branches
     */
    logic   [4:0]   rs1, rs2, rd;
    pipe_id_ex      ex;         // THINK WHY THIS IS EX...
    logic           stall_by_load_use;
//    logic   [1:0]   stall_by_regwr_branch;   // branch result is decided in ID stage, this is not explained in the textbook
    logic           flush_by_branch;
    
    logic           id_stall, stall_check;
    assign rs1 = id.inst[19:15];
    assign rs2 = id.inst[24:20];
    assign rd  = id.inst[11:7];
    always_comb begin
        if(|ex.rd) stall_by_load_use = ~|(ex.rd ^ rs1)| ~|(ex.rd ^ rs2);
        else stall_by_load_use = 'b0;
    end
//    always_comb begin
//        if(stall_check) stall_by_load_use = 'b1;
//        else stall_by_load_use = 'b0;
//    end
//    assign stall_by_load_use = ex.mem_to_reg & ((ex.rd == rs1)| (ex.rd == rs2));// FILL THIS: STALL BY LOAD-USE
//    assign stall_by_regwr_branch[0] = // FILL THIS: STALL BY INST-BRANCH (CONDITION 1)
//    assign stall_by_regwr_branch[1] = // FILL THIS: STALL BY INST-BRANCH (CONDITION 2)

    

    assign id_stall = stall_by_load_use;
    assign if_stall = stall_by_load_use;// FILL THIS
    assign pc_write = ~stall_by_load_use;// FILL THIS
    always_comb begin
        case(flush_by_branch)
            1'b1: begin
                {signal, alu_op, pctrl[0]} = 'b0;
                if_flush = 'b1;
            end default: begin
                {signal, alu_op, pctrl[0]} = decoder(opcode);
                if_flush = 'b0;
            end
        endcase
    end    
    assign pctrl[1] = pctrl[0] & opcode[5]; // LUI= 11, AUIPC= 01
//    assign {signal, alu_op, pctrl[0]} = flush_by_branch? 'b0: decoder(opcode);
    // ----------------------------------------------------------------------

    
    // regfile/
    pipe_mem_wb     wb;

    logic   [REG_WIDTH-1:0] rd_din;
    logic   [REG_WIDTH-1:0] rs1_dout, rs2_dout;
    
//    assign rd_din = wb.mem_to_reg? wb.dmem_dout : wb.alu_result;
    // rd, rd_din, and reg_write will be determined in WB stage
    
    // instnatiation of register file
    regfile #(
        .REG_WIDTH          (REG_WIDTH)
    ) u_regfile_0 (
        .clk                (clk),
        .rs1                (rs1),
        .rs2                (rs2),
        .rd                 (wb.rd),
        .rd_din             (rd_din),
        .reg_write          (wb.reg_write),
        .rs1_dout           (rs1_dout),
        .rs2_dout           (rs2_dout)
    );

    // assign regfile_zero = // don't use at this time

    // ------------------------------------------------------------------

    // -------------------------------------------------------------------
    /* ID/EX pipeline register
     * - Supporting pipeline stalls
     */
   
    logic   [6:0]   funct7;
    logic   [2:0]   funct3;
    logic           id_flush;
    // THE FOLLOWING SIGNALS WILL BE USED FOR ALU CONTROL
    assign id_flush = flush_by_branch;
    assign funct7 = id.inst[31:25];// FILL THIS
    assign funct3 = id.inst[14:12];// FILL THIS

    // COMPLETE ID/EX PIPELINE REGISTER
    always_ff @ (posedge clk or negedge reset_b) begin
        $display("[%0t-decode] stall_by_load_use=[%1b], rd_din=[0x%16h], func3=[%3b], id.rs1: %h, id.rs2: %h, id.rd: %h, id.opcode:%7b", $time, stall_by_load_use, rd_din, funct3, rs1, rs2, rd, opcode[6:0]);
        $display("[%0t-ex] ex.pc=[%4h], ex.rs1_dout=[%4h], ex.rs2_dout=[%4h], ex.imm=[%4h], ex.signal=[%7b], ex.rs1:%2d, ex.rs2:%2d, ex.rd:%2d", 
                $time, ex.pc, ex.rs1_dout, ex.rs2_dout, ex.imm64, {ex.jump, ex.branch, ex.mem_read, ex.mem_to_reg, ex.mem_write, ex.alu_src, ex.reg_write}, ex.rs1, ex.rs2, ex.rd);
        if (~reset_b) begin
            ex <= 'b0;
        end else if (id_stall | id_flush) begin
            ex <= 'b0;
            $display("[$monitor] ex flush by condition: [%2b]", {id_stall, id_flush});
        end else begin
            // FILL THIS
            ex.pc       <= #(`FF) id.pc;
            ex.rs1_dout <= #(`FF) rs1_dout;
            ex.rs2_dout <= #(`FF) rs2_dout;
            ex.imm64[63:0] <= #(`FF) imm64[63:0];
            ex.funct3[2:0] <= #(`FF) funct3[2:0];
            ex.funct7[6:0] <= #(`FF) funct7[6:0];
            ex.jump <= #(`FF) jump;
            ex.branch <= #(`FF) branch;
            ex.mem_read <= #(`FF) mem_read;
            ex.mem_to_reg <= #(`FF) mem_to_reg;
            ex.mem_write <= #(`FF) mem_write;
            ex.alu_src <= #(`FF) alu_src;
            ex.reg_write <= #(`FF) reg_write;
            ex.alu_op[1:0] <= #(`FF) alu_op[1:0];
            ex.rs1[4:0] <= #(`FF) rs1[4:0];
            ex.rs2[4:0] <= #(`FF) rs2[4:0];
            ex.rd[4:0] <= #(`FF)  rd[4:0]; 
            ex.pctrl[1:0] <= #(`FF)  pctrl[1:0];  
        end
    end

    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    /* Excution stage:
     * - ALU & ALU control
     * - Data forwarding unit
     */

    // --------------------------------------------------------------------
    /* ALU control unit:
     * ALU control unit generate alu_control signal which selects ALU operations
     * Generating control signals using alu_op, funct7, and funct3 fileds
     */

    logic   [3:0]   alu_control;    // ALU control signal

    // COMPLETE ALU CONTROL UNIT
    assign alu_control = ctrl(ex.alu_op, ex.funct3, ex.funct7, ex.pctrl);
    // ---------------------------------------------------------------------

    // ----------------------------------------------------------------------
    /* Forwarding unit:
     * - Forwarding from EX/MEM and MEM/WB
     */
    logic   [1:0]   forward_a, forward_b;
    logic   [63:0]  alu_fwd_in1, alu_fwd_in2;   // outputs of forward MUXes

    // COMPLETE FORWARDING MUXES
    /* 
    function [1:0] forward;
    input ex_rs;
    input mem_rd;
    input wb_rd;
    input mem_reg_write;
    input wb_reg_write;
    */
    logic   [63:0]  dmem_din, dmem_dout; // mem stage
    /*
    input [1:0] forward;
    input   mem_mem2reg, wb_mem2reg;
    input [63:0] mem_alu_result, wb_alu_result, mem_dmem_dout, wb_dmem_dout, ex_dout;
    forward_a[1]? (mem.mem_to_reg? dmem_dout: mem.alu_result): (wb.mem_to_reg? wb.dmem_dout: wb.alu_result);
    */
    
    
    assign alu_fwd_in1 = alu_select(forward_a, mem.mem_to_reg, wb.mem_to_reg, mem.alu_result, 
                                    wb.alu_result, dmem_dout, wb.dmem_dout, ex.rs1_dout);
    assign alu_fwd_in2 = alu_select(forward_b, mem.mem_to_reg, wb.mem_to_reg, mem.alu_result, 
                                    wb.alu_result, dmem_dout, wb.dmem_dout, ex.rs2_dout);
    // COMPLETE THE FORWARDING UNIT
    // Need to prioritize forwarding conditions
    /*
    function [1:0] forward;
        input [4:0]ex_rs;
        input [4:0]mem_rd;
        input [4:0]wb_rd;
        input mem_reg_write;
        input wb_reg_write;
        logic [1:0] check;
        check = {(mem_reg_write & (|mem_rd) & (~|(ex_rs ^ mem_rd))), (wb_reg_write & (|wb_rd) & (~|(ex_rs ^ wb_rd)))};
        $monitor("[$monitor] check: %2b, [|wb_rd:%1b], [|mem_rd:%1b], [~|(ex_rs ^ mem_rd):%1b], [~|(ex_rs ^ wb_rd):%1b]", check[1:0], |wb_rd, |mem_rd, ~|(ex_rs ^ mem_rd), ~|(ex_rs ^ wb_rd) );
        casex(check)
             2'b00: forward = 2'b00;
             2'b1x: forward = 2'b10;
             2'b01: forward = 2'b01;
            default: forward = 2'b00;
        endcase
    endfunction
    */
    logic [1:0] check1, check2;
    assign check1 = {(mem.reg_write & (|mem.rd) & (~|(ex.rs1 ^ mem.rd))), (wb.reg_write & (|wb.rd) & (~|(ex.rs1 ^ wb.rd)))};
    assign check2 = {(mem.reg_write & (|mem.rd) & (~|(ex.rs2 ^ mem.rd))), (wb.reg_write & (|wb.rd) & (~|(ex.rs2 ^ wb.rd)))};
    always_comb begin
        casex(check1)
             2'b00: forward_a = 2'b00;
             2'b1x: forward_a = 2'b10;
             2'b01: forward_a = 2'b01;
            default: forward_a = 2'b00;
        endcase
        casex(check2)
             2'b00: forward_b = 2'b00;
             2'b1x: forward_b = 2'b10;
             2'b01: forward_b = 2'b01;
            default: forward_b = 2'b00;
        endcase
        
    end
    
//    assign forward_a = forward(ex.rs1[4:0], mem.rd[4:0], wb.rd[4:0], mem.reg_write, wb.reg_write);
//    assign forward_b = forward(ex.rs2[4:0], mem.rd[4:0], wb.rd[4:0], mem.reg_write, wb.reg_write);
    


    // -----------------------------------------------------------------------

    // ALU
    logic   [REG_WIDTH-1:0] alu_in1, alu_in2;
    logic   [REG_WIDTH-1:0] _alu_result, alu_result, jump_reg;
    
    // Computing branch target
    assign pc_next_branch = ex.imm64 + ex.pc;// FILL THIS
    assign pc_next_jump = (alu_control[3]&jump)? pc_next_branch: _alu_result; // JAL, JALR
//    assign alu_in1 = alu_fwd_in1;// FILL THIS
//    assign alu_in2 = alu_fwd_in2;// FILL THIS
    assign jump_reg = ex.pc + 3'b100;
//    assign alu_result = ~ex.jump? (^ex.pctrl? pc_next_branch: _alu_result): jump_reg;
    assign branch_hit = ex.branch & branch_chk(ex.funct3, alu_in1, alu_in2);
    assign flush_by_branch = ex.jump | branch_hit;// FILL THIS: FLUSH CONDITION
    assign pc_next = pc_decision(branch_hit, ex.jump, pc_next_branch, pc_next_jump, pc_next_plus4); // FILL THIS
    // instantiation: ALU
    always_comb  begin
    //alu_in2    
        if(ex.alu_src) begin
            alu_in2 = ex.imm64;
        end
        else alu_in2 = alu_fwd_in2;
    //alu_in1
        if(pctrl == 2'b10) alu_in1 = ex.pc;
        else            alu_in1 = alu_fwd_in1;    

    end
    
    always_comb begin
        if(ex.jump) alu_result = jump_reg;
        else        alu_result = _alu_result;        
    end
    alu #(
        .REG_WIDTH          (REG_WIDTH)
    ) u_alu_0 (
        .in1                (alu_in1),
        .in2                (alu_in2),
        .alu_control        (alu_control),
        .result             (_alu_result),
        .zero               (alu_zero)
    );

    // -------------------------------------------------------------------------
    /* Ex/MEM pipeline register
     */
    pipe_ex_mem     mem;

    always_ff @ (posedge clk or negedge reset_b) begin
        $display("[$forward] check1:%2b, ~|(ex.rs1 ^ mem.rd):%1b, wb.reg_write & (|wb.rd) & (~|(ex.rs1 ^ wb.rd)): %1b", check1, ~|(ex.rs1 ^ mem.rd), wb.reg_write & (|wb.rd) & (~|(ex.rs1 ^ wb.rd)));
        $display("[$forward] check2:%2b, ~|(ex.rs2 ^ mem.rd):%1b, wb.reg_write & (|wb.rd) & (~|(ex.rs2 ^ wb.rd)): %1b", check2, ~|(ex.rs2 ^ mem.rd), wb.reg_write & (|wb.rd) & (~|(ex.rs2 ^ wb.rd)));
        $display("[%0t-alu] result:[%8h], branch_hit:[%1b], pc_next:[%4h], alu_control:[%4b], forward_a:[%2b], forward_b:[%2b], alu_in1:%5d, alu_in2:%5d", 
                    $time, alu_result[31:0], branch_hit, pc_next[15:0], alu_control, forward_a, forward_b, alu_in1, alu_in2);
        $display("[%0t-mem] mem.pc=[%4h], mem.alu_result:[%4h], mem.mem_dout:[%4h], mem.rd:[%5b], |mem.rd:%1b, mem.signal:%4b", $time, mem.pc, mem.alu_result, dmem_dout, mem.rd, |mem.rd, {mem.mem_read, mem.mem_to_reg, mem.mem_write, mem.reg_write});
        if (~reset_b) begin
            mem <= 'b0;
        end else begin
           // FILL THIS
            mem.pc          <=#(`FF) ex.pc;
            mem.alu_result <= #(`FF) alu_result;
            mem.rs2_dout <= #(`FF) ex.rs2_dout;
            mem.mem_read <= #(`FF) ex.mem_read;
            mem.mem_write <= #(`FF) ex.mem_write;
            mem.rd <= #(`FF) ex.rd;
            mem.reg_write <= #(`FF) ex.reg_write;
            mem.mem_to_reg <= #(`FF) ex.mem_to_reg;
            mem.size    <= #(`FF) ex.funct3;
            
        end
    end


    // --------------------------------------------------------------------------
    /* Memory srage
     * - Data memory accesses
     */

    // dmem
    logic   [DMEM_ADDR_WIDTH-1:0]    dmem_addr;
    

    assign dmem_addr = mem.alu_result[DMEM_ADDR_WIDTH-1:0];// memory address
    assign dmem_din = mem.rs2_dout;// store data
    
    // instantiation: data memory
    dmem #(
        .DMEM_DEPTH         (DMEM_DEPTH),
        .DMEM_ADDR_WIDTH    (DMEM_ADDR_WIDTH)
    ) u_dmem_0 (
        .clk                (clk),
        .addr               (dmem_addr),
        .din                (dmem_din),
        .rd_en           (mem.mem_read),
        .wr_en          (mem.mem_write ),
        .dout               (dmem_dout),
        .funct3                 (mem.size)
    );

    // -----------------------------------------------------------------------
    /* MEM/WB pipeline register
     */


    always_ff @ (posedge clk or negedge reset_b) begin
        $display("[%0t-wb] wb.pc=[%4h], wb.alu_result:[%4d], wb.dmem_dout:[%4d], wb.rd:[%2d], |wb.rd:[%1b], rd_din:[%4d], {wb.reg_write, wb.mem2reg}:%2b", 
                $time, wb.pc, wb.alu_result, wb.dmem_dout, wb.rd, |wb.rd, rd_din, {wb.reg_write, wb.mem_to_reg});
        if (~reset_b) begin
            wb <= 'b0;
        end else begin
            // FILL THIS
            wb.pc       <=#(`FF) mem.pc;
            wb.alu_result  <= #(`FF) mem.alu_result;
            wb.dmem_dout  <= #(`FF) dmem_dout;
            wb.rd        <= #(`FF) mem.rd;
            wb.reg_write  <= #(`FF) mem.reg_write;
            wb.mem_to_reg  <= #(`FF) mem.mem_to_reg;
        end
    end

    // ----------------------------------------------------------------------
    /* Writeback stage
     * - Write results to regsiter file
     */
    always_comb begin
        if (wb.reg_write) begin
            if(wb.mem_to_reg) rd_din = wb.dmem_dout;
            else rd_din = wb.alu_result;
        end else rd_din = 'b0;
    end
endmodule