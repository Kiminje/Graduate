/* ***********************************************
 *  COSE222 Lab #3
 *
 *  Module: testbench for single_cycle_cpu.sv
 *  -
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 **************************************************
 */

`timescale 1ns/100ps
`define CLK_T   20        // FILL THIS

module tb_single_cycle_cpu 
    #(  parameter IMEM_DEPTH = 1024,    // imem depth (default: 1024 entries = 4 KB)
              IMEM_ADDR_WIDTH = 10,
              REG_WIDTH = 64,
              DMEM_DEPTH = 1024,    // dmem depth (default: 1024 entries = 8 KB)
              DMEM_ADDR_WIDTH = 10 )
    ();
    
    logic           clk, reset_b;

    initial clk = 1'b1;
    always #(`CLK_T/2) clk = ~clk;

    initial begin
        clk = 1'b1;
        reset_b = 1'b0;
        repeat (2) @ (posedge clk);
        #(1) reset_b = 1'b1;
    end

    int fout;

    initial begin
        fout = $fopen("./report.txt", "w");
        if (fout)
            $display("File was opened successfully: %d", fout);
        else
            $display("Failed to open the file: %d", fout);

        repeat (46) @ (posedge clk);
            
        #(`CLK_T/2)
        $display("simulation finish");
        for (int i = 0; i < 32; i++) begin
            $fwrite(fout, "RF[%02d]: %016X\n", i, dut.u_regfile_0.rf_data[i]);
        end
        for (int i = 0; i < 20; i++) begin
            $fwrite(fout, "DMEM[%02d]: %016X\n", i, dut.u_dmem_0.data[i]);
        end
        
        $fclose(fout);
        $stop(1);
    end

    // INSTANTIATE DUT
    single_cycle_cpu #(
        .IMEM_DEPTH     (IMEM_DEPTH),
        .IMEM_ADDR_WIDTH(IMEM_ADDR_WIDTH),
        .REG_WIDTH      (REG_WIDTH),
        .DMEM_DEPTH     (DMEM_DEPTH),
        .DMEM_ADDR_WIDTH(DMEM_ADDR_WIDTH)
    ) dut   (
        .clk            (clk),
        .reset_b        (reset_b));

endmodule