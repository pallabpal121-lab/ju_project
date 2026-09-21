// =============================================================================
// File Name   : newton_axi_tb_top.sv
// Module Name : newton_axi_tb_top (Top-Level Testbench Module)
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

import uvm_pkg::*;
`include "uvm_macros.svh"
import newton_multivar_pkg::*;
import newton_axi_regs_pkg::*;
import newton_axi_agent_pkg::*;
import newton_axi_seq_pkg::*;
import newton_axi_env_pkg::*;
import newton_axi_test_pkg::*;
import newton_axi_tb_pkg::*;

module newton_axi_tb_top;

    // Clock & Reset Generation
    logic clk;
    logic rst_n;

    // 100MHz System Clock (10ns Period)
    initial begin
        clk = 1'b0;
        forever #5ns clk = ~clk;
    end

    // Active-Low Reset Sequence
    initial begin
        rst_n = 1'b0;
        #25ns;
        rst_n = 1'b1;
    end

    // Interface Instantiation
    newton_axi_if vif (
        .s_axi_aclk    (clk),
        .s_axi_aresetn (rst_n)
    );

    // DUT Instantiation
    newton_bcd_axi_lite u_dut (
        .s_axi_aclk    (vif.s_axi_aclk),
        .s_axi_aresetn (vif.s_axi_aresetn_eff),
        .s_axi_awaddr  (vif.s_axi_awaddr),
        .s_axi_awprot  (vif.s_axi_awprot),
        .s_axi_awvalid (vif.s_axi_awvalid),
        .s_axi_awready (vif.s_axi_awready),
        .s_axi_wdata   (vif.s_axi_wdata),
        .s_axi_wstrb   (vif.s_axi_wstrb),
        .s_axi_wvalid  (vif.s_axi_wvalid),
        .s_axi_wready  (vif.s_axi_wready),
        .s_axi_bresp   (vif.s_axi_bresp),
        .s_axi_bvalid  (vif.s_axi_bvalid),
        .s_axi_bready  (vif.s_axi_bready),
        .s_axi_araddr  (vif.s_axi_araddr),
        .s_axi_arprot  (vif.s_axi_arprot),
        .s_axi_arvalid (vif.s_axi_arvalid),
        .s_axi_arready (vif.s_axi_arready),
        .s_axi_rdata   (vif.s_axi_rdata),
        .s_axi_rresp   (vif.s_axi_rresp),
        .s_axi_rvalid  (vif.s_axi_rvalid),
        .s_axi_rready  (vif.s_axi_rready),
        .irq_done      (vif.irq_done)
    );

    // UVM Setup & Waveform Control
    initial begin
        string dump_file;
        uvm_config_db#(virtual newton_axi_if)::set(null, "*", "vif", vif);

        // Waveform dumping controlled via plusargs
        if ($test$plusargs("DUMP")) begin
            if ($test$plusargs("FSDB")) begin
                $fsdbDumpfile("outputs/novas.fsdb");
                $fsdbDumpvars(0, newton_axi_tb_top);
            end else begin
                if (!$value$plusargs("DUMPFILE=%s", dump_file)) begin
                    dump_file = "outputs/sim_waveform.vcd";
                end
                $dumpfile(dump_file);
                $dumpvars(0, newton_axi_tb_top);
            end
        end

        run_test();
    end

endmodule : newton_axi_tb_top
