// =============================================================================
// File Name   : newton_tb_top.sv
// Module Name : newton_tb_top (Top-Level Testbench Module)
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

import uvm_pkg::*;
`include "uvm_macros.svh"
import newton_types_pkg::*;
import newton_tb_pkg::*;

module newton_tb_top;

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

    // Interface & DUT Instantiation
    newton_if vif (
        .clk   (clk),
        .rst_n (rst_n)
    );

    newton_2nd_order_top u_dut (
        .clk        (vif.clk),
        .rst_n      (vif.rst_n),
        .prog_en    (vif.prog_en),
        .prog_addr  (vif.prog_addr),
        .prog_data  (vif.prog_data),
        .start      (vif.start),
        .x_init     (vif.x_init),
        .tolerance  (vif.tolerance),
        .step_alpha (vif.step_alpha),
        .lambda_reg (vif.lambda_reg),
        .max_iters  (vif.max_iters),
        .x_optimal  (vif.x_optimal),
        .f_optimal  (vif.f_optimal),
        .g_final    (vif.g_final),
        .iter_count (vif.iter_count),
        .status     (vif.status),
        .done       (vif.done),
        .busy       (vif.busy)
    );

    // UVM Setup & Waveform Dump
    initial begin
        string dump_file;
        uvm_config_db#(virtual newton_if)::set(null, "*", "vif", vif);

        if (!$value$plusargs("DUMPFILE=%s", dump_file)) begin
            dump_file = "outputs/sim_waveform.vcd";
        end
        $dumpfile(dump_file);
        $dumpvars(0, newton_tb_top);

        run_test();
    end

endmodule : newton_tb_top
