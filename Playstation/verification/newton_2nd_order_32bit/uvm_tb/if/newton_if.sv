// =============================================================================
// File Name   : newton_if.sv
// Module Name : newton_if (SystemVerilog Interface)
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

interface newton_if (input logic clk, input logic rst_n);
    import newton_types_pkg::*;

    // Microcode Programming Interface Signals
    logic        prog_en;
    logic [4:0]  prog_addr;
    instr_t      prog_data;

    // Optimization Control & Parameter Signals
    logic        start;
    q16_t        x_init;
    q16_t        tolerance;
    q16_t        step_alpha;
    q16_t        lambda_reg;
    logic [7:0]  max_iters;

    // Output & Status Signals
    q16_t        x_optimal;
    q16_t        f_optimal;
    q16_t        g_final;
    logic [7:0]  iter_count;
    status_t     status;
    logic        done;
    logic        busy;

    // Clocking Block: Driver
    clocking drv_cb @(posedge clk);
        default input #1ns output #1ns;
        output prog_en;
        output prog_addr;
        output prog_data;
        output start;
        output x_init;
        output tolerance;
        output step_alpha;
        output lambda_reg;
        output max_iters;
        input  x_optimal;
        input  f_optimal;
        input  g_final;
        input  iter_count;
        input  status;
        input  done;
        input  busy;
    endclocking

    // Clocking Block: Monitor
    clocking mon_cb @(posedge clk);
        default input #1ns output #1ns;
        input  prog_en;
        input  prog_addr;
        input  prog_data;
        input  start;
        input  x_init;
        input  tolerance;
        input  step_alpha;
        input  lambda_reg;
        input  max_iters;
        input  x_optimal;
        input  f_optimal;
        input  g_final;
        input  iter_count;
        input  status;
        input  done;
        input  busy;
    endclocking

    modport drv_mp (clocking drv_cb, input clk, input rst_n);
    modport mon_mp (clocking mon_cb, input clk, input rst_n);
    modport dut_mp (
        input  clk, rst_n, prog_en, prog_addr, prog_data,
        input  start, x_init, tolerance, step_alpha, lambda_reg, max_iters,
        output x_optimal, f_optimal, g_final, iter_count, status, done, busy
    );

endinterface : newton_if
