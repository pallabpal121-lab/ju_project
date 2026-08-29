// =============================================================================
// File Name   : tb_levenberg_marquardt.sv
// Module Name : tb_levenberg_marquardt
// Project     : Levenberg-Marquardt (LM) Non-Linear Least Squares Accelerator (Solver #2)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for Levenberg-Marquardt hardware accelerator.
//   Verifies convergence of non-linear parameter estimation & robotics SLAM problems.
// =============================================================================

`timescale 1ns / 1ps

import lm_types_pkg::*;
`include "lm_helpers.svh"

module tb_levenberg_marquardt;

    logic        clk;
    logic        rst_n;

    // Programming Interface
    logic        prog_en;
    logic [4:0]  prog_addr;
    instr_t      prog_data;

    // Observation Dataset Interface
    logic        obs_we;
    logic [2:0]  obs_addr;
    q16_t        obs_t_in;
    q16_t        obs_y_in;

    // Controls & Inputs
    logic        start;
    logic [2:0]  num_params;
    logic [3:0]  num_obs;
    vec_t        x_init;
    q16_t        tolerance;
    q16_t        lambda_init;
    logic [7:0]  max_iters;

    // Results & Outputs
    vec_t        x_optimal;
    q16_t        cost_optimal;
    q16_t        g_norm_inf;
    q16_t        lambda_final;
    logic [7:0]  iter_count;
    status_t     status;
    logic        done;
    logic        busy;

    // Instantiate Top Module
    levenberg_marquardt_top dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .prog_en      (prog_en),
        .prog_addr    (prog_addr),
        .prog_data    (prog_data),
        .obs_we       (obs_we),
        .obs_addr     (obs_addr),
        .obs_t_in     (obs_t_in),
        .obs_y_in     (obs_y_in),
        .start        (start),
        .num_params   (num_params),
        .num_obs      (num_obs),
        .x_init       (x_init),
        .tolerance    (tolerance),
        .lambda_init  (lambda_init),
        .max_iters    (max_iters),
        .x_optimal    (x_optimal),
        .cost_optimal (cost_optimal),
        .g_norm_inf   (g_norm_inf),
        .lambda_final (lambda_final),
        .iter_count   (iter_count),
        .status       (status),
        .done         (done),
        .busy         (busy)
    );

    // 100MHz Clock
    initial clk = 0;
    always #5 clk = ~clk;

    // Real Number Conversion Helpers
    function real q16_to_real(input q16_t val);
        q16_to_real = real'(val) / 65536.0;
    endfunction

    function q16_t real_to_q16(input real val);
        real_to_q16 = q16_t'(int'(val * 65536.0));
    endfunction

    // Task to program microcode
    task write_instr(input logic [4:0] addr, input opcode_t op, input logic [3:0] dst, input logic [3:0] src_a, input logic [3:0] src_b, input logic signed [15:0] imm);
        begin
            @(posedge clk);
            prog_en         <= 1'b1;
            prog_addr       <= addr;
            prog_data.op    <= op;
            prog_data.dst   <= dst;
            prog_data.src_a <= src_a;
            prog_data.src_b <= src_b;
            prog_data.imm   <= imm;
            @(posedge clk);
            prog_en         <= 1'b0;
        end
    endtask

    // Task to program observation dataset
    task write_obs(input logic [2:0] addr, input real t_val, input real y_val);
        begin
            @(posedge clk);
            obs_we   <= 1'b1;
            obs_addr <= addr;
            obs_t_in <= real_to_q16(t_val);
            obs_y_in <= real_to_q16(y_val);
            @(posedge clk);
            obs_we   <= 1'b0;
        end
    endtask

    initial begin
        $display("==================================================================");
        $display(" Levenberg-Marquardt (LM) Non-Linear Least Squares Accelerator TB");
        $display("==================================================================");

        // Reset
        rst_n       = 0;
        prog_en     = 0;
        prog_addr   = '0;
        prog_data   = '0;
        obs_we      = 0;
        obs_addr    = '0;
        obs_t_in    = '0;
        obs_y_in    = '0;
        start       = 0;
        num_params  = 3'd3;
        num_obs     = 4'd4;
        tolerance   = 0;
        lambda_init = 0;
        max_iters   = 0;
        x_init      = '0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: Parabola Non-Linear Parameter Estimation: y(t) = a*t^2 + b*t + c
        // True Parameters: a* = 1.0, b* = -2.0, c* = 3.0
        // Data: (-1, 6), (0, 3), (1, 2), (2, 3)
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Non-Linear Parameter Estimation: y(t) = a*t^2 + b*t + c");
        $display("Target Parameters: a* = 1.000, b* = -2.000, c* = 3.000");

        // Microcode for f(t, a, b, c):
        // r0 = a, r1 = b, r2 = c, r14 = t
        write_instr(5'd0, OP_MUL,   4'd3, 4'd14, 4'd14, 16'sd0); // r3  = t^2
        write_instr(5'd1, OP_MUL,   4'd4, 4'd0,  4'd3,  16'sd0); // r4  = a*t^2
        write_instr(5'd2, OP_MUL,   4'd5, 4'd1,  4'd14, 16'sd0); // r5  = b*t
        write_instr(5'd3, OP_ADD,   4'd6, 4'd4,  4'd5,  16'sd0); // r6  = a*t^2 + b*t
        write_instr(5'd4, OP_ADD,   4'd15,4'd6,  4'd2,  16'sd0); // r15 = a*t^2 + b*t + c
        write_instr(5'd5, OP_END,   4'd0, 4'd0,  4'd0,  16'sd0); // END

        // Load 4 observation data points
        write_obs(3'd0, -1.0, 6.0);
        write_obs(3'd1,  0.0, 3.0);
        write_obs(3'd2,  1.0, 2.0);
        write_obs(3'd3,  2.0, 3.0);

        @(posedge clk);
        num_params  = 3'd3;
        num_obs     = 4'd4;
        // Start from initial guess: (a0, b0, c0) = (5.0, 5.0, 0.0)
        x_init      = set_vec(set_vec(set_vec(x_init, 2'd0, real_to_q16(5.0)), 2'd1, real_to_q16(5.0)), 2'd2, real_to_q16(0.0));
        tolerance   = 32'h0000_0080; // tol ≈ 0.0019
        lambda_init = 32'h0000_1000; // lambda_0 = 0.0625
        max_iters   = 8'd50;
        start       = 1'b1;
        @(posedge clk);
        start       = 1'b0;

        $display("Starting Levenberg-Marquardt Solver from guess (a, b, c) = (5.0, 5.0, 0.0)...");
        @(posedge done);
        #1;

        $display("--> LM SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    a_optimal:    %f (Expected: ~ 1.000000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    b_optimal:    %f (Expected: ~ -2.000000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    c_optimal:    %f (Expected: ~ 3.000000)", q16_to_real(get_vec(x_optimal, 2'd2)));
        $display("    cost_optimal: %f (Expected: ~ 0.000000)", q16_to_real(cost_optimal));
        $display("    g_norm_inf:   %f", q16_to_real(g_norm_inf));
        $display("    lambda_final: %f", q16_to_real(lambda_final));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 0.95 && q16_to_real(get_vec(x_optimal, 2'd0)) < 1.05 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > -2.05 && q16_to_real(get_vec(x_optimal, 2'd1)) < -1.95 &&
            q16_to_real(get_vec(x_optimal, 2'd2)) > 2.95 && q16_to_real(get_vec(x_optimal, 2'd2)) < 3.05) begin
            $display("[TEST 1 PASSED] Successfully converged parameter estimation in hardware!");
        end else begin
            $display("[TEST 1 FAILED] Did not converge to target parameters.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 2D Robotics SLAM / Range Target Localization: y(t) = (t - xc)^2 + yc
        // Target Minimum: xc* = 2.0, yc* = 3.0
        // Data: (0, 7), (1, 4), (2, 3), (3, 4), (4, 7)
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 2D Robotics SLAM Range Localization: y(t) = (t - xc)^2 + yc");
        $display("Target Coordinates: xc* = 2.000, yc* = 3.000");

        // Microcode for f(t, xc, yc):
        // r0 = xc, r1 = yc, r14 = t
        write_instr(5'd0, OP_SUB,   4'd2, 4'd14, 4'd0,  16'sd0); // r2  = t - xc
        write_instr(5'd1, OP_MUL,   4'd3, 4'd2,  4'd2,  16'sd0); // r3  = (t - xc)^2
        write_instr(5'd2, OP_ADD,   4'd15,4'd3,  4'd1,  16'sd0); // r15 = (t - xc)^2 + yc
        write_instr(5'd3, OP_END,   4'd0, 4'd0,  4'd0,  16'sd0); // END

        // Load 5 observation data points
        write_obs(3'd0, 0.0, 7.0);
        write_obs(3'd1, 1.0, 4.0);
        write_obs(3'd2, 2.0, 3.0);
        write_obs(3'd3, 3.0, 4.0);
        write_obs(3'd4, 4.0, 7.0);

        @(posedge clk);
        num_params  = 3'd2;
        num_obs     = 4'd5;
        // Start from distant initial guess: (xc0, yc0) = (10.0, 10.0)
        x_init      = set_vec(set_vec(x_init, 2'd0, real_to_q16(10.0)), 2'd1, real_to_q16(10.0));
        tolerance   = 32'h0000_0080;
        lambda_init = 32'h0000_1000;
        max_iters   = 8'd50;
        start       = 1'b1;
        @(posedge clk);
        start       = 1'b0;

        $display("Starting Levenberg-Marquardt Solver from distant guess (xc, yc) = (10.0, 10.0)...");
        @(posedge done);
        #1;

        $display("--> LM SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    xc_optimal:   %f (Expected: ~ 2.000000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    yc_optimal:   %f (Expected: ~ 3.000000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    cost_optimal: %f (Expected: ~ 0.000000)", q16_to_real(cost_optimal));
        $display("    g_norm_inf:   %f", q16_to_real(g_norm_inf));
        $display("    lambda_final: %f", q16_to_real(lambda_final));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 1.95 && q16_to_real(get_vec(x_optimal, 2'd0)) < 2.05 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 2.95 && q16_to_real(get_vec(x_optimal, 2'd1)) < 3.05) begin
            $display("[TEST 2 PASSED] Successfully converged 2D SLAM localization in hardware!");
        end else begin
            $display("[TEST 2 FAILED] Did not converge to target coordinates.");
        end

        $display("\n==================================================================");
        $display(" ALL LEVENBERG-MARQUARDT HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
