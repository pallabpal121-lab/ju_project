// =============================================================================
// File Name   : tb_pgd.sv
// Module Name : tb_pgd
// Project     : Projected Gradient Descent (PGD) Accelerator (Solver #12)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for the PGD Accelerator.
//   Verifies Non-Negative Orthant, Euclidean L2 Ball, and Probability Simplex
//   constrained optimizations.
// =============================================================================

`timescale 1ns / 1ps

import pgd_types_pkg::*;
`include "pgd_helpers.svh"

module tb_pgd;

    logic         clk;
    logic         rst_n;

    // Programming Interface
    logic         prog_en;
    logic [4:0]   prog_addr;
    instr_t       prog_data;

    // Controls & Inputs
    logic         start;
    logic [2:0]   num_params;
    vec_t         x_init;
    proj_mode_t   proj_mode;
    vec_t         box_lower;
    vec_t         box_upper;
    q16_t         ball_radius;
    q16_t         step_size;
    q16_t         tolerance;
    logic [7:0]   max_iters;

    // Results & Outputs
    vec_t         x_optimal;
    q16_t         cost_optimal;
    q16_t         g_norm_inf;
    q16_t         delta_x_norm;
    logic [7:0]   iter_count;
    status_t      status;
    logic         done;
    logic         busy;

    // Instantiate Top Module
    pgd_top dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .prog_en     (prog_en),
        .prog_addr   (prog_addr),
        .prog_data   (prog_data),
        .start       (start),
        .num_params  (num_params),
        .x_init      (x_init),
        .proj_mode   (proj_mode),
        .box_lower   (box_lower),
        .box_upper   (box_upper),
        .ball_radius (ball_radius),
        .step_size   (step_size),
        .tolerance   (tolerance),
        .max_iters   (max_iters),
        .x_optimal   (x_optimal),
        .cost_optimal(cost_optimal),
        .g_norm_inf  (g_norm_inf),
        .delta_x_norm(delta_x_norm),
        .iter_count  (iter_count),
        .status      (status),
        .done        (done),
        .busy        (busy)
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

    initial begin
        $display("==================================================================");
        $display(" Projected Gradient Descent (PGD) Accelerator TB (Solver #12)");
        $display("==================================================================");

        // Reset
        rst_n       = 0;
        prog_en     = 0;
        prog_addr   = '0;
        prog_data   = '0;
        start       = 0;
        num_params  = 3'd2;
        x_init      = '0;
        proj_mode   = PROJ_NONE;
        box_lower   = '0;
        box_upper   = '0;
        ball_radius = 0;
        step_size   = 0;
        tolerance   = 0;
        max_iters   = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: Non-Negative Constrained Optimization (x0 >= 0, x1 >= 0)
        // Objective: f(x0, x1) = (x0 + 2)^2 + (x1 - 4)^2
        // Unconstrained min: (-2.0, 4.0), Constrained min on x >= 0: (0.0, 4.0)
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Non-Negative Constrained Optimization (x0 >= 0, x1 >= 0)");
        $display("Objective: f(x0, x1) = (x0 + 2)^2 + (x1 - 4)^2");
        $display("Unconstrained Minimum: (-2.000000, 4.000000)");
        $display("Target Constrained Minimum: x0* = 0.000000, x1* = 4.000000, f(x*) = 4.000000");

        // DFG Program for (x0 + 2)^2 + (x1 - 4)^2:
        write_instr(5'd0, OP_LOADI, 4'd4, 4'd0, 4'd0, 16'sd2); // R4 = 2.0
        write_instr(5'd1, OP_ADD,   4'd4, 4'd0, 4'd4, 16'sd0); // R4 = x0 + 2
        write_instr(5'd2, OP_MUL,   4'd4, 4'd4, 4'd4, 16'sd0); // R4 = (x0 + 2)^2
        write_instr(5'd3, OP_LOADI, 4'd5, 4'd0, 4'd0, 16'sd4); // R5 = 4.0
        write_instr(5'd4, OP_SUB,   4'd5, 4'd1, 4'd5, 16'sd0); // R5 = x1 - 4
        write_instr(5'd5, OP_MUL,   4'd5, 4'd5, 4'd5, 16'sd0); // R5 = (x1 - 4)^2
        write_instr(5'd6, OP_ADD,   4'd0, 4'd4, 4'd5, 16'sd0); // R0 = R4 + R5
        write_instr(5'd7, OP_NOP,   4'd0, 4'd0, 4'd0, 16'sd0); // End

        @(posedge clk);
        num_params = 3'd2;
        x_init     = set_vec(set_vec(x_init, 2'd0, real_to_q16(5.0)), 2'd1, real_to_q16(8.0));
        proj_mode  = PROJ_NON_NEGATIVE;
        step_size  = real_to_q16(0.125);
        tolerance  = 32'h0000_0020;
        max_iters  = 8'd50;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting PGD Solver from initial guess (5.0, 8.0)...");
        @(posedge done);
        #1;

        $display("--> PGD SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Iterations:     %0d", iter_count);
        $display("    x0_optimal:     %f (Expected:   0.000000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:     %f (Expected: ~ 4.000000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    f_optimal:      %f (Expected: ~ 4.000000)", q16_to_real(cost_optimal));
        $display("    delta_x_norm:   %f", q16_to_real(delta_x_norm));

        if (status == STATUS_CONVERGED &&
            get_vec(x_optimal, 2'd0) == Q16_ZERO &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 3.98 && q16_to_real(get_vec(x_optimal, 2'd1)) < 4.02) begin
            $display("[TEST 1 PASSED] Successfully converged non-negative constrained optimization!");
        end else begin
            $display("[TEST 1 FAILED] Did not reach expected constrained optimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: Euclidean L2 Ball Constrained Optimization (||x||_2 <= 2.0)
        // Objective: f(x0, x1) = (x0 - 3)^2 + (x1 - 4)^2
        // Unconstrained min: (3.0, 4.0) with ||(3,4)||_2 = 5.0
        // Constrained min on ||x||_2 <= 2.0: x* = (2/5)*(3, 4) = (1.200000, 1.600000)
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Euclidean L2 Ball Constrained Optimization (||x||_2 <= 2.0)");
        $display("Objective: f(x0, x1) = (x0 - 3)^2 + (x1 - 4)^2");
        $display("Unconstrained Minimum: (3.000000, 4.000000) (Radius = 5.0)");
        $display("Target Constrained Minimum: x0* = 1.200000, x1* = 1.600000, ||x*||_2 = 2.000000");

        // DFG Program for (x0 - 3)^2 + (x1 - 4)^2:
        write_instr(5'd0, OP_LOADI, 4'd4, 4'd0, 4'd0, 16'sd3); // R4 = 3.0
        write_instr(5'd1, OP_SUB,   4'd4, 4'd0, 4'd4, 16'sd0); // R4 = x0 - 3
        write_instr(5'd2, OP_MUL,   4'd4, 4'd4, 4'd4, 16'sd0); // R4 = (x0 - 3)^2
        write_instr(5'd3, OP_LOADI, 4'd5, 4'd0, 4'd0, 16'sd4); // R5 = 4.0
        write_instr(5'd4, OP_SUB,   4'd5, 4'd1, 4'd5, 16'sd0); // R5 = x1 - 4
        write_instr(5'd5, OP_MUL,   4'd5, 4'd5, 4'd5, 16'sd0); // R5 = (x1 - 4)^2
        write_instr(5'd6, OP_ADD,   4'd0, 4'd4, 4'd5, 16'sd0); // R0 = R4 + R5
        write_instr(5'd7, OP_NOP,   4'd0, 4'd0, 4'd0, 16'sd0); // End

        @(posedge clk);
        num_params  = 3'd2;
        x_init      = set_vec(set_vec(x_init, 2'd0, real_to_q16(0.0)), 2'd1, real_to_q16(0.0));
        proj_mode   = PROJ_L2_BALL;
        ball_radius = real_to_q16(2.0);
        step_size   = real_to_q16(0.125);
        tolerance   = 32'h0000_0020;
        max_iters   = 8'd50;
        start       = 1'b1;
        @(posedge clk);
        start       = 1'b0;

        $display("Starting PGD Solver on L2 Ball...");
        @(posedge done);
        #1;

        $display("--> PGD SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Iterations:     %0d", iter_count);
        $display("    x0_optimal:     %f (Expected: ~ 1.200000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:     %f (Expected: ~ 1.600000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    Norm ||x*||_2:  %f (Expected: ~ 2.000000)", $sqrt(q16_to_real(get_vec(x_optimal, 2'd0))**2 + q16_to_real(get_vec(x_optimal, 2'd1))**2));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 1.18 && q16_to_real(get_vec(x_optimal, 2'd0)) < 1.22 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 1.58 && q16_to_real(get_vec(x_optimal, 2'd1)) < 1.62) begin
            $display("[TEST 2 PASSED] Successfully converged Euclidean L2 Ball constrained optimization!");
        end else begin
            $display("[TEST 2 FAILED] Did not reach expected L2 boundary optimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: Probability Simplex Constrained Optimization (sum x_i = 1.0, x_i >= 0)
        // Objective: f(x0, x1, x2) = (x0 - 1)^2 + (x1 - 2)^2 + (x2 + 1)^2
        // Target Constrained Optimum: x* = (0.000000, 1.000000, 0.000000)
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Probability Simplex Constrained Optimization (sum x_i = 1.0, x_i >= 0)");
        $display("Objective: f(x0, x1, x2) = (x0 - 1)^2 + (x1 - 2)^2 + (x2 + 1)^2");
        $display("Unconstrained Minimum: (1.000000, 2.000000, -1.000000)");
        $display("Target Constrained Minimum: x* = (0.000000, 1.000000, 0.000000), sum x_i* = 1.0");

        // DFG Program for (x0 - 1)^2 + (x1 - 2)^2 + (x2 + 1)^2:
        write_instr(5'd0, OP_LOADI, 4'd4, 4'd0, 4'd0, 16'sd1); // R4 = 1.0
        write_instr(5'd1, OP_SUB,   4'd4, 4'd0, 4'd4, 16'sd0); // R4 = x0 - 1
        write_instr(5'd2, OP_MUL,   4'd4, 4'd4, 4'd4, 16'sd0); // R4 = (x0 - 1)^2
        write_instr(5'd3, OP_LOADI, 4'd5, 4'd0, 4'd0, 16'sd2); // R5 = 2.0
        write_instr(5'd4, OP_SUB,   4'd5, 4'd1, 4'd5, 16'sd0); // R5 = x1 - 2
        write_instr(5'd5, OP_MUL,   4'd5, 4'd5, 4'd5, 16'sd0); // R5 = (x1 - 2)^2
        write_instr(5'd6, OP_LOADI, 4'd6, 4'd0, 4'd0, 16'sd1); // R6 = 1.0
        write_instr(5'd7, OP_ADD,   4'd6, 4'd2, 4'd6, 16'sd0); // R6 = x2 + 1
        write_instr(5'd8, OP_MUL,   4'd6, 4'd6, 4'd6, 16'sd0); // R6 = (x2 + 1)^2
        write_instr(5'd9, OP_ADD,   4'd0, 4'd4, 4'd5, 16'sd0); // R0 = (x0-1)^2 + (x1-2)^2
        write_instr(5'd10,OP_ADD,   4'd0, 4'd0, 4'd6, 16'sd0); // R0 = total
        write_instr(5'd11,OP_NOP,   4'd0, 4'd0, 4'd0, 16'sd0); // End

        @(posedge clk);
        num_params = 3'd3;
        x_init     = set_vec(set_vec(set_vec(x_init, 2'd0, real_to_q16(0.333333)), 2'd1, real_to_q16(0.333333)), 2'd2, real_to_q16(0.333334));
        proj_mode  = PROJ_SIMPLEX;
        step_size  = real_to_q16(0.125);
        tolerance  = 32'h0000_0008;
        max_iters  = 8'd50;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting PGD Solver on Probability Simplex...");
        @(posedge done);
        #1;

        $display("--> PGD SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Iterations:     %0d", iter_count);
        $display("    x0_optimal:     %f (Expected:   0.000000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:     %f (Expected: ~ 1.000000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    x2_optimal:     %f (Expected:   0.000000)", q16_to_real(get_vec(x_optimal, 2'd2)));
        $display("    Simplex Sum:    %f (Expected:   1.000000)", q16_to_real(get_vec(x_optimal, 2'd0)) + q16_to_real(get_vec(x_optimal, 2'd1)) + q16_to_real(get_vec(x_optimal, 2'd2)));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) < 0.01 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 0.98 && q16_to_real(get_vec(x_optimal, 2'd1)) < 1.02 &&
            get_vec(x_optimal, 2'd2) == Q16_ZERO) begin
            $display("[TEST 3 PASSED] Successfully converged Probability Simplex constrained optimization!");
        end else begin
            $display("[TEST 3 FAILED] Simplex constraint satisfaction failed.");
        end

        $display("\n==================================================================");
        $display(" ALL PGD HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
