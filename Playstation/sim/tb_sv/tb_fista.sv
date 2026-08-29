// =============================================================================
// File Name   : tb_fista.sv
// Module Name : tb_fista
// Project     : Fast Iterative Shrinkage-Thresholding Algorithm (FISTA) Accelerator (Solver #15)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for the FISTA Accelerator.
//   Verifies:
//   1. Accelerated Smooth Convex Gradient Descent (λ = 0.0)
//   2. Sparse Feature Selection via L1 Soft-Thresholding (λ = 1.0)
//   3. 4-Dimensional Compressed Sensing Sparse Recovery (λ = 0.5)
// =============================================================================

`timescale 1ns / 1ps

import fista_types_pkg::*;
`include "fista_helpers.svh"

module tb_fista;

    logic        clk;
    logic        rst_n;

    // Programming Interface
    logic        prog_en;
    logic [4:0]  prog_addr;
    instr_t      prog_data;

    // Algorithm Controls & Inputs
    logic        start;
    logic [2:0]  num_dims;
    q16_t        gamma_step;
    q16_t        lambda_reg;
    vec_t        x_init;
    q16_t        tolerance;
    logic [7:0]  max_iters;

    // Results & Outputs
    vec_t        x_optimal;
    q16_t        f_optimal;
    q16_t        f_composite;
    logic [2:0]  sparsity_count;
    logic [7:0]  iter_count;
    q16_t        dx_norm_inf;
    status_t     status;
    logic        done;
    logic        busy;

    // Instantiate Top Module
    fista_top dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .prog_en        (prog_en),
        .prog_addr      (prog_addr),
        .prog_data      (prog_data),
        .start          (start),
        .num_dims       (num_dims),
        .gamma_step     (gamma_step),
        .lambda_reg     (lambda_reg),
        .x_init         (x_init),
        .tolerance      (tolerance),
        .max_iters      (max_iters),
        .x_optimal      (x_optimal),
        .f_optimal      (f_optimal),
        .f_composite    (f_composite),
        .sparsity_count (sparsity_count),
        .iter_count     (iter_count),
        .dx_norm_inf    (dx_norm_inf),
        .status         (status),
        .done           (done),
        .busy           (busy)
    );

    // 100MHz Clock Generator
    initial clk = 0;
    always #5 clk = ~clk;

    // Fixed-point Real Number Conversion Helpers
    function real q16_to_real(input q16_t val);
        q16_to_real = real'(val) / 65536.0;
    endfunction

    function q16_t real_to_q16(input real val);
        real_to_q16 = q16_t'(int'(val * 65536.0));
    endfunction

    // Task to program microcode instruction
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
        $display(" Fast Iterative Shrinkage-Thresholding (FISTA) TB (Solver #15)");
        $display("==================================================================");

        // Reset
        rst_n      = 0;
        prog_en    = 0;
        prog_addr  = '0;
        prog_data  = '0;
        start      = 0;
        num_dims   = 3'd2;
        gamma_step = 0;
        lambda_reg = 0;
        x_init     = '0;
        tolerance  = 0;
        max_iters  = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: Accelerated Smooth Gradient Descent (lambda = 0.0, gamma = 0.2)
        // Objective: f(x0, x1) = (x0 - 3)^2 + 2*(x1 - 4)^2
        // Target Minimum: x0* = 3.000000, x1* = 4.000000, f(x*) = 0.000000
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Accelerated Smooth Convex Optimization (lambda = 0.0, gamma = 0.2)");
        $display("Objective: f(x0, x1) = (x0 - 3)^2 + 2*(x1 - 4)^2");
        $display("Target Optimum: x0* = 3.000000, x1* = 4.000000, f(x*) = 0.000000");

        // Microcode Program:
        // r0 = x0, r1 = x1
        // r4 = 3.0, r5 = 4.0, r6 = 2.0
        // r7 = x0 - 3.0
        // r8 = (x0 - 3.0)^2
        // r9 = x1 - 4.0
        // r10 = (x1 - 4.0)^2
        // r11 = 2 * (x1 - 4.0)^2
        // r15 = r8 + r11
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd3);  // r4  = 3.0
        write_instr(5'd1, OP_LOADC, 4'd5,  4'd0, 4'd0, 16'sd4);  // r5  = 4.0
        write_instr(5'd2, OP_LOADC, 4'd6,  4'd0, 4'd0, 16'sd2);  // r6  = 2.0
        write_instr(5'd3, OP_SUB,   4'd7,  4'd0, 4'd4, 16'sd0);  // r7  = x0 - 3.0
        write_instr(5'd4, OP_MUL,   4'd8,  4'd7, 4'd7, 16'sd0);  // r8  = (x0 - 3.0)^2
        write_instr(5'd5, OP_SUB,   4'd9,  4'd1, 4'd5, 16'sd0);  // r9  = x1 - 4.0
        write_instr(5'd6, OP_MUL,   4'd10, 4'd9, 4'd9, 16'sd0);  // r10 = (x1 - 4.0)^2
        write_instr(5'd7, OP_MUL,   4'd11, 4'd6, 4'd10, 16'sd0); // r11 = 2 * (x1 - 4.0)^2
        write_instr(5'd8, OP_ADD,   4'd15, 4'd8, 4'd11, 16'sd0); // r15 = f(x)
        write_instr(5'd9, OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);  // END

        @(posedge clk);
        num_dims   = 3'd2;
        gamma_step = real_to_q16(0.20);
        lambda_reg = Q16_ZERO; // lambda = 0.0 (smooth unconstrained)
        x_init     = set_vec(set_vec('0, 2'd0, real_to_q16(0.0)), 2'd1, real_to_q16(0.0));
        tolerance  = 32'h0000_0080; // tol ≈ 0.00195
        max_iters  = 8'd40;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting FISTA Solver from initial guess (0.0, 0.0)...");
        @(posedge done);
        #1;

        $display("--> FISTA SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Iterations:     %0d", iter_count);
        $display("    x0_optimal:     %f (Expected: ~ 3.000000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:     %f (Expected: ~ 4.000000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    f_optimal:      %f (Expected: ~ 0.000000)", q16_to_real(f_optimal));
        $display("    dx_norm_inf:    %f", q16_to_real(dx_norm_inf));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 2.98 && q16_to_real(get_vec(x_optimal, 2'd0)) < 3.02 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 3.98 && q16_to_real(get_vec(x_optimal, 2'd1)) < 4.02 &&
            q16_to_real(f_optimal) < 0.001) begin
            $display("[TEST 1 PASSED] Successfully converged smooth accelerated optimization via FISTA!");
        end else begin
            $display("[TEST 1 FAILED] Did not converge to target optimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: Sparse Feature Selection via L1 Soft-Thresholding (lambda = 1.0, gamma = 0.1)
        // Objective: f(x0, x1, x2) = (x0 - 4)^2 + 2*x1^2 + 2*x2^2
        // With lambda = 1.0, x0 is active (x0* ≈ 3.50), x1* = 0.0, x2* = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Sparse Feature Selection via L1 Regularization (lambda = 1.0, gamma = 0.1)");
        $display("Objective: f(x0, x1, x2) = (x0 - 4)^2 + 2*x1^2 + 2*x2^2 + 1.0*||x||_1");
        $display("Target Sparsity: x0* ≈ 3.500000, x1* = 0.000000, x2* = 0.000000 (Exact Zeros!)");

        // Microcode Program:
        // r0 = x0, r1 = x1, r2 = x2
        // r4 = 4.0, r5 = 2.0
        // r6 = x0 - 4.0
        // r7 = (x0 - 4.0)^2
        // r8 = x1^2
        // r9 = 2 * x1^2
        // r10 = x2^2
        // r11 = 2 * x2^2
        // r12 = r7 + r9
        // r15 = r12 + r11
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd4);   // r4  = 4.0
        write_instr(5'd1, OP_LOADC, 4'd5,  4'd0, 4'd0, 16'sd2);   // r5  = 2.0
        write_instr(5'd2, OP_SUB,   4'd6,  4'd0, 4'd4, 16'sd0);   // r6  = x0 - 4.0
        write_instr(5'd3, OP_MUL,   4'd7,  4'd6, 4'd6, 16'sd0);   // r7  = (x0 - 4.0)^2
        write_instr(5'd4, OP_MUL,   4'd8,  4'd1, 4'd1, 16'sd0);   // r8  = x1^2
        write_instr(5'd5, OP_MUL,   4'd9,  4'd5, 4'd8, 16'sd0);   // r9  = 2 * x1^2
        write_instr(5'd6, OP_MUL,   4'd10, 4'd2, 4'd2, 16'sd0);   // r10 = x2^2
        write_instr(5'd7, OP_MUL,   4'd11, 4'd5, 4'd10, 16'sd0);  // r11 = 2 * x2^2
        write_instr(5'd8, OP_ADD,   4'd12, 4'd7, 4'd9, 16'sd0);   // r12 = r7 + r9
        write_instr(5'd9, OP_ADD,   4'd15, 4'd12, 4'd11, 16'sd0); // r15 = f(x)
        write_instr(5'd10,OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);   // END

        @(posedge clk);
        num_dims   = 3'd3;
        gamma_step = real_to_q16(0.10);
        lambda_reg = real_to_q16(1.0);
        x_init     = set_vec(set_vec(set_vec('0, 2'd0, real_to_q16(1.0)), 2'd1, real_to_q16(2.0)), 2'd2, real_to_q16(-2.0));
        tolerance  = 32'h0000_0080;
        max_iters  = 8'd40;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting FISTA Sparse Feature Selection Solver...");
        @(posedge done);
        #1;

        $display("--> FISTA SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Iterations:     %0d", iter_count);
        $display("    x0_optimal:     %f (Active Feature)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:     %f (Noise Feature - Exact Zero)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    x2_optimal:     %f (Noise Feature - Exact Zero)", q16_to_real(get_vec(x_optimal, 2'd2)));
        $display("    f_smooth:       %f", q16_to_real(f_optimal));
        $display("    f_composite:    %f", q16_to_real(f_composite));
        $display("    sparsity_count: %0d / 3 zero coefficients", sparsity_count);

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 3.40 && q16_to_real(get_vec(x_optimal, 2'd0)) < 3.60 &&
            get_vec(x_optimal, 2'd1) == Q16_ZERO &&
            get_vec(x_optimal, 2'd2) == Q16_ZERO &&
            sparsity_count == 3'd2) begin
            $display("[TEST 2 PASSED] Successfully eliminated irrelevant noise features via FISTA soft-thresholding!");
        end else begin
            $display("[TEST 2 FAILED] Did not produce exact sparsity.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 4-Dimensional Compressed Sensing Sparse Recovery (lambda = 0.5, gamma = 0.1)
        // Objective: f(x0, x1, x2, x3) = (x0 - 1.5)^2 + 2*x1^2 + (x2 - 2.5)^2 + 2*x3^2
        // Target: x0* ≈ 1.25, x1* = 0.0, x2* ≈ 2.25, x3* = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 4D Compressed Sensing Sparse Signal Recovery (lambda = 0.5, gamma = 0.1)");
        $display("Target Sparse Signal: x* = [1.250000, 0.000000, 2.250000, 0.000000]");

        // Microcode Program:
        // r0 = x0, r1 = x1, r2 = x2, r3 = x3
        // r4 = 1.5, r5 = 2.5, r6 = 2.0
        // r7 = x0 - 1.5
        // r8 = (x0 - 1.5)^2
        // r9 = x1^2
        // r10 = 2 * x1^2
        // r11 = x2 - 2.5
        // r12 = (x2 - 2.5)^2
        // r13 = x3^2
        // r14 = 2 * x3^2
        // r15 = r8 + r10 + r12 + r14
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd1);   // r4  = 1.0 -> will add 0.5 below
        write_instr(5'd1, OP_LOADC, 4'd5,  4'd0, 4'd0, 16'sd2);   // r5  = 2.0 -> will add 0.5 below
        write_instr(5'd2, OP_LOADC, 4'd6,  4'd0, 4'd0, 16'sd2);   // r6  = 2.0
        write_instr(5'd3, OP_SUB,   4'd7,  4'd0, 4'd4, 16'sd0);   // r7  = x0 - 1.0
        write_instr(5'd4, OP_LOADC, 4'd14, 4'd0, 4'd0, 16'sd1);   // r14 = 1.0
        write_instr(5'd5, OP_MOV,   4'd14, 4'd14, 4'd0, 16'sd0);  // shift
        // r7 = x0 - 1.5
        // Let's use clean constants:
        // 2*(x0 - 1.5) = (2*x0 - 3)
        // Or simply load constants:
        // (x0 - 1)^2 + 2*x1^2 + (x2 - 2)^2 + 2*x3^2
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd1);   // r4  = 1.0
        write_instr(5'd1, OP_LOADC, 4'd5,  4'd0, 4'd0, 16'sd2);   // r5  = 2.0
        write_instr(5'd2, OP_LOADC, 4'd6,  4'd0, 4'd0, 16'sd2);   // r6  = 2.0
        write_instr(5'd3, OP_SUB,   4'd7,  4'd0, 4'd4, 16'sd0);   // r7  = x0 - 1.0
        write_instr(5'd4, OP_MUL,   4'd8,  4'd7, 4'd7, 16'sd0);   // r8  = (x0 - 1.0)^2
        write_instr(5'd5, OP_MUL,   4'd9,  4'd1, 4'd1, 16'sd0);   // r9  = x1^2
        write_instr(5'd6, OP_MUL,   4'd10, 4'd6, 4'd9, 16'sd0);   // r10 = 2 * x1^2
        write_instr(5'd7, OP_SUB,   4'd11, 4'd2, 4'd5, 16'sd0);   // r11 = x2 - 2.0
        write_instr(5'd8, OP_MUL,   4'd12, 4'd11, 4'd11, 16'sd0); // r12 = (x2 - 2.0)^2
        write_instr(5'd9, OP_MUL,   4'd13, 4'd3, 4'd3, 16'sd0);   // r13 = x3^2
        write_instr(5'd10,OP_MUL,   4'd14, 4'd6, 4'd13, 16'sd0);  // r14 = 2 * x3^2
        write_instr(5'd11,OP_ADD,   4'd7,  4'd8, 4'd10, 16'sd0);  // r7  = r8 + r10
        write_instr(5'd12,OP_ADD,   4'd8,  4'd12, 4'd14, 16'sd0); // r8  = r12 + r14
        write_instr(5'd13,OP_ADD,   4'd15, 4'd7, 4'd8, 16'sd0);   // r15 = r7 + r8
        write_instr(5'd14,OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);   // END

        @(posedge clk);
        num_dims   = 3'd4;
        gamma_step = real_to_q16(0.10);
        lambda_reg = real_to_q16(0.50);
        x_init     = set_vec(set_vec(set_vec(set_vec('0, 2'd0, real_to_q16(0.0)), 2'd1, real_to_q16(1.0)), 2'd2, real_to_q16(0.0)), 2'd3, real_to_q16(-1.0));
        tolerance  = 32'h0000_0080;
        max_iters  = 8'd40;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting FISTA Compressed Sensing Recovery...");
        @(posedge done);
        #1;

        $display("--> FISTA SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Iterations:     %0d", iter_count);
        $display("    x0_optimal:     %f (Expected: ~ 0.750000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:     %f (Expected:   0.000000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    x2_optimal:     %f (Expected: ~ 1.750000)", q16_to_real(get_vec(x_optimal, 2'd2)));
        $display("    x3_optimal:     %f (Expected:   0.000000)", q16_to_real(get_vec(x_optimal, 2'd3)));
        $display("    f_smooth:       %f", q16_to_real(f_optimal));
        $display("    f_composite:    %f", q16_to_real(f_composite));
        $display("    sparsity_count: %0d / 4 zero coefficients", sparsity_count);

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 0.70 && q16_to_real(get_vec(x_optimal, 2'd0)) < 0.80 &&
            get_vec(x_optimal, 2'd1) == Q16_ZERO &&
            q16_to_real(get_vec(x_optimal, 2'd2)) > 1.70 && q16_to_real(get_vec(x_optimal, 2'd2)) < 1.80 &&
            get_vec(x_optimal, 2'd3) == Q16_ZERO &&
            sparsity_count == 3'd2) begin
            $display("[TEST 3 PASSED] Successfully recovered sparse signal via FISTA!");
        end else begin
            $display("[TEST 3 FAILED] Sparse recovery failed.");
        end

        $display("\n==================================================================");
        $display(" ALL FISTA ACCELERATOR HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
