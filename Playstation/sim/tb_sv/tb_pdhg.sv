// =============================================================================
// File Name   : tb_pdhg.sv
// Module Name : tb_pdhg
// Project     : Primal-Dual Hybrid Gradient (PDHG / Chambolle-Pock) Accelerator (Solver #19)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for PDHG / Chambolle-Pock Accelerator.
//   Verifies:
//   1. Total Variation (TV-L2) 1D Step Signal Denoising
//   2. Basis Pursuit / L1 Sparse Signal Recovery
//   3. Non-Negative Constrained Linear Inversion
// =============================================================================

`timescale 1ns / 1ps

import pdhg_types_pkg::*;
`include "pdhg_helpers.svh"

module tb_pdhg;

    logic        clk;
    logic        rst_n;

    // Controls & Inputs
    logic        start;
    logic [2:0]  dim_m;
    logic [2:0]  dim_n;
    pdhg_mode_t  mode;
    mat_t        k_matrix;
    vec_t        b_target;
    vec_t        x_init;
    vec_t        y_init;
    q16_t        tau_step;
    q16_t        sigma_step;
    q16_t        theta_relax;
    q16_t        lambda_param;
    q16_t        tol_residual;
    logic [15:0] max_iters;

    // Results & Outputs
    vec_t        x_optimal;
    vec_t        y_optimal;
    logic [15:0] iter_count;
    q16_t        primal_res;
    q16_t        dual_res;
    status_t     status;
    logic        done;
    logic        busy;

    // Instantiate Top Module
    pdhg_top dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (start),
        .dim_m       (dim_m),
        .dim_n       (dim_n),
        .mode        (mode),
        .k_matrix    (k_matrix),
        .b_target    (b_target),
        .x_init      (x_init),
        .y_init      (y_init),
        .tau_step    (tau_step),
        .sigma_step  (sigma_step),
        .theta_relax (theta_relax),
        .lambda_param(lambda_param),
        .tol_residual(tol_residual),
        .max_iters   (max_iters),
        .x_optimal   (x_optimal),
        .y_optimal   (y_optimal),
        .iter_count  (iter_count),
        .primal_res  (primal_res),
        .dual_res    (dual_res),
        .status      (status),
        .done        (done),
        .busy        (busy)
    );

    // 100MHz Clock Generator
    initial clk = 0;
    always #5 clk = ~clk;

    // Fixed-point Real Conversion Helpers
    function real q16_to_real(input q16_t val);
        q16_to_real = real'(val) / 65536.0;
    endfunction

    function q16_t real_to_q16(input real val);
        real_to_q16 = q16_t'(int'(val * 65536.0));
    endfunction

    initial begin
        $display("==================================================================");
        $display(" PDHG / Chambolle-Pock Accelerator TB (Solver #19)");
        $display("==================================================================");

        // Reset
        rst_n        = 0;
        start        = 0;
        dim_m        = 3'd3;
        dim_n        = 3'd4;
        mode         = MODE_TV_L2;
        k_matrix     = '0;
        b_target     = '0;
        x_init       = '0;
        y_init       = '0;
        tau_step     = 0;
        sigma_step   = 0;
        theta_relax  = 0;
        lambda_param = 0;
        tol_residual = 0;
        max_iters    = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: Total Variation (TV-L2) 1D Step Signal Denoising
        // Problem: min 0.5*||x - b||_2^2 + λ * ||K*x||_1
        // Noisy signal: b = [1.2, 0.8, 4.2, 3.8]^T
        // Discrete difference operator K (3x4):
        // K = [ [-1, 1, 0, 0], [0, -1, 1, 0], [0, 0, -1, 1] ]
        // Expected TV Denoised output: x* ≈ [1.0, 1.0, 4.0, 4.0]^T
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Total Variation (TV-L2) 1D Signal Denoising");
        $display("Noisy Step Signal: b = [1.200000, 0.800000, 4.200000, 3.800000]");
        $display("Target Denoised:   x* ≈ [1.000000, 1.000000, 4.000000, 4.000000]");

        k_matrix = '0;
        // K row 0: [-1, 1, 0, 0]
        k_matrix = set_mat(k_matrix, 2'd0, 2'd0, real_to_q16(-1.0));
        k_matrix = set_mat(k_matrix, 2'd0, 2'd1, real_to_q16(1.0));
        // K row 1: [0, -1, 1, 0]
        k_matrix = set_mat(k_matrix, 2'd1, 2'd1, real_to_q16(-1.0));
        k_matrix = set_mat(k_matrix, 2'd1, 2'd2, real_to_q16(1.0));
        // K row 2: [0, 0, -1, 1]
        k_matrix = set_mat(k_matrix, 2'd2, 2'd2, real_to_q16(-1.0));
        k_matrix = set_mat(k_matrix, 2'd2, 2'd3, real_to_q16(1.0));

        b_target[0] = real_to_q16(1.2);
        b_target[1] = real_to_q16(0.8);
        b_target[2] = real_to_q16(4.2);
        b_target[3] = real_to_q16(3.8);

        x_init = b_target; // Initialize primal with observation
        y_init = '0;

        @(posedge clk);
        dim_m        = 3'd3;
        dim_n        = 3'd4;
        mode         = MODE_TV_L2;
        tau_step     = real_to_q16(0.25);
        sigma_step   = real_to_q16(0.25);
        theta_relax  = real_to_q16(1.0);
        lambda_param = real_to_q16(0.5); // λ = 0.5
        tol_residual = real_to_q16(0.0005);
        max_iters    = 16'd100;
        start        = 1'b1;
        @(posedge clk);
        start        = 1'b0;

        $display("Starting PDHG / Chambolle-Pock TV-L2 Denoising Engine...");
        @(posedge done);
        #1;

        $display("--> PDHG TV-L2 DENOISER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    Primal Res:   %f", q16_to_real(primal_res));
        $display("    Dual Res:     %f", q16_to_real(dual_res));
        $display("    x0_denoised:  %f (Expected: ~ 1.250000)", q16_to_real(x_optimal[0]));
        $display("    x1_denoised:  %f (Expected: ~ 1.250000)", q16_to_real(x_optimal[1]));
        $display("    x2_denoised:  %f (Expected: ~ 3.750000)", q16_to_real(x_optimal[2]));
        $display("    x3_denoised:  %f (Expected: ~ 3.750000)", q16_to_real(x_optimal[3]));

        if (q16_to_real(x_optimal[0]) > 1.20 && q16_to_real(x_optimal[0]) < 1.30 &&
            q16_to_real(x_optimal[1]) > 1.20 && q16_to_real(x_optimal[1]) < 1.30 &&
            q16_to_real(x_optimal[2]) > 3.70 && q16_to_real(x_optimal[2]) < 3.80 &&
            q16_to_real(x_optimal[3]) > 3.70 && q16_to_real(x_optimal[3]) < 3.80) begin
            $display("[TEST 1 PASSED] Successfully denoised step signal while preserving sharp edge!");
        end else begin
            $display("[TEST 1 FAILED] TV denoised signal outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: Basis Pursuit / L1 Sparse Recovery (MODE_LASSO_L1)
        // Problem: min 0.5*||K*x - b||_2^2 + λ * ||x||_1
        // True Sparse Signal: x_true = [2.0, 0.0, 3.0, 0.0]^T
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Basis Pursuit / L1 Sparse Signal Recovery");
        $display("Ground Truth: x_true = [2.000000, 0.000000, 3.000000, 0.000000]");

        k_matrix = '0;
        k_matrix = set_mat(k_matrix, 2'd0, 2'd0, real_to_q16(1.0));
        k_matrix = set_mat(k_matrix, 2'd0, 2'd1, real_to_q16(0.5));
        k_matrix = set_mat(k_matrix, 2'd1, 2'd2, real_to_q16(1.0));
        k_matrix = set_mat(k_matrix, 2'd1, 2'd3, real_to_q16(0.5));
        k_matrix = set_mat(k_matrix, 2'd2, 2'd0, real_to_q16(0.5));
        k_matrix = set_mat(k_matrix, 2'd2, 2'd2, real_to_q16(0.5));
        k_matrix = set_mat(k_matrix, 2'd3, 2'd1, real_to_q16(0.5));
        k_matrix = set_mat(k_matrix, 2'd3, 2'd3, real_to_q16(1.0));

        // b = K * x_true = [2.0, 3.0, 2.5, 0.0]^T
        b_target[0] = real_to_q16(2.0);
        b_target[1] = real_to_q16(3.0);
        b_target[2] = real_to_q16(2.5);
        b_target[3] = real_to_q16(0.0);

        x_init = '0;
        y_init = '0;

        @(posedge clk);
        dim_m        = 3'd4;
        dim_n        = 3'd4;
        mode         = MODE_LASSO_L1;
        tau_step     = real_to_q16(0.20);
        sigma_step   = real_to_q16(0.20);
        theta_relax  = real_to_q16(1.0);
        lambda_param = real_to_q16(0.05); // λ = 0.05
        tol_residual = real_to_q16(0.0005);
        max_iters    = 16'd150;
        start        = 1'b1;
        @(posedge clk);
        start        = 1'b0;

        $display("Starting PDHG Sparse L1 Recovery Engine...");
        @(posedge done);
        #1;

        $display("--> PDHG SPARSE RECOVERY FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    x0_recovered: %f (Expected: ~ 2.000000)", q16_to_real(x_optimal[0]));
        $display("    x1_recovered: %f (Expected: ~ 0.000000)", q16_to_real(x_optimal[1]));
        $display("    x2_recovered: %f (Expected: ~ 3.000000)", q16_to_real(x_optimal[2]));
        $display("    x3_recovered: %f (Expected: ~ 0.000000)", q16_to_real(x_optimal[3]));

        if (q16_to_real(x_optimal[0]) > 1.90 && q16_to_real(x_optimal[0]) < 2.10 &&
            q16_to_real(x_optimal[1]) > -0.05 && q16_to_real(x_optimal[1]) < 0.05 &&
            q16_to_real(x_optimal[2]) > 2.90 && q16_to_real(x_optimal[2]) < 3.10 &&
            q16_to_real(x_optimal[3]) > -0.05 && q16_to_real(x_optimal[3]) < 0.05) begin
            $display("[TEST 2 PASSED] Successfully recovered sparse ground truth signal!");
        end else begin
            $display("[TEST 2 FAILED] Sparse recovery outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: Non-Negative Constrained Linear Inversion (MODE_NONNEG_L2)
        // Problem: min 0.5*||K*x - b||_2^2 subject to x >= 0
        // Unconstrained minimum would be [-1.0, 2.0], clamped to [0.0, 2.0]
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Non-Negative Constrained Linear Inversion");
        $display("Problem: min 0.5*||x - [-1.0, 2.0]||^2  s.t.  x >= 0");
        $display("Target Non-Negative Output: x* = [0.000000, 2.000000]");

        k_matrix = '0;
        k_matrix = set_mat(k_matrix, 2'd0, 2'd0, real_to_q16(1.0));
        k_matrix = set_mat(k_matrix, 2'd1, 2'd1, real_to_q16(1.0));

        b_target[0] = real_to_q16(-1.0);
        b_target[1] = real_to_q16(2.0);
        b_target[2] = Q16_ZERO;
        b_target[3] = Q16_ZERO;

        x_init = '0;
        y_init = '0;

        @(posedge clk);
        dim_m        = 3'd2;
        dim_n        = 3'd2;
        mode         = MODE_NONNEG_L2;
        tau_step     = real_to_q16(0.25);
        sigma_step   = real_to_q16(0.25);
        theta_relax  = real_to_q16(1.0);
        lambda_param = Q16_ZERO;
        tol_residual = real_to_q16(0.0005);
        max_iters    = 16'd100;
        start        = 1'b1;
        @(posedge clk);
        start        = 1'b0;

        $display("Starting PDHG Non-Negative Inversion Engine...");
        @(posedge done);
        #1;

        $display("--> PDHG NON-NEGATIVE FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    x0_optimal:   %f (Expected: ~ 0.000000)", q16_to_real(x_optimal[0]));
        $display("    x1_optimal:   %f (Expected: ~ 2.000000)", q16_to_real(x_optimal[1]));

        if (q16_to_real(x_optimal[0]) >= 0.0 && q16_to_real(x_optimal[0]) < 0.05 &&
            q16_to_real(x_optimal[1]) > 1.90 && q16_to_real(x_optimal[1]) < 2.10) begin
            $display("[TEST 3 PASSED] Successfully solved non-negative constrained inversion!");
        end else begin
            $display("[TEST 3 FAILED] Non-negative inversion outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL PDHG / CHAMBOLLE-POCK HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
