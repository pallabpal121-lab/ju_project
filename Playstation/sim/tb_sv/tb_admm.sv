// =============================================================================
// File Name   : tb_admm.sv
// Module Name : tb_admm
// Project     : Alternating Direction Method of Multipliers (ADMM) Accelerator (Solver #11)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for the ADMM Accelerator.
//   Verifies Linear Inversion (lambda = 0), LASSO Feature Selection (lambda = 1.0),
//   and Compressed Sensing Sparse Recovery (lambda = 0.5).
// =============================================================================

`timescale 1ns / 1ps

import admm_types_pkg::*;
`include "admm_helpers.svh"

module tb_admm;

    logic         clk;
    logic         rst_n;

    // Controls & Inputs
    logic         start;
    logic [2:0]   num_params;
    logic [3:0]   num_obs;
    dataset_mat_t a_matrix;
    obs_vec_t     b_obs;
    q16_t         lambda_reg;
    q16_t         rho_val;
    q16_t         tolerance;
    logic [7:0]   max_iters;

    // Results & Outputs
    vec_t         z_optimal;
    vec_t         x_primal;
    vec_t         u_dual;
    q16_t         r_pri_norm;
    q16_t         s_dual_norm;
    logic [2:0]   sparsity_count;
    logic [7:0]   iter_count;
    status_t      status;
    logic         done;
    logic         busy;

    // Instantiate Top Module
    admm_top dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (start),
        .num_params    (num_params),
        .num_obs       (num_obs),
        .a_matrix      (a_matrix),
        .b_obs         (b_obs),
        .lambda_reg    (lambda_reg),
        .rho_val       (rho_val),
        .tolerance     (tolerance),
        .max_iters     (max_iters),
        .z_optimal     (z_optimal),
        .x_primal      (x_primal),
        .u_dual        (u_dual),
        .r_pri_norm    (r_pri_norm),
        .s_dual_norm   (s_dual_norm),
        .sparsity_count(sparsity_count),
        .iter_count    (iter_count),
        .status        (status),
        .done          (done),
        .busy          (busy)
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

    initial begin
        $display("==================================================================");
        $display(" Alternating Direction Method of Multipliers (ADMM) TB (Solver #11)");
        $display("==================================================================");

        // Reset
        rst_n      = 0;
        start      = 0;
        num_params = 3'd2;
        num_obs    = 4'd4;
        a_matrix   = '0;
        b_obs      = '0;
        lambda_reg = 0;
        rho_val    = 0;
        tolerance  = 0;
        max_iters  = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: ADMM Linear Inversion / Consensus (lambda = 0.0, rho = 1.0)
        // Model: b = 2.0*x0 + 3.0*x1
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] ADMM Linear Inversion / Consensus (lambda = 0.0, rho = 1.0)");
        $display("Target Equation: b = 2.0*x0 + 3.0*x1");
        $display("Target Weights : x0* = z0* = 2.000000, x1* = z1* = 3.000000");

        // Load 4 data samples:
        // Sample 0: (1.0, 1.0) -> b = 5.0
        // Sample 1: (2.0, 1.0) -> b = 7.0
        // Sample 2: (1.0, 3.0) -> b = 11.0
        // Sample 3: (3.0, 2.0) -> b = 12.0
        a_matrix = set_data_elem(a_matrix, 3'd0, 2'd0, real_to_q16(1.0));
        a_matrix = set_data_elem(a_matrix, 3'd0, 2'd1, real_to_q16(1.0));
        b_obs    = set_obs(b_obs, 3'd0, real_to_q16(5.0));

        a_matrix = set_data_elem(a_matrix, 3'd1, 2'd0, real_to_q16(2.0));
        a_matrix = set_data_elem(a_matrix, 3'd1, 2'd1, real_to_q16(1.0));
        b_obs    = set_obs(b_obs, 3'd1, real_to_q16(7.0));

        a_matrix = set_data_elem(a_matrix, 3'd2, 2'd0, real_to_q16(1.0));
        a_matrix = set_data_elem(a_matrix, 3'd2, 2'd1, real_to_q16(3.0));
        b_obs    = set_obs(b_obs, 3'd2, real_to_q16(11.0));

        a_matrix = set_data_elem(a_matrix, 3'd3, 2'd0, real_to_q16(3.0));
        a_matrix = set_data_elem(a_matrix, 3'd3, 2'd1, real_to_q16(2.0));
        b_obs    = set_obs(b_obs, 3'd3, real_to_q16(12.0));

        @(posedge clk);
        num_params = 3'd2;
        num_obs    = 4'd4;
        lambda_reg = real_to_q16(0.0);
        rho_val    = real_to_q16(1.0);
        tolerance  = 32'h0000_0040;
        max_iters  = 8'd50;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting ADMM Solver for Linear Inversion...");
        @(posedge done);
        #1;

        $display("--> ADMM SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Iterations:     %0d", iter_count);
        $display("    z0_optimal:     %f (Expected: ~ 2.000000)", q16_to_real(get_vec(z_optimal, 2'd0)));
        $display("    z1_optimal:     %f (Expected: ~ 3.000000)", q16_to_real(get_vec(z_optimal, 2'd1)));
        $display("    r_pri_norm:     %f", q16_to_real(r_pri_norm));
        $display("    s_dual_norm:    %f", q16_to_real(s_dual_norm));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(z_optimal, 2'd0)) > 1.98 && q16_to_real(get_vec(z_optimal, 2'd0)) < 2.02 &&
            q16_to_real(get_vec(z_optimal, 2'd1)) > 2.98 && q16_to_real(get_vec(z_optimal, 2'd1)) < 3.02) begin
            $display("[TEST 1 PASSED] Successfully converged ADMM linear consensus in hardware!");
        end else begin
            $display("[TEST 1 FAILED] Did not converge to target weights.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: ADMM LASSO Sparse Feature Selection (lambda = 1.0, rho = 1.0)
        // Model: b = 4.0*x0 (x1 and x2 are noise/irrelevant features)
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] ADMM Sparse Feature Selection (lambda = 1.0, rho = 1.0)");
        $display("Ground Truth: Only x0 is predictive (b = 4.0*x0), x1 and x2 are noise.");
        $display("Target Sparsity: z0* ≈ 4.000000, z1* = 0.000000, z2* = 0.000000 (Exact Zero!)");

        a_matrix = '0;
        b_obs    = '0;

        // Sample 0: (1.0, 0.5, 0.2) -> b = 4.0
        a_matrix = set_data_elem(a_matrix, 3'd0, 2'd0, real_to_q16(1.0));
        a_matrix = set_data_elem(a_matrix, 3'd0, 2'd1, real_to_q16(0.5));
        a_matrix = set_data_elem(a_matrix, 3'd0, 2'd2, real_to_q16(0.2));
        b_obs    = set_obs(b_obs, 3'd0, real_to_q16(4.0));

        // Sample 1: (2.0, 0.8, 0.1) -> b = 8.0
        a_matrix = set_data_elem(a_matrix, 3'd1, 2'd0, real_to_q16(2.0));
        a_matrix = set_data_elem(a_matrix, 3'd1, 2'd1, real_to_q16(0.8));
        a_matrix = set_data_elem(a_matrix, 3'd1, 2'd2, real_to_q16(0.1));
        b_obs    = set_obs(b_obs, 3'd1, real_to_q16(8.0));

        // Sample 2: (3.0, 0.2, 0.9) -> b = 12.0
        a_matrix = set_data_elem(a_matrix, 3'd2, 2'd0, real_to_q16(3.0));
        a_matrix = set_data_elem(a_matrix, 3'd2, 2'd1, real_to_q16(0.2));
        a_matrix = set_data_elem(a_matrix, 3'd2, 2'd2, real_to_q16(0.9));
        b_obs    = set_obs(b_obs, 3'd2, real_to_q16(12.0));

        // Sample 3: (4.0, 0.6, 0.4) -> b = 16.0
        a_matrix = set_data_elem(a_matrix, 3'd3, 2'd0, real_to_q16(4.0));
        a_matrix = set_data_elem(a_matrix, 3'd3, 2'd1, real_to_q16(0.6));
        a_matrix = set_data_elem(a_matrix, 3'd3, 2'd2, real_to_q16(0.4));
        b_obs    = set_obs(b_obs, 3'd3, real_to_q16(16.0));

        @(posedge clk);
        num_params = 3'd3;
        num_obs    = 4'd4;
        lambda_reg = real_to_q16(1.0);
        rho_val    = real_to_q16(1.0);
        tolerance  = 32'h0000_0040;
        max_iters  = 8'd50;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting ADMM Feature Selection with lambda = 1.0...");
        @(posedge done);
        #1;

        $display("--> ADMM SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Iterations:     %0d", iter_count);
        $display("    z0_optimal:     %f (Active Feature)", q16_to_real(get_vec(z_optimal, 2'd0)));
        $display("    z1_optimal:     %f (Noise Feature - Exact Zero)", q16_to_real(get_vec(z_optimal, 2'd1)));
        $display("    z2_optimal:     %f (Noise Feature - Exact Zero)", q16_to_real(get_vec(z_optimal, 2'd2)));
        $display("    sparsity_count: %0d / 3 zero coefficients", sparsity_count);

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(z_optimal, 2'd0)) > 3.80 &&
            get_vec(z_optimal, 2'd1) == Q16_ZERO &&
            get_vec(z_optimal, 2'd2) == Q16_ZERO) begin
            $display("[TEST 2 PASSED] Successfully eliminated irrelevant features via ADMM!");
        end else begin
            $display("[TEST 2 FAILED] Irrelevant features were not zeroed.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: ADMM Compressed Sensing Sparse Signal Recovery (lambda = 0.5, rho = 1.0)
        // 4 features, 6 observations, sparse signal z* = [1.5, 0.0, 2.5, 0.0]
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] ADMM Compressed Sensing Sparse Recovery (lambda = 0.5, rho = 1.0)");
        $display("Target Sparse Signal: z* = [1.500000, 0.000000, 2.500000, 0.000000]");

        a_matrix = '0;
        b_obs    = '0;

        // Load 6 observation equations:
        // b_m = 1.5*A_{m,0} + 2.5*A_{m,2}
        a_matrix = set_data_elem(a_matrix, 3'd0, 2'd0, real_to_q16(1.0));
        a_matrix = set_data_elem(a_matrix, 3'd0, 2'd1, real_to_q16(0.3));
        a_matrix = set_data_elem(a_matrix, 3'd0, 2'd2, real_to_q16(2.0));
        a_matrix = set_data_elem(a_matrix, 3'd0, 2'd3, real_to_q16(0.1));
        b_obs    = set_obs(b_obs, 3'd0, real_to_q16(6.5)); // 1.5*1 + 2.5*2 = 6.5

        a_matrix = set_data_elem(a_matrix, 3'd1, 2'd0, real_to_q16(2.0));
        a_matrix = set_data_elem(a_matrix, 3'd1, 2'd1, real_to_q16(0.1));
        a_matrix = set_data_elem(a_matrix, 3'd1, 2'd2, real_to_q16(1.0));
        a_matrix = set_data_elem(a_matrix, 3'd1, 2'd3, real_to_q16(0.4));
        b_obs    = set_obs(b_obs, 3'd1, real_to_q16(5.5)); // 1.5*2 + 2.5*1 = 5.5

        a_matrix = set_data_elem(a_matrix, 3'd2, 2'd0, real_to_q16(0.0));
        a_matrix = set_data_elem(a_matrix, 3'd2, 2'd1, real_to_q16(0.9));
        a_matrix = set_data_elem(a_matrix, 3'd2, 2'd2, real_to_q16(3.0));
        a_matrix = set_data_elem(a_matrix, 3'd2, 2'd3, real_to_q16(0.2));
        b_obs    = set_obs(b_obs, 3'd2, real_to_q16(7.5)); // 2.5*3 = 7.5

        a_matrix = set_data_elem(a_matrix, 3'd3, 2'd0, real_to_q16(3.0));
        a_matrix = set_data_elem(a_matrix, 3'd3, 2'd1, real_to_q16(0.2));
        a_matrix = set_data_elem(a_matrix, 3'd3, 2'd2, real_to_q16(0.0));
        a_matrix = set_data_elem(a_matrix, 3'd3, 2'd3, real_to_q16(0.5));
        b_obs    = set_obs(b_obs, 3'd3, real_to_q16(4.5)); // 1.5*3 = 4.5

        a_matrix = set_data_elem(a_matrix, 3'd4, 2'd0, real_to_q16(2.0));
        a_matrix = set_data_elem(a_matrix, 3'd4, 2'd1, real_to_q16(0.5));
        a_matrix = set_data_elem(a_matrix, 3'd4, 2'd2, real_to_q16(2.0));
        a_matrix = set_data_elem(a_matrix, 3'd4, 2'd3, real_to_q16(0.1));
        b_obs    = set_obs(b_obs, 3'd4, real_to_q16(8.0)); // 1.5*2 + 2.5*2 = 8.0

        a_matrix = set_data_elem(a_matrix, 3'd5, 2'd0, real_to_q16(1.0));
        a_matrix = set_data_elem(a_matrix, 3'd5, 2'd1, real_to_q16(0.4));
        a_matrix = set_data_elem(a_matrix, 3'd5, 2'd2, real_to_q16(1.0));
        a_matrix = set_data_elem(a_matrix, 3'd5, 2'd3, real_to_q16(0.8));
        b_obs    = set_obs(b_obs, 3'd5, real_to_q16(4.0)); // 1.5*1 + 2.5*1 = 4.0

        @(posedge clk);
        num_params = 3'd4;
        num_obs    = 4'd6;
        lambda_reg = real_to_q16(0.5);
        rho_val    = real_to_q16(1.0);
        tolerance  = 32'h0000_0040;
        max_iters  = 8'd50;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting ADMM Compressed Sensing Recovery...");
        @(posedge done);
        #1;

        $display("--> ADMM SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Iterations:     %0d", iter_count);
        $display("    z0_optimal:     %f (Expected: ~ 1.500000)", q16_to_real(get_vec(z_optimal, 2'd0)));
        $display("    z1_optimal:     %f (Expected:   0.000000)", q16_to_real(get_vec(z_optimal, 2'd1)));
        $display("    z2_optimal:     %f (Expected: ~ 2.500000)", q16_to_real(get_vec(z_optimal, 2'd2)));
        $display("    z3_optimal:     %f (Expected:   0.000000)", q16_to_real(get_vec(z_optimal, 2'd3)));
        $display("    sparsity_count: %0d / 4 zero coefficients", sparsity_count);

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(z_optimal, 2'd0)) > 1.40 &&
            get_vec(z_optimal, 2'd1) == Q16_ZERO &&
            q16_to_real(get_vec(z_optimal, 2'd2)) > 2.40 &&
            get_vec(z_optimal, 2'd3) == Q16_ZERO) begin
            $display("[TEST 3 PASSED] Successfully recovered sparse signal via ADMM!");
        end else begin
            $display("[TEST 3 FAILED] ADMM sparse recovery failed.");
        end

        $display("\n==================================================================");
        $display(" ALL ADMM HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
