// =============================================================================
// File Name   : tb_lasso.sv
// Module Name : tb_lasso
// Project     : Coordinate Descent / LASSO L1 Sparsity Accelerator (Solver #10)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for the LASSO Accelerator.
//   Verifies OLS Regression (lambda = 0), Sparse Feature Selection with exact
//   zero-clamping (lambda = 1.0), and Compressed Sensing Sparse Signal Recovery.
// =============================================================================

`timescale 1ns / 1ps

import lasso_types_pkg::*;
`include "lasso_helpers.svh"

module tb_lasso;

    logic         clk;
    logic         rst_n;

    // Controls & Inputs
    logic         start;
    logic [2:0]   num_params;
    logic [3:0]   num_obs;
    dataset_mat_t x_matrix;
    obs_vec_t     y_obs;
    vec_t         w_init;
    q16_t         lambda_reg;
    q16_t         tolerance;
    logic [7:0]   max_iters;

    // Results & Outputs
    vec_t         w_optimal;
    q16_t         cost_optimal;
    q16_t         max_delta_w;
    logic [2:0]   sparsity_count;
    logic [7:0]   iter_count;
    status_t      status;
    logic         done;
    logic         busy;

    // Instantiate Top Module
    lasso_top dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (start),
        .num_params    (num_params),
        .num_obs       (num_obs),
        .x_matrix      (x_matrix),
        .y_obs         (y_obs),
        .w_init        (w_init),
        .lambda_reg    (lambda_reg),
        .tolerance     (tolerance),
        .max_iters     (max_iters),
        .w_optimal     (w_optimal),
        .cost_optimal  (cost_optimal),
        .max_delta_w   (max_delta_w),
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
        $display(" Coordinate Descent / LASSO L1 Sparsity Accelerator TB (Solver #10)");
        $display("==================================================================");

        // Reset
        rst_n      = 0;
        start      = 0;
        num_params = 3'd2;
        num_obs    = 4'd4;
        x_matrix   = '0;
        y_obs      = '0;
        w_init     = '0;
        lambda_reg = 0;
        tolerance  = 0;
        max_iters  = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: Ordinary Least Squares (OLS) Linear Regression (lambda = 0.0)
        // Model: y = 2.0*x0 + 3.0*x1
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Ordinary Least Squares Linear Regression (lambda = 0.0)");
        $display("Target Equation: y = 2.0*x0 + 3.0*x1");
        $display("Target Weights : w0* = 2.000000, w1* = 3.000000");

        // Load 4 data samples:
        // Sample 0: (1.0, 1.0) -> y = 5.0
        // Sample 1: (2.0, 1.0) -> y = 7.0
        // Sample 2: (1.0, 3.0) -> y = 11.0
        // Sample 3: (3.0, 2.0) -> y = 12.0
        x_matrix = set_mat_elem(x_matrix, 3'd0, 2'd0, real_to_q16(1.0));
        x_matrix = set_mat_elem(x_matrix, 3'd0, 2'd1, real_to_q16(1.0));
        y_obs    = set_obs(y_obs, 3'd0, real_to_q16(5.0));

        x_matrix = set_mat_elem(x_matrix, 3'd1, 2'd0, real_to_q16(2.0));
        x_matrix = set_mat_elem(x_matrix, 3'd1, 2'd1, real_to_q16(1.0));
        y_obs    = set_obs(y_obs, 3'd1, real_to_q16(7.0));

        x_matrix = set_mat_elem(x_matrix, 3'd2, 2'd0, real_to_q16(1.0));
        x_matrix = set_mat_elem(x_matrix, 3'd2, 2'd1, real_to_q16(3.0));
        y_obs    = set_obs(y_obs, 3'd2, real_to_q16(11.0));

        x_matrix = set_mat_elem(x_matrix, 3'd3, 2'd0, real_to_q16(3.0));
        x_matrix = set_mat_elem(x_matrix, 3'd3, 2'd1, real_to_q16(2.0));
        y_obs    = set_obs(y_obs, 3'd3, real_to_q16(12.0));

        @(posedge clk);
        num_params = 3'd2;
        num_obs    = 4'd4;
        w_init     = '0;
        lambda_reg = real_to_q16(0.0);
        tolerance  = 32'h0000_0020;
        max_iters  = 8'd50;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting LASSO Solver for OLS Regression...");
        @(posedge done);
        #1;

        $display("--> LASSO SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Cycles:         %0d", iter_count);
        $display("    w0_optimal:     %f (Expected: ~ 2.000000)", q16_to_real(get_vec(w_optimal, 2'd0)));
        $display("    w1_optimal:     %f (Expected: ~ 3.000000)", q16_to_real(get_vec(w_optimal, 2'd1)));
        $display("    cost_optimal:   %f (RSS)", q16_to_real(cost_optimal));
        $display("    max_delta_w:    %f", q16_to_real(max_delta_w));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(w_optimal, 2'd0)) > 1.98 && q16_to_real(get_vec(w_optimal, 2'd0)) < 2.02 &&
            q16_to_real(get_vec(w_optimal, 2'd1)) > 2.98 && q16_to_real(get_vec(w_optimal, 2'd1)) < 3.02) begin
            $display("[TEST 1 PASSED] Successfully converged OLS regression in hardware!");
        end else begin
            $display("[TEST 1 FAILED] Did not converge to target weights.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: Sparse Feature Selection with Irrelevant Features (lambda = 1.0)
        // Model: y = 4.0*x0 (x1 and x2 are noise/irrelevant features)
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Sparse Feature Selection (lambda = 1.0)");
        $display("Ground Truth: Only x0 is predictive (y = 4.0*x0), x1 and x2 are noise.");
        $display("Target Sparsity: w0* ≈ 4.000000, w1* = 0.000000, w2* = 0.000000 (Exact Zero!)");

        x_matrix = '0;
        y_obs    = '0;

        // Sample 0: (1.0, 0.5, 0.2) -> y = 4.0
        x_matrix = set_mat_elem(x_matrix, 3'd0, 2'd0, real_to_q16(1.0));
        x_matrix = set_mat_elem(x_matrix, 3'd0, 2'd1, real_to_q16(0.5));
        x_matrix = set_mat_elem(x_matrix, 3'd0, 2'd2, real_to_q16(0.2));
        y_obs    = set_obs(y_obs, 3'd0, real_to_q16(4.0));

        // Sample 1: (2.0, 0.8, 0.1) -> y = 8.0
        x_matrix = set_mat_elem(x_matrix, 3'd1, 2'd0, real_to_q16(2.0));
        x_matrix = set_mat_elem(x_matrix, 3'd1, 2'd1, real_to_q16(0.8));
        x_matrix = set_mat_elem(x_matrix, 3'd1, 2'd2, real_to_q16(0.1));
        y_obs    = set_obs(y_obs, 3'd1, real_to_q16(8.0));

        // Sample 2: (3.0, 0.2, 0.9) -> y = 12.0
        x_matrix = set_mat_elem(x_matrix, 3'd2, 2'd0, real_to_q16(3.0));
        x_matrix = set_mat_elem(x_matrix, 3'd2, 2'd1, real_to_q16(0.2));
        x_matrix = set_mat_elem(x_matrix, 3'd2, 2'd2, real_to_q16(0.9));
        y_obs    = set_obs(y_obs, 3'd2, real_to_q16(12.0));

        // Sample 3: (4.0, 0.6, 0.4) -> y = 16.0
        x_matrix = set_mat_elem(x_matrix, 3'd3, 2'd0, real_to_q16(4.0));
        x_matrix = set_mat_elem(x_matrix, 3'd3, 2'd1, real_to_q16(0.6));
        x_matrix = set_mat_elem(x_matrix, 3'd3, 2'd2, real_to_q16(0.4));
        y_obs    = set_obs(y_obs, 3'd3, real_to_q16(16.0));

        @(posedge clk);
        num_params = 3'd3;
        num_obs    = 4'd4;
        w_init     = '0;
        lambda_reg = real_to_q16(1.0);
        tolerance  = 32'h0000_0020;
        max_iters  = 8'd50;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting LASSO Feature Selection with lambda = 1.0...");
        @(posedge done);
        #1;

        $display("--> LASSO SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Cycles:         %0d", iter_count);
        $display("    w0_optimal:     %f (Active Feature)", q16_to_real(get_vec(w_optimal, 2'd0)));
        $display("    w1_optimal:     %f (Noise Feature - Exact Zero)", q16_to_real(get_vec(w_optimal, 2'd1)));
        $display("    w2_optimal:     %f (Noise Feature - Exact Zero)", q16_to_real(get_vec(w_optimal, 2'd2)));
        $display("    sparsity_count: %0d / 3 zero coefficients", sparsity_count);

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(w_optimal, 2'd0)) > 3.80 &&
            get_vec(w_optimal, 2'd1) == Q16_ZERO &&
            get_vec(w_optimal, 2'd2) == Q16_ZERO) begin
            $display("[TEST 2 PASSED] Successfully eliminated irrelevant features to exact silicon zero!");
        end else begin
            $display("[TEST 2 FAILED] Irrelevant features were not zeroed.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: Compressed Sensing Sparse Recovery (lambda = 0.5)
        // 4 features, 6 observations, sparse signal w* = [1.5, 0.0, 2.5, 0.0]
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Compressed Sensing Sparse Recovery (lambda = 0.5)");
        $display("Target Sparse Signal: w* = [1.500000, 0.000000, 2.500000, 0.000000]");

        x_matrix = '0;
        y_obs    = '0;

        // Load 6 observation equations for 4 features:
        // y_m = 1.5*X_{m,0} + 2.5*X_{m,2}
        x_matrix = set_mat_elem(x_matrix, 3'd0, 2'd0, real_to_q16(1.0));
        x_matrix = set_mat_elem(x_matrix, 3'd0, 2'd1, real_to_q16(0.3));
        x_matrix = set_mat_elem(x_matrix, 3'd0, 2'd2, real_to_q16(2.0));
        x_matrix = set_mat_elem(x_matrix, 3'd0, 2'd3, real_to_q16(0.1));
        y_obs    = set_obs(y_obs, 3'd0, real_to_q16(6.5)); // 1.5*1 + 2.5*2 = 6.5

        x_matrix = set_mat_elem(x_matrix, 3'd1, 2'd0, real_to_q16(2.0));
        x_matrix = set_mat_elem(x_matrix, 3'd1, 2'd1, real_to_q16(0.1));
        x_matrix = set_mat_elem(x_matrix, 3'd1, 2'd2, real_to_q16(1.0));
        x_matrix = set_mat_elem(x_matrix, 3'd1, 2'd3, real_to_q16(0.4));
        y_obs    = set_obs(y_obs, 3'd1, real_to_q16(5.5)); // 1.5*2 + 2.5*1 = 5.5

        x_matrix = set_mat_elem(x_matrix, 3'd2, 2'd0, real_to_q16(0.0));
        x_matrix = set_mat_elem(x_matrix, 3'd2, 2'd1, real_to_q16(0.9));
        x_matrix = set_mat_elem(x_matrix, 3'd2, 2'd2, real_to_q16(3.0));
        x_matrix = set_mat_elem(x_matrix, 3'd2, 2'd3, real_to_q16(0.2));
        y_obs    = set_obs(y_obs, 3'd2, real_to_q16(7.5)); // 2.5*3 = 7.5

        x_matrix = set_mat_elem(x_matrix, 3'd3, 2'd0, real_to_q16(3.0));
        x_matrix = set_mat_elem(x_matrix, 3'd3, 2'd1, real_to_q16(0.2));
        x_matrix = set_mat_elem(x_matrix, 3'd3, 2'd2, real_to_q16(0.0));
        x_matrix = set_mat_elem(x_matrix, 3'd3, 2'd3, real_to_q16(0.5));
        y_obs    = set_obs(y_obs, 3'd3, real_to_q16(4.5)); // 1.5*3 = 4.5

        x_matrix = set_mat_elem(x_matrix, 3'd4, 2'd0, real_to_q16(2.0));
        x_matrix = set_mat_elem(x_matrix, 3'd4, 2'd1, real_to_q16(0.5));
        x_matrix = set_mat_elem(x_matrix, 3'd4, 2'd2, real_to_q16(2.0));
        x_matrix = set_mat_elem(x_matrix, 3'd4, 2'd3, real_to_q16(0.1));
        y_obs    = set_obs(y_obs, 3'd4, real_to_q16(8.0)); // 1.5*2 + 2.5*2 = 8.0

        x_matrix = set_mat_elem(x_matrix, 3'd5, 2'd0, real_to_q16(1.0));
        x_matrix = set_mat_elem(x_matrix, 3'd5, 2'd1, real_to_q16(0.4));
        x_matrix = set_mat_elem(x_matrix, 3'd5, 2'd2, real_to_q16(1.0));
        x_matrix = set_mat_elem(x_matrix, 3'd5, 2'd3, real_to_q16(0.8));
        y_obs    = set_obs(y_obs, 3'd5, real_to_q16(4.0)); // 1.5*1 + 2.5*1 = 4.0

        @(posedge clk);
        num_params = 3'd4;
        num_obs    = 4'd6;
        w_init     = '0;
        lambda_reg = real_to_q16(0.5);
        tolerance  = 32'h0000_0020;
        max_iters  = 8'd50;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting Compressed Sensing Recovery...");
        @(posedge done);
        #1;

        $display("--> LASSO SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Cycles:         %0d", iter_count);
        $display("    w0_optimal:     %f (Expected: ~ 1.500000)", q16_to_real(get_vec(w_optimal, 2'd0)));
        $display("    w1_optimal:     %f (Expected:   0.000000)", q16_to_real(get_vec(w_optimal, 2'd1)));
        $display("    w2_optimal:     %f (Expected: ~ 2.500000)", q16_to_real(get_vec(w_optimal, 2'd2)));
        $display("    w3_optimal:     %f (Expected:   0.000000)", q16_to_real(get_vec(w_optimal, 2'd3)));
        $display("    sparsity_count: %0d / 4 zero coefficients", sparsity_count);

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(w_optimal, 2'd0)) > 1.40 &&
            get_vec(w_optimal, 2'd1) == Q16_ZERO &&
            q16_to_real(get_vec(w_optimal, 2'd2)) > 2.40 &&
            get_vec(w_optimal, 2'd3) == Q16_ZERO) begin
            $display("[TEST 3 PASSED] Successfully recovered sparse compressed sensing signal!");
        end else begin
            $display("[TEST 3 FAILED] Sparse recovery failed.");
        end

        $display("\n==================================================================");
        $display(" ALL LASSO HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
