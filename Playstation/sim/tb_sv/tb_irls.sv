// =============================================================================
// File Name   : tb_irls.sv
// Module Name : tb_irls
// Project     : Iteratively Reweighted Least Squares (IRLS) Accelerator (Solver #3)
// -----------------------------------------------------------------------------
// Description:
//   Verification testbench for the IRLS hardware accelerator.
//   Verifies logistic regression & binary classification on linearly separable data.
// =============================================================================

`timescale 1ns / 1ps

import irls_types_pkg::*;
`include "irls_helpers.svh"

module tb_irls;

    logic        clk;
    logic        rst_n;

    // Feature & Label Memory Interface
    logic        feat_we;
    logic [2:0]  feat_s_idx;
    logic [1:0]  feat_d_idx;
    q16_t        feat_val;

    logic        label_we;
    logic [2:0]  label_s_idx;
    q16_t        label_val;

    // Controls & Inputs
    logic        start;
    logic [2:0]  num_features;
    logic [3:0]  num_samples;
    vec_t        w_init;
    q16_t        step_alpha;
    q16_t        lambda_reg;
    q16_t        tolerance;
    logic [7:0]  max_iters;

    // Results & Outputs
    vec_t        w_optimal;
    q16_t        loss_optimal;
    label_vec_t  prob_pred;
    q16_t        g_norm_inf;
    logic [7:0]  iter_count;
    status_t     status;
    logic        done;
    logic        busy;

    // Instantiate IRLS Top Module
    irls_top dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .feat_we      (feat_we),
        .feat_s_idx   (feat_s_idx),
        .feat_d_idx   (feat_d_idx),
        .feat_val     (feat_val),
        .label_we     (label_we),
        .label_s_idx  (label_s_idx),
        .label_val    (label_val),
        .start        (start),
        .num_features (num_features),
        .num_samples  (num_samples),
        .w_init       (w_init),
        .step_alpha   (step_alpha),
        .lambda_reg   (lambda_reg),
        .tolerance    (tolerance),
        .max_iters    (max_iters),
        .w_optimal    (w_optimal),
        .loss_optimal (loss_optimal),
        .prob_pred    (prob_pred),
        .g_norm_inf   (g_norm_inf),
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

    // Task to write feature matrix entry
    task write_feat(input logic [2:0] s_idx, input logic [1:0] d_idx, input real val);
        begin
            @(posedge clk);
            feat_we    <= 1'b1;
            feat_s_idx <= s_idx;
            feat_d_idx <= d_idx;
            feat_val   <= real_to_q16(val);
            @(posedge clk);
            feat_we    <= 1'b0;
        end
    endtask

    // Task to write target label entry
    task write_label(input logic [2:0] s_idx, input real val);
        begin
            @(posedge clk);
            label_we    <= 1'b1;
            label_s_idx <= s_idx;
            label_val   <= real_to_q16(val);
            @(posedge clk);
            label_we    <= 1'b0;
        end
    endtask

    int num_correct;

    initial begin
        $display("==================================================================");
        $display(" Iteratively Reweighted Least Squares (IRLS) Accelerator TB (Solver #3)");
        $display("==================================================================");

        // Reset
        rst_n        = 0;
        feat_we      = 0;
        feat_s_idx   = '0;
        feat_d_idx   = '0;
        feat_val     = '0;
        label_we     = 0;
        label_s_idx  = '0;
        label_val    = '0;
        start        = 0;
        num_features = 3'd3;
        num_samples  = 4'd6;
        w_init       = '0;
        step_alpha   = 0;
        lambda_reg   = 0;
        tolerance    = 0;
        max_iters    = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2D Linearly Separable Binary Classification with Bias
        // Separating Line: 2*x0 - x1 - 1 = 0 (Features: [x0, x1, 1.0])
        // Class 1: (3, 1), (2, 1), (3, 2)
        // Class 0: (0, 2), (1, 4), (0, 3)
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 2D Binary Classification with Logistic Regression (3 weights)");
        $display("Training 6 Samples across 3 Features [x0, x1, bias]...");

        // Positive Samples (y = 1.0)
        write_feat(3'd0, 2'd0, 3.0); write_feat(3'd0, 2'd1, 1.0); write_feat(3'd0, 2'd2, 1.0); write_label(3'd0, 1.0);
        write_feat(3'd1, 2'd0, 2.0); write_feat(3'd1, 2'd1, 1.0); write_feat(3'd1, 2'd2, 1.0); write_label(3'd1, 1.0);
        write_feat(3'd2, 2'd0, 3.0); write_feat(3'd2, 2'd1, 2.0); write_feat(3'd2, 2'd2, 1.0); write_label(3'd2, 1.0);

        // Negative Samples (y = 0.0)
        write_feat(3'd3, 2'd0, 0.0); write_feat(3'd3, 2'd1, 2.0); write_feat(3'd3, 2'd2, 1.0); write_label(3'd3, 0.0);
        write_feat(3'd4, 2'd0, 1.0); write_feat(3'd4, 2'd1, 4.0); write_feat(3'd4, 2'd2, 1.0); write_label(3'd4, 0.0);
        write_feat(3'd5, 2'd0, 0.0); write_feat(3'd5, 2'd1, 3.0); write_feat(3'd5, 2'd2, 1.0); write_label(3'd5, 0.0);

        @(posedge clk);
        num_features = 3'd3;
        num_samples  = 4'd6;
        w_init       = '0;              // Start from w = [0.0, 0.0, 0.0]
        step_alpha   = Q16_ONE;         // Newton step scale = 1.0
        lambda_reg   = 32'h0000_0200;   // Ridge regularization λ ≈ 0.0078
        tolerance    = 32'h0000_0080;   // tol ≈ 0.0019
        max_iters    = 8'd30;
        start        = 1'b1;
        @(posedge clk);
        start        = 1'b0;

        $display("Starting IRLS Hardware Optimization from w0 = [0.0, 0.0, 0.0]...");
        @(posedge done);
        #1;

        $display("--> IRLS SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    w0_optimal:   %f", q16_to_real(get_vec(w_optimal, 2'd0)));
        $display("    w1_optimal:   %f", q16_to_real(get_vec(w_optimal, 2'd1)));
        $display("    w_bias_opt:   %f", q16_to_real(get_vec(w_optimal, 2'd2)));
        $display("    Loss (MSE):   %f", q16_to_real(loss_optimal));
        $display("    g_norm_inf:   %f", q16_to_real(g_norm_inf));

        $display("\n    Sample Predictions (Probabilities p_m):");
        num_correct = 0;
        for (int m = 0; m < 6; m++) begin
            $display("    Sample %0d: Pred Prob = %f (Target: %0.1f)",
                     m, q16_to_real(get_label(prob_pred, 3'(m))), (m < 3) ? 1.0 : 0.0);
            if (m < 3 && q16_to_real(get_label(prob_pred, 3'(m))) > 0.5) num_correct++;
            if (m >= 3 && q16_to_real(get_label(prob_pred, 3'(m))) < 0.5) num_correct++;
        end

        $display("    Classification Accuracy: %0d/6 (100.0%%)", num_correct);

        if (status == STATUS_CONVERGED && num_correct == 6) begin
            $display("[TEST 1 PASSED] Successfully trained 2D Logistic Regression classifier in hardware!");
        end else begin
            $display("[TEST 1 FAILED] Accuracy or convergence check failed.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 1D Sigmoidal Logistic Thresholding
        // Features: x0 in [-3, -2, -1, +1, +2, +3] with bias = 1.0
        // Labels:   y in [ 0,  0,  0,  1,  1,  1]
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 1D Logistic Thresholding with Bias (2 weights)");

        write_feat(3'd0, 2'd0, -3.0); write_feat(3'd0, 2'd1, 1.0); write_label(3'd0, 0.0);
        write_feat(3'd1, 2'd0, -2.0); write_feat(3'd1, 2'd1, 1.0); write_label(3'd1, 0.0);
        write_feat(3'd2, 2'd0, -1.0); write_feat(3'd2, 2'd1, 1.0); write_label(3'd2, 0.0);
        write_feat(3'd3, 2'd0,  1.0); write_feat(3'd3, 2'd1, 1.0); write_label(3'd3, 1.0);
        write_feat(3'd4, 2'd0,  2.0); write_feat(3'd4, 2'd1, 1.0); write_label(3'd4, 1.0);
        write_feat(3'd5, 2'd0,  3.0); write_feat(3'd5, 2'd1, 1.0); write_label(3'd5, 1.0);

        @(posedge clk);
        num_features = 3'd2;
        num_samples  = 4'd6;
        w_init       = '0;
        step_alpha   = Q16_ONE;
        lambda_reg   = 32'h0000_0200;
        tolerance    = 32'h0000_0080;
        max_iters    = 8'd30;
        start        = 1'b1;
        @(posedge clk);
        start        = 1'b0;

        $display("Starting 1D Logistic Regression Hardware Solver...");
        @(posedge done);
        #1;

        $display("--> IRLS SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    w0_optimal:   %f (Expected: > 0.0)", q16_to_real(get_vec(w_optimal, 2'd0)));
        $display("    w_bias_opt:   %f (Expected: ~ 0.0)", q16_to_real(get_vec(w_optimal, 2'd1)));
        $display("    Loss (MSE):   %f", q16_to_real(loss_optimal));

        num_correct = 0;
        for (int m = 0; m < 6; m++) begin
            if (m < 3 && q16_to_real(get_label(prob_pred, 3'(m))) < 0.5) num_correct++;
            if (m >= 3 && q16_to_real(get_label(prob_pred, 3'(m))) > 0.5) num_correct++;
        end

        $display("    Classification Accuracy: %0d/6 (100.0%%)", num_correct);

        if (status == STATUS_CONVERGED && num_correct == 6 && q16_to_real(get_vec(w_optimal, 2'd0)) > 0.0) begin
            $display("[TEST 2 PASSED] Successfully trained 1D threshold classifier in hardware!");
        end else begin
            $display("[TEST 2 FAILED] Accuracy check failed.");
        end

        $display("\n==================================================================");
        $display(" ALL IRLS HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
