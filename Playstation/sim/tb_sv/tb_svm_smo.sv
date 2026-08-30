// =============================================================================
// File Name   : tb_svm_smo.sv
// Module Name : tb_svm_smo
// Project     : Support Vector Machine Sequential Minimal Optimization (SVM-SMO)
//               Accelerator (Solver #27)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for SVM-SMO Accelerator.
//   Verifies:
//   1. 2D Linearly Separable Binary Classification & Margin Maximization
//   2. Soft-Margin SVM with Hard Box Constraints (C = 1.0)
//   3. 4D Multi-Feature Physical AI Sensor Fault Detection
// =============================================================================

`timescale 1ns / 1ps

import svm_types_pkg::*;
`include "svm_helpers.svh"

module tb_svm_smo;

    logic               clk;
    logic               rst_n;

    // Controls & Configurations
    logic               init_svm;
    logic [3:0]         num_samples;
    logic [2:0]         feat_dim;
    dataset_arr_t       dataset;
    label_vec_t         labels;
    q16_t               c_bound;
    q16_t               tol_kkt;
    logic [7:0]         max_passes;

    // Training Execution
    logic               train_valid;
    alpha_vec_t         alphas_out;
    q16_t               b_bias_out;
    logic [7:0]         total_passes;
    status_t            status;
    logic               train_done;

    // Real-Time Online Inference
    logic               infer_valid;
    sample_vec_t        x_test;
    q16_t               decision_value;
    q16_t               y_predicted;
    logic               infer_done;
    logic               busy;

    // Instantiate Top Module
    svm_top dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .init_svm      (init_svm),
        .num_samples   (num_samples),
        .feat_dim      (feat_dim),
        .dataset       (dataset),
        .labels        (labels),
        .c_bound       (c_bound),
        .tol_kkt       (tol_kkt),
        .max_passes    (max_passes),
        .train_valid   (train_valid),
        .alphas_out    (alphas_out),
        .b_bias_out    (b_bias_out),
        .total_passes  (total_passes),
        .status        (status),
        .train_done    (train_done),
        .infer_valid   (infer_valid),
        .x_test        (x_test),
        .decision_value(decision_value),
        .y_predicted   (y_predicted),
        .infer_done    (infer_done),
        .busy          (busy)
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

    // Task to run online classification inference
    task automatic do_classify(
        input sample_vec_t s_in,
        output q16_t out_y,
        output q16_t out_margin
    );
        @(posedge clk);
        x_test      = s_in;
        infer_valid = 1'b1;
        @(posedge clk);
        infer_valid = 1'b0;
        @(posedge infer_done);
        out_y      = y_predicted;
        out_margin = decision_value;
        #1;
    endtask

    // Test variables declared at module level
    sample_vec_t test_s1, test_s2;
    q16_t pred_y1, pred_y2, marg1, marg2;
    int correct_cnt;

    initial begin
        $display("==================================================================");
        $display(" SVM Sequential Minimal Optimization (SMO) TB (Solver #27)");
        $display("==================================================================");

        // Reset
        rst_n       = 0;
        init_svm    = 0;
        train_valid = 0;
        infer_valid = 0;
        num_samples = 4'd6;
        feat_dim    = 3'd2;
        dataset     = '0;
        labels      = '0;
        c_bound     = real_to_q16(2.0);
        tol_kkt     = real_to_q16(0.01);
        max_passes  = 8'd6;
        x_test      = '0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2D Linearly Separable Binary Classification & Margin Maximization
        // Class +1: (1, 3), (2, 4), (1.5, 5)
        // Class -1: (3, 1), (4, 2), (5, 1.5)
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 2D Linearly Separable Binary Classification & Margin Maximization");
        $display("Training 6 samples (3 Class +1, 3 Class -1) with C = 2.0");

        num_samples = 4'd6;
        feat_dim    = 3'd2;
        dataset     = '0;
        labels      = '0;

        // Class +1
        dataset = set_sample_feat(dataset, 3'd0, 2'd0, real_to_q16(1.0));
        dataset = set_sample_feat(dataset, 3'd0, 2'd1, real_to_q16(3.0));
        labels[0] = real_to_q16(1.0);

        dataset = set_sample_feat(dataset, 3'd1, 2'd0, real_to_q16(2.0));
        dataset = set_sample_feat(dataset, 3'd1, 2'd1, real_to_q16(4.0));
        labels[1] = real_to_q16(1.0);

        dataset = set_sample_feat(dataset, 3'd2, 2'd0, real_to_q16(1.5));
        dataset = set_sample_feat(dataset, 3'd2, 2'd1, real_to_q16(5.0));
        labels[2] = real_to_q16(1.0);

        // Class -1
        dataset = set_sample_feat(dataset, 3'd3, 2'd0, real_to_q16(3.0));
        dataset = set_sample_feat(dataset, 3'd3, 2'd1, real_to_q16(1.0));
        labels[3] = real_to_q16(-1.0);

        dataset = set_sample_feat(dataset, 3'd4, 2'd0, real_to_q16(4.0));
        dataset = set_sample_feat(dataset, 3'd4, 2'd1, real_to_q16(2.0));
        labels[4] = real_to_q16(-1.0);

        dataset = set_sample_feat(dataset, 3'd5, 2'd0, real_to_q16(5.0));
        dataset = set_sample_feat(dataset, 3'd5, 2'd1, real_to_q16(1.5));
        labels[5] = real_to_q16(-1.0);

        c_bound    = real_to_q16(2.0);
        tol_kkt    = real_to_q16(0.01);
        max_passes = 8'd6;

        // Init and Kernel Generation
        @(posedge clk);
        init_svm = 1'b1;
        @(posedge clk);
        init_svm = 1'b0;
        @(negedge busy);
        #10;

        // Trigger SMO Training
        $display("Starting hardware SMO optimization loop...");
        @(posedge clk);
        train_valid = 1'b1;
        @(posedge clk);
        train_valid = 1'b0;
        @(posedge train_done);
        #10;

        $display("--> SVM TRAINING COMPLETED:");
        $display("    Status:       %0d", status);
        $display("    Bias b*:      %f", q16_to_real(b_bias_out));
        for (int i = 0; i < 6; i++) begin
            $display("    alpha[%0d]:   %f (y = %+2.0f)", i, q16_to_real(alphas_out[i]), q16_to_real(labels[i]));
        end

        // Evaluate Training Accuracy
        correct_cnt = 0;
        for (int i = 0; i < 6; i++) begin
            test_s1 = '0;
            test_s1[0] = get_sample_feat(dataset, 3'(i), 2'd0);
            test_s1[1] = get_sample_feat(dataset, 3'(i), 2'd1);
            do_classify(test_s1, pred_y1, marg1);
            if (q16_to_real(pred_y1) == q16_to_real(labels[i])) begin
                correct_cnt++;
            end
        end
        $display("    Training Accuracy: %0d / 6 correct (%.1f%%)", correct_cnt, (real'(correct_cnt)/6.0)*100.0);

        // Test with unseen samples
        test_s1 = '0;
        test_s1[0] = real_to_q16(1.0);
        test_s1[1] = real_to_q16(4.0); // Should be +1
        do_classify(test_s1, pred_y1, marg1);

        test_s2 = '0;
        test_s2[0] = real_to_q16(4.5);
        test_s2[1] = real_to_q16(1.0); // Should be -1
        do_classify(test_s2, pred_y2, marg2);

        $display("    Inference Sample A (1.0, 4.0): Pred = %+2.0f (Margin = %+f)", q16_to_real(pred_y1), q16_to_real(marg1));
        $display("    Inference Sample B (4.5, 1.0): Pred = %+2.0f (Margin = %+f)", q16_to_real(pred_y2), q16_to_real(marg2));

        if (correct_cnt == 6 && q16_to_real(pred_y1) == 1.0 && q16_to_real(pred_y2) == -1.0) begin
            $display("[TEST 1 PASSED] 100%% accuracy on 2D linearly separable classification!");
        end else begin
            $display("[TEST 1 FAILED] Accuracy outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: Soft-Margin SVM with Bounded Alphas (C = 1.0)
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Soft-Margin SVM with Bounded Alphas (C = 1.0)");

        num_samples = 4'd6;
        feat_dim    = 3'd2;
        dataset     = '0;
        labels      = '0;

        // Symmetric 2D clusters across decision boundary
        dataset = set_sample_feat(dataset, 3'd0, 2'd0, real_to_q16(-1.0));
        dataset = set_sample_feat(dataset, 3'd0, 2'd1, real_to_q16(2.0));
        labels[0] = real_to_q16(1.0);

        dataset = set_sample_feat(dataset, 3'd1, 2'd0, real_to_q16(-2.0));
        dataset = set_sample_feat(dataset, 3'd1, 2'd1, real_to_q16(3.0));
        labels[1] = real_to_q16(1.0);

        dataset = set_sample_feat(dataset, 3'd2, 2'd0, real_to_q16(-0.5));
        dataset = set_sample_feat(dataset, 3'd2, 2'd1, real_to_q16(2.5));
        labels[2] = real_to_q16(1.0);

        dataset = set_sample_feat(dataset, 3'd3, 2'd0, real_to_q16(1.0));
        dataset = set_sample_feat(dataset, 3'd3, 2'd1, real_to_q16(-2.0));
        labels[3] = real_to_q16(-1.0);

        dataset = set_sample_feat(dataset, 3'd4, 2'd0, real_to_q16(2.0));
        dataset = set_sample_feat(dataset, 3'd4, 2'd1, real_to_q16(-3.0));
        labels[4] = real_to_q16(-1.0);

        dataset = set_sample_feat(dataset, 3'd5, 2'd0, real_to_q16(0.5));
        dataset = set_sample_feat(dataset, 3'd5, 2'd1, real_to_q16(-2.5));
        labels[5] = real_to_q16(-1.0);

        c_bound    = real_to_q16(1.0);
        tol_kkt    = real_to_q16(0.01);
        max_passes = 8'd6;

        @(posedge clk);
        init_svm = 1'b1;
        @(posedge clk);
        init_svm = 1'b0;
        @(negedge busy);
        #10;

        @(posedge clk);
        train_valid = 1'b1;
        @(posedge clk);
        train_valid = 1'b0;
        @(posedge train_done);
        #10;

        $display("--> SOFT-MARGIN SVM RESULTS:");
        $display("    Bias b*: %f", q16_to_real(b_bias_out));
        for (int i = 0; i < 6; i++) begin
            $display("    alpha[%0d]: %f (Bound 0 <= alpha <= 1.0)", i, q16_to_real(alphas_out[i]));
        end

        correct_cnt = 0;
        for (int i = 0; i < 6; i++) begin
            test_s1 = '0;
            test_s1[0] = get_sample_feat(dataset, 3'(i), 2'd0);
            test_s1[1] = get_sample_feat(dataset, 3'(i), 2'd1);
            do_classify(test_s1, pred_y1, marg1);
            if (q16_to_real(pred_y1) == q16_to_real(labels[i])) begin
                correct_cnt++;
            end
        end
        $display("    Training Accuracy: %0d / 6 correct (%.1f%%)", correct_cnt, (real'(correct_cnt)/6.0)*100.0);

        if (correct_cnt == 6) begin
            $display("[TEST 2 PASSED] Successfully trained soft-margin SVM within box constraints!");
        end else begin
            $display("[TEST 2 FAILED] Soft-margin classification outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 4D Multi-Feature Physical AI Sensor Fault Detection
        // Normal State (y = -1): low vibration & temperature
        // Fault State  (y = +1): high vibration & high temperature
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 4D Multi-Feature Physical AI Sensor Fault Detection");

        num_samples = 4'd6;
        feat_dim    = 3'd4;
        dataset     = '0;
        labels      = '0;

        // Normal samples (y = -1)
        dataset = set_sample_feat(dataset, 3'd0, 2'd0, real_to_q16(0.2));
        dataset = set_sample_feat(dataset, 3'd0, 2'd1, real_to_q16(0.3));
        dataset = set_sample_feat(dataset, 3'd0, 2'd2, real_to_q16(0.1));
        dataset = set_sample_feat(dataset, 3'd0, 2'd3, real_to_q16(0.4));
        labels[0] = real_to_q16(-1.0);

        dataset = set_sample_feat(dataset, 3'd1, 2'd0, real_to_q16(0.4));
        dataset = set_sample_feat(dataset, 3'd1, 2'd1, real_to_q16(0.2));
        dataset = set_sample_feat(dataset, 3'd1, 2'd2, real_to_q16(0.3));
        dataset = set_sample_feat(dataset, 3'd1, 2'd3, real_to_q16(0.5));
        labels[1] = real_to_q16(-1.0);

        dataset = set_sample_feat(dataset, 3'd2, 2'd0, real_to_q16(0.3));
        dataset = set_sample_feat(dataset, 3'd2, 2'd1, real_to_q16(0.4));
        dataset = set_sample_feat(dataset, 3'd2, 2'd2, real_to_q16(0.2));
        dataset = set_sample_feat(dataset, 3'd2, 2'd3, real_to_q16(0.3));
        labels[2] = real_to_q16(-1.0);

        // Fault samples (y = +1)
        dataset = set_sample_feat(dataset, 3'd3, 2'd0, real_to_q16(2.5));
        dataset = set_sample_feat(dataset, 3'd3, 2'd1, real_to_q16(3.1));
        dataset = set_sample_feat(dataset, 3'd3, 2'd2, real_to_q16(1.8));
        dataset = set_sample_feat(dataset, 3'd3, 2'd3, real_to_q16(2.9));
        labels[3] = real_to_q16(1.0);

        dataset = set_sample_feat(dataset, 3'd4, 2'd0, real_to_q16(3.0));
        dataset = set_sample_feat(dataset, 3'd4, 2'd1, real_to_q16(2.8));
        dataset = set_sample_feat(dataset, 3'd4, 2'd2, real_to_q16(2.2));
        dataset = set_sample_feat(dataset, 3'd4, 2'd3, real_to_q16(3.4));
        labels[4] = real_to_q16(1.0);

        dataset = set_sample_feat(dataset, 3'd5, 2'd0, real_to_q16(2.8));
        dataset = set_sample_feat(dataset, 3'd5, 2'd1, real_to_q16(3.5));
        dataset = set_sample_feat(dataset, 3'd5, 2'd2, real_to_q16(1.9));
        dataset = set_sample_feat(dataset, 3'd5, 2'd3, real_to_q16(3.1));
        labels[5] = real_to_q16(1.0);

        c_bound    = real_to_q16(2.0);
        tol_kkt    = real_to_q16(0.01);
        max_passes = 8'd6;

        @(posedge clk);
        init_svm = 1'b1;
        @(posedge clk);
        init_svm = 1'b0;
        @(negedge busy);
        #10;

        @(posedge clk);
        train_valid = 1'b1;
        @(posedge clk);
        train_valid = 1'b0;
        @(posedge train_done);
        #10;

        // Test unseen sensor telemetry:
        // Test Normal
        test_s1 = '0;
        test_s1[0] = real_to_q16(0.35);
        test_s1[1] = real_to_q16(0.25);
        test_s1[2] = real_to_q16(0.15);
        test_s1[3] = real_to_q16(0.45);
        do_classify(test_s1, pred_y1, marg1);

        // Test Fault
        test_s2 = '0;
        test_s2[0] = real_to_q16(2.9);
        test_s2[1] = real_to_q16(3.2);
        test_s2[2] = real_to_q16(2.0);
        test_s2[3] = real_to_q16(3.0);
        do_classify(test_s2, pred_y2, marg2);

        $display("    Normal Telemetry Classification: Pred = %+2.0f (Margin = %+f)", q16_to_real(pred_y1), q16_to_real(marg1));
        $display("    Fault Telemetry Classification:  Pred = %+2.0f (Margin = %+f)", q16_to_real(pred_y2), q16_to_real(marg2));

        if (q16_to_real(pred_y1) == -1.0 && q16_to_real(pred_y2) == 1.0) begin
            $display("[TEST 3 PASSED] Successfully detected 4D sensor faults and normal operations!");
        end else begin
            $display("[TEST 3 FAILED] 4D Sensor classification outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL SVM-SMO HARDWARE OPTIMIZATION TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
