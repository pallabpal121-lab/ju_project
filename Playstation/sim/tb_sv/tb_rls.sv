// =============================================================================
// File Name   : tb_rls.sv
// Module Name : tb_rls
// Project     : Recursive Least Squares (RLS) Accelerator (Solver #23)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for RLS Adaptive Filtering Accelerator.
//   Verifies:
//   1. 2-Tap Real-Time Adaptive System Identification (FIR Plant Estimation)
//   2. 3-Tap Acoustic Echo / Noise Cancellation (Exponential Forgetting λ = 0.95)
//   3. 4-Tap Multi-Axis Physical Sensor Calibration
// =============================================================================

`timescale 1ns / 1ps

import rls_types_pkg::*;
`include "rls_helpers.svh"

module tb_rls;

    logic               clk;
    logic               rst_n;

    // Controls & Inputs
    logic               init_rls;
    logic [2:0]         num_taps;
    q16_t               lambda_factor;
    q16_t               delta_init;
    vec_t               w_init;

    logic               sample_valid;
    vec_t               x_sample;
    q16_t               d_meas;

    // Results & Outputs
    vec_t               w_optimal;
    mat_t               p_matrix;
    q16_t               a_priori_err;
    vec_t               k_gain_out;
    status_t            status;
    logic               sample_done;
    logic               busy;

    // Instantiate Top Module
    rls_top dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .init_rls     (init_rls),
        .num_taps     (num_taps),
        .lambda_factor(lambda_factor),
        .delta_init   (delta_init),
        .w_init       (w_init),
        .sample_valid (sample_valid),
        .x_sample     (x_sample),
        .d_meas       (d_meas),
        .w_optimal    (w_optimal),
        .p_matrix     (p_matrix),
        .a_priori_err (a_priori_err),
        .k_gain_out   (k_gain_out),
        .status       (status),
        .sample_done  (sample_done),
        .busy         (busy)
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

    // Task to feed a single sample and wait for update
    task automatic feed_sample(input vec_t x, input q16_t d);
        @(posedge clk);
        x_sample     = x;
        d_meas       = d;
        sample_valid = 1'b1;
        @(posedge clk);
        sample_valid = 1'b0;
        @(posedge sample_done);
        #1;
    endtask

    vec_t x_in;
    q16_t d_in;

    initial begin
        $display("==================================================================");
        $display(" Recursive Least Squares (RLS) Accelerator TB (Solver #23)");
        $display("==================================================================");

        // Reset
        rst_n         = 0;
        init_rls      = 0;
        num_taps      = 3'd2;
        lambda_factor = Q16_ONE;
        delta_init    = real_to_q16(100.0);
        w_init        = '0;
        sample_valid  = 0;
        x_sample      = '0;
        d_meas        = Q16_ZERO;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2-Tap Real-Time Adaptive System Identification (FIR Plant)
        // Unknown plant: d_t = 1.5 * x0_t - 2.5 * x1_t
        // Target: w* = [1.500000, -2.500000]
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 2-Tap Real-Time Adaptive System Identification (FIR Plant)");
        $display("True Plant Model: d_t = 1.5 * x0_t - 2.5 * x1_t");
        $display("Target Tap Weights: w* = [1.500000, -2.500000]");

        @(posedge clk);
        num_taps      = 3'd2;
        lambda_factor = real_to_q16(1.0);
        delta_init    = real_to_q16(100.0);
        w_init        = '0;
        init_rls      = 1'b1;
        @(posedge clk);
        init_rls      = 1'b0;

        // Wait until RLS is initialized
        @(posedge clk);
        while (busy) @(posedge clk);

        $display("Streaming online training samples into RLS pipeline...");

        // Sample 1: x = [1.0, 0.0], d = 1.5
        x_in = '0; x_in[0] = real_to_q16(1.0); x_in[1] = real_to_q16(0.0);
        d_in = real_to_q16(1.5);
        feed_sample(x_in, d_in);
        $display("  Sample 1 Processed: w = [%f, %f], err = %f",
                 q16_to_real(w_optimal[0]), q16_to_real(w_optimal[1]), q16_to_real(a_priori_err));

        // Sample 2: x = [0.0, 1.0], d = -2.5
        x_in = '0; x_in[0] = real_to_q16(0.0); x_in[1] = real_to_q16(1.0);
        d_in = real_to_q16(-2.5);
        feed_sample(x_in, d_in);
        $display("  Sample 2 Processed: w = [%f, %f], err = %f",
                 q16_to_real(w_optimal[0]), q16_to_real(w_optimal[1]), q16_to_real(a_priori_err));

        // Sample 3: x = [1.0, 1.0], d = -1.0
        x_in = '0; x_in[0] = real_to_q16(1.0); x_in[1] = real_to_q16(1.0);
        d_in = real_to_q16(-1.0);
        feed_sample(x_in, d_in);
        $display("  Sample 3 Processed: w = [%f, %f], err = %f",
                 q16_to_real(w_optimal[0]), q16_to_real(w_optimal[1]), q16_to_real(a_priori_err));

        // Sample 4: x = [2.0, -1.0], d = 5.5
        x_in = '0; x_in[0] = real_to_q16(2.0); x_in[1] = real_to_q16(-1.0);
        d_in = real_to_q16(5.5);
        feed_sample(x_in, d_in);
        $display("  Sample 4 Processed: w = [%f, %f], err = %f",
                 q16_to_real(w_optimal[0]), q16_to_real(w_optimal[1]), q16_to_real(a_priori_err));

        // Sample 5: x = [-1.0, 2.0], d = -6.5
        x_in = '0; x_in[0] = real_to_q16(-1.0); x_in[1] = real_to_q16(2.0);
        d_in = real_to_q16(-6.5);
        feed_sample(x_in, d_in);
        $display("  Sample 5 Processed: w = [%f, %f], err = %f",
                 q16_to_real(w_optimal[0]), q16_to_real(w_optimal[1]), q16_to_real(a_priori_err));

        // Sample 6: x = [0.5, 0.5], d = -0.5
        x_in = '0; x_in[0] = real_to_q16(0.5); x_in[1] = real_to_q16(0.5);
        d_in = real_to_q16(-0.5);
        feed_sample(x_in, d_in);
        $display("  Sample 6 Processed: w = [%f, %f], err = %f",
                 q16_to_real(w_optimal[0]), q16_to_real(w_optimal[1]), q16_to_real(a_priori_err));

        $display("--> RLS 2-TAP IDENTIFICATION RESULTS:");
        $display("    w0_estimated: %f (Expected: ~ 1.500000)", q16_to_real(w_optimal[0]));
        $display("    w1_estimated: %f (Expected: ~ -2.500000)", q16_to_real(w_optimal[1]));
        $display("    Final Error:  %f", q16_to_real(a_priori_err));

        if (q16_to_real(w_optimal[0]) > 1.48 && q16_to_real(w_optimal[0]) < 1.52 &&
            q16_to_real(w_optimal[1]) > -2.52 && q16_to_real(w_optimal[1]) < -2.48) begin
            $display("[TEST 1 PASSED] Successfully identified 2-tap FIR plant weights!");
        end else begin
            $display("[TEST 1 FAILED] RLS 2-tap identification outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 3-Tap Acoustic Echo / Noise Cancellation (λ = 0.95)
        // Unknown echo filter: w* = [0.800000, -1.200000, 2.000000]
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 3-Tap Acoustic Echo / Noise Cancellation (λ = 0.95)");
        $display("Target Echo Filter Weights: w* = [0.800000, -1.200000, 2.000000]");

        @(posedge clk);
        num_taps      = 3'd3;
        lambda_factor = real_to_q16(0.95);
        delta_init    = real_to_q16(100.0);
        w_init        = '0;
        init_rls      = 1'b1;
        @(posedge clk);
        init_rls      = 1'b0;

        @(posedge clk);
        while (busy) @(posedge clk);

        // Stream 20 persistent excitation samples
        for (int i = 0; i < 20; i++) begin
            x_in = '0;
            x_in[0] = real_to_q16(1.0 + 0.5 * real'((i * 7 + 3) % 5) - 1.0);
            x_in[1] = real_to_q16(-0.5 + 0.4 * real'((i * 11 + 2) % 7) - 1.2);
            x_in[2] = real_to_q16(0.8 - 0.3 * real'((i * 13 + 5) % 6) + 0.5);
            // Ensure non-zero energy
            if (x_in[0] == 0 && x_in[1] == 0 && x_in[2] == 0) x_in[0] = real_to_q16(1.0);
            d_in = real_to_q16(0.8 * q16_to_real(x_in[0]) - 1.2 * q16_to_real(x_in[1]) + 2.0 * q16_to_real(x_in[2]));
            feed_sample(x_in, d_in);
        end

        $display("--> RLS 3-TAP ECHO CANCELLATION RESULTS:");
        $display("    w0_estimated: %f (Expected: ~ 0.800000)", q16_to_real(w_optimal[0]));
        $display("    w1_estimated: %f (Expected: ~ -1.200000)", q16_to_real(w_optimal[1]));
        $display("    w2_estimated: %f (Expected: ~ 2.000000)", q16_to_real(w_optimal[2]));
        $display("    Final Error:  %f", q16_to_real(a_priori_err));

        if (q16_to_real(w_optimal[0]) > 0.78 && q16_to_real(w_optimal[0]) < 0.82 &&
            q16_to_real(w_optimal[1]) > -1.22 && q16_to_real(w_optimal[1]) < -1.18 &&
            q16_to_real(w_optimal[2]) > 1.98 && q16_to_real(w_optimal[2]) < 2.02) begin
            $display("[TEST 2 PASSED] Successfully cancelled acoustic echo with adaptive RLS!");
        end else begin
            $display("[TEST 2 FAILED] RLS 3-tap echo cancellation outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 4-Tap Multi-Axis Physical Sensor Calibration
        // Target: w* = [0.500000, 1.000000, -0.500000, 2.500000]
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 4-Tap Multi-Axis Physical Sensor Calibration");
        $display("Target Calibration Weights: w* = [0.500000, 1.000000, -0.500000, 2.500000]");

        @(posedge clk);
        num_taps      = 3'd4;
        lambda_factor = real_to_q16(0.98);
        delta_init    = real_to_q16(100.0);
        w_init        = '0;
        init_rls      = 1'b1;
        @(posedge clk);
        init_rls      = 1'b0;

        @(posedge clk);
        while (busy) @(posedge clk);

        // Stream 25 persistent excitation samples
        for (int i = 0; i < 25; i++) begin
            x_in = '0;
            x_in[0] = real_to_q16(1.0 + 0.3 * real'((i * 3 + 1) % 5) - 0.6);
            x_in[1] = real_to_q16(0.5 - 0.4 * real'((i * 7 + 4) % 6) + 0.8);
            x_in[2] = real_to_q16(-1.0 + 0.5 * real'((i * 11 + 2) % 7) - 1.0);
            x_in[3] = real_to_q16(0.2 + 0.3 * real'((i * 17 + 3) % 5) - 0.5);
            if (x_in[0] == 0 && x_in[1] == 0 && x_in[2] == 0 && x_in[3] == 0) x_in[0] = real_to_q16(1.0);
            d_in = real_to_q16(0.5 * q16_to_real(x_in[0]) + 1.0 * q16_to_real(x_in[1]) - 0.5 * q16_to_real(x_in[2]) + 2.5 * q16_to_real(x_in[3]));
            feed_sample(x_in, d_in);
        end

        $display("--> RLS 4-TAP SENSOR CALIBRATION RESULTS:");
        $display("    w0_estimated: %f (Expected: ~ 0.500000)", q16_to_real(w_optimal[0]));
        $display("    w1_estimated: %f (Expected: ~ 1.000000)", q16_to_real(w_optimal[1]));
        $display("    w2_estimated: %f (Expected: ~ -0.500000)", q16_to_real(w_optimal[2]));
        $display("    w3_estimated: %f (Expected: ~ 2.500000)", q16_to_real(w_optimal[3]));
        $display("    Final Error:  %f", q16_to_real(a_priori_err));

        if (q16_to_real(w_optimal[0]) > 0.48 && q16_to_real(w_optimal[0]) < 0.52 &&
            q16_to_real(w_optimal[1]) > 0.98 && q16_to_real(w_optimal[1]) < 1.02 &&
            q16_to_real(w_optimal[2]) > -0.52 && q16_to_real(w_optimal[2]) < -0.48 &&
            q16_to_real(w_optimal[3]) > 2.48 && q16_to_real(w_optimal[3]) < 2.52) begin
            $display("[TEST 3 PASSED] Successfully calibrated 4-axis physical sensor!");
        end else begin
            $display("[TEST 3 FAILED] RLS 4-tap sensor calibration outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL RLS ADAPTIVE FILTERING HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
