// =============================================================================
// File Name   : tb_ukf.sv
// Module Name : tb_ukf
// Project     : Unscented Kalman Filter (UKF) Accelerator (Solver #25)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for UKF Accelerator.
//   Verifies:
//   1. 2D Non-Linear Radar Range & Bearing Tracking (Range & Azimuth)
//   2. 2D Autonomous Vehicle Polar Range Kinematics
//   3. 4D Multi-Sensor Kinematic Target Tracking (Pos & Vel, 9 Sigma Points)
// =============================================================================

`timescale 1ns / 1ps

import ukf_types_pkg::*;
`include "ukf_helpers.svh"

module tb_ukf;

    logic               clk;
    logic               rst_n;

    // Controls & Configurations
    logic               init_ukf;
    logic [2:0]         state_dim;
    logic [1:0]         meas_dim;
    state_vec_t         x_init;
    state_mat_t         p_init;
    q16_t               gamma_sq;
    weights_arr_t       weights_m;
    weights_arr_t       weights_c;

    // Sigma Points Interface
    logic               gen_sigmas_valid;
    sigma_state_arr_t   sigma_points_out;
    logic               sigmas_ready;

    // Predict Interface
    logic               predict_valid;
    sigma_state_arr_t   sigma_propagated;
    state_mat_t         q_mat;
    state_vec_t         x_prior_out;
    state_mat_t         p_prior_out;
    logic               predict_done;

    // Correct Interface
    logic               correct_valid;
    sigma_meas_arr_t    sigma_meas_in;
    meas_vec_t          z_meas;
    innov_mat_t         r_mat;
    state_vec_t         x_estimated;
    state_mat_t         p_covariance;
    meas_vec_t          innov_residual;
    cross_mat_t         k_gain_out;
    status_t            status;
    logic               correct_done;
    logic               busy;

    // Instantiate Top Module
    ukf_top dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .init_ukf        (init_ukf),
        .state_dim       (state_dim),
        .meas_dim        (meas_dim),
        .x_init          (x_init),
        .p_init          (p_init),
        .gamma_sq        (gamma_sq),
        .weights_m       (weights_m),
        .weights_c       (weights_c),
        .gen_sigmas_valid(gen_sigmas_valid),
        .sigma_points_out(sigma_points_out),
        .sigmas_ready    (sigmas_ready),
        .predict_valid   (predict_valid),
        .sigma_propagated(sigma_propagated),
        .q_mat           (q_mat),
        .x_prior_out     (x_prior_out),
        .p_prior_out     (p_prior_out),
        .predict_done    (predict_done),
        .correct_valid   (correct_valid),
        .sigma_meas_in   (sigma_meas_in),
        .z_meas          (z_meas),
        .r_mat           (r_mat),
        .x_estimated     (x_estimated),
        .p_covariance    (p_covariance),
        .innov_residual  (innov_residual),
        .k_gain_out      (k_gain_out),
        .status          (status),
        .correct_done    (correct_done),
        .busy            (busy)
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

    // Task to generate sigma points
    task automatic do_gen_sigmas();
        @(posedge clk);
        gen_sigmas_valid = 1'b1;
        @(posedge clk);
        gen_sigmas_valid = 1'b0;
        @(posedge sigmas_ready);
        #1;
    endtask

    // Task for predict step
    task automatic do_predict(
        input sigma_state_arr_t sigmas_f,
        input state_mat_t q_noise
    );
        @(posedge clk);
        sigma_propagated = sigmas_f;
        q_mat            = q_noise;
        predict_valid    = 1'b1;
        @(posedge clk);
        predict_valid    = 1'b0;
        @(posedge predict_done);
        #1;
    endtask

    // Task for correct step
    task automatic do_correct(
        input sigma_meas_arr_t sigmas_h,
        input meas_vec_t z_raw,
        input innov_mat_t r_noise
    );
        @(posedge clk);
        sigma_meas_in = sigmas_h;
        z_meas        = z_raw;
        r_mat         = r_noise;
        correct_valid = 1'b1;
        @(posedge clk);
        correct_valid = 1'b0;
        @(posedge correct_done);
        #1;
    endtask

    // Test variables declared at module level
    real true_px, true_py, true_vx, true_vy, dt;
    real r_val, th_val, s_px, s_py;
    sigma_state_arr_t sig_f_in;
    sigma_meas_arr_t  sig_h_in;
    state_mat_t q_m;
    innov_mat_t r_m;
    meas_vec_t  z_in;

    initial begin
        $display("==================================================================");
        $display(" Unscented Kalman Filter (UKF) Accelerator TB (Solver #25)");
        $display("==================================================================");

        // Reset
        rst_n            = 0;
        init_ukf         = 0;
        gen_sigmas_valid = 0;
        predict_valid    = 0;
        correct_valid    = 0;
        state_dim        = 3'd2;
        meas_dim         = 2'd2;
        x_init           = '0;
        p_init           = '0;
        gamma_sq         = real_to_q16(3.0);
        weights_m        = '0;
        weights_c        = '0;
        sigma_propagated = '0;
        q_mat            = '0;
        sigma_meas_in    = '0;
        z_meas           = '0;
        r_mat            = '0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2D Highly Non-Linear Radar Range & Bearing Tracking (Range & Azimuth)
        // Target: px(t) = 4.0 + 0.5*t, py(t) = 8.0 + 0.8*t
        // Measurements: r = sqrt(px^2 + py^2), th = py / px
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 2D Non-Linear Radar Range & Bearing Tracking (Range & Azimuth)");
        $display("True Kinematics: p_x(t) = 4.0 + 0.5*t, p_y(t) = 8.0 + 0.8*t");

        dt = 0.5;
        state_dim = 3'd2;
        meas_dim  = 2'd2;

        x_init = '0;
        x_init[0] = real_to_q16(3.5);
        x_init[1] = real_to_q16(7.5);

        p_init = '0;
        p_init = set_smat(p_init, 2'd0, 2'd0, real_to_q16(4.0));
        p_init = set_smat(p_init, 2'd1, 2'd1, real_to_q16(4.0));

        gamma_sq = real_to_q16(3.0);

        // Standard UT weights for N=2, gamma_sq = 3.0:
        // W0_m = 0.333333, W0_c = 0.333333
        // Wi_m = Wi_c = 0.166667 (i = 1..4)
        weights_m = '0;
        weights_c = '0;
        weights_m[0] = real_to_q16(0.333333);
        weights_c[0] = real_to_q16(0.333333);
        for (int i = 1; i <= 4; i++) begin
            weights_m[i] = real_to_q16(0.166667);
            weights_c[i] = real_to_q16(0.166667);
        end

        @(posedge clk);
        init_ukf = 1'b1;
        @(posedge clk);
        init_ukf = 1'b0;
        #10;

        q_m = '0;
        q_m = set_smat(q_m, 2'd0, 2'd0, real_to_q16(0.01));
        q_m = set_smat(q_m, 2'd1, 2'd1, real_to_q16(0.01));

        r_m = '0;
        r_m = set_imat(r_m, 1'b0, 1'b0, real_to_q16(0.04));
        r_m = set_imat(r_m, 1'b1, 1'b1, real_to_q16(0.04));

        true_px = 4.0;
        true_py = 8.0;

        for (int k = 1; k <= 8; k++) begin
            true_px = true_px + 0.5 * dt;
            true_py = true_py + 0.8 * dt;

            // 1. Generate Sigmas
            do_gen_sigmas();

            // 2. Propagate state sigmas with motion model [0.25, 0.40]
            sig_f_in = '0;
            for (int p = 0; p < 5; p++) begin
                sig_f_in = set_sigma_state(sig_f_in, 4'(p), 2'd0,
                                           real_to_q16(q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd0)) + 0.25));
                sig_f_in = set_sigma_state(sig_f_in, 4'(p), 2'd1,
                                           real_to_q16(q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd1)) + 0.40));
            end
            do_predict(sig_f_in, q_m);

            // 3. Propagate measurement sigmas: r = sqrt(px^2 + py^2), th = py / px
            sig_h_in = '0;
            for (int p = 0; p < 5; p++) begin
                s_px = q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd0)) + 0.25;
                s_py = q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd1)) + 0.40;
                if (s_px < 0.1) s_px = 0.1;
                sig_h_in = set_sigma_meas(sig_h_in, 4'(p), 1'b0, real_to_q16($sqrt(s_px * s_px + s_py * s_py)));
                sig_h_in = set_sigma_meas(sig_h_in, 4'(p), 1'b1, real_to_q16(s_py / s_px));
            end

            // Actual measurement with slight noise
            z_in = '0;
            z_in[0] = real_to_q16($sqrt(true_px * true_px + true_py * true_py) + 0.02 * real'((k % 2) ? 1 : -1));
            z_in[1] = real_to_q16((true_py / true_px) - 0.01 * real'((k % 2) ? 1 : -1));

            do_correct(sig_h_in, z_in, r_m);

            $display("  Step %0d: True [px,py] = [%4.2f, %4.2f] | Est [px,py] = [%4.2f, %4.2f]",
                     k, true_px, true_py, q16_to_real(x_estimated[0]), q16_to_real(x_estimated[1]));
        end

        $display("--> UKF RADAR TRACKING RESULTS:");
        $display("    True Position:      px = %f, py = %f", true_px, true_py);
        $display("    Estimated Position: px = %f, py = %f", q16_to_real(x_estimated[0]), q16_to_real(x_estimated[1]));

        if (q16_to_real(x_estimated[0]) > true_px - 0.3 && q16_to_real(x_estimated[0]) < true_px + 0.3 &&
            q16_to_real(x_estimated[1]) > true_py - 0.3 && q16_to_real(x_estimated[1]) < true_py + 0.3) begin
            $display("[TEST 1 PASSED] Successfully tracked non-linear range-bearing kinematics via UKF!");
        end else begin
            $display("[TEST 1 FAILED] UKF Radar tracking outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 2D Autonomous Vehicle Kinematic Motion
        // Target: px = 6.0, py = 8.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 2D Autonomous Vehicle Kinematic Motion");
        $display("Target Kinematics: vx = 1.5 m/s, vy = 2.0 m/s");

        state_dim = 3'd2;
        meas_dim  = 2'd2;

        x_init = '0;
        x_init[0] = real_to_q16(0.0);
        x_init[1] = real_to_q16(0.0);

        p_init = '0;
        p_init = set_smat(p_init, 2'd0, 2'd0, real_to_q16(3.0));
        p_init = set_smat(p_init, 2'd1, 2'd1, real_to_q16(3.0));

        gamma_sq = real_to_q16(3.0);

        @(posedge clk);
        init_ukf = 1'b1;
        @(posedge clk);
        init_ukf = 1'b0;
        #10;

        q_m = '0;
        q_m = set_smat(q_m, 2'd0, 2'd0, real_to_q16(0.02));
        q_m = set_smat(q_m, 2'd1, 2'd1, real_to_q16(0.02));

        r_m = '0;
        r_m = set_imat(r_m, 1'b0, 1'b0, real_to_q16(0.08));
        r_m = set_imat(r_m, 1'b1, 1'b1, real_to_q16(0.08));

        true_px = 0.0;
        true_py = 0.0;

        for (int k = 1; k <= 8; k++) begin
            true_px = true_px + 1.5 * 0.5;
            true_py = true_py + 2.0 * 0.5;

            do_gen_sigmas();

            sig_f_in = '0;
            for (int p = 0; p < 5; p++) begin
                sig_f_in = set_sigma_state(sig_f_in, 4'(p), 2'd0,
                                           real_to_q16(q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd0)) + 0.75));
                sig_f_in = set_sigma_state(sig_f_in, 4'(p), 2'd1,
                                           real_to_q16(q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd1)) + 1.00));
            end
            do_predict(sig_f_in, q_m);

            sig_h_in = '0;
            for (int p = 0; p < 5; p++) begin
                sig_h_in = set_sigma_meas(sig_h_in, 4'(p), 1'b0,
                                          real_to_q16(q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd0)) + 0.75));
                sig_h_in = set_sigma_meas(sig_h_in, 4'(p), 1'b1,
                                          real_to_q16(q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd1)) + 1.00));
            end

            z_in = '0;
            z_in[0] = real_to_q16(true_px + 0.03 * real'((k % 2) ? 1 : -1));
            z_in[1] = real_to_q16(true_py - 0.03 * real'((k % 2) ? 1 : -1));

            do_correct(sig_h_in, z_in, r_m);

            $display("  Step %0d: True [px,py] = [%4.2f, %4.2f] | Est [px,py] = [%4.2f, %4.2f]",
                     k, true_px, true_py, q16_to_real(x_estimated[0]), q16_to_real(x_estimated[1]));
        end

        $display("--> UKF VEHICLE STATE ESTIMATION RESULTS:");
        $display("    True Position:      px = %f, py = %f", true_px, true_py);
        $display("    Estimated Position: px = %f, py = %f", q16_to_real(x_estimated[0]), q16_to_real(x_estimated[1]));

        if (q16_to_real(x_estimated[0]) > true_px - 0.25 && q16_to_real(x_estimated[0]) < true_px + 0.25 &&
            q16_to_real(x_estimated[1]) > true_py - 0.25 && q16_to_real(x_estimated[1]) < true_py + 0.25) begin
            $display("[TEST 2 PASSED] Successfully estimated 2D vehicle kinematic trajectory!");
        end else begin
            $display("[TEST 2 FAILED] 2D vehicle UKF tracking outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 4D Multi-Sensor Kinematic Target Tracking (9 Sigma Points)
        // State: [px, py, vx, vy], Measurements: [px, py]
        // Target: vx = 1.0 m/s, vy = -1.5 m/s
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 4D Multi-Sensor Kinematic Target Tracking (9 Sigma Points)");
        $display("Target Kinematics: vx = 1.0 m/s, vy = -1.5 m/s");

        state_dim = 3'd4;
        meas_dim  = 2'd2;

        x_init = '0;
        x_init[0] = real_to_q16(0.0);
        x_init[1] = real_to_q16(0.0);
        x_init[2] = real_to_q16(0.8);
        x_init[3] = real_to_q16(-1.2);

        p_init = '0;
        p_init = set_smat(p_init, 2'd0, 2'd0, real_to_q16(4.0));
        p_init = set_smat(p_init, 2'd1, 2'd1, real_to_q16(4.0));
        p_init = set_smat(p_init, 2'd2, 2'd2, real_to_q16(4.0));
        p_init = set_smat(p_init, 2'd3, 2'd3, real_to_q16(4.0));

        gamma_sq = real_to_q16(5.0);

        // N=4, gamma_sq = 5.0 -> W0 = 0.2, Wi = 0.1
        weights_m = '0;
        weights_c = '0;
        weights_m[0] = real_to_q16(0.200000);
        weights_c[0] = real_to_q16(0.200000);
        for (int i = 1; i <= 8; i++) begin
            weights_m[i] = real_to_q16(0.100000);
            weights_c[i] = real_to_q16(0.100000);
        end

        @(posedge clk);
        init_ukf = 1'b1;
        @(posedge clk);
        init_ukf = 1'b0;
        #10;

        q_m = '0;
        q_m = set_smat(q_m, 2'd0, 2'd0, real_to_q16(0.01));
        q_m = set_smat(q_m, 2'd1, 2'd1, real_to_q16(0.01));
        q_m = set_smat(q_m, 2'd2, 2'd2, real_to_q16(0.01));
        q_m = set_smat(q_m, 2'd3, 2'd3, real_to_q16(0.01));

        r_m = '0;
        r_m = set_imat(r_m, 1'b0, 1'b0, real_to_q16(0.06));
        r_m = set_imat(r_m, 1'b1, 1'b1, real_to_q16(0.06));

        true_px = 0.0;
        true_py = 0.0;
        true_vx = 1.0;
        true_vy = -1.5;

        for (int k = 1; k <= 10; k++) begin
            true_px = true_px + true_vx * 0.5;
            true_py = true_py + true_vy * 0.5;

            // 1. Generate 9 Sigma Points
            do_gen_sigmas();

            // 2. Propagate state sigmas: [px + vx*dt, py + vy*dt, vx, vy]
            sig_f_in = '0;
            for (int p = 0; p < 9; p++) begin
                sig_f_in = set_sigma_state(sig_f_in, 4'(p), 2'd0,
                    real_to_q16(q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd0)) +
                                q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd2)) * 0.5));
                sig_f_in = set_sigma_state(sig_f_in, 4'(p), 2'd1,
                    real_to_q16(q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd1)) +
                                q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd3)) * 0.5));
                sig_f_in = set_sigma_state(sig_f_in, 4'(p), 2'd2, get_sigma_state(sigma_points_out, 4'(p), 2'd2));
                sig_f_in = set_sigma_state(sig_f_in, 4'(p), 2'd3, get_sigma_state(sigma_points_out, 4'(p), 2'd3));
            end
            do_predict(sig_f_in, q_m);

            // 3. Measurement sigmas: [px, py]
            sig_h_in = '0;
            for (int p = 0; p < 9; p++) begin
                sig_h_in = set_sigma_meas(sig_h_in, 4'(p), 1'b0,
                    real_to_q16(q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd0)) +
                                q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd2)) * 0.5));
                sig_h_in = set_sigma_meas(sig_h_in, 4'(p), 1'b1,
                    real_to_q16(q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd1)) +
                                q16_to_real(get_sigma_state(sigma_points_out, 4'(p), 2'd3)) * 0.5));
            end

            z_in = '0;
            z_in[0] = real_to_q16(true_px + 0.04 * real'((k % 3) - 1));
            z_in[1] = real_to_q16(true_py - 0.04 * real'((k % 3) - 1));

            do_correct(sig_h_in, z_in, r_m);

            $display("  Step %0d: True [px,py,vx,vy] = [%4.2f, %4.2f, %4.2f, %4.2f] | Est = [%4.2f, %4.2f, %4.2f, %4.2f]",
                     k, true_px, true_py, true_vx, true_vy,
                     q16_to_real(x_estimated[0]), q16_to_real(x_estimated[1]),
                     q16_to_real(x_estimated[2]), q16_to_real(x_estimated[3]));
        end

        $display("--> UKF 4D TRACKING RESULTS:");
        $display("    True State:      [%f, %f, %f, %f]", true_px, true_py, true_vx, true_vy);
        $display("    Estimated State: [%f, %f, %f, %f]",
                 q16_to_real(x_estimated[0]), q16_to_real(x_estimated[1]),
                 q16_to_real(x_estimated[2]), q16_to_real(x_estimated[3]));

        if (q16_to_real(x_estimated[0]) > true_px - 0.35 && q16_to_real(x_estimated[0]) < true_px + 0.35 &&
            q16_to_real(x_estimated[1]) > true_py - 0.35 && q16_to_real(x_estimated[1]) < true_py + 0.35 &&
            q16_to_real(x_estimated[2]) > 0.8 && q16_to_real(x_estimated[2]) < 1.2 &&
            q16_to_real(x_estimated[3]) > -1.7 && q16_to_real(x_estimated[3]) < -1.3) begin
            $display("[TEST 3 PASSED] Successfully estimated 4D target state via 9-point UKF!");
        end else begin
            $display("[TEST 3 FAILED] 4D UKF target tracking outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL UNSCENTED KALMAN FILTER (UKF) HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
