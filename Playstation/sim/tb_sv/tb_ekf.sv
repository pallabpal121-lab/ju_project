// =============================================================================
// File Name   : tb_ekf.sv
// Module Name : tb_ekf
// Project     : Extended Kalman Filter (EKF) Accelerator (Solver #24)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for EKF Accelerator.
//   Verifies:
//   1. 2D Non-Linear Radar / Range Tracking (z = sqrt(p_x^2 + y_0^2))
//   2. 2D Autonomous Vehicle Kinematic Motion (GPS + Odometry Fusion)
//   3. 4D Multi-Sensor Kinematic Target Tracking (Position & Velocity)
// =============================================================================

`timescale 1ns / 1ps

import ekf_types_pkg::*;
`include "ekf_helpers.svh"

module tb_ekf;

    logic               clk;
    logic               rst_n;

    // Controls & Configurations
    logic               init_ekf;
    logic [2:0]         state_dim;
    logic [1:0]         meas_dim;
    state_vec_t         x_init;
    state_mat_t         p_init;

    // Predict Interface
    logic               predict_valid;
    state_vec_t         f_x_pred;
    state_mat_t         f_mat;
    state_mat_t         q_mat;

    // Correct Interface
    logic               correct_valid;
    meas_vec_t          z_meas;
    meas_vec_t          h_x_pred;
    meas_mat_t          h_mat;
    innov_mat_t         r_mat;

    // Outputs
    state_vec_t         x_estimated;
    state_mat_t         p_covariance;
    meas_vec_t          innov_residual;
    gain_mat_t          k_gain_out;
    status_t            status;
    logic               predict_done;
    logic               correct_done;
    logic               busy;

    // Instantiate Top Module
    ekf_top dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .init_ekf      (init_ekf),
        .state_dim     (state_dim),
        .meas_dim      (meas_dim),
        .x_init        (x_init),
        .p_init        (p_init),
        .predict_valid (predict_valid),
        .f_x_pred      (f_x_pred),
        .f_mat         (f_mat),
        .q_mat         (q_mat),
        .correct_valid (correct_valid),
        .z_meas        (z_meas),
        .h_x_pred      (h_x_pred),
        .h_mat         (h_mat),
        .r_mat         (r_mat),
        .x_estimated   (x_estimated),
        .p_covariance  (p_covariance),
        .innov_residual(innov_residual),
        .k_gain_out    (k_gain_out),
        .status        (status),
        .predict_done  (predict_done),
        .correct_done  (correct_done),
        .busy          (busy)
    );

    // 100MHz Clock Generator
    initial clk = 0;
    always #5 clk = ~clk;

    // Helper functions
    function real q16_to_real(input q16_t val);
        q16_to_real = real'(val) / 65536.0;
    endfunction

    function q16_t real_to_q16(input real val);
        real_to_q16 = q16_t'(int'(val * 65536.0));
    endfunction

    // Task for predict step
    task automatic do_predict(
        input state_vec_t f_pred,
        input state_mat_t f_jacobian,
        input state_mat_t q_noise
    );
        @(posedge clk);
        f_x_pred      = f_pred;
        f_mat         = f_jacobian;
        q_mat         = q_noise;
        predict_valid = 1'b1;
        @(posedge clk);
        predict_valid = 1'b0;
        @(posedge predict_done);
        #1;
    endtask

    // Task for correct step
    task automatic do_correct(
        input meas_vec_t  z_raw,
        input meas_vec_t  h_pred,
        input meas_mat_t  h_jacobian,
        input innov_mat_t r_noise
    );
        @(posedge clk);
        z_meas        = z_raw;
        h_x_pred      = h_pred;
        h_mat         = h_jacobian;
        r_mat         = r_noise;
        correct_valid = 1'b1;
        @(posedge clk);
        correct_valid = 1'b0;
        @(posedge correct_done);
        #1;
    endtask

    // Test variables
    real true_px, true_vx, dt, y0, range_meas, h_val, h_dh;
    real true_x, true_y;
    real true_4d_px, true_4d_py, true_4d_vx, true_4d_vy;
    state_vec_t f_in;
    state_mat_t f_m, q_m, p_m;
    meas_vec_t  z_in, h_in;
    meas_mat_t  h_m;
    innov_mat_t r_m;

    initial begin
        $display("==================================================================");
        $display(" Extended Kalman Filter (EKF) Accelerator TB (Solver #24)");
        $display("==================================================================");

        // Reset
        rst_n         = 0;
        init_ekf      = 0;
        predict_valid = 0;
        correct_valid = 0;
        state_dim     = 3'd2;
        meas_dim      = 2'd1;
        x_init        = '0;
        p_init        = '0;
        f_x_pred      = '0;
        f_mat         = '0;
        q_mat         = '0;
        z_meas        = '0;
        h_x_pred      = '0;
        h_mat         = '0;
        r_mat         = '0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2D Non-Linear Radar / Range Tracking (z = sqrt(p_x^2 + y_0^2))
        // Target: True position px(t) = 5.0 + 2.0*t, vx = 2.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 2D Non-Linear Radar / Range Tracking (z = sqrt(p_x^2 + y_0^2))");
        $display("Target Kinematics: p_x(t) = 5.0 + 2.0*t, v_x = 2.0 m/s");

        dt = 0.5;
        y0 = 10.0; // Radar offset altitude

        // State: [px, vx]
        state_dim = 3'd2;
        meas_dim  = 2'd1;

        // Initial guess: px_0 = 4.0, vx_0 = 0.0
        x_init    = '0;
        x_init[0] = real_to_q16(4.0);
        x_init[1] = real_to_q16(0.0);

        // P_0 = diag(5.0, 5.0)
        p_init = '0;
        p_init = set_smat(p_init, 2'd0, 2'd0, real_to_q16(5.0));
        p_init = set_smat(p_init, 2'd1, 2'd1, real_to_q16(5.0));

        @(posedge clk);
        init_ekf = 1'b1;
        @(posedge clk);
        init_ekf = 1'b0;
        #10;

        // Process noise Q
        q_m = '0;
        q_m = set_smat(q_m, 2'd0, 2'd0, real_to_q16(0.01));
        q_m = set_smat(q_m, 2'd1, 2'd1, real_to_q16(0.01));

        // State transition Jacobian F = [1, dt; 0, 1]
        f_m = '0;
        f_m = set_smat(f_m, 2'd0, 2'd0, real_to_q16(1.0));
        f_m = set_smat(f_m, 2'd0, 2'd1, real_to_q16(dt));
        f_m = set_smat(f_m, 2'd1, 2'd1, real_to_q16(1.0));

        // Measurement noise R = 0.05
        r_m = '0;
        r_m = set_imat(r_m, 1'b0, 1'b0, real_to_q16(0.05));

        true_px = 5.0;
        true_vx = 2.0;

        for (int k = 1; k <= 10; k++) begin
            // 1. Simulate true motion
            true_px = true_px + true_vx * dt;
            range_meas = $sqrt(true_px * true_px + y0 * y0) + 0.05 * real'((k % 3) - 1);

            // 2. Predict Step
            f_in = '0;
            f_in[0] = real_to_q16(q16_to_real(x_estimated[0]) + q16_to_real(x_estimated[1]) * dt);
            f_in[1] = x_estimated[1];
            do_predict(f_in, f_m, q_m);

            // 3. Measurement Jacobian at predicted state: H = [px / sqrt(px^2 + y0^2), 0]
            h_val = $sqrt(q16_to_real(x_estimated[0]) * q16_to_real(x_estimated[0]) + y0 * y0);
            h_dh  = q16_to_real(x_estimated[0]) / h_val;

            z_in = '0;
            z_in[0] = real_to_q16(range_meas);

            h_in = '0;
            h_in[0] = real_to_q16(h_val);

            h_m = '0;
            h_m = set_mmat(h_m, 1'b0, 2'd0, real_to_q16(h_dh));
            h_m = set_mmat(h_m, 1'b0, 2'd1, real_to_q16(0.0));

            do_correct(z_in, h_in, h_m, r_m);

            $display("  Step %0d: True px = %5.2f, Est px = %5.2f, Est vx = %5.2f, Innov = %6.3f",
                     k, true_px, q16_to_real(x_estimated[0]), q16_to_real(x_estimated[1]), q16_to_real(innov_residual[0]));
        end

        $display("--> EKF RADAR TRACKING RESULTS:");
        $display("    True State:      px = %f, vx = %f", true_px, true_vx);
        $display("    Estimated State: px = %f, vx = %f", q16_to_real(x_estimated[0]), q16_to_real(x_estimated[1]));
        $display("    Covariance P00:  %f, P11: %f", q16_to_real(get_smat(p_covariance, 2'd0, 2'd0)),
                                                    q16_to_real(get_smat(p_covariance, 2'd1, 2'd1)));

        if (q16_to_real(x_estimated[0]) > true_px - 0.5 && q16_to_real(x_estimated[0]) < true_px + 0.5 &&
            q16_to_real(x_estimated[1]) > 1.7 && q16_to_real(x_estimated[1]) < 2.3) begin
            $display("[TEST 1 PASSED] Successfully tracked non-linear radar kinematics!");
        end else begin
            $display("[TEST 1 FAILED] EKF Radar tracking outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 2D Autonomous Vehicle Kinematic Motion (GPS + Odometry Fusion)
        // State: [x, y], Control u = [vx, vy]*dt = [1.5, 2.0]*0.5
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 2D Autonomous Vehicle Kinematic Motion (GPS + Odometry Fusion)");
        $display("Target Kinematics: vx = 1.5 m/s, vy = 2.0 m/s");

        state_dim = 3'd2;
        meas_dim  = 2'd2;

        x_init = '0;
        x_init[0] = real_to_q16(0.0);
        x_init[1] = real_to_q16(0.0);

        p_init = '0;
        p_init = set_smat(p_init, 2'd0, 2'd0, real_to_q16(4.0));
        p_init = set_smat(p_init, 2'd1, 2'd1, real_to_q16(4.0));

        @(posedge clk);
        init_ekf = 1'b1;
        @(posedge clk);
        init_ekf = 1'b0;
        #10;

        f_m = '0;
        f_m = set_smat(f_m, 2'd0, 2'd0, real_to_q16(1.0));
        f_m = set_smat(f_m, 2'd1, 2'd1, real_to_q16(1.0));

        q_m = '0;
        q_m = set_smat(q_m, 2'd0, 2'd0, real_to_q16(0.02));
        q_m = set_smat(q_m, 2'd1, 2'd1, real_to_q16(0.02));

        h_m = '0;
        h_m = set_mmat(h_m, 1'b0, 2'd0, real_to_q16(1.0));
        h_m = set_mmat(h_m, 1'b1, 2'd1, real_to_q16(1.0));

        r_m = '0;
        r_m = set_imat(r_m, 1'b0, 1'b0, real_to_q16(0.1));
        r_m = set_imat(r_m, 1'b1, 1'b1, real_to_q16(0.1));

        true_x = 0.0;
        true_y = 0.0;

        for (int k = 1; k <= 8; k++) begin
            true_x = true_x + 1.5 * 0.5;
            true_y = true_y + 2.0 * 0.5;

            // Predict with odometry control [0.75, 1.0]
            f_in = '0;
            f_in[0] = real_to_q16(q16_to_real(x_estimated[0]) + 0.75);
            f_in[1] = real_to_q16(q16_to_real(x_estimated[1]) + 1.00);
            do_predict(f_in, f_m, q_m);

            // Correct with GPS measurement
            z_in = '0;
            z_in[0] = real_to_q16(true_x + 0.05 * real'((k % 2) ? 1 : -1));
            z_in[1] = real_to_q16(true_y - 0.04 * real'((k % 2) ? 1 : -1));

            h_in = '0;
            h_in[0] = x_estimated[0];
            h_in[1] = x_estimated[1];

            do_correct(z_in, h_in, h_m, r_m);

            $display("  Step %0d: True [x,y] = [%4.2f, %4.2f], Est [x,y] = [%4.2f, %4.2f]",
                     k, true_x, true_y, q16_to_real(x_estimated[0]), q16_to_real(x_estimated[1]));
        end

        $display("--> EKF VEHICLE STATE ESTIMATION RESULTS:");
        $display("    True Position:      x = %f, y = %f", true_x, true_y);
        $display("    Estimated Position: x = %f, y = %f", q16_to_real(x_estimated[0]), q16_to_real(x_estimated[1]));

        if (q16_to_real(x_estimated[0]) > true_x - 0.3 && q16_to_real(x_estimated[0]) < true_x + 0.3 &&
            q16_to_real(x_estimated[1]) > true_y - 0.3 && q16_to_real(x_estimated[1]) < true_y + 0.3) begin
            $display("[TEST 2 PASSED] Successfully estimated 2D vehicle kinematic position!");
        end else begin
            $display("[TEST 2 FAILED] 2D vehicle state estimation outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 4D Multi-Sensor Kinematic Target Tracking (Pos & Vel)
        // State: [px, py, vx, vy], Measurements: [px, py]
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 4D Multi-Sensor Kinematic Target Tracking (Pos & Vel)");
        $display("Target Kinematics: vx = 1.0 m/s, vy = -1.5 m/s");

        state_dim = 3'd4;
        meas_dim  = 2'd2;

        x_init = '0;
        x_init[0] = real_to_q16(0.0);
        x_init[1] = real_to_q16(0.0);
        x_init[2] = real_to_q16(0.5);
        x_init[3] = real_to_q16(-1.0);

        p_init = '0;
        p_init = set_smat(p_init, 2'd0, 2'd0, real_to_q16(5.0));
        p_init = set_smat(p_init, 2'd1, 2'd1, real_to_q16(5.0));
        p_init = set_smat(p_init, 2'd2, 2'd2, real_to_q16(5.0));
        p_init = set_smat(p_init, 2'd3, 2'd3, real_to_q16(5.0));

        @(posedge clk);
        init_ekf = 1'b1;
        @(posedge clk);
        init_ekf = 1'b0;
        #10;

        // F = [1, 0, dt, 0; 0, 1, 0, dt; 0, 0, 1, 0; 0, 0, 0, 1]
        f_m = '0;
        f_m = set_smat(f_m, 2'd0, 2'd0, real_to_q16(1.0));
        f_m = set_smat(f_m, 2'd0, 2'd2, real_to_q16(0.5));
        f_m = set_smat(f_m, 2'd1, 2'd1, real_to_q16(1.0));
        f_m = set_smat(f_m, 2'd1, 2'd3, real_to_q16(0.5));
        f_m = set_smat(f_m, 2'd2, 2'd2, real_to_q16(1.0));
        f_m = set_smat(f_m, 2'd3, 2'd3, real_to_q16(1.0));

        q_m = '0;
        q_m = set_smat(q_m, 2'd0, 2'd0, real_to_q16(0.01));
        q_m = set_smat(q_m, 2'd1, 2'd1, real_to_q16(0.01));
        q_m = set_smat(q_m, 2'd2, 2'd2, real_to_q16(0.01));
        q_m = set_smat(q_m, 2'd3, 2'd3, real_to_q16(0.01));

        h_m = '0;
        h_m = set_mmat(h_m, 1'b0, 2'd0, real_to_q16(1.0));
        h_m = set_mmat(h_m, 1'b1, 2'd1, real_to_q16(1.0));

        r_m = '0;
        r_m = set_imat(r_m, 1'b0, 1'b0, real_to_q16(0.08));
        r_m = set_imat(r_m, 1'b1, 1'b1, real_to_q16(0.08));

        true_4d_px = 0.0;
        true_4d_py = 0.0;
        true_4d_vx = 1.0;
        true_4d_vy = -1.5;

        for (int k = 1; k <= 10; k++) begin
            true_4d_px = true_4d_px + true_4d_vx * 0.5;
            true_4d_py = true_4d_py + true_4d_vy * 0.5;

            // Predict
            f_in = '0;
            f_in[0] = real_to_q16(q16_to_real(x_estimated[0]) + q16_to_real(x_estimated[2]) * 0.5);
            f_in[1] = real_to_q16(q16_to_real(x_estimated[1]) + q16_to_real(x_estimated[3]) * 0.5);
            f_in[2] = x_estimated[2];
            f_in[3] = x_estimated[3];
            do_predict(f_in, f_m, q_m);

            // Correct
            z_in = '0;
            z_in[0] = real_to_q16(true_4d_px + 0.06 * real'((k % 3) - 1));
            z_in[1] = real_to_q16(true_4d_py - 0.05 * real'((k % 3) - 1));

            h_in = '0;
            h_in[0] = x_estimated[0];
            h_in[1] = x_estimated[1];

            do_correct(z_in, h_in, h_m, r_m);

            $display("  Step %0d: True [px,py,vx,vy] = [%4.2f, %4.2f, %4.2f, %4.2f] | Est = [%4.2f, %4.2f, %4.2f, %4.2f]",
                     k, true_4d_px, true_4d_py, true_4d_vx, true_4d_vy,
                     q16_to_real(x_estimated[0]), q16_to_real(x_estimated[1]),
                     q16_to_real(x_estimated[2]), q16_to_real(x_estimated[3]));
        end

        $display("--> EKF 4D TRACKING RESULTS:");
        $display("    True State:      [%f, %f, %f, %f]", true_4d_px, true_4d_py, true_4d_vx, true_4d_vy);
        $display("    Estimated State: [%f, %f, %f, %f]",
                 q16_to_real(x_estimated[0]), q16_to_real(x_estimated[1]),
                 q16_to_real(x_estimated[2]), q16_to_real(x_estimated[3]));

        if (q16_to_real(x_estimated[0]) > true_4d_px - 0.4 && q16_to_real(x_estimated[0]) < true_4d_px + 0.4 &&
            q16_to_real(x_estimated[1]) > true_4d_py - 0.4 && q16_to_real(x_estimated[1]) < true_4d_py + 0.4 &&
            q16_to_real(x_estimated[2]) > 0.8 && q16_to_real(x_estimated[2]) < 1.2 &&
            q16_to_real(x_estimated[3]) > -1.7 && q16_to_real(x_estimated[3]) < -1.3) begin
            $display("[TEST 3 PASSED] Successfully estimated 4D position and velocity kinematics!");
        end else begin
            $display("[TEST 3 FAILED] 4D target tracking outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL EXTENDED KALMAN FILTER (EKF) HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
