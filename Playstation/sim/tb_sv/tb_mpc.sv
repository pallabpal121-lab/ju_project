// =============================================================================
// File Name   : tb_mpc.sv
// Module Name : tb_mpc
// Project     : Model Predictive Control (MPC) Accelerator (Solver #26)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for MPC Accelerator.
//   Verifies:
//   1. 1D Double Integrator Position & Velocity Setpoint Regulation (|u| <= 1.0)
//   2. 2D Autonomous Mobile Robot Speed & Yaw Rate Tracking ([-2.0, 2.0])
//   3. Inverted Pendulum on Cart Stabilization under Hard Force Constraints
// =============================================================================

`timescale 1ns / 1ps

import mpc_types_pkg::*;
`include "mpc_helpers.svh"

module tb_mpc;

    logic               clk;
    logic               rst_n;

    // Controls & Configurations
    logic               init_mpc;
    logic [2:0]         state_dim;
    logic [1:0]         ctrl_dim;
    logic [2:0]         stacked_dim;
    hessian_mat_t       h_mpc_mat;
    grad_mat_t          m_x_mat;
    grad_mat_t          m_ref_mat;
    stacked_vec_t       u_min;
    stacked_vec_t       u_max;
    q16_t               step_alpha;
    q16_t               mom_beta;
    logic [7:0]         max_iters;
    q16_t               tol_eps;

    // Real-Time Receding Horizon Interface
    logic               solve_valid;
    state_vec_t         x_curr;
    state_vec_t         x_ref;
    ctrl_vec_t          u_applied;
    stacked_vec_t       u_opt_horizon;
    stacked_vec_t       g_gradient;
    logic [7:0]         iterations_done;
    status_t            status;
    logic               solve_done;
    logic               busy;

    // Instantiate Top Module
    mpc_top dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .init_mpc       (init_mpc),
        .state_dim      (state_dim),
        .ctrl_dim       (ctrl_dim),
        .stacked_dim    (stacked_dim),
        .h_mpc_mat      (h_mpc_mat),
        .m_x_mat        (m_x_mat),
        .m_ref_mat      (m_ref_mat),
        .u_min          (u_min),
        .u_max          (u_max),
        .step_alpha     (step_alpha),
        .mom_beta       (mom_beta),
        .max_iters      (max_iters),
        .tol_eps        (tol_eps),
        .solve_valid    (solve_valid),
        .x_curr         (x_curr),
        .x_ref          (x_ref),
        .u_applied      (u_applied),
        .u_opt_horizon  (u_opt_horizon),
        .g_gradient     (g_gradient),
        .iterations_done(iterations_done),
        .status         (status),
        .solve_done     (solve_done),
        .busy           (busy)
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

    // Task to trigger MPC solve cycle
    task automatic do_mpc_step(
        input state_vec_t x_now,
        input state_vec_t x_target
    );
        @(posedge clk);
        x_curr      = x_now;
        x_ref       = x_target;
        solve_valid = 1'b1;
        @(posedge clk);
        solve_valid = 1'b0;
        @(posedge solve_done);
        #1;
    endtask

    // Test variables declared at module level
    real pos, vel, u_ctrl, dt;
    real v_speed, w_yaw, u_l, u_r;
    real p_cart, th_pend, v_cart, w_pend, f_cart;
    state_vec_t x_in, target_in;

    initial begin
        $display("==================================================================");
        $display(" Model Predictive Control (MPC) Accelerator TB (Solver #26)");
        $display("==================================================================");

        // Reset
        rst_n       = 0;
        init_mpc    = 0;
        solve_valid = 0;
        state_dim   = 3'd2;
        ctrl_dim    = 2'd1;
        stacked_dim = 3'd4;
        h_mpc_mat   = '0;
        m_x_mat     = '0;
        m_ref_mat   = '0;
        u_min       = '0;
        u_max       = '0;
        step_alpha  = real_to_q16(0.05);
        mom_beta    = real_to_q16(0.5);
        max_iters   = 8'd30;
        tol_eps     = real_to_q16(0.0001);
        x_curr      = '0;
        x_ref       = '0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 1D Double Integrator Position & Velocity Setpoint Regulation
        // Target: p_ref = 3.0 m, v_ref = 0.0 m/s
        // Constraints: -1.0 <= u <= +1.0 N
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 1D Double Integrator Position & Velocity Setpoint Regulation");
        $display("Target Setpoint: p_ref = 3.0 m, v_ref = 0.0 m/s | Actuator Limits: [-1.0, 1.0] N");

        state_dim   = 3'd2;
        ctrl_dim    = 2'd1;
        stacked_dim = 3'd4;

        h_mpc_mat = '0;
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd0, 2'd0, real_to_q16(2.45));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd0, 2'd1, real_to_q16(1.82));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd0, 2'd2, real_to_q16(1.24));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd0, 2'd3, real_to_q16(0.70));

        h_mpc_mat = set_hmat(h_mpc_mat, 2'd1, 2'd0, real_to_q16(1.82));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd1, 2'd1, real_to_q16(2.15));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd1, 2'd2, real_to_q16(1.48));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd1, 2'd3, real_to_q16(0.85));

        h_mpc_mat = set_hmat(h_mpc_mat, 2'd2, 2'd0, real_to_q16(1.24));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd2, 2'd1, real_to_q16(1.48));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd2, 2'd2, real_to_q16(1.85));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd2, 2'd3, real_to_q16(1.10));

        h_mpc_mat = set_hmat(h_mpc_mat, 2'd3, 2'd0, real_to_q16(0.70));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd3, 2'd1, real_to_q16(0.85));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd3, 2'd2, real_to_q16(1.10));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd3, 2'd3, real_to_q16(1.55));

        m_x_mat = '0;
        m_x_mat = set_hmat(m_x_mat, 2'd0, 2'd0, real_to_q16(3.50));
        m_x_mat = set_hmat(m_x_mat, 2'd0, 2'd1, real_to_q16(4.50));
        m_x_mat = set_hmat(m_x_mat, 2'd1, 2'd0, real_to_q16(2.60));
        m_x_mat = set_hmat(m_x_mat, 2'd1, 2'd1, real_to_q16(3.40));
        m_x_mat = set_hmat(m_x_mat, 2'd2, 2'd0, real_to_q16(1.80));
        m_x_mat = set_hmat(m_x_mat, 2'd2, 2'd1, real_to_q16(2.40));
        m_x_mat = set_hmat(m_x_mat, 2'd3, 2'd0, real_to_q16(1.00));
        m_x_mat = set_hmat(m_x_mat, 2'd3, 2'd1, real_to_q16(1.50));

        m_ref_mat = m_x_mat;

        for (int i = 0; i < 4; i++) begin
            u_min[i] = real_to_q16(-1.0);
            u_max[i] = real_to_q16(1.0);
        end

        step_alpha = real_to_q16(0.12);
        mom_beta   = real_to_q16(0.40);
        max_iters  = 8'd25;
        tol_eps    = real_to_q16(0.0001);

        @(posedge clk);
        init_mpc = 1'b1;
        @(posedge clk);
        init_mpc = 1'b0;
        #10;

        pos = 0.0;
        vel = 0.0;
        dt  = 0.2;

        target_in = '0;
        target_in[0] = real_to_q16(3.0); // p_ref = 3.0
        target_in[1] = real_to_q16(0.0); // v_ref = 0.0

        for (int t = 1; t <= 24; t++) begin
            x_in = '0;
            x_in[0] = real_to_q16(pos);
            x_in[1] = real_to_q16(vel);

            do_mpc_step(x_in, target_in);

            u_ctrl = q16_to_real(u_applied[0]);

            pos = pos + vel * dt + 0.5 * u_ctrl * dt * dt;
            vel = vel + u_ctrl * dt;

            $display("  Step %2d: Pos = %5.2f m | Vel = %5.2f m/s | u_applied = %5.2f N | Status = %0d",
                     t, pos, vel, u_ctrl, status);
        end

        $display("--> MPC DOUBLE INTEGRATOR CLOSED-LOOP RESULTS:");
        $display("    Target State:    p = 3.000000, v = 0.000000");
        $display("    Final State:     p = %f, v = %f", pos, vel);

        if (pos > 2.85 && pos < 3.15 && vel > -0.15 && vel < 0.15) begin
            $display("[TEST 1 PASSED] Successfully regulated double integrator position to setpoint!");
        end else begin
            $display("[TEST 1 FAILED] Double integrator MPC outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 2D Autonomous Mobile Robot Speed & Yaw Rate Tracking
        // Target: v_ref = 1.5 m/s, w_ref = 0.5 rad/s
        // Control limits: [-2.0, 2.0] N*m
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 2D Autonomous Mobile Robot Speed & Yaw Rate Tracking");
        $display("Target Kinematics: v_ref = 1.5 m/s, w_ref = 0.5 rad/s | Torque Limits: [-2.0, 2.0] N*m");

        state_dim   = 3'd2;
        ctrl_dim    = 2'd2;
        stacked_dim = 3'd4;

        h_mpc_mat = '0;
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd0, 2'd0, real_to_q16(2.00));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd0, 2'd1, real_to_q16(0.00));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd0, 2'd2, real_to_q16(0.80));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd0, 2'd3, real_to_q16(0.00));

        h_mpc_mat = set_hmat(h_mpc_mat, 2'd1, 2'd0, real_to_q16(0.00));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd1, 2'd1, real_to_q16(2.00));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd1, 2'd2, real_to_q16(0.00));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd1, 2'd3, real_to_q16(0.80));

        h_mpc_mat = set_hmat(h_mpc_mat, 2'd2, 2'd0, real_to_q16(0.80));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd2, 2'd1, real_to_q16(0.00));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd2, 2'd2, real_to_q16(1.80));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd2, 2'd3, real_to_q16(0.00));

        h_mpc_mat = set_hmat(h_mpc_mat, 2'd3, 2'd0, real_to_q16(0.00));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd3, 2'd1, real_to_q16(0.80));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd3, 2'd2, real_to_q16(0.00));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd3, 2'd3, real_to_q16(1.80));

        // Mapping matrix with balanced tracking gains
        m_x_mat = '0;
        m_x_mat = set_hmat(m_x_mat, 2'd0, 2'd0, real_to_q16(2.40));
        m_x_mat = set_hmat(m_x_mat, 2'd0, 2'd1, real_to_q16(-2.40));
        m_x_mat = set_hmat(m_x_mat, 2'd1, 2'd0, real_to_q16(2.40));
        m_x_mat = set_hmat(m_x_mat, 2'd1, 2'd1, real_to_q16(2.40));
        m_x_mat = set_hmat(m_x_mat, 2'd2, 2'd0, real_to_q16(1.40));
        m_x_mat = set_hmat(m_x_mat, 2'd2, 2'd1, real_to_q16(-1.40));
        m_x_mat = set_hmat(m_x_mat, 2'd3, 2'd0, real_to_q16(1.40));
        m_x_mat = set_hmat(m_x_mat, 2'd3, 2'd1, real_to_q16(1.40));

        m_ref_mat = m_x_mat;

        for (int i = 0; i < 4; i++) begin
            u_min[i] = real_to_q16(-2.0);
            u_max[i] = real_to_q16(2.0);
        end

        step_alpha = real_to_q16(0.18);
        mom_beta   = real_to_q16(0.40);
        max_iters  = 8'd25;

        @(posedge clk);
        init_mpc = 1'b1;
        @(posedge clk);
        init_mpc = 1'b0;
        #10;

        v_speed = 0.0;
        w_yaw   = 0.0;

        target_in = '0;
        target_in[0] = real_to_q16(1.5);
        target_in[1] = real_to_q16(0.5);

        for (int t = 1; t <= 12; t++) begin
            x_in = '0;
            x_in[0] = real_to_q16(v_speed);
            x_in[1] = real_to_q16(w_yaw);

            do_mpc_step(x_in, target_in);

            u_l = q16_to_real(u_applied[0]);
            u_r = q16_to_real(u_applied[1]);

            // Closed-loop robot velocity updates:
            v_speed = v_speed + (0.5 * (u_l + u_r)) * 0.2;
            w_yaw   = w_yaw   + (0.5 * (u_r - u_l)) * 0.2;

            $display("  Step %2d: Speed = %4.2f m/s | YawRate = %4.2f rad/s | u = [%4.2f, %4.2f]",
                     t, v_speed, w_yaw, u_l, u_r);
        end

        $display("--> MPC ROBOT SPEED & YAW RATE RESULTS:");
        $display("    Target State:    v = 1.500000, w = 0.500000");
        $display("    Final State:     v = %f, w = %f", v_speed, w_yaw);

        if (v_speed > 1.35 && v_speed < 1.65 && w_yaw > 0.35 && w_yaw < 0.65) begin
            $display("[TEST 2 PASSED] Successfully tracked mobile robot speed and yaw rate!");
        end else begin
            $display("[TEST 2 FAILED] Mobile robot MPC outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: Inverted Pendulum on Cart Stabilization (|u| <= 4.0 N)
        // Initial disturbance: theta_0 = 0.2 rad (~11.5 deg)
        // Target: [0, 0, 0, 0] upright equilibrium
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Inverted Pendulum Balancing on Cart (Cart-Pole Stabilization)");
        $display("Initial Angle Disturbance: theta_0 = 0.20 rad (11.5 deg) | Force Limits: [-4.0, 4.0] N");

        state_dim   = 3'd4;
        ctrl_dim    = 2'd1;
        stacked_dim = 3'd4;

        h_mpc_mat = '0;
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd0, 2'd0, real_to_q16(3.50));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd0, 2'd1, real_to_q16(2.20));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd0, 2'd2, real_to_q16(1.40));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd0, 2'd3, real_to_q16(0.80));

        h_mpc_mat = set_hmat(h_mpc_mat, 2'd1, 2'd0, real_to_q16(2.20));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd1, 2'd1, real_to_q16(3.10));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd1, 2'd2, real_to_q16(1.80));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd1, 2'd3, real_to_q16(1.10));

        h_mpc_mat = set_hmat(h_mpc_mat, 2'd2, 2'd0, real_to_q16(1.40));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd2, 2'd1, real_to_q16(1.80));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd2, 2'd2, real_to_q16(2.70));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd2, 2'd3, real_to_q16(1.40));

        h_mpc_mat = set_hmat(h_mpc_mat, 2'd3, 2'd0, real_to_q16(0.80));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd3, 2'd1, real_to_q16(1.10));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd3, 2'd2, real_to_q16(1.40));
        h_mpc_mat = set_hmat(h_mpc_mat, 2'd3, 2'd3, real_to_q16(2.20));

        // High restorative gain on tilt angle theta and angular velocity
        m_x_mat = '0;
        m_x_mat = set_hmat(m_x_mat, 2'd0, 2'd0, real_to_q16(0.80));
        m_x_mat = set_hmat(m_x_mat, 2'd0, 2'd1, real_to_q16(-35.00));
        m_x_mat = set_hmat(m_x_mat, 2'd0, 2'd2, real_to_q16(0.50));
        m_x_mat = set_hmat(m_x_mat, 2'd0, 2'd3, real_to_q16(-12.00));

        m_x_mat = set_hmat(m_x_mat, 2'd1, 2'd0, real_to_q16(0.60));
        m_x_mat = set_hmat(m_x_mat, 2'd1, 2'd1, real_to_q16(-25.00));
        m_x_mat = set_hmat(m_x_mat, 2'd1, 2'd2, real_to_q16(0.40));
        m_x_mat = set_hmat(m_x_mat, 2'd1, 2'd3, real_to_q16(-8.50));

        m_x_mat = set_hmat(m_x_mat, 2'd2, 2'd0, real_to_q16(0.40));
        m_x_mat = set_hmat(m_x_mat, 2'd2, 2'd1, real_to_q16(-16.00));
        m_x_mat = set_hmat(m_x_mat, 2'd2, 2'd2, real_to_q16(0.30));
        m_x_mat = set_hmat(m_x_mat, 2'd2, 2'd3, real_to_q16(-5.00));

        m_x_mat = set_hmat(m_x_mat, 2'd3, 2'd0, real_to_q16(0.20));
        m_x_mat = set_hmat(m_x_mat, 2'd3, 2'd1, real_to_q16(-8.00));
        m_x_mat = set_hmat(m_x_mat, 2'd3, 2'd2, real_to_q16(0.15));
        m_x_mat = set_hmat(m_x_mat, 2'd3, 2'd3, real_to_q16(-2.50));

        m_ref_mat = m_x_mat;

        for (int i = 0; i < 4; i++) begin
            u_min[i] = real_to_q16(-4.0);
            u_max[i] = real_to_q16(4.0);
        end

        step_alpha = real_to_q16(0.08);
        mom_beta   = real_to_q16(0.40);
        max_iters  = 8'd30;

        @(posedge clk);
        init_mpc = 1'b1;
        @(posedge clk);
        init_mpc = 1'b0;
        #10;

        p_cart  = 0.0;
        th_pend = 0.20; // 0.2 rad initial perturbation
        v_cart  = 0.0;
        w_pend  = 0.0;

        target_in = '0; // Equilibrium: [0, 0, 0, 0]

        for (int t = 1; t <= 15; t++) begin
            x_in = '0;
            x_in[0] = real_to_q16(p_cart);
            x_in[1] = real_to_q16(th_pend);
            x_in[2] = real_to_q16(v_cart);
            x_in[3] = real_to_q16(w_pend);

            do_mpc_step(x_in, target_in);

            f_cart = q16_to_real(u_applied[0]);

            // Stabilizing cart-pole dynamics:
            w_pend  = w_pend  + (9.8 * th_pend - 2.5 * f_cart - 1.2 * w_pend) * 0.1;
            th_pend = th_pend + w_pend * 0.1;
            v_cart  = v_cart  + (0.4 * f_cart - 0.1 * th_pend - 0.2 * v_cart) * 0.1;
            p_cart  = p_cart  + v_cart * 0.1;

            $display("  Step %2d: CartPos = %5.2f m | PendAngle = %5.3f rad | Force = %5.2f N",
                     t, p_cart, th_pend, f_cart);
        end

        $display("--> MPC CART-POLE STABILIZATION RESULTS:");
        $display("    Target Equilibrium: theta = 0.000000 rad");
        $display("    Final Angle:        theta = %f rad", th_pend);

        if (th_pend > -0.05 && th_pend < 0.05) begin
            $display("[TEST 3 PASSED] Successfully balanced inverted pendulum to upright vertical!");
        end else begin
            $display("[TEST 3 FAILED] Cart-pole MPC stabilization outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL MODEL PREDICTIVE CONTROL (MPC) HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
