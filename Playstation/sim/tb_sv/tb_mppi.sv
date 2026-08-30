// =============================================================================
// File Name   : tb_mppi.sv
// Module Name : tb_mppi
// Project     : Model Predictive Path Integral (MPPI) Accelerator (Solver #33)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for Model Predictive Path Integral Accelerator.
//   Verifies:
//   1. 1D Double Integrator Position & Velocity Setpoint Regulation (Target: (3.0, 0.0))
//   2. 2D Autonomous Mobile Robot Speed & Yaw Rate Tracking (Target: (1.5, 0.5))
//   3. Inverted Pendulum Upright Balancing Stabilization (Target: (0.0, 0.0))
// =============================================================================

`timescale 1ns / 1ps

import mppi_types_pkg::*;
`include "mppi_helpers.svh"

module tb_mppi;

    logic               clk;
    logic               rst_n;

    // Controls & Configurations
    logic               init_mppi;
    logic [2:0]         horizon;
    logic [2:0]         num_rollouts;
    q16_t               lambda_inv;
    q16_t               noise_sigma;
    ctrl_vec_t          lb_ctrl;
    ctrl_vec_t          ub_ctrl;
    mat22_t             A_mat;
    mat22_t             B_mat;
    state_vec_t         Q_diag;
    ctrl_vec_t          R_diag;
    state_vec_t         Qf_diag;
    ctrl_seq_t          init_nominal;

    // Execution & Outputs
    logic               step_mppi;
    state_vec_t         x_curr;
    state_vec_t         x_ref;
    ctrl_vec_t          u_opt_current;
    ctrl_seq_t          u_sequence;
    q16_t               best_cost;
    weight_arr_t        weights;
    status_t            status;
    logic               mppi_done;
    logic               busy;

    // Instantiate Top Module
    mppi_top dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .init_mppi    (init_mppi),
        .horizon      (horizon),
        .num_rollouts (num_rollouts),
        .lambda_inv   (lambda_inv),
        .noise_sigma  (noise_sigma),
        .lb_ctrl      (lb_ctrl),
        .ub_ctrl      (ub_ctrl),
        .A_mat        (A_mat),
        .B_mat        (B_mat),
        .Q_diag       (Q_diag),
        .R_diag       (R_diag),
        .Qf_diag      (Qf_diag),
        .init_nominal (init_nominal),
        .step_mppi    (step_mppi),
        .x_curr       (x_curr),
        .x_ref        (x_ref),
        .u_opt_current(u_opt_current),
        .u_sequence   (u_sequence),
        .best_cost    (best_cost),
        .weights      (weights),
        .status       (status),
        .mppi_done    (mppi_done),
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

    // Simulation loop variables
    state_vec_t sim_state;
    real p_curr, v_curr, u0_act;
    real w_curr, th_curr;

    initial begin
        $display("==================================================================");
        $display(" Model Predictive Path Integral (MPPI) Controller TB (Solver #33)");
        $display("==================================================================");

        // Reset
        rst_n        = 0;
        init_mppi    = 0;
        step_mppi    = 0;
        horizon      = 3'd4;
        num_rollouts = 3'd4;
        lambda_inv   = real_to_q16(0.05);
        noise_sigma  = real_to_q16(1.50);
        lb_ctrl      = '0;
        ub_ctrl      = '0;
        A_mat        = '0;
        B_mat        = '0;
        Q_diag       = '0;
        R_diag       = '0;
        Qf_diag      = '0;
        init_nominal = '0;
        x_curr       = '0;
        x_ref        = '0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 1D Double Integrator Position & Velocity Setpoint Regulation
        // State: (p, v) | Target: (3.0, 0.0) | Control: acceleration a
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 1D Double Integrator Position & Velocity Setpoint Regulation");
        $display("Target State: x* = (3.000000, 0.000000) from x_0 = (0.0, 0.0)");

        horizon      = 3'd4;
        num_rollouts = 3'd4;
        lambda_inv   = real_to_q16(0.05);
        noise_sigma  = real_to_q16(2.00);

        // Dynamics: dt = 0.4 s -> A = [1.0, 0.4; 0.0, 1.0], B = [0.08, 0.0; 0.4, 0.0]
        A_mat[0] = real_to_q16(1.0); A_mat[1] = real_to_q16(0.4);
        A_mat[2] = real_to_q16(0.0); A_mat[3] = real_to_q16(1.0);

        B_mat[0] = real_to_q16(0.08); B_mat[1] = real_to_q16(0.0);
        B_mat[2] = real_to_q16(0.40); B_mat[3] = real_to_q16(0.0);

        Q_diag[0]  = real_to_q16(5.0);  // Position error weight
        Q_diag[1]  = real_to_q16(1.0);  // Velocity error weight
        R_diag[0]  = real_to_q16(0.01); // Actuator effort weight
        R_diag[1]  = real_to_q16(0.0);
        Qf_diag[0] = real_to_q16(10.0);
        Qf_diag[1] = real_to_q16(2.0);

        lb_ctrl[0] = real_to_q16(-3.0);
        lb_ctrl[1] = real_to_q16(0.0);
        ub_ctrl[0] = real_to_q16(3.0);
        ub_ctrl[1] = real_to_q16(0.0);

        init_nominal = '0;

        @(posedge clk);
        init_mppi = 1'b1;
        @(posedge clk);
        init_mppi = 1'b0;
        #10;

        sim_state[0] = real_to_q16(0.0); // p = 0.0
        sim_state[1] = real_to_q16(0.0); // v = 0.0
        x_ref[0]     = real_to_q16(3.0); // target p = 3.0
        x_ref[1]     = real_to_q16(0.0); // target v = 0.0

        $display("Starting closed-loop MPPI control rollout execution (24 steps)...");
        for (int step = 0; step < 24; step++) begin
            x_curr = sim_state;
            @(posedge clk);
            step_mppi = 1'b1;
            @(posedge clk);
            step_mppi = 1'b0;
            @(posedge mppi_done);
            #10;

            sim_state = step_dynamics(sim_state, u_opt_current, A_mat, B_mat);
        end

        p_curr = q16_to_real(sim_state[0]);
        v_curr = q16_to_real(sim_state[1]);
        u0_act = q16_to_real(u_opt_current[0]);

        $display("--> CLOSED-LOOP MPPI POSITION RESULTS:");
        $display("    Final Position p:     %f (Target: 3.000000)", p_curr);
        $display("    Final Velocity v:     %f (Target: 0.000000)", v_curr);
        $display("    Optimal Control u0*:  %f", u0_act);

        if (p_curr > 2.50 && p_curr < 3.50 && v_curr > -0.60 && v_curr < 0.60) begin
            $display("[TEST 1 PASSED] Successfully regulated 1D double integrator setpoint via MPPI!");
        end else begin
            $display("[TEST 1 FAILED] Final state outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 2D Autonomous Mobile Robot Speed & Yaw Rate Tracking
        // State: (v, w) | Target: (1.5, 0.5)
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 2D Autonomous Mobile Robot Speed & Yaw Rate Tracking");
        $display("Target State: x* = (1.500000, 0.500000) from x_0 = (0.0, 0.0)");

        horizon      = 3'd4;
        num_rollouts = 3'd4;
        lambda_inv   = real_to_q16(0.10);
        noise_sigma  = real_to_q16(2.00);

        A_mat[0] = real_to_q16(0.70); A_mat[1] = real_to_q16(0.0);
        A_mat[2] = real_to_q16(0.0);  A_mat[3] = real_to_q16(0.70);

        B_mat[0] = real_to_q16(0.45); B_mat[1] = real_to_q16(0.0);
        B_mat[2] = real_to_q16(0.0);  B_mat[3] = real_to_q16(0.45);

        Q_diag[0]  = real_to_q16(5.0);
        Q_diag[1]  = real_to_q16(5.0);
        R_diag[0]  = real_to_q16(0.01);
        R_diag[1]  = real_to_q16(0.01);
        Qf_diag[0] = real_to_q16(10.0);
        Qf_diag[1] = real_to_q16(10.0);

        lb_ctrl[0] = real_to_q16(-3.0);
        lb_ctrl[1] = real_to_q16(-3.0);
        ub_ctrl[0] = real_to_q16(3.0);
        ub_ctrl[1] = real_to_q16(3.0);

        init_nominal = '0;

        @(posedge clk);
        init_mppi = 1'b1;
        @(posedge clk);
        init_mppi = 1'b0;
        #10;

        sim_state[0] = real_to_q16(0.0);
        sim_state[1] = real_to_q16(0.0);
        x_ref[0]     = real_to_q16(1.5);
        x_ref[1]     = real_to_q16(0.5);

        for (int step = 0; step < 24; step++) begin
            x_curr = sim_state;
            @(posedge clk);
            step_mppi = 1'b1;
            @(posedge clk);
            step_mppi = 1'b0;
            @(posedge mppi_done);
            #10;

            sim_state = step_dynamics(sim_state, u_opt_current, A_mat, B_mat);
        end

        v_curr = q16_to_real(sim_state[0]);
        w_curr = q16_to_real(sim_state[1]);

        $display("--> CLOSED-LOOP ROBOT NAVIGATION RESULTS:");
        $display("    Final Speed v:        %f (Target: 1.500000)", v_curr);
        $display("    Final Yaw Rate w:     %f (Target: 0.500000)", w_curr);

        if (v_curr > 1.20 && v_curr < 1.80 && w_curr > 0.25 && w_curr < 0.75) begin
            $display("[TEST 2 PASSED] Successfully tracked robot speed & yaw rate via MPPI!");
        end else begin
            $display("[TEST 2 FAILED] Robot velocity outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: Inverted Pendulum Upright Balancing Stabilization
        // State: (θ, θ_dot) | Target: (0.0, 0.0) | Initial θ = 0.20 rad
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Inverted Pendulum Upright Balancing Stabilization");
        $display("Target State: x* = (0.000000, 0.000000) from θ_0 = 0.200000 rad");

        horizon      = 3'd4;
        num_rollouts = 3'd4;
        lambda_inv   = real_to_q16(0.05);
        noise_sigma  = real_to_q16(4.00);

        A_mat[0] = real_to_q16(0.95); A_mat[1] = real_to_q16(0.1);
        A_mat[2] = real_to_q16(0.50); A_mat[3] = real_to_q16(0.95);

        B_mat[0] = real_to_q16(0.0); B_mat[1] = real_to_q16(0.0);
        B_mat[2] = real_to_q16(0.2); B_mat[3] = real_to_q16(0.0);

        Q_diag[0]  = real_to_q16(20.0);
        Q_diag[1]  = real_to_q16(2.0);
        R_diag[0]  = real_to_q16(0.005);
        R_diag[1]  = real_to_q16(0.0);
        Qf_diag[0] = real_to_q16(40.0);
        Qf_diag[1] = real_to_q16(4.0);

        lb_ctrl[0] = real_to_q16(-10.0);
        lb_ctrl[1] = real_to_q16(0.0);
        ub_ctrl[0] = real_to_q16(10.0);
        ub_ctrl[1] = real_to_q16(0.0);

        init_nominal = '0;

        @(posedge clk);
        init_mppi = 1'b1;
        @(posedge clk);
        init_mppi = 1'b0;
        #10;

        sim_state[0] = real_to_q16(0.20); // Initial angle perturbation
        sim_state[1] = real_to_q16(0.00);
        x_ref[0]     = real_to_q16(0.00);
        x_ref[1]     = real_to_q16(0.00);

        for (int step = 0; step < 20; step++) begin
            x_curr = sim_state;
            @(posedge clk);
            step_mppi = 1'b1;
            @(posedge clk);
            step_mppi = 1'b0;
            @(posedge mppi_done);
            #10;

            sim_state = step_dynamics(sim_state, u_opt_current, A_mat, B_mat);
        end

        th_curr = q16_to_real(sim_state[0]);

        $display("--> INVERTED PENDULUM BALANCING RESULTS:");
        $display("    Final Angle θ:        %f rad (Target: 0.000000)", th_curr);

        if (th_curr > -0.15 && th_curr < 0.15) begin
            $display("[TEST 3 PASSED] Successfully stabilized inverted pendulum upright via MPPI!");
        end else begin
            $display("[TEST 3 FAILED] Pendulum angle outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL MPPI HARDWARE ACCELERATOR TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
