// =============================================================================
// File Name   : tb_nes.sv
// Module Name : tb_nes
// Project     : Natural Evolution Strategies (NES) Accelerator (Solver #32)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for Natural Evolution Strategies Accelerator.
//   Verifies:
//   1. 2D Decoupled Quadratic Policy Maximization (Target: (3.0, 4.0))
//   2. 2D Non-Convex Curved Rosenbrock Ridge Search (Target: (1.0, 1.0))
//   3. 3D Multi-Parameter Policy Optimization (Target: (0.0, 0.0, 0.0))
// =============================================================================

`timescale 1ns / 1ps

import nes_types_pkg::*;
`include "nes_helpers.svh"

module tb_nes;

    logic               clk;
    logic               rst_n;

    // Controls & Configurations
    logic               init_nes;
    reward_fn_t         fn_type;
    logic [2:0]         num_pairs;
    logic [2:0]         dim;
    policy_vec_t        init_policy;
    q16_t               init_sigma;
    policy_vec_t        lb_vec;
    policy_vec_t        ub_vec;
    logic [7:0]         max_iters;
    q16_t               target_tol;
    q16_t               learning_rate;
    q16_t               momentum_rate;
    q16_t               sigma_decay;
    q16_t               sigma_min;

    // Execution & Outputs
    logic               opt_valid;
    policy_vec_t        best_policy;
    q16_t               best_reward;
    q16_t               final_sigma;
    logic [7:0]         iter_count;
    status_t            status;
    logic               opt_done;
    logic               busy;

    // Instantiate Top Module
    nes_top dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .init_nes     (init_nes),
        .fn_type      (fn_type),
        .num_pairs    (num_pairs),
        .dim          (dim),
        .init_policy  (init_policy),
        .init_sigma   (init_sigma),
        .lb_vec       (lb_vec),
        .ub_vec       (ub_vec),
        .max_iters    (max_iters),
        .target_tol   (target_tol),
        .learning_rate(learning_rate),
        .momentum_rate(momentum_rate),
        .sigma_decay  (sigma_decay),
        .sigma_min    (sigma_min),
        .opt_valid    (opt_valid),
        .best_policy  (best_policy),
        .best_reward  (best_reward),
        .final_sigma  (final_sigma),
        .iter_count   (iter_count),
        .status       (status),
        .opt_done     (opt_done),
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

    // Test variables declared at module level
    real th0_best, th1_best, th2_best, r_best;

    initial begin
        $display("==================================================================");
        $display(" Natural Evolution Strategies (NES) Policy Search TB (Solver #32)");
        $display("==================================================================");

        // Reset
        rst_n         = 0;
        init_nes      = 0;
        opt_valid     = 0;
        fn_type       = FN_QUADRATIC;
        num_pairs     = 3'd4;
        dim           = 3'd2;
        init_policy   = '0;
        init_sigma    = real_to_q16(1.0);
        lb_vec        = '0;
        ub_vec        = '0;
        max_iters     = 8'd40;
        target_tol    = real_to_q16(0.001);
        learning_rate = real_to_q16(0.20);
        momentum_rate = real_to_q16(0.85);
        sigma_decay   = real_to_q16(0.97);
        sigma_min     = real_to_q16(0.05);

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2D Decoupled Quadratic Policy Maximization
        // R(θ0, θ1) = -(θ0 - 3)^2 - 2(θ1 - 4)^2 | Target: (3.0, 4.0), R* = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 2D Decoupled Quadratic Policy Maximization");
        $display("Target Policy: θ* = (3.000000, 4.000000) | Target R* = 0.000000");

        fn_type       = FN_QUADRATIC;
        num_pairs     = 3'd4;
        dim           = 3'd2;
        max_iters     = 8'd40;
        target_tol    = real_to_q16(0.0001);

        init_policy[0]= real_to_q16(0.0);
        init_policy[1]= real_to_q16(0.0);
        init_sigma    = real_to_q16(1.0);

        lb_vec[0] = real_to_q16(-5.0);
        lb_vec[1] = real_to_q16(-5.0);
        ub_vec[0] = real_to_q16(5.0);
        ub_vec[1] = real_to_q16(5.0);

        learning_rate = real_to_q16(0.20);
        momentum_rate = real_to_q16(0.85);
        sigma_decay   = real_to_q16(0.97);
        sigma_min     = real_to_q16(0.05);

        @(posedge clk);
        init_nes = 1'b1;
        @(posedge clk);
        init_nes = 1'b0;
        #10;

        $display("Starting NES policy gradient optimization loop...");
        @(posedge clk);
        opt_valid = 1'b1;
        @(posedge clk);
        opt_valid = 1'b0;
        @(posedge opt_done);
        #10;

        th0_best = q16_to_real(best_policy[0]);
        th1_best = q16_to_real(best_policy[1]);
        r_best   = q16_to_real(best_reward);

        $display("--> NES OPTIMIZATION RESULTS:");
        $display("    Completed Iterations:  %0d", iter_count);
        $display("    Best Policy θ*:        (%f, %f)", th0_best, th1_best);
        $display("    Maximum Reward R*:     %f", r_best);
        $display("    Final Exploration σ:   %f", q16_to_real(final_sigma));

        if (r_best > -0.20 && th0_best > 2.50 && th0_best < 3.50 && th1_best > 3.50 && th1_best < 4.50) begin
            $display("[TEST 1 PASSED] Successfully climbed quadratic reward landscape via NES!");
        end else begin
            $display("[TEST 1 FAILED] Solution outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 2D Non-Convex Curved Rosenbrock Ridge Search
        // R(θ0, θ1) = -10(θ1 - θ0^2)^2 - (1 - θ0)^2 | Target: (1.0, 1.0), R* = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 2D Non-Convex Curved Rosenbrock Ridge Search");
        $display("Target Policy: θ* = (1.000000, 1.000000) | Target R* = 0.000000");

        fn_type       = FN_ROSENBROCK;
        num_pairs     = 3'd4;
        dim           = 3'd2;
        max_iters     = 8'd40;
        target_tol    = real_to_q16(0.0001);

        init_policy[0]= real_to_q16(0.0);
        init_policy[1]= real_to_q16(0.0);
        init_sigma    = real_to_q16(0.8);

        lb_vec[0] = real_to_q16(-2.0);
        lb_vec[1] = real_to_q16(-2.0);
        ub_vec[0] = real_to_q16(2.0);
        ub_vec[1] = real_to_q16(2.0);

        learning_rate = real_to_q16(0.15);
        momentum_rate = real_to_q16(0.85);
        sigma_decay   = real_to_q16(0.97);
        sigma_min     = real_to_q16(0.05);

        @(posedge clk);
        init_nes = 1'b1;
        @(posedge clk);
        init_nes = 1'b0;
        #10;

        @(posedge clk);
        opt_valid = 1'b1;
        @(posedge clk);
        opt_valid = 1'b0;
        @(posedge opt_done);
        #10;

        th0_best = q16_to_real(best_policy[0]);
        th1_best = q16_to_real(best_policy[1]);
        r_best   = q16_to_real(best_reward);

        $display("--> ROSENBROCK NES RESULTS:");
        $display("    Completed Iterations:  %0d", iter_count);
        $display("    Best Policy θ*:        (%f, %f)", th0_best, th1_best);
        $display("    Maximum Reward R*:     %f", r_best);

        if (r_best > -0.20 && th0_best > 0.70 && th0_best < 1.30 && th1_best > 0.50 && th1_best < 1.60) begin
            $display("[TEST 2 PASSED] Successfully tracked Rosenbrock non-convex curved ridge to maximum!");
        end else begin
            $display("[TEST 2 FAILED] Rosenbrock solution outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 3D Multi-Parameter Policy Optimization
        // R(θ) = -∑ (θ_d^2 + θ_d^4) | Target: (0.0, 0.0, 0.0), R* = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 3D Multi-Parameter Policy Optimization");
        $display("Target Policy: θ* = (0.000000, 0.000000, 0.000000) | Target R* = 0.000000");

        fn_type       = FN_RASTRIGIN;
        num_pairs     = 3'd4;
        dim           = 3'd3;
        max_iters     = 8'd40;
        target_tol    = real_to_q16(0.0001);

        init_policy[0]= real_to_q16(1.5);
        init_policy[1]= real_to_q16(-1.5);
        init_policy[2]= real_to_q16(2.0);
        init_sigma    = real_to_q16(1.0);

        lb_vec[0] = real_to_q16(-3.0);
        lb_vec[1] = real_to_q16(-3.0);
        lb_vec[2] = real_to_q16(-3.0);
        ub_vec[0] = real_to_q16(3.0);
        ub_vec[1] = real_to_q16(3.0);
        ub_vec[2] = real_to_q16(3.0);

        learning_rate = real_to_q16(0.15);
        momentum_rate = real_to_q16(0.85);
        sigma_decay   = real_to_q16(0.97);
        sigma_min     = real_to_q16(0.05);

        @(posedge clk);
        init_nes = 1'b1;
        @(posedge clk);
        init_nes = 1'b0;
        #10;

        @(posedge clk);
        opt_valid = 1'b1;
        @(posedge clk);
        opt_valid = 1'b0;
        @(posedge opt_done);
        #10;

        th0_best = q16_to_real(best_policy[0]);
        th1_best = q16_to_real(best_policy[1]);
        th2_best = q16_to_real(best_policy[2]);
        r_best   = q16_to_real(best_reward);

        $display("--> 3D MULTI-PARAMETER NES RESULTS:");
        $display("    Completed Iterations:  %0d", iter_count);
        $display("    Best Policy θ*:        (%f, %f, %f)", th0_best, th1_best, th2_best);
        $display("    Maximum Reward R*:     %f", r_best);

        if (r_best > -0.10 && th0_best > -0.30 && th0_best < 0.30 &&
            th1_best > -0.30 && th1_best < 0.30 && th2_best > -0.30 && th2_best < 0.30) begin
            $display("[TEST 3 PASSED] Successfully optimized 3D policy parameters via NES!");
        end else begin
            $display("[TEST 3 FAILED] 3D policy solution outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL NATURAL EVOLUTION STRATEGIES (NES) HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
