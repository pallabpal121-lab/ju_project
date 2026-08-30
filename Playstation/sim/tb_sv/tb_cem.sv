// =============================================================================
// File Name   : tb_cem.sv
// Module Name : tb_cem
// Project     : Cross-Entropy Method (CEM) Accelerator (Solver #31)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for Cross-Entropy Method Accelerator.
//   Verifies:
//   1. 2D Decoupled Quadratic Trajectory Minimization (Target: (3.0, 4.0))
//   2. 2D Non-Convex Curved Rosenbrock Valley (Target: (1.0, 1.0))
//   3. 3D Multi-Modal Landscape Optimization (Target: (0.0, 0.0, 0.0))
// =============================================================================

`timescale 1ns / 1ps

import cem_types_pkg::*;
`include "cem_helpers.svh"

module tb_cem;

    logic               clk;
    logic               rst_n;

    // Controls & Configurations
    logic               init_cem;
    fitness_fn_t        fn_type;
    logic [3:0]         num_samples;
    logic [2:0]         num_elites;
    logic [2:0]         dim;
    param_vec_t         init_mean;
    param_vec_t         init_sigma;
    param_vec_t         lb_vec;
    param_vec_t         ub_vec;
    logic [7:0]         max_iters;
    q16_t               target_tol;
    q16_t               alpha_smooth;
    q16_t               sigma_min;

    // Execution & Outputs
    logic               opt_valid;
    param_vec_t         best_solution;
    q16_t               best_fitness;
    param_vec_t         final_mean;
    param_vec_t         final_sigma;
    logic [7:0]         iter_count;
    status_t            status;
    logic               opt_done;
    logic               busy;

    // Instantiate Top Module
    cem_top dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .init_cem     (init_cem),
        .fn_type      (fn_type),
        .num_samples  (num_samples),
        .num_elites   (num_elites),
        .dim          (dim),
        .init_mean    (init_mean),
        .init_sigma   (init_sigma),
        .lb_vec       (lb_vec),
        .ub_vec       (ub_vec),
        .max_iters    (max_iters),
        .target_tol   (target_tol),
        .alpha_smooth (alpha_smooth),
        .sigma_min    (sigma_min),
        .opt_valid    (opt_valid),
        .best_solution(best_solution),
        .best_fitness (best_fitness),
        .final_mean   (final_mean),
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
    real x0_best, x1_best, x2_best, fit_best;

    initial begin
        $display("==================================================================");
        $display(" Cross-Entropy Method (CEM) Accelerator TB (Solver #31)");
        $display("==================================================================");

        // Reset
        rst_n        = 0;
        init_cem     = 0;
        opt_valid    = 0;
        fn_type      = FN_QUADRATIC;
        num_samples  = 4'd6;
        num_elites   = 3'd2;
        dim          = 3'd2;
        init_mean    = '0;
        init_sigma   = '0;
        lb_vec       = '0;
        ub_vec       = '0;
        max_iters    = 8'd40;
        target_tol   = real_to_q16(0.001);
        alpha_smooth = real_to_q16(0.70);
        sigma_min    = real_to_q16(0.05);

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2D Decoupled Quadratic Trajectory Minimization
        // f(x0, x1) = (x0 - 3)^2 + 2(x1 - 4)^2 | Target: (3.0, 4.0), F* = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 2D Decoupled Quadratic Trajectory Minimization");
        $display("Target Minimum: x* = (3.000000, 4.000000) | Target F* = 0.000000");

        fn_type     = FN_QUADRATIC;
        num_samples = 4'd6;
        num_elites  = 3'd2;
        dim         = 3'd2;
        max_iters   = 8'd40;
        target_tol  = real_to_q16(0.0001);

        init_mean[0]  = real_to_q16(0.0);
        init_mean[1]  = real_to_q16(0.0);
        init_sigma[0] = real_to_q16(2.0);
        init_sigma[1] = real_to_q16(2.0);

        lb_vec[0] = real_to_q16(-5.0);
        lb_vec[1] = real_to_q16(-5.0);
        ub_vec[0] = real_to_q16(5.0);
        ub_vec[1] = real_to_q16(5.0);

        alpha_smooth = real_to_q16(0.70);
        sigma_min    = real_to_q16(0.05);

        @(posedge clk);
        init_cem = 1'b1;
        @(posedge clk);
        init_cem = 1'b0;
        #10;

        $display("Starting Cross-Entropy Method optimization loop...");
        @(posedge clk);
        opt_valid = 1'b1;
        @(posedge clk);
        opt_valid = 1'b0;
        @(posedge opt_done);
        #10;

        x0_best  = q16_to_real(best_solution[0]);
        x1_best  = q16_to_real(best_solution[1]);
        fit_best = q16_to_real(best_fitness);

        $display("--> CEM OPTIMIZATION RESULTS:");
        $display("    Completed Iterations:  %0d", iter_count);
        $display("    Best Solution x*:      (%f, %f)", x0_best, x1_best);
        $display("    Minimum Fitness F*:    %f", fit_best);
        $display("    Final Distribution μ:  (%f, %f)",
                 q16_to_real(final_mean[0]), q16_to_real(final_mean[1]));
        $display("    Final Distribution σ:  (%f, %f)",
                 q16_to_real(final_sigma[0]), q16_to_real(final_sigma[1]));

        if (fit_best < 0.20 && x0_best > 2.50 && x0_best < 3.50 && x1_best > 3.50 && x1_best < 4.50) begin
            $display("[TEST 1 PASSED] Successfully converged Gaussian distribution to quadratic minimum!");
        end else begin
            $display("[TEST 1 FAILED] Solution outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 2D Non-Convex Curved Rosenbrock Valley
        // f(x0, x1) = 10(x1 - x0^2)^2 + (1 - x0)^2 | Target: (1.0, 1.0), F* = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 2D Non-Convex Curved Rosenbrock Valley");
        $display("Target Optimum: x* = (1.000000, 1.000000) | Target F* = 0.000000");

        fn_type     = FN_ROSENBROCK;
        num_samples = 4'd6;
        num_elites  = 3'd2;
        dim         = 3'd2;
        max_iters   = 8'd40;
        target_tol  = real_to_q16(0.0001);

        init_mean[0]  = real_to_q16(0.0);
        init_mean[1]  = real_to_q16(0.0);
        init_sigma[0] = real_to_q16(1.5);
        init_sigma[1] = real_to_q16(1.5);

        lb_vec[0] = real_to_q16(-2.0);
        lb_vec[1] = real_to_q16(-2.0);
        ub_vec[0] = real_to_q16(2.0);
        ub_vec[1] = real_to_q16(2.0);

        alpha_smooth = real_to_q16(0.70);
        sigma_min    = real_to_q16(0.05);

        @(posedge clk);
        init_cem = 1'b1;
        @(posedge clk);
        init_cem = 1'b0;
        #10;

        @(posedge clk);
        opt_valid = 1'b1;
        @(posedge clk);
        opt_valid = 1'b0;
        @(posedge opt_done);
        #10;

        x0_best  = q16_to_real(best_solution[0]);
        x1_best  = q16_to_real(best_solution[1]);
        fit_best = q16_to_real(best_fitness);

        $display("--> ROSENBROCK CEM RESULTS:");
        $display("    Completed Iterations:  %0d", iter_count);
        $display("    Best Solution x*:      (%f, %f)", x0_best, x1_best);
        $display("    Minimum Fitness F*:    %f", fit_best);

        if (fit_best < 0.20 && x0_best > 0.70 && x0_best < 1.30 && x1_best > 0.50 && x1_best < 1.60) begin
            $display("[TEST 2 PASSED] Successfully tracked Rosenbrock banana valley to global optimum!");
        end else begin
            $display("[TEST 2 FAILED] Rosenbrock solution outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 3D Multi-Modal Landscape Optimization
        // f(x) = ∑ (x_d^2 + x_d^4) | Target: (0.0, 0.0, 0.0), F* = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 3D Multi-Modal Landscape Optimization");
        $display("Target Optimum: x* = (0.000000, 0.000000, 0.000000) | Target F* = 0.000000");

        fn_type     = FN_RASTRIGIN;
        num_samples = 4'd6;
        num_elites  = 3'd2;
        dim         = 3'd3;
        max_iters   = 8'd40;
        target_tol  = real_to_q16(0.0001);

        init_mean[0]  = real_to_q16(1.5);
        init_mean[1]  = real_to_q16(-1.5);
        init_mean[2]  = real_to_q16(2.0);
        init_sigma[0] = real_to_q16(1.5);
        init_sigma[1] = real_to_q16(1.5);
        init_sigma[2] = real_to_q16(1.5);

        lb_vec[0] = real_to_q16(-3.0);
        lb_vec[1] = real_to_q16(-3.0);
        lb_vec[2] = real_to_q16(-3.0);
        ub_vec[0] = real_to_q16(3.0);
        ub_vec[1] = real_to_q16(3.0);
        ub_vec[2] = real_to_q16(3.0);

        alpha_smooth = real_to_q16(0.70);
        sigma_min    = real_to_q16(0.05);

        @(posedge clk);
        init_cem = 1'b1;
        @(posedge clk);
        init_cem = 1'b0;
        #10;

        @(posedge clk);
        opt_valid = 1'b1;
        @(posedge clk);
        opt_valid = 1'b0;
        @(posedge opt_done);
        #10;

        x0_best  = q16_to_real(best_solution[0]);
        x1_best  = q16_to_real(best_solution[1]);
        x2_best  = q16_to_real(best_solution[2]);
        fit_best = q16_to_real(best_fitness);

        $display("--> 3D MULTI-MODAL CEM RESULTS:");
        $display("    Completed Iterations:  %0d", iter_count);
        $display("    Best Solution x*:      (%f, %f, %f)", x0_best, x1_best, x2_best);
        $display("    Minimum Fitness F*:    %f", fit_best);

        if (fit_best < 0.10 && x0_best > -0.30 && x0_best < 0.30 &&
            x1_best > -0.30 && x1_best < 0.30 && x2_best > -0.30 && x2_best < 0.30) begin
            $display("[TEST 3 PASSED] Successfully optimized 3D multi-modal trajectory via Cross-Entropy Method!");
        end else begin
            $display("[TEST 3 FAILED] 3D fitness solution outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL CROSS-ENTROPY METHOD (CEM) HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
