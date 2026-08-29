// =============================================================================
// File Name   : tb_frank_wolfe.sv
// Module Name : tb_frank_wolfe
// Project     : Frank-Wolfe / Conditional Gradient Accelerator (Solver #20)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for Frank-Wolfe Accelerator.
//   Verifies:
//   1. L1 Ball Constrained Quadratic Optimization (Sparse Vertex Recovery)
//   2. Probability Simplex Constrained Optimization (Distribution Tracking)
//   3. Hyperbox Constrained Quadratic Optimization
// =============================================================================

`timescale 1ns / 1ps

import fw_types_pkg::*;
`include "fw_helpers.svh"

module tb_frank_wolfe;

    logic          clk;
    logic          rst_n;

    // Controls & Inputs
    logic          start;
    logic [2:0]    dim_n;
    fw_geom_t      geom;
    fw_step_mode_t step_mode;
    mat_t          q_matrix;
    vec_t          b_vector;
    vec_t          x_init;
    q16_t          radius_l1;
    vec_t          box_lower;
    vec_t          box_upper;
    q16_t          tol_duality_gap;
    logic [15:0]   max_iters;

    // Results & Outputs
    vec_t          x_optimal;
    q16_t          f_optimal;
    q16_t          duality_gap;
    logic [15:0]   iter_count;
    status_t       status;
    logic          done;
    logic          busy;

    // Instantiate Top Module
    fw_top dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .dim_n          (dim_n),
        .geom           (geom),
        .step_mode      (step_mode),
        .q_matrix       (q_matrix),
        .b_vector       (b_vector),
        .x_init         (x_init),
        .radius_l1      (radius_l1),
        .box_lower      (box_lower),
        .box_upper      (box_upper),
        .tol_duality_gap(tol_duality_gap),
        .max_iters      (max_iters),
        .x_optimal      (x_optimal),
        .f_optimal      (f_optimal),
        .duality_gap    (duality_gap),
        .iter_count     (iter_count),
        .status         (status),
        .done           (done),
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

    initial begin
        $display("==================================================================");
        $display(" Frank-Wolfe / Conditional Gradient Accelerator TB (Solver #20)");
        $display("==================================================================");

        // Reset
        rst_n           = 0;
        start           = 0;
        dim_n           = 3'd2;
        geom            = GEOM_L1_BALL;
        step_mode       = STEP_EXACT_LINE_SEARCH;
        q_matrix        = '0;
        b_vector        = '0;
        x_init          = '0;
        radius_l1       = Q16_ONE;
        box_lower       = '0;
        box_upper       = '0;
        tol_duality_gap = 0;
        max_iters       = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: L1 Ball Constrained Quadratic Optimization (Sparse FW)
        // Problem: min 0.5*(x0^2 + x1^2) - (2*x0 + 1*x1)  s.t.  ||x||_1 <= 1.0
        // Target constrained vertex optimum: x* = [1.000000, 0.000000]
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] L1 Ball Constrained Quadratic Optimization (Sparse FW)");
        $display("Problem: min 0.5*(x0^2 + x1^2) - (2*x0 + x1)  s.t.  ||x||_1 <= 1.0");
        $display("Target Sparse Vertex: x* = [1.000000, 0.000000]");

        q_matrix = '0;
        q_matrix = set_mat(q_matrix, 2'd0, 2'd0, real_to_q16(1.0));
        q_matrix = set_mat(q_matrix, 2'd1, 2'd1, real_to_q16(1.0));

        b_vector[0] = real_to_q16(2.0);
        b_vector[1] = real_to_q16(1.0);
        b_vector[2] = Q16_ZERO;
        b_vector[3] = Q16_ZERO;

        x_init[0] = real_to_q16(0.0);
        x_init[1] = real_to_q16(0.0);
        x_init[2] = Q16_ZERO;
        x_init[3] = Q16_ZERO;

        @(posedge clk);
        dim_n           = 3'd2;
        geom            = GEOM_L1_BALL;
        step_mode       = STEP_EXACT_LINE_SEARCH;
        radius_l1       = real_to_q16(1.0);
        tol_duality_gap = real_to_q16(0.001);
        max_iters       = 16'd50;
        start           = 1'b1;
        @(posedge clk);
        start           = 1'b0;

        $display("Starting Frank-Wolfe L1 Ball Optimizer from (0, 0)...");
        @(posedge done);
        #1;

        $display("--> FRANK-WOLFE L1 SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    Duality Gap:  %f", q16_to_real(duality_gap));
        $display("    x0_optimal:   %f (Expected: ~ 1.000000)", q16_to_real(x_optimal[0]));
        $display("    x1_optimal:   %f (Expected: ~ 0.000000)", q16_to_real(x_optimal[1]));
        $display("    f_optimal:    %f (Expected: ~ -1.500000)", q16_to_real(f_optimal));

        if (q16_to_real(x_optimal[0]) > 0.95 && q16_to_real(x_optimal[0]) < 1.05 &&
            q16_to_real(x_optimal[1]) > -0.05 && q16_to_real(x_optimal[1]) < 0.05) begin
            $display("[TEST 1 PASSED] Successfully converged to sparse L1 ball vertex!");
        end else begin
            $display("[TEST 1 FAILED] L1 optimizer outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: Probability Simplex Constrained Optimization (Simplex FW)
        // Problem: min 0.5*||x - [0.1, 0.7, 0.8, 0.0]||^2  s.t.  sum(x_i) = 1, x_i >= 0
        // Target Simplex Optimum: x* ≈ [0.000000, 0.450000, 0.550000, 0.000000]
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Probability Simplex Constrained Optimization (Simplex FW)");
        $display("Target Point to Project: b = [0.100000, 0.700000, 0.800000, 0.000000]");
        $display("Target Simplex Solution: x* ≈ [0.000000, 0.450000, 0.550000, 0.000000]");

        q_matrix = '0;
        for (int i = 0; i < 4; i++) begin
            q_matrix = set_mat(q_matrix, 2'(i), 2'(i), real_to_q16(1.0));
        end

        b_vector[0] = real_to_q16(0.1);
        b_vector[1] = real_to_q16(0.7);
        b_vector[2] = real_to_q16(0.8);
        b_vector[3] = real_to_q16(0.0);

        // Initial feasible simplex point: [0.25, 0.25, 0.25, 0.25]
        x_init[0] = real_to_q16(0.25);
        x_init[1] = real_to_q16(0.25);
        x_init[2] = real_to_q16(0.25);
        x_init[3] = real_to_q16(0.25);

        @(posedge clk);
        dim_n           = 3'd4;
        geom            = GEOM_SIMPLEX;
        step_mode       = STEP_EXACT_LINE_SEARCH;
        tol_duality_gap = real_to_q16(0.005);
        max_iters       = 16'd50;
        start           = 1'b1;
        @(posedge clk);
        start           = 1'b0;

        $display("Starting Frank-Wolfe Simplex Optimizer...");
        @(posedge done);
        #1;

        $display("--> FRANK-WOLFE SIMPLEX FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    Duality Gap:  %f", q16_to_real(duality_gap));
        $display("    x0_optimal:   %f (Expected: ~ 0.000000)", q16_to_real(x_optimal[0]));
        $display("    x1_optimal:   %f (Expected: ~ 0.450000)", q16_to_real(x_optimal[1]));
        $display("    x2_optimal:   %f (Expected: ~ 0.550000)", q16_to_real(x_optimal[2]));
        $display("    x3_optimal:   %f (Expected: ~ 0.000000)", q16_to_real(x_optimal[3]));
        $display("    Sum(x_i):     %f (Expected: 1.000000)",
                 q16_to_real(x_optimal[0]) + q16_to_real(x_optimal[1]) +
                 q16_to_real(x_optimal[2]) + q16_to_real(x_optimal[3]));

        if (q16_to_real(x_optimal[0]) > -0.05 && q16_to_real(x_optimal[0]) < 0.05 &&
            q16_to_real(x_optimal[1]) > 0.40 && q16_to_real(x_optimal[1]) < 0.50 &&
            q16_to_real(x_optimal[2]) > 0.50 && q16_to_real(x_optimal[2]) < 0.60 &&
            q16_to_real(x_optimal[3]) > -0.05 && q16_to_real(x_optimal[3]) < 0.05) begin
            $display("[TEST 2 PASSED] Successfully solved probability simplex constrained optimization!");
        end else begin
            $display("[TEST 2 FAILED] Simplex optimizer outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: Hyperbox Constrained Quadratic Optimization (Box FW)
        // Problem: min 0.5*(x0^2 + x1^2) - (2.5*x0 + 0.5*x1)  s.t.  -1 <= x0, x1 <= 1
        // Target Box Solution: x0* = 1.000000 (at active bound), x1* = 0.500000 (interior)
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Hyperbox Constrained Quadratic Optimization (Box FW)");
        $display("Problem: min 0.5*(x0^2 + x1^2) - (2.5*x0 + 0.5*x1)  s.t.  [-1.0, -1.0] <= x <= [1.0, 1.0]");
        $display("Target Solution: x* = [1.000000, 0.500000]");

        q_matrix = '0;
        q_matrix = set_mat(q_matrix, 2'd0, 2'd0, real_to_q16(1.0));
        q_matrix = set_mat(q_matrix, 2'd1, 2'd1, real_to_q16(1.0));

        b_vector[0] = real_to_q16(2.5);
        b_vector[1] = real_to_q16(0.5);
        b_vector[2] = Q16_ZERO;
        b_vector[3] = Q16_ZERO;

        box_lower[0] = real_to_q16(-1.0);
        box_lower[1] = real_to_q16(-1.0);
        box_upper[0] = real_to_q16(1.0);
        box_upper[1] = real_to_q16(1.0);

        x_init[0] = real_to_q16(0.0);
        x_init[1] = real_to_q16(0.0);

        @(posedge clk);
        dim_n           = 3'd2;
        geom            = GEOM_BOX;
        step_mode       = STEP_EXACT_LINE_SEARCH;
        tol_duality_gap = real_to_q16(0.001);
        max_iters       = 16'd50;
        start           = 1'b1;
        @(posedge clk);
        start           = 1'b0;

        $display("Starting Frank-Wolfe Hyperbox Optimizer...");
        @(posedge done);
        #1;

        $display("--> FRANK-WOLFE BOX FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    Duality Gap:  %f", q16_to_real(duality_gap));
        $display("    x0_optimal:   %f (Expected: ~ 1.000000)", q16_to_real(x_optimal[0]));
        $display("    x1_optimal:   %f (Expected: ~ 0.500000)", q16_to_real(x_optimal[1]));
        $display("    f_optimal:    %f (Expected: ~ -2.125000)", q16_to_real(f_optimal));

        if (q16_to_real(x_optimal[0]) > 0.95 && q16_to_real(x_optimal[0]) < 1.05 &&
            q16_to_real(x_optimal[1]) > 0.45 && q16_to_real(x_optimal[1]) < 0.55) begin
            $display("[TEST 3 PASSED] Successfully converged on active box constraint!");
        end else begin
            $display("[TEST 3 FAILED] Box optimizer outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL FRANK-WOLFE / CONDITIONAL GRADIENT TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
