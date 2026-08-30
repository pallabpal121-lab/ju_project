// =============================================================================
// File Name   : tb_alm.sv
// Module Name : tb_alm
// Project     : Augmented Lagrangian Method (ALM) Accelerator (Solver #21)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for ALM Accelerator.
//   Verifies:
//   1. Coupled Quadratic with Linear Equality Constraint (A x = b)
//   2. Quadratic with Linear Inequality Constraint (C x <= d)
//   3. 3D Multi-Constraint Actuator / Power Allocation (Equality + Inequality)
// =============================================================================

`timescale 1ns / 1ps

import alm_types_pkg::*;
`include "alm_helpers.svh"

module tb_alm;

    logic          clk;
    logic          rst_n;

    // Controls & Inputs
    logic          start;
    logic [2:0]    dim_n;
    logic [2:0]    num_eq;
    logic [2:0]    num_ineq;
    mat_t          q_matrix;
    vec_t          p_vector;
    mat_t          a_matrix;
    vec_t          b_vector;
    mat_t          c_matrix;
    vec_t          d_vector;
    vec_t          x_init;
    q16_t          tol_feas;
    q16_t          tol_opt;
    logic [15:0]   max_iters;

    // Results & Outputs
    vec_t          x_optimal;
    vec_t          lambda_optimal;
    vec_t          mu_optimal;
    q16_t          f_optimal;
    q16_t          feas_error;
    logic [15:0]   iter_count;
    status_t       status;
    logic          done;
    logic          busy;

    // Instantiate Top Module
    alm_top dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (start),
        .dim_n         (dim_n),
        .num_eq        (num_eq),
        .num_ineq      (num_ineq),
        .q_matrix      (q_matrix),
        .p_vector      (p_vector),
        .a_matrix      (a_matrix),
        .b_vector      (b_vector),
        .c_matrix      (c_matrix),
        .d_vector      (d_vector),
        .x_init        (x_init),
        .tol_feas      (tol_feas),
        .tol_opt       (tol_opt),
        .max_iters     (max_iters),
        .x_optimal     (x_optimal),
        .lambda_optimal(lambda_optimal),
        .mu_optimal    (mu_optimal),
        .f_optimal     (f_optimal),
        .feas_error    (feas_error),
        .iter_count    (iter_count),
        .status        (status),
        .done          (done),
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

    initial begin
        $display("==================================================================");
        $display(" Augmented Lagrangian Method (ALM) Accelerator TB (Solver #21)");
        $display("==================================================================");

        // Reset
        rst_n     = 0;
        start     = 0;
        dim_n     = 3'd2;
        num_eq    = 3'd0;
        num_ineq  = 3'd0;
        q_matrix  = '0;
        p_vector  = '0;
        a_matrix  = '0;
        b_vector  = '0;
        c_matrix  = '0;
        d_vector  = '0;
        x_init    = '0;
        tol_feas  = 0;
        tol_opt   = 0;
        max_iters = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: Coupled Quadratic with Linear Equality Constraint (A x = b)
        // Problem: min 0.5*(x0 - 2)^2 + 0.5*(x1 - 4)^2  s.t.  x0 + x1 = 4.0
        // Target constrained minimum: x* = [1.000000, 3.000000], λ* = 1.000000
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Coupled Quadratic with Linear Equality Constraint (A x = b)");
        $display("Problem: min 0.5*(x0 - 2)^2 + 0.5*(x1 - 4)^2  s.t.  x0 + x1 = 4.0");
        $display("Target Constrained Optimum: x* = [1.000000, 3.000000], λ* = 1.000000");

        q_matrix = '0;
        q_matrix = set_mat(q_matrix, 2'd0, 2'd0, real_to_q16(1.0));
        q_matrix = set_mat(q_matrix, 2'd1, 2'd1, real_to_q16(1.0));

        p_vector[0] = real_to_q16(2.0);
        p_vector[1] = real_to_q16(4.0);
        p_vector[2] = Q16_ZERO;
        p_vector[3] = Q16_ZERO;

        a_matrix = '0;
        a_matrix = set_mat(a_matrix, 2'd0, 2'd0, real_to_q16(1.0));
        a_matrix = set_mat(a_matrix, 2'd0, 2'd1, real_to_q16(1.0));

        b_vector[0] = real_to_q16(4.0);
        b_vector[1] = Q16_ZERO;

        x_init[0] = real_to_q16(0.0);
        x_init[1] = real_to_q16(0.0);

        @(posedge clk);
        dim_n     = 3'd2;
        num_eq    = 3'd1;
        num_ineq  = 3'd0;
        tol_feas  = real_to_q16(0.001);
        tol_opt   = real_to_q16(0.001);
        max_iters = 16'd50;
        start     = 1'b1;
        @(posedge clk);
        start     = 1'b0;

        $display("Starting ALM Equality Constrained Optimizer from (0, 0)...");
        @(posedge done);
        #1;

        $display("--> ALM EQUALITY SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    Feas Error:   %f", q16_to_real(feas_error));
        $display("    x0_optimal:   %f (Expected: ~ 1.000000)", q16_to_real(x_optimal[0]));
        $display("    x1_optimal:   %f (Expected: ~ 3.000000)", q16_to_real(x_optimal[1]));
        $display("    λ0_optimal:   %f (Expected: ~ 1.000000)", q16_to_real(lambda_optimal[0]));
        $display("    Constraint:   x0 + x1 = %f (Expected: 4.000000)",
                 q16_to_real(x_optimal[0]) + q16_to_real(x_optimal[1]));

        if (q16_to_real(x_optimal[0]) > 0.95 && q16_to_real(x_optimal[0]) < 1.05 &&
            q16_to_real(x_optimal[1]) > 2.95 && q16_to_real(x_optimal[1]) < 3.05 &&
            q16_to_real(feas_error) < 0.05) begin
            $display("[TEST 1 PASSED] Successfully converged to equality constrained optimum!");
        end else begin
            $display("[TEST 1 FAILED] ALM equality solver outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: Quadratic with Linear Inequality Constraint (C x <= d)
        // Problem: min 0.5*(x0 - 3)^2 + 0.5*(x1 - 3)^2  s.t.  x0 + 2*x1 <= 3.0
        // Target active inequality optimum: x* = [1.800000, 0.600000], μ* = 1.200000
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Quadratic with Linear Inequality Constraint (C x <= d)");
        $display("Problem: min 0.5*(x0 - 3)^2 + 0.5*(x1 - 3)^2  s.t.  x0 + 2*x1 <= 3.0");
        $display("Target Active Optimum: x* = [1.800000, 0.600000], μ* = 1.200000");

        q_matrix = '0;
        q_matrix = set_mat(q_matrix, 2'd0, 2'd0, real_to_q16(1.0));
        q_matrix = set_mat(q_matrix, 2'd1, 2'd1, real_to_q16(1.0));

        p_vector[0] = real_to_q16(3.0);
        p_vector[1] = real_to_q16(3.0);

        a_matrix = '0;
        b_vector = '0;

        c_matrix = '0;
        c_matrix = set_mat(c_matrix, 2'd0, 2'd0, real_to_q16(1.0));
        c_matrix = set_mat(c_matrix, 2'd0, 2'd1, real_to_q16(2.0));

        d_vector[0] = real_to_q16(3.0);
        d_vector[1] = Q16_ZERO;

        x_init[0] = real_to_q16(0.0);
        x_init[1] = real_to_q16(0.0);

        @(posedge clk);
        dim_n     = 3'd2;
        num_eq    = 3'd0;
        num_ineq  = 3'd1;
        tol_feas  = real_to_q16(0.001);
        tol_opt   = real_to_q16(0.001);
        max_iters = 16'd50;
        start     = 1'b1;
        @(posedge clk);
        start     = 1'b0;

        $display("Starting ALM Inequality Constrained Optimizer...");
        @(posedge done);
        #1;

        $display("--> ALM INEQUALITY SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    Feas Error:   %f", q16_to_real(feas_error));
        $display("    x0_optimal:   %f (Expected: ~ 1.800000)", q16_to_real(x_optimal[0]));
        $display("    x1_optimal:   %f (Expected: ~ 0.600000)", q16_to_real(x_optimal[1]));
        $display("    μ0_optimal:   %f (Expected: ~ 1.200000)", q16_to_real(mu_optimal[0]));
        $display("    Constraint:   x0 + 2*x1 = %f (Expected: <= 3.000000)",
                 q16_to_real(x_optimal[0]) + 2.0 * q16_to_real(x_optimal[1]));

        if (q16_to_real(x_optimal[0]) > 1.70 && q16_to_real(x_optimal[0]) < 1.90 &&
            q16_to_real(x_optimal[1]) > 0.50 && q16_to_real(x_optimal[1]) < 0.70 &&
            q16_to_real(feas_error) < 0.05) begin
            $display("[TEST 2 PASSED] Successfully converged to active inequality bound!");
        end else begin
            $display("[TEST 2 FAILED] ALM inequality solver outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 3D Multi-Constraint Actuator Allocation (Equality + Inequality)
        // Problem: min 0.5*(x0^2 + x1^2 + x2^2) - (x0 + 2*x1 + 3*x2)
        //          s.t.  x0 + x1 + x2 = 3.0,  x2 <= 1.5
        // Target Solution: x* = [0.250000, 1.250000, 1.500000]
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 3D Multi-Constraint Actuator Allocation (Equality + Inequality)");
        $display("Problem: min 0.5*||x||^2 - (x0 + 2*x1 + 3*x2)  s.t.  sum(x)=3.0, x2 <= 1.5");
        $display("Target Solution: x* = [0.250000, 1.250000, 1.500000]");

        q_matrix = '0;
        q_matrix = set_mat(q_matrix, 2'd0, 2'd0, real_to_q16(1.0));
        q_matrix = set_mat(q_matrix, 2'd1, 2'd1, real_to_q16(1.0));
        q_matrix = set_mat(q_matrix, 2'd2, 2'd2, real_to_q16(1.0));

        p_vector[0] = real_to_q16(1.0);
        p_vector[1] = real_to_q16(2.0);
        p_vector[2] = real_to_q16(3.0);
        p_vector[3] = Q16_ZERO;

        a_matrix = '0;
        a_matrix = set_mat(a_matrix, 2'd0, 2'd0, real_to_q16(1.0));
        a_matrix = set_mat(a_matrix, 2'd0, 2'd1, real_to_q16(1.0));
        a_matrix = set_mat(a_matrix, 2'd0, 2'd2, real_to_q16(1.0));

        b_vector[0] = real_to_q16(3.0);
        b_vector[1] = Q16_ZERO;

        c_matrix = '0;
        c_matrix = set_mat(c_matrix, 2'd0, 2'd2, real_to_q16(1.0));

        d_vector[0] = real_to_q16(1.5);
        d_vector[1] = Q16_ZERO;

        x_init[0] = real_to_q16(0.0);
        x_init[1] = real_to_q16(0.0);
        x_init[2] = real_to_q16(0.0);

        @(posedge clk);
        dim_n     = 3'd3;
        num_eq    = 3'd1;
        num_ineq  = 3'd1;
        tol_feas  = real_to_q16(0.001);
        tol_opt   = real_to_q16(0.001);
        max_iters = 16'd50;
        start     = 1'b1;
        @(posedge clk);
        start     = 1'b0;

        $display("Starting ALM Mixed Equality/Inequality Optimizer...");
        @(posedge done);
        #1;

        $display("--> ALM MULTI-CONSTRAINT SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    Feas Error:   %f", q16_to_real(feas_error));
        $display("    x0_optimal:   %f (Expected: ~ 0.250000)", q16_to_real(x_optimal[0]));
        $display("    x1_optimal:   %f (Expected: ~ 1.250000)", q16_to_real(x_optimal[1]));
        $display("    x2_optimal:   %f (Expected: ~ 1.500000)", q16_to_real(x_optimal[2]));
        $display("    sum(x_i):     %f (Expected: 3.000000)",
                 q16_to_real(x_optimal[0]) + q16_to_real(x_optimal[1]) + q16_to_real(x_optimal[2]));

        if (q16_to_real(x_optimal[0]) > 0.20 && q16_to_real(x_optimal[0]) < 0.30 &&
            q16_to_real(x_optimal[1]) > 1.20 && q16_to_real(x_optimal[1]) < 1.30 &&
            q16_to_real(x_optimal[2]) > 1.45 && q16_to_real(x_optimal[2]) < 1.55 &&
            q16_to_real(feas_error) < 0.05) begin
            $display("[TEST 3 PASSED] Successfully solved mixed equality and inequality constrained QP!");
        end else begin
            $display("[TEST 3 FAILED] ALM mixed solver outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL AUGMENTED LAGRANGIAN (ALM) HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
