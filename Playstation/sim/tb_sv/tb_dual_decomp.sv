// =============================================================================
// File Name   : tb_dual_decomp.sv
// Module Name : tb_dual_decomp
// Project     : Dual Decomposition Engine (Solver #22)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for Dual Decomposition Engine.
//   Verifies:
//   1. 3-Agent Microgrid Distributed Power Allocation (Equality Budget)
//   2. 2-Agent Resource Allocation with Inequality Capacity Budget
//   3. Multi-Resource 2D Coupled Multi-Agent Allocation (2 Shared Resources)
// =============================================================================

`timescale 1ns / 1ps

import dd_types_pkg::*;
`include "dd_helpers.svh"

module tb_dual_decomp;

    logic               clk;
    logic               rst_n;

    // Controls & Inputs
    logic               start;
    logic [2:0]         num_agents;
    logic [1:0]         local_dim;
    logic [1:0]         num_res;
    dd_couple_mode_t    couple_mode;
    agent_mat_t         q_inv_mats[MAX_AGENTS];
    agent_vec_t         p_vectors[MAX_AGENTS];
    couple_mat_t        a_matrices[MAX_AGENTS];
    res_vec_t           c_budget;
    res_vec_t           lambda_init;
    q16_t               step_alpha;
    q16_t               tol_feas;
    q16_t               tol_opt;
    logic [15:0]        max_iters;

    // Results & Outputs
    all_agents_vec_t    x_optimal;
    res_vec_t           lambda_optimal;
    q16_t               feas_error;
    logic [15:0]        iter_count;
    status_t            status;
    logic               done;
    logic               busy;

    // Instantiate Top Module
    dd_top dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (start),
        .num_agents    (num_agents),
        .local_dim     (local_dim),
        .num_res       (num_res),
        .couple_mode   (couple_mode),
        .q_inv_mats    (q_inv_mats),
        .p_vectors     (p_vectors),
        .a_matrices    (a_matrices),
        .c_budget      (c_budget),
        .lambda_init   (lambda_init),
        .step_alpha    (step_alpha),
        .tol_feas      (tol_feas),
        .tol_opt       (tol_opt),
        .max_iters     (max_iters),
        .x_optimal     (x_optimal),
        .lambda_optimal(lambda_optimal),
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
        $display(" Dual Decomposition Engine TB (Solver #22)");
        $display("==================================================================");

        // Reset
        rst_n       = 0;
        start       = 0;
        num_agents  = 3'd3;
        local_dim   = 2'd1;
        num_res     = 2'd1;
        couple_mode = COUPLE_EQ;
        for (int s = 0; s < MAX_AGENTS; s++) begin
            q_inv_mats[s] = '0;
            p_vectors[s]  = '0;
            a_matrices[s] = '0;
        end
        c_budget    = '0;
        lambda_init = '0;
        step_alpha  = 0;
        tol_feas    = 0;
        tol_opt     = 0;
        max_iters   = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 3-Agent Microgrid Distributed Power Allocation (Equality Budget)
        // Generator 1: f1 = x1^2 - 4*x1       => Q1_inv = 0.5, p1 = 4.0
        // Generator 2: f2 = 0.5*x2^2 - 3*x2   => Q2_inv = 1.0, p2 = 3.0
        // Generator 3: f3 = 1.5*x3^2 - 6*x3   => Q3_inv = 0.333333, p3 = 6.0
        // Coupling: x1 + x2 + x3 = 6.0 MW
        // Target: x* = [1.727273, 2.454545, 1.818182], λ* = 0.545455
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 3-Agent Microgrid Distributed Power Allocation (Equality Budget)");
        $display("Problem: min sum(f_s(x_s))  s.t.  x1 + x2 + x3 = 6.0 MW");
        $display("Target Decisions: x1* ≈ 1.727273, x2* ≈ 2.454545, x3* ≈ 1.818182, λ* ≈ 0.545455");

        // Agent 0
        q_inv_mats[0] = set_mat2(q_inv_mats[0], 1'b0, 1'b0, real_to_q16(0.5));
        p_vectors[0][0] = real_to_q16(4.0);
        a_matrices[0] = set_mat2(a_matrices[0], 1'b0, 1'b0, real_to_q16(1.0));

        // Agent 1
        q_inv_mats[1] = set_mat2(q_inv_mats[1], 1'b0, 1'b0, real_to_q16(1.0));
        p_vectors[1][0] = real_to_q16(3.0);
        a_matrices[1] = set_mat2(a_matrices[1], 1'b0, 1'b0, real_to_q16(1.0));

        // Agent 2
        q_inv_mats[2] = set_mat2(q_inv_mats[2], 1'b0, 1'b0, real_to_q16(0.333333));
        p_vectors[2][0] = real_to_q16(6.0);
        a_matrices[2] = set_mat2(a_matrices[2], 1'b0, 1'b0, real_to_q16(1.0));

        c_budget[0] = real_to_q16(6.0);
        c_budget[1] = Q16_ZERO;

        lambda_init[0] = real_to_q16(0.0);
        lambda_init[1] = Q16_ZERO;

        @(posedge clk);
        num_agents  = 3'd3;
        local_dim   = 2'd1;
        num_res     = 2'd1;
        couple_mode = COUPLE_EQ;
        step_alpha  = real_to_q16(0.4);
        tol_feas    = real_to_q16(0.005);
        tol_opt     = real_to_q16(0.001);
        max_iters   = 16'd100;
        start       = 1'b1;
        @(posedge clk);
        start       = 1'b0;

        $display("Starting Dual Decomposition Microgrid Coordinator...");
        @(posedge done);
        #1;

        $display("--> DUAL DECOMPOSITION MICROGRID FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    Feas Error:   %f", q16_to_real(feas_error));
        $display("    x1_optimal:   %f (Expected: ~ 1.727273)", q16_to_real(x_optimal[0][0]));
        $display("    x2_optimal:   %f (Expected: ~ 2.454545)", q16_to_real(x_optimal[1][0]));
        $display("    x3_optimal:   %f (Expected: ~ 1.818182)", q16_to_real(x_optimal[2][0]));
        $display("    λ_market:     %f (Expected: ~ 0.545455)", q16_to_real(lambda_optimal[0]));
        $display("    Total Power:  %f MW (Expected: 6.000000 MW)",
                 q16_to_real(x_optimal[0][0]) + q16_to_real(x_optimal[1][0]) + q16_to_real(x_optimal[2][0]));

        if (q16_to_real(x_optimal[0][0]) > 1.65 && q16_to_real(x_optimal[0][0]) < 1.80 &&
            q16_to_real(x_optimal[1][0]) > 2.35 && q16_to_real(x_optimal[1][0]) < 2.55 &&
            q16_to_real(x_optimal[2][0]) > 1.75 && q16_to_real(x_optimal[2][0]) < 1.90 &&
            q16_to_real(feas_error) < 0.05) begin
            $display("[TEST 1 PASSED] Successfully coordinated distributed microgrid power generation!");
        end else begin
            $display("[TEST 1 FAILED] Dual Decomposition microgrid allocation outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 2-Agent Resource Allocation with Inequality Capacity Budget
        // Agent 1: f1 = 0.5*(x1 - 4)^2 => Q1_inv = 1.0, p1 = 4.0
        // Agent 2: f2 = 0.5*(x2 - 4)^2 => Q2_inv = 1.0, p2 = 4.0
        // Capacity limit: x1 + x2 <= 5.0
        // Target: x1* = 2.500000, x2* = 2.500000, λ* = 1.500000
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 2-Agent Resource Allocation with Inequality Capacity Budget");
        $display("Problem: min sum(0.5*(x_s - 4)^2)  s.t.  x1 + x2 <= 5.0");
        $display("Target Capacity Split: x1* = 2.500000, x2* = 2.500000, λ* = 1.500000");

        for (int s = 0; s < MAX_AGENTS; s++) begin
            q_inv_mats[s] = '0;
            p_vectors[s]  = '0;
            a_matrices[s] = '0;
        end

        // Agent 0
        q_inv_mats[0] = set_mat2(q_inv_mats[0], 1'b0, 1'b0, real_to_q16(1.0));
        p_vectors[0][0] = real_to_q16(4.0);
        a_matrices[0] = set_mat2(a_matrices[0], 1'b0, 1'b0, real_to_q16(1.0));

        // Agent 1
        q_inv_mats[1] = set_mat2(q_inv_mats[1], 1'b0, 1'b0, real_to_q16(1.0));
        p_vectors[1][0] = real_to_q16(4.0);
        a_matrices[1] = set_mat2(a_matrices[1], 1'b0, 1'b0, real_to_q16(1.0));

        c_budget[0] = real_to_q16(5.0);
        c_budget[1] = Q16_ZERO;

        lambda_init[0] = real_to_q16(0.0);

        @(posedge clk);
        num_agents  = 3'd2;
        local_dim   = 2'd1;
        num_res     = 2'd1;
        couple_mode = COUPLE_INEQ;
        step_alpha  = real_to_q16(0.4);
        tol_feas    = real_to_q16(0.005);
        tol_opt     = real_to_q16(0.001);
        max_iters   = 16'd100;
        start       = 1'b1;
        @(posedge clk);
        start       = 1'b0;

        $display("Starting Dual Decomposition Capacity Allocation...");
        @(posedge done);
        #1;

        $display("--> DUAL DECOMPOSITION CAPACITY SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    Feas Error:   %f", q16_to_real(feas_error));
        $display("    x1_optimal:   %f (Expected: ~ 2.500000)", q16_to_real(x_optimal[0][0]));
        $display("    x2_optimal:   %f (Expected: ~ 2.500000)", q16_to_real(x_optimal[1][0]));
        $display("    λ_market:     %f (Expected: ~ 1.500000)", q16_to_real(lambda_optimal[0]));
        $display("    Total Alloc:  %f (Expected: <= 5.000000)",
                 q16_to_real(x_optimal[0][0]) + q16_to_real(x_optimal[1][0]));

        if (q16_to_real(x_optimal[0][0]) > 2.40 && q16_to_real(x_optimal[0][0]) < 2.60 &&
            q16_to_real(x_optimal[1][0]) > 2.40 && q16_to_real(x_optimal[1][0]) < 2.60 &&
            q16_to_real(feas_error) < 0.05) begin
            $display("[TEST 2 PASSED] Successfully allocated capacity with shadow pricing!");
        end else begin
            $display("[TEST 2 FAILED] Dual Decomposition capacity allocation outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: Multi-Resource 2D Coupled Multi-Agent Allocation (2 Shared Resources)
        // Agent 1: Q1_inv = I_2, p1 = [2.0, 1.0]^T, A1 = I_2
        // Agent 2: Q2_inv = I_2, p2 = [1.0, 2.0]^T, A2 = I_2
        // Coupling: x1 + x2 = [2.0, 2.0]^T
        // Target: x1* = [1.500000, 0.500000], x2* = [0.500000, 1.500000], λ* = [0.500000, 0.500000]
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Multi-Resource 2D Coupled Multi-Agent Allocation (2 Shared Resources)");
        $display("Problem: min sum(f_s(x_s))  s.t.  x1 + x2 = [2.000000, 2.000000]^T");
        $display("Target Decisions: x1* = [1.500000, 0.500000], x2* = [0.500000, 1.500000], λ* = [0.500000, 0.500000]");

        for (int s = 0; s < MAX_AGENTS; s++) begin
            q_inv_mats[s] = '0;
            p_vectors[s]  = '0;
            a_matrices[s] = '0;
        end

        // Agent 0 (2D)
        q_inv_mats[0] = set_mat2(q_inv_mats[0], 1'b0, 1'b0, real_to_q16(1.0));
        q_inv_mats[0] = set_mat2(q_inv_mats[0], 1'b1, 1'b1, real_to_q16(1.0));
        p_vectors[0][0] = real_to_q16(2.0);
        p_vectors[0][1] = real_to_q16(1.0);
        a_matrices[0] = set_mat2(a_matrices[0], 1'b0, 1'b0, real_to_q16(1.0));
        a_matrices[0] = set_mat2(a_matrices[0], 1'b1, 1'b1, real_to_q16(1.0));

        // Agent 1 (2D)
        q_inv_mats[1] = set_mat2(q_inv_mats[1], 1'b0, 1'b0, real_to_q16(1.0));
        q_inv_mats[1] = set_mat2(q_inv_mats[1], 1'b1, 1'b1, real_to_q16(1.0));
        p_vectors[1][0] = real_to_q16(1.0);
        p_vectors[1][1] = real_to_q16(2.0);
        a_matrices[1] = set_mat2(a_matrices[1], 1'b0, 1'b0, real_to_q16(1.0));
        a_matrices[1] = set_mat2(a_matrices[1], 1'b1, 1'b1, real_to_q16(1.0));

        c_budget[0] = real_to_q16(2.0);
        c_budget[1] = real_to_q16(2.0);

        lambda_init[0] = real_to_q16(0.0);
        lambda_init[1] = real_to_q16(0.0);

        @(posedge clk);
        num_agents  = 3'd2;
        local_dim   = 2'd2;
        num_res     = 2'd2;
        couple_mode = COUPLE_EQ;
        step_alpha  = real_to_q16(0.4);
        tol_feas    = real_to_q16(0.005);
        tol_opt     = real_to_q16(0.001);
        max_iters   = 16'd100;
        start       = 1'b1;
        @(posedge clk);
        start       = 1'b0;

        $display("Starting Dual Decomposition 2D Multi-Resource Coordinator...");
        @(posedge done);
        #1;

        $display("--> DUAL DECOMPOSITION 2D SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    Feas Error:   %f", q16_to_real(feas_error));
        $display("    x1_optimal:   [%f, %f] (Expected: ~ [1.500000, 0.500000])",
                 q16_to_real(x_optimal[0][0]), q16_to_real(x_optimal[0][1]));
        $display("    x2_optimal:   [%f, %f] (Expected: ~ [0.500000, 1.500000])",
                 q16_to_real(x_optimal[1][0]), q16_to_real(x_optimal[1][1]));
        $display("    λ_market:     [%f, %f] (Expected: ~ [0.500000, 0.500000])",
                 q16_to_real(lambda_optimal[0]), q16_to_real(lambda_optimal[1]));
        $display("    Total Res 1:  %f (Expected: 2.000000)",
                 q16_to_real(x_optimal[0][0]) + q16_to_real(x_optimal[1][0]));
        $display("    Total Res 2:  %f (Expected: 2.000000)",
                 q16_to_real(x_optimal[0][1]) + q16_to_real(x_optimal[1][1]));

        if (q16_to_real(x_optimal[0][0]) > 1.40 && q16_to_real(x_optimal[0][0]) < 1.60 &&
            q16_to_real(x_optimal[0][1]) > 0.40 && q16_to_real(x_optimal[0][1]) < 0.60 &&
            q16_to_real(x_optimal[1][0]) > 0.40 && q16_to_real(x_optimal[1][0]) < 0.60 &&
            q16_to_real(x_optimal[1][1]) > 1.40 && q16_to_real(x_optimal[1][1]) < 1.60 &&
            q16_to_real(feas_error) < 0.05) begin
            $display("[TEST 3 PASSED] Successfully balanced multi-dimensional coupled resources!");
        end else begin
            $display("[TEST 3 FAILED] Dual Decomposition 2D multi-resource solver outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL DUAL DECOMPOSITION HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
