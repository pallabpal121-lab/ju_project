// =============================================================================
// File Name   : sqp_top.sv
// Module Name : sqp_top
// Project     : Sequential Quadratic Programming (SQP) Accelerator (Solver #7)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Top-level SoC Module for the SQP Constrained Optimization Accelerator.
//   Minimizes arbitrary non-linear objective functions subject to physical box
//   inequality bounds:
//     min f(x)  subject to  l_i <= x_i <= u_i
//   Solves the local QP subproblem via active-set projection and Cholesky factorization.
// =============================================================================

`timescale 1ns / 1ps

import sqp_types_pkg::*;
`include "sqp_helpers.svh"

module sqp_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Model Microcode Programming Port
    // -------------------------------------------------------------------------
    input  logic               prog_en,       // Microcode Write Enable
    input  logic [4:0]         prog_addr,     // Microcode Address (0..31)
    input  instr_t             prog_data,     // 32-bit Microcode Instruction

    // -------------------------------------------------------------------------
    // Interface 2: Optimization Controls, Bounds & Parameters
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start pulse
    input  logic [2:0]         num_params,    // Number of parameters N (1..4)
    input  vec_t               x_init,        // Initial parameter guess [x3, x2, x1, x0]
    input  box_bounds_t        bounds,        // Lower (l) and Upper (u) physical bounds
    input  q16_t               step_alpha,    // Step size scale α
    input  q16_t               tolerance,     // Convergence threshold on free gradient norm
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 3: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,     // Converged feasible parameter vector x*
    output q16_t               f_optimal,     // Final objective function value f(x*)
    output q16_t               g_free_norm,   // Max free gradient component norm
    output active_mask_t       active_mask,   // Bitmask of active boundary constraints
    output vec_t               vec_mu,        // Lagrange multipliers μ_i for active bounds
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while solving
);

    // -------------------------------------------------------------------------
    // Master FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        SQP_IDLE        = 3'd0,
        SQP_START_HESS  = 3'd1,
        SQP_WAIT_HESS   = 3'd2,
        SQP_START_ACT   = 3'd3,
        SQP_WAIT_ACT    = 3'd4,
        SQP_CHECK_CONV  = 3'd5,
        SQP_START_SOLVE = 3'd6,
        SQP_WAIT_SOLVE  = 3'd7
    } sqp_state_t;

    sqp_state_t state;

    // Configuration & State Registers
    vec_t        x_reg;
    logic [2:0]  num_params_reg;
    box_bounds_t bounds_reg;
    q16_t        alpha_reg;
    q16_t        tol_reg;
    logic [7:0]  max_iters_reg;

    // Hessian Engine Interconnect
    logic hess_start;
    q16_t hess_f_0;
    vec_t hess_vec_g;
    mat_t hess_mat_h;
    logic hess_done, hess_busy;

    // Active-Set Engine Interconnect
    logic         act_start;
    active_mask_t act_mask_out;
    mat_t         act_h_proj;
    vec_t         act_g_proj;
    vec_t         act_mu_out;
    q16_t         act_g_free_norm;
    logic         act_done, act_busy;

    // Cholesky Solver Interconnect
    logic chol_start;
    vec_t chol_vec_p;
    logic chol_done, chol_singular, chol_busy;

    // Helper fixed-point multiplier
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Submodule 1: Finite-Difference Objective, Gradient & Hessian Engine
    sqp_hessian_engine u_hess_engine (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (hess_start),
        .num_params (num_params_reg),
        .x_curr     (x_reg),
        .f_0_out    (hess_f_0),
        .vec_g_out  (hess_vec_g),
        .mat_h_out  (hess_mat_h),
        .done       (hess_done),
        .busy       (hess_busy)
    );

    // Submodule 2: Active-Set Constraint Classifier & KKT Builder
    sqp_active_set_engine u_act_engine (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (act_start),
        .num_params      (num_params_reg),
        .x_curr          (x_reg),
        .vec_g           (hess_vec_g),
        .mat_h           (hess_mat_h),
        .bounds          (bounds_reg),
        .active_mask     (act_mask_out),
        .mat_h_proj      (act_h_proj),
        .vec_g_proj      (act_g_proj),
        .vec_mu          (act_mu_out),
        .g_free_norm_inf (act_g_free_norm),
        .done            (act_done),
        .busy            (act_busy)
    );

    // Submodule 3: Hardware Cholesky Linear Solver: H_proj · p = -g_proj
    cholesky_solver_engine u_chol_solver (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (chol_start),
        .num_vars   (num_params_reg),
        .mat_a      (act_h_proj),
        .vec_g      (act_g_proj),
        .vec_p      (chol_vec_p),
        .done       (chol_done),
        .singular   (chol_singular),
        .busy       (chol_busy)
    );

    vec_t tmp_next_x;
    q16_t cand_x, cur_l, cur_u;

    // Master SQP Loop FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= SQP_IDLE;
            num_params_reg <= 3'd2;
            bounds_reg     <= '0;
            alpha_reg      <= Q16_ONE;
            tol_reg        <= Q16_EPS_DEF;
            max_iters_reg  <= 8'd50;
            iter_count     <= 8'd0;
            status         <= STATUS_IDLE;
            done           <= 1'b0;
            busy           <= 1'b0;
            x_optimal      <= '0;
            f_optimal      <= Q16_ZERO;
            g_free_norm    <= Q16_ZERO;
            active_mask    <= '0;
            vec_mu         <= '0;
            x_reg          <= '0;
            hess_start     <= 1'b0;
            act_start      <= 1'b0;
            chol_start     <= 1'b0;
        end else begin
            hess_start <= 1'b0;
            act_start  <= 1'b0;
            chol_start <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: SQP_IDLE
                // -------------------------------------------------------------
                SQP_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy           <= 1'b1;
                        num_params_reg <= (num_params != 3'd0)     ? num_params : 3'd2;
                        bounds_reg     <= bounds;
                        alpha_reg      <= (step_alpha != Q16_ZERO) ? step_alpha : Q16_ONE;
                        tol_reg        <= (tolerance != Q16_ZERO)  ? tolerance  : Q16_EPS_DEF;
                        max_iters_reg  <= (max_iters != 8'd0)      ? max_iters  : 8'd50;
                        iter_count     <= 8'd0;
                        status         <= STATUS_RUNNING;

                        // Project initial guess into feasible box
                        tmp_next_x = x_init;
                        for (int i = 0; i < MAX_PARAMS; i++) begin
                            if (i < num_params) begin
                                tmp_next_x = set_vec(tmp_next_x, 2'(i),
                                    clamp_q16(get_vec(x_init, 2'(i)),
                                              get_vec(bounds.lower_bound, 2'(i)),
                                              get_vec(bounds.upper_bound, 2'(i))));
                            end
                        end
                        x_reg <= tmp_next_x;
                        state <= SQP_START_HESS;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // 1. EVALUATE OBJECTIVE f(x), GRADIENT g(x), AND HESSIAN H(x)
                // -------------------------------------------------------------
                SQP_START_HESS: begin
                    hess_start <= 1'b1;
                    state      <= SQP_WAIT_HESS;
                end

                SQP_WAIT_HESS: begin
                    if (hess_done) begin
                        state <= SQP_START_ACT;
                    end
                end

                // -------------------------------------------------------------
                // 2. CLASSIFY ACTIVE CONSTRAINTS & BUILD PROJECTED KKT SYSTEM
                // -------------------------------------------------------------
                SQP_START_ACT: begin
                    act_start <= 1'b1;
                    state     <= SQP_WAIT_ACT;
                end

                SQP_WAIT_ACT: begin
                    if (act_done) begin
                        state <= SQP_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // 3. CHECK CONVERGENCE (||g_free||_inf <= tolerance or max iters)
                // -------------------------------------------------------------
                SQP_CHECK_CONV: begin
                    // Condition 1: Free gradient norm converged to zero
                    if (act_g_free_norm <= tol_reg) begin
                        status      <= STATUS_CONVERGED;
                        x_optimal   <= x_reg;
                        f_optimal   <= hess_f_0;
                        g_free_norm <= act_g_free_norm;
                        active_mask <= act_mask_out;
                        vec_mu      <= act_mu_out;
                        done        <= 1'b1;
                        busy        <= 1'b0;
                        state       <= SQP_IDLE;

                    // Condition 2: Max iterations reached
                    end else if (iter_count >= max_iters_reg) begin
                        status      <= STATUS_MAX_ITERS;
                        x_optimal   <= x_reg;
                        f_optimal   <= hess_f_0;
                        g_free_norm <= act_g_free_norm;
                        active_mask <= act_mask_out;
                        vec_mu      <= act_mu_out;
                        done        <= 1'b1;
                        busy        <= 1'b0;
                        state       <= SQP_IDLE;

                    // Condition 3: Solve QP subproblem: H_proj · p = -g_proj
                    end else begin
                        chol_start <= 1'b1;
                        state      <= SQP_START_SOLVE;
                    end
                end

                // -------------------------------------------------------------
                // 4. WAIT FOR CHOLESKY LINEAR SOLVER
                // -------------------------------------------------------------
                SQP_START_SOLVE: begin
                    state <= SQP_WAIT_SOLVE;
                end

                SQP_WAIT_SOLVE: begin
                    if (chol_done) begin
                        if (chol_singular) begin
                            status      <= STATUS_SINGULAR;
                            x_optimal   <= x_reg;
                            f_optimal   <= hess_f_0;
                            g_free_norm <= act_g_free_norm;
                            active_mask <= act_mask_out;
                            vec_mu      <= act_mu_out;
                            done        <= 1'b1;
                            busy        <= 1'b0;
                            state       <= SQP_IDLE;
                        end else begin
                            // 5. Projected Feasible Step: x = clamp(x + α*p, l, u)
                            tmp_next_x = x_reg;
                            for (int i = 0; i < MAX_PARAMS; i++) begin
                                if (i < num_params_reg) begin
                                    cand_x = get_vec(x_reg, 2'(i)) + q16_mul(alpha_reg, get_vec(chol_vec_p, 2'(i)));
                                    cur_l  = get_vec(bounds_reg.lower_bound, 2'(i));
                                    cur_u  = get_vec(bounds_reg.upper_bound, 2'(i));
                                    tmp_next_x = set_vec(tmp_next_x, 2'(i), clamp_q16(cand_x, cur_l, cur_u));
                                end
                            end
                            x_reg      <= tmp_next_x;
                            iter_count <= iter_count + 1'b1;
                            state      <= SQP_START_HESS;
                        end
                    end
                end

                default: state <= SQP_IDLE;
            endcase
        end
    end

endmodule
