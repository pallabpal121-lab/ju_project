// =============================================================================
// File Name   : fw_top.sv
// Module Name : fw_top
// Project     : Frank-Wolfe / Conditional Gradient Accelerator (Solver #20)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Module for Frank-Wolfe / Conditional Gradient Accelerator.
//   Executes projection-free constrained optimization over L1 balls, hyperboxes,
//   and probability simplices.
//   Alternates:
//   1. Gradient evaluation: g_k = Q * x_k - b
//   2. Linear Minimization Oracle: s_k = argmin_{s in C} g_k^T s
//   3. Exact line search or diminishing step: gamma_k in [0, 1]
//   4. Convex update: x_{k+1} = (1 - gamma)*x_k + gamma*s_k
//   5. Duality gap certificate: gap = g_k^T (x_k - s_k) <= tol
// =============================================================================

`timescale 1ns / 1ps

import fw_types_pkg::*;
`include "fw_helpers.svh"

module fw_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Problem Formulation & Controls
    // -------------------------------------------------------------------------
    input  logic               start,            // 1-cycle start trigger
    input  logic [2:0]         dim_n,            // Problem dimension N (1..4)
    input  fw_geom_t           geom,             // Constraint geometry (L1, Box, Simplex)
    input  fw_step_mode_t      step_mode,        // Line search mode
    input  mat_t               q_matrix,         // Quadratic Hessian matrix Q (N x N)
    input  vec_t               b_vector,         // Linear objective vector b
    input  vec_t               x_init,           // Initial feasible point x_0
    input  q16_t               radius_l1,        // L1 ball radius R
    input  vec_t               box_lower,        // Box lower bounds l
    input  vec_t               box_upper,        // Box upper bounds u
    input  q16_t               tol_duality_gap,  // Duality gap tolerance ε
    input  logic [15:0]        max_iters,        // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 2: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,        // Optimal constrained point x*
    output q16_t               f_optimal,        // Minimum objective value f(x*)
    output q16_t               duality_gap,      // Final Frank-Wolfe duality gap
    output logic [15:0]        iter_count,       // Iterations completed
    output status_t            status,           // Convergence status code
    output logic               done,             // 1-cycle completion strobe
    output logic               busy              // High while active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        FW_IDLE       = 3'd0,
        FW_GRAD       = 3'd1,
        FW_START_LMO  = 3'd2,
        FW_WAIT_LMO   = 3'd3,
        FW_START_STEP = 3'd4,
        FW_WAIT_STEP  = 3'd5,
        FW_CHECK_CONV = 3'd6,
        FW_EVAL_F     = 3'd7
    } fw_state_t;

    fw_state_t state;

    // Registers
    vec_t        x_curr;
    vec_t        grad_curr;
    vec_t        s_lmo_reg;
    vec_t        x_next_reg;
    q16_t        gap_reg;
    q16_t        f_reg;
    logic [15:0] iters_cnt;
    status_t     status_reg;

    // Latched Parameters
    logic [2:0]    dim_n_reg;
    fw_geom_t      geom_reg;
    fw_step_mode_t step_mode_reg;
    mat_t          q_mat_reg;
    vec_t          b_vec_reg;
    q16_t          radius_reg;
    vec_t          box_l_reg;
    vec_t          box_u_reg;
    q16_t          tol_gap_reg;
    logic [15:0]   max_iters_reg;

    // Sub-Engine 1: LMO Engine
    logic start_lmo;
    vec_t lmo_s_out;
    logic lmo_done, lmo_busy;

    fw_lmo_engine u_lmo (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start_lmo),
        .dim_n     (dim_n_reg),
        .geom      (geom_reg),
        .grad_in   (grad_curr),
        .radius_l1 (radius_reg),
        .box_lower (box_l_reg),
        .box_upper (box_u_reg),
        .s_lmo     (lmo_s_out),
        .done      (lmo_done),
        .busy      (lmo_busy)
    );

    // Sub-Engine 2: Step Engine
    logic start_step;
    vec_t step_x_out;
    q16_t step_gamma_out;
    q16_t step_gap_out;
    logic step_done, step_busy;

    fw_step_engine u_step (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start_step),
        .dim_n      (dim_n_reg),
        .step_mode  (step_mode_reg),
        .iter_k     (iters_cnt),
        .q_matrix   (q_mat_reg),
        .x_curr     (x_curr),
        .s_lmo      (s_lmo_reg),
        .grad_curr  (grad_curr),
        .x_next     (step_x_out),
        .gamma_used (step_gamma_out),
        .duality_gap(step_gap_out),
        .done       (step_done),
        .busy       (step_busy)
    );

    // Outputs
    assign x_optimal   = x_curr;
    assign f_optimal   = f_reg;
    assign duality_gap = gap_reg;
    assign iter_count  = iters_cnt;
    assign status      = status_reg;

    vec_t q_x;
    q16_t half_x_q_x, b_x;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= FW_IDLE;
            x_curr        <= '0;
            grad_curr     <= '0;
            s_lmo_reg     <= '0;
            x_next_reg    <= '0;
            gap_reg       <= Q16_ZERO;
            f_reg         <= Q16_ZERO;
            iters_cnt     <= 16'd0;
            status_reg    <= STATUS_IDLE;
            dim_n_reg     <= 3'd2;
            geom_reg      <= GEOM_L1_BALL;
            step_mode_reg <= STEP_EXACT_LINE_SEARCH;
            q_mat_reg     <= '0;
            b_vec_reg     <= '0;
            radius_reg    <= Q16_ONE;
            box_l_reg     <= '0;
            box_u_reg     <= '0;
            tol_gap_reg   <= Q16_TOL_DEF;
            max_iters_reg <= 16'd100;
            start_lmo     <= 1'b0;
            start_step    <= 1'b0;
            done          <= 1'b0;
            busy          <= 1'b0;
        end else begin
            start_lmo  <= 1'b0;
            start_step <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: FW_IDLE - Latch User Configuration & Initialize
                // -------------------------------------------------------------
                FW_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy          <= 1'b1;
                        dim_n_reg     <= dim_n;
                        geom_reg      <= geom;
                        step_mode_reg <= step_mode;
                        q_mat_reg     <= q_matrix;
                        b_vec_reg     <= b_vector;
                        x_curr        <= x_init;
                        radius_reg    <= (radius_l1 != Q16_ZERO) ? radius_l1 : Q16_ONE;
                        box_l_reg     <= box_lower;
                        box_u_reg     <= box_upper;
                        tol_gap_reg   <= (tol_duality_gap != Q16_ZERO) ? tol_duality_gap : Q16_TOL_DEF;
                        max_iters_reg <= (max_iters != 16'd0) ? max_iters : 16'd100;
                        iters_cnt     <= 16'd0;
                        status_reg    <= STATUS_RUNNING;
                        state         <= FW_GRAD;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: FW_GRAD - Compute Gradient g_k = Q * x_k - b
                // -------------------------------------------------------------
                FW_GRAD: begin
                    q_x = mat_vec_mul(q_mat_reg, x_curr, dim_n_reg);
                    grad_curr <= vec_sub(q_x, b_vec_reg, dim_n_reg);
                    state     <= FW_START_LMO;
                end

                // -------------------------------------------------------------
                // STATE 2: FW_START_LMO - Trigger Linear Minimization Oracle
                // -------------------------------------------------------------
                FW_START_LMO: begin
                    start_lmo <= 1'b1;
                    state     <= FW_WAIT_LMO;
                end

                // -------------------------------------------------------------
                // STATE 3: FW_WAIT_LMO - Latch Extreme Point s_k
                // -------------------------------------------------------------
                FW_WAIT_LMO: begin
                    if (lmo_done) begin
                        s_lmo_reg <= lmo_s_out;
                        state     <= FW_START_STEP;
                    end
                end

                // -------------------------------------------------------------
                // STATE 4: FW_START_STEP - Trigger Step & Duality Gap Engine
                // -------------------------------------------------------------
                FW_START_STEP: begin
                    start_step <= 1'b1;
                    state      <= FW_WAIT_STEP;
                end

                // -------------------------------------------------------------
                // STATE 5: FW_WAIT_STEP - Latch Next Point x_{k+1} & Duality Gap
                // -------------------------------------------------------------
                FW_WAIT_STEP: begin
                    if (step_done) begin
                        x_next_reg <= step_x_out;
                        gap_reg    <= step_gap_out;
                        iters_cnt  <= iters_cnt + 1'b1;
                        state      <= FW_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // STATE 6: FW_CHECK_CONV - Convergence & Max-Iter Decision
                // -------------------------------------------------------------
                FW_CHECK_CONV: begin
                    if (gap_reg <= tol_gap_reg) begin
                        x_curr     <= x_next_reg;
                        status_reg <= STATUS_CONVERGED;
                        state      <= FW_EVAL_F;
                    end else if (iters_cnt >= max_iters_reg) begin
                        x_curr     <= x_next_reg;
                        status_reg <= STATUS_MAX_ITERS;
                        state      <= FW_EVAL_F;
                    end else begin
                        x_curr <= x_next_reg;
                        state  <= FW_GRAD;
                    end
                end

                // -------------------------------------------------------------
                // STATE 7: FW_EVAL_F - Compute Final Objective f(x*) = 0.5*x^T Q x - b^T x
                // -------------------------------------------------------------
                FW_EVAL_F: begin
                    q_x        = mat_vec_mul(q_mat_reg, x_curr, dim_n_reg);
                    half_x_q_x = q16_mul(Q16_HALF, dot_product(x_curr, q_x, dim_n_reg));
                    b_x        = dot_product(b_vec_reg, x_curr, dim_n_reg);
                    f_reg      <= half_x_q_x - b_x;
                    done       <= 1'b1;
                    busy       <= 1'b0;
                    state      <= FW_IDLE;
                end

                default: state <= FW_IDLE;
            endcase
        end
    end

endmodule
