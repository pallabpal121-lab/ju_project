// =============================================================================
// File Name   : pdhg_top.sv
// Module Name : pdhg_top
// Project     : Primal-Dual Hybrid Gradient (PDHG / Chambolle-Pock) Accelerator (Solver #19)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Module for Chambolle-Pock First-Order Primal-Dual Hybrid Gradient
//   algorithm. Solves min_x f(x) + g(K*x) by alternating:
//   1. Dual update: y_{k+1} = prox_{sigma * g*}(y_k + sigma * K * x_bar_k)
//   2. Primal update: x_{k+1} = prox_{tau * f}(x_k - tau * K^T * y_{k+1})
//   3. Extrapolation: x_bar_{k+1} = x_{k+1} + theta * (x_{k+1} - x_k)
// =============================================================================

`timescale 1ns / 1ps

import pdhg_types_pkg::*;
`include "pdhg_helpers.svh"

module pdhg_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Problem Formulation & Controls
    // -------------------------------------------------------------------------
    input  logic               start,            // 1-cycle start trigger
    input  logic [2:0]         dim_m,            // Dual dimension M (1..4)
    input  logic [2:0]         dim_n,            // Primal dimension N (1..4)
    input  pdhg_mode_t         mode,             // Operating mode (TV_L2, LASSO, NONNEG)
    input  mat_t               k_matrix,         // Operator Matrix K (M x N)
    input  vec_t               b_target,         // Target observation vector b
    input  vec_t               x_init,           // Initial primal vector x_0
    input  vec_t               y_init,           // Initial dual vector y_0
    input  q16_t               tau_step,         // Primal step size τ
    input  q16_t               sigma_step,       // Dual step size σ
    input  q16_t               theta_relax,      // Relaxation factor θ (e.g. 1.0)
    input  q16_t               lambda_param,     // Regularization parameter λ
    input  q16_t               tol_residual,     // Convergence tolerance ε
    input  logic [15:0]        max_iters,        // Maximum iteration count

    // -------------------------------------------------------------------------
    // Interface 2: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,        // Optimal primal vector x*
    output vec_t               y_optimal,        // Optimal dual vector y*
    output logic [15:0]        iter_count,       // Iterations completed
    output q16_t               primal_res,       // Primal residual ||Δx||_inf
    output q16_t               dual_res,         // Dual residual ||Δy||_inf
    output status_t            status,           // Convergence status code
    output logic               done,             // 1-cycle completion strobe
    output logic               busy              // High while active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        PDHG_IDLE         = 3'd0,
        PDHG_START_DUAL   = 3'd1,
        PDHG_WAIT_DUAL    = 3'd2,
        PDHG_START_PRIMAL = 3'd3,
        PDHG_WAIT_PRIMAL  = 3'd4,
        PDHG_CHECK_CONV   = 3'd5,
        PDHG_DONE         = 3'd6
    } pdhg_state_t;

    pdhg_state_t state;

    // Registers
    vec_t        x_curr;
    vec_t        x_bar_curr;
    vec_t        y_curr;
    vec_t        x_next_reg;
    vec_t        x_bar_next_reg;
    vec_t        y_next_reg;
    q16_t        primal_res_reg;
    q16_t        dual_res_reg;
    logic [15:0] iters_cnt;
    status_t     status_reg;

    // Latched Parameters
    logic [2:0]  dim_m_reg;
    logic [2:0]  dim_n_reg;
    pdhg_mode_t  mode_reg;
    mat_t        k_mat_reg;
    vec_t        b_reg;
    q16_t        tau_reg;
    q16_t        sigma_reg;
    q16_t        theta_reg;
    q16_t        lambda_reg;
    q16_t        tol_reg;
    logic [15:0] max_iters_reg;

    // Sub-Engine 1: Dual Engine
    logic start_dual;
    vec_t dual_y_out;
    q16_t dual_res_out;
    logic dual_done, dual_busy;

    pdhg_dual_engine u_dual (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (start_dual),
        .dim_m       (dim_m_reg),
        .dim_n       (dim_n_reg),
        .mode        (mode_reg),
        .k_matrix    (k_mat_reg),
        .y_curr      (y_curr),
        .x_bar       (x_bar_curr),
        .b_target    (b_reg),
        .sigma_step  (sigma_reg),
        .lambda_param(lambda_reg),
        .y_next      (dual_y_out),
        .res_dual    (dual_res_out),
        .done        (dual_done),
        .busy        (dual_busy)
    );

    // Sub-Engine 2: Primal Engine
    logic start_primal;
    vec_t primal_x_out;
    vec_t primal_xbar_out;
    q16_t primal_res_out;
    logic primal_done, primal_busy;

    pdhg_primal_engine u_primal (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (start_primal),
        .dim_m       (dim_m_reg),
        .dim_n       (dim_n_reg),
        .mode        (mode_reg),
        .k_matrix    (k_mat_reg),
        .x_curr      (x_curr),
        .y_next      (y_next_reg),
        .b_target    (b_reg),
        .tau_step    (tau_reg),
        .theta_relax (theta_reg),
        .lambda_param(lambda_reg),
        .x_next      (primal_x_out),
        .x_bar_next  (primal_xbar_out),
        .res_primal  (primal_res_out),
        .done        (primal_done),
        .busy        (primal_busy)
    );

    // Outputs
    assign x_optimal  = x_curr;
    assign y_optimal  = y_curr;
    assign iter_count = iters_cnt;
    assign primal_res = primal_res_reg;
    assign dual_res   = dual_res_reg;
    assign status     = status_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= PDHG_IDLE;
            x_curr         <= '0;
            x_bar_curr     <= '0;
            y_curr         <= '0;
            x_next_reg     <= '0;
            x_bar_next_reg <= '0;
            y_next_reg     <= '0;
            primal_res_reg <= Q16_ZERO;
            dual_res_reg   <= Q16_ZERO;
            iters_cnt      <= 16'd0;
            status_reg     <= STATUS_IDLE;
            dim_m_reg      <= 3'd3;
            dim_n_reg      <= 3'd4;
            mode_reg       <= MODE_TV_L2;
            k_mat_reg      <= '0;
            b_reg          <= '0;
            tau_reg        <= Q16_TAU_DEF;
            sigma_reg      <= Q16_SIGMA_DEF;
            theta_reg      <= Q16_THETA_DEF;
            lambda_reg     <= Q16_LAMBDA_DEF;
            tol_reg        <= Q16_TOL_DEF;
            max_iters_reg  <= 16'd200;
            start_dual     <= 1'b0;
            start_primal   <= 1'b0;
            done           <= 1'b0;
            busy           <= 1'b0;
        end else begin
            start_dual   <= 1'b0;
            start_primal <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: PDHG_IDLE - Latch User Configuration & Initialize
                // -------------------------------------------------------------
                PDHG_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy          <= 1'b1;
                        dim_m_reg     <= dim_m;
                        dim_n_reg     <= dim_n;
                        mode_reg      <= mode;
                        k_mat_reg     <= k_matrix;
                        b_reg         <= b_target;
                        x_curr        <= x_init;
                        x_bar_curr    <= x_init;
                        y_curr        <= y_init;
                        tau_reg       <= (tau_step != Q16_ZERO)     ? tau_step     : Q16_TAU_DEF;
                        sigma_reg     <= (sigma_step != Q16_ZERO)   ? sigma_step   : Q16_SIGMA_DEF;
                        theta_reg     <= (theta_relax != Q16_ZERO)  ? theta_relax  : Q16_THETA_DEF;
                        lambda_reg    <= (lambda_param != Q16_ZERO) ? lambda_param : Q16_LAMBDA_DEF;
                        tol_reg       <= (tol_residual != Q16_ZERO) ? tol_residual : Q16_TOL_DEF;
                        max_iters_reg <= (max_iters != 16'd0)       ? max_iters    : 16'd200;
                        iters_cnt     <= 16'd0;
                        status_reg    <= STATUS_RUNNING;
                        state         <= PDHG_START_DUAL;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: PDHG_START_DUAL - Trigger Dual Engine
                // -------------------------------------------------------------
                PDHG_START_DUAL: begin
                    start_dual <= 1'b1;
                    state      <= PDHG_WAIT_DUAL;
                end

                // -------------------------------------------------------------
                // STATE 2: PDHG_WAIT_DUAL - Latch Dual Update y_{k+1}
                // -------------------------------------------------------------
                PDHG_WAIT_DUAL: begin
                    if (dual_done) begin
                        y_next_reg   <= dual_y_out;
                        dual_res_reg <= dual_res_out;
                        state        <= PDHG_START_PRIMAL;
                    end
                end

                // -------------------------------------------------------------
                // STATE 3: PDHG_START_PRIMAL - Trigger Primal Engine
                // -------------------------------------------------------------
                PDHG_START_PRIMAL: begin
                    start_primal <= 1'b1;
                    state        <= PDHG_WAIT_PRIMAL;
                end

                // -------------------------------------------------------------
                // STATE 4: PDHG_WAIT_PRIMAL - Latch Primal Update x_{k+1}
                // -------------------------------------------------------------
                PDHG_WAIT_PRIMAL: begin
                    if (primal_done) begin
                        x_next_reg     <= primal_x_out;
                        x_bar_next_reg <= primal_xbar_out;
                        primal_res_reg <= primal_res_out;
                        iters_cnt      <= iters_cnt + 1'b1;
                        state          <= PDHG_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // STATE 5: PDHG_CHECK_CONV - Convergence & Max-Iter Decision
                // -------------------------------------------------------------
                PDHG_CHECK_CONV: begin
                    if (primal_res_reg <= tol_reg && dual_res_reg <= tol_reg) begin
                        x_curr     <= x_next_reg;
                        x_bar_curr <= x_bar_next_reg;
                        y_curr     <= y_next_reg;
                        status_reg <= STATUS_CONVERGED;
                        state      <= PDHG_DONE;
                    end else if (iters_cnt >= max_iters_reg) begin
                        x_curr     <= x_next_reg;
                        x_bar_curr <= x_bar_next_reg;
                        y_curr     <= y_next_reg;
                        status_reg <= STATUS_MAX_ITERS;
                        state      <= PDHG_DONE;
                    end else begin
                        x_curr     <= x_next_reg;
                        x_bar_curr <= x_bar_next_reg;
                        y_curr     <= y_next_reg;
                        state      <= PDHG_START_DUAL;
                    end
                end

                // -------------------------------------------------------------
                // STATE 6: PDHG_DONE - Completion Strobe
                // -------------------------------------------------------------
                PDHG_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= PDHG_IDLE;
                end

                default: state <= PDHG_IDLE;
            endcase
        end
    end

endmodule
