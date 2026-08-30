// =============================================================================
// File Name   : mpc_top.sv
// Module Name : mpc_top
// Project     : Model Predictive Control (MPC) Accelerator (Solver #26)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Controller for Real-Time Model Predictive Control (MPC).
//   Supports complete receding horizon workflow:
//   1. Initialization: Configures H_mpc, M_x, M_ref, box bounds, solver parameters
//   2. Gradient Condensing: mpc_condense_engine evaluates g = M_x * x - M_ref * x_ref
//   3. QP Optimization:    mpc_qp_engine finds optimal stacked control sequence U*
//   4. Receding Horizon:   Streams immediate control action u_0 to actuators,
//                          shifts warm-start buffer for next control cycle
// =============================================================================

`timescale 1ns / 1ps

import mpc_types_pkg::*;
`include "mpc_helpers.svh"

module mpc_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Configuration & Initialization
    // -------------------------------------------------------------------------
    input  logic               init_mpc,         // 1-cycle initialization strobe
    input  logic [2:0]         state_dim,        // State dimension Nx (1..4)
    input  logic [1:0]         ctrl_dim,         // Control dimension Nu (1..2)
    input  logic [2:0]         stacked_dim,      // Stacked horizon dimension (1..4)
    input  hessian_mat_t       h_mpc_mat,        // Condensed Hessian H_mpc (4x4)
    input  grad_mat_t          m_x_mat,          // State mapping matrix M_x (4x4)
    input  grad_mat_t          m_ref_mat,        // Reference mapping matrix M_ref (4x4)
    input  stacked_vec_t       u_min,            // Lower box limits U_min (4x1)
    input  stacked_vec_t       u_max,            // Upper box limits U_max (4x1)
    input  q16_t               step_alpha,       // Step size α = 1/L (Q16.16)
    input  q16_t               mom_beta,         // Nesterov momentum β (Q16.16)
    input  logic [7:0]         max_iters,        // Max iterations per QP solve
    input  q16_t               tol_eps,          // Stopping tolerance ε

    // -------------------------------------------------------------------------
    // Interface 2: Real-Time Receding Horizon Solve Step
    // -------------------------------------------------------------------------
    input  logic               solve_valid,      // Trigger new MPC optimization cycle
    input  state_vec_t         x_curr,           // Current measured state x_0
    input  state_vec_t         x_ref,            // Reference target state x_ref
    output ctrl_vec_t          u_applied,        // Immediate actuator control u_0 (2x1)
    output stacked_vec_t       u_opt_horizon,    // Full optimal control sequence U* (4x1)
    output stacked_vec_t       g_gradient,       // Evaluated gradient vector g (4x1)
    output logic [7:0]         iterations_done,  // Number of QP iterations completed
    output status_t            status,           // Status code
    output logic               solve_done,       // Optimization complete strobe
    output logic               busy              // High while engine is active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        MPC_IDLE         = 3'd0,
        MPC_CONDENSE     = 3'd1,
        MPC_WAIT_COND    = 3'd2,
        MPC_START_QP     = 3'd3,
        MPC_WAIT_QP      = 3'd4,
        MPC_DONE         = 3'd5
    } mpc_state_t;

    mpc_state_t state;

    // Internal Configuration Registers
    logic [2:0]   state_dim_reg;
    logic [1:0]   ctrl_dim_reg;
    logic [2:0]   stacked_dim_reg;
    hessian_mat_t h_reg;
    grad_mat_t    mx_reg;
    grad_mat_t    mref_reg;
    stacked_vec_t u_min_reg;
    stacked_vec_t u_max_reg;
    q16_t         alpha_reg;
    q16_t         beta_reg;
    logic [7:0]   max_iter_reg;
    q16_t         tol_reg;
    status_t      status_reg;

    // Receding Horizon Warm-Start Buffer
    stacked_vec_t u_warm;

    // Sub-engine 1: mpc_condense_engine
    logic         start_cond;
    stacked_vec_t g_cond_out;
    logic         cond_done;
    logic         cond_busy;

    mpc_condense_engine u_cond (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start_cond),
        .state_dim  (state_dim_reg),
        .stacked_dim(stacked_dim_reg),
        .x_curr     (x_curr),
        .x_ref      (x_ref),
        .m_x_mat    (mx_reg),
        .m_ref_mat  (mref_reg),
        .g_mpc      (g_cond_out),
        .done       (cond_done),
        .busy       (cond_busy)
    );

    // Sub-engine 2: mpc_qp_engine
    logic         start_qp;
    stacked_vec_t qp_u_out;
    logic [7:0]   qp_iters;
    logic         qp_sat;
    logic         qp_done;
    logic         qp_busy;

    mpc_qp_engine u_qp (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start_qp),
        .stacked_dim(stacked_dim_reg),
        .u_init     (u_warm),
        .h_mat      (h_reg),
        .g_vec      (g_cond_out),
        .u_min      (u_min_reg),
        .u_max      (u_max_reg),
        .step_alpha (alpha_reg),
        .mom_beta   (beta_reg),
        .max_iters  (max_iter_reg),
        .tol_eps    (tol_reg),
        .u_opt      (qp_u_out),
        .iters_taken(qp_iters),
        .saturated  (qp_sat),
        .done       (qp_done),
        .busy       (qp_busy)
    );

    // Outputs
    ctrl_vec_t    u_app_reg;
    stacked_vec_t u_opt_reg;
    stacked_vec_t g_latched;
    logic [7:0]   iter_latched;

    assign u_applied       = u_app_reg;
    assign u_opt_horizon   = u_opt_reg;
    assign g_gradient      = g_latched;
    assign iterations_done = iter_latched;
    assign status          = status_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= MPC_IDLE;
            state_dim_reg   <= 3'd2;
            ctrl_dim_reg    <= 2'd1;
            stacked_dim_reg <= 3'd4;
            h_reg           <= '0;
            mx_reg          <= '0;
            mref_reg        <= '0;
            u_min_reg       <= '0;
            u_max_reg       <= '0;
            alpha_reg       <= 32'h0000_1000;
            beta_reg        <= 32'h0000_8000;
            max_iter_reg    <= 8'd25;
            tol_reg         <= 32'h0000_0010;
            status_reg      <= STATUS_IDLE;
            u_warm          <= '0;
            u_app_reg       <= '0;
            u_opt_reg       <= '0;
            g_latched       <= '0;
            iter_latched    <= '0;
            start_cond      <= 1'b0;
            start_qp        <= 1'b0;
            solve_done      <= 1'b0;
            busy            <= 1'b0;
        end else begin
            start_cond <= 1'b0;
            start_qp   <= 1'b0;
            solve_done <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: MPC_IDLE - Handle Init and Solve Requests
                // -------------------------------------------------------------
                MPC_IDLE: begin
                    if (init_mpc) begin
                        state_dim_reg   <= state_dim;
                        ctrl_dim_reg    <= ctrl_dim;
                        stacked_dim_reg <= stacked_dim;
                        h_reg           <= h_mpc_mat;
                        mx_reg          <= m_x_mat;
                        mref_reg        <= m_ref_mat;
                        u_min_reg       <= u_min;
                        u_max_reg       <= u_max;
                        alpha_reg       <= (step_alpha != 32'sd0) ? step_alpha : 32'h0000_1000;
                        beta_reg        <= (mom_beta != 32'sd0) ? mom_beta : 32'h0000_8000;
                        max_iter_reg    <= (max_iters != 8'd0) ? max_iters : 8'd25;
                        tol_reg         <= (tol_eps != 32'sd0) ? tol_eps : 32'h0000_0010;
                        status_reg      <= STATUS_IDLE;
                        u_warm          <= '0;
                        busy            <= 1'b0;
                    end else if (solve_valid) begin
                        busy       <= 1'b1;
                        start_cond <= 1'b1;
                        state      <= MPC_WAIT_COND;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: MPC_WAIT_COND - Wait for Linear Gradient Engine
                // -------------------------------------------------------------
                MPC_WAIT_COND: begin
                    if (cond_done) begin
                        g_latched  <= g_cond_out;
                        status_reg <= STATUS_CONDENSED;
                        start_qp   <= 1'b1;
                        state      <= MPC_WAIT_QP;
                    end
                end

                // -------------------------------------------------------------
                // STATE 2: MPC_WAIT_QP - Wait for QP Solver
                // -------------------------------------------------------------
                MPC_WAIT_QP: begin
                    if (qp_done) begin
                        u_opt_reg    <= qp_u_out;
                        iter_latched <= qp_iters;
                        status_reg   <= qp_sat ? STATUS_SATURATED : STATUS_OPTIMIZED;

                        // Extract immediate control action u_0
                        if (ctrl_dim_reg == 2'd1) begin
                            u_app_reg[0] <= qp_u_out[0];
                            u_app_reg[1] <= 32'sd0;
                        end else begin
                            u_app_reg[0] <= qp_u_out[0];
                            u_app_reg[1] <= qp_u_out[1];
                        end

                        // Shift warm-start vector for next receding horizon step:
                        // U_warm = [u_1, u_2, u_3, u_3]
                        if (ctrl_dim_reg == 2'd1) begin
                            u_warm[0] <= qp_u_out[1];
                            u_warm[1] <= qp_u_out[2];
                            u_warm[2] <= qp_u_out[3];
                            u_warm[3] <= qp_u_out[3];
                        end else begin
                            u_warm[0] <= qp_u_out[2];
                            u_warm[1] <= qp_u_out[3];
                            u_warm[2] <= qp_u_out[2];
                            u_warm[3] <= qp_u_out[3];
                        end

                        solve_done <= 1'b1;
                        busy       <= 1'b0;
                        state      <= MPC_IDLE;
                    end
                end

                default: state <= MPC_IDLE;
            endcase
        end
    end

endmodule
