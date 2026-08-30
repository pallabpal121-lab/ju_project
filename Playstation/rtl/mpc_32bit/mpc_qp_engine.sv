// =============================================================================
// File Name   : mpc_qp_engine.sv
// Module Name : mpc_qp_engine
// Project     : Model Predictive Control (MPC) Accelerator (Solver #26)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Nesterov Accelerated Projected Gradient Quadratic Programming Solver.
//   Solves:
//     min_U 0.5 * U^T * H * U + g^T * U
//     s.t.  U_min <= U <= U_max
// =============================================================================

`timescale 1ns / 1ps

import mpc_types_pkg::*;
`include "mpc_helpers.svh"

module mpc_qp_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [2:0]         stacked_dim,      // Stacked control dimension (1..4)
    input  stacked_vec_t       u_init,           // Warm-start initial control
    input  hessian_mat_t       h_mat,            // Condensed Hessian matrix H (4x4)
    input  stacked_vec_t       g_vec,            // Linear gradient vector g (4x1)
    input  stacked_vec_t       u_min,            // Lower box limits U_min
    input  stacked_vec_t       u_max,            // Upper box limits U_max
    input  q16_t               step_alpha,       // Step size α = 1/L (Q16.16)
    input  q16_t               mom_beta,         // Momentum parameter β (Q16.16)
    input  logic [7:0]         max_iters,        // Maximum QP iterations
    input  q16_t               tol_eps,          // Stopping tolerance ε

    output stacked_vec_t       u_opt,            // Optimal control sequence U*
    output logic [7:0]         iters_taken,      // Completed iterations
    output logic               saturated,        // High if any actuator hits limits
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        QP_IDLE = 3'd0,
        QP_INIT = 3'd1,
        QP_GRAD = 3'd2,
        QP_STEP = 3'd3,
        QP_DONE = 3'd4
    } qp_state_t;

    qp_state_t state;

    stacked_vec_t v_reg;
    stacked_vec_t y_reg;
    stacked_vec_t v_prev;
    logic [7:0]   iter_cnt;
    logic         sat_reg;

    assign u_opt       = v_reg;
    assign iters_taken = iter_cnt;
    assign saturated   = sat_reg;

    stacked_vec_t hy_comb;
    stacked_vec_t grad_comb;
    stacked_vec_t u_unconstrained;
    stacked_vec_t v_new_comb;
    stacked_vec_t diff_comb;
    q16_t         diff_norm_sq;
    q16_t         diff_i, prod_diff;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= QP_IDLE;
            v_reg    <= '0;
            y_reg    <= '0;
            v_prev   <= '0;
            iter_cnt <= '0;
            sat_reg  <= 1'b0;
            done     <= 1'b0;
            busy     <= 1'b0;
        end else begin
            case (state)
                QP_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy     <= 1'b1;
                        v_reg    <= u_init;
                        y_reg    <= u_init;
                        v_prev   <= u_init;
                        iter_cnt <= 8'd0;
                        sat_reg  <= 1'b0;
                        state    <= QP_GRAD;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Compute ∇f(Y) = H * Y + g, and V_new = clamp(Y - α ∇f(Y))
                QP_GRAD: begin
                    hy_comb = mat_vec_mul_4x4(h_mat, y_reg, stacked_dim);

                    for (int i = 0; i < MAX_STACKED_DIM; i++) begin
                        if (i < stacked_dim) begin
                            grad_comb[i]         = hy_comb[i] + g_vec[i];
                            u_unconstrained[i]   = y_reg[i] - q16_mul(step_alpha, grad_comb[i]);
                        end else begin
                            grad_comb[i]       = 32'sd0;
                            u_unconstrained[i] = 32'sd0;
                        end
                    end

                    // Projected clamping
                    v_new_comb = vec_box_clamp(u_unconstrained, u_min, u_max, stacked_dim);

                    // Evaluate diff norm squared ||V_new - V||^2
                    diff_norm_sq = 32'sd0;
                    for (int i = 0; i < MAX_STACKED_DIM; i++) begin
                        if (i < stacked_dim) begin
                            diff_i       = v_new_comb[i] - v_reg[i];
                            prod_diff    = q16_mul(diff_i, diff_i);
                            diff_norm_sq = diff_norm_sq + prod_diff;
                        end
                    end

                    // Accelerated extrapolation Y_new = V_new + β (V_new - V)
                    for (int i = 0; i < MAX_STACKED_DIM; i++) begin
                        if (i < stacked_dim) begin
                            diff_comb[i] = v_new_comb[i] - v_reg[i];
                            y_reg[i]    <= v_new_comb[i] + q16_mul(mom_beta, diff_comb[i]);
                        end else begin
                            y_reg[i] <= 32'sd0;
                        end
                    end

                    v_prev   <= v_reg;
                    v_reg    <= v_new_comb;
                    iter_cnt <= iter_cnt + 1'b1;

                    // Convergence check
                    if ((diff_norm_sq < tol_eps && iter_cnt > 0) || (iter_cnt + 1'b1 >= max_iters)) begin
                        state <= QP_DONE;
                    end else begin
                        state <= QP_GRAD;
                    end
                end

                // Step 2: Finalize & check physical saturation
                QP_DONE: begin
                    sat_reg <= 1'b0;
                    for (int i = 0; i < MAX_STACKED_DIM; i++) begin
                        if (i < stacked_dim) begin
                            if (v_reg[i] <= u_min[i] || v_reg[i] >= u_max[i]) begin
                                sat_reg <= 1'b1;
                            end
                        end
                    end
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= QP_IDLE;
                end

                default: state <= QP_IDLE;
            endcase
        end
    end

endmodule
