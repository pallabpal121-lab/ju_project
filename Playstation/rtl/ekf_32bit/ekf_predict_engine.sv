// =============================================================================
// File Name   : ekf_predict_engine.sv
// Module Name : ekf_predict_engine
// Project     : Extended Kalman Filter (EKF) Accelerator (Solver #24)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Time-Update (Prediction Step) Engine for EKF.
//   1. A priori state prediction:      x_prior = f(x_{k-1}, u_k)
//   2. A priori error covariance:      P_prior = F * P * F^T + Q
//   3. Symmetric covariance assurance: P_prior = 0.5 * (P_prior + P_prior^T)
// =============================================================================

`timescale 1ns / 1ps

import ekf_types_pkg::*;
`include "ekf_helpers.svh"

module ekf_predict_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [2:0]         state_dim,        // State dimension N (1..4)
    input  state_vec_t         f_x_pred,         // Predicted state f(x, u)
    input  state_mat_t         p_curr,           // Previous covariance P_{k-1}
    input  state_mat_t         f_mat,            // State transition Jacobian F_k
    input  state_mat_t         q_mat,            // Process noise covariance Q_k

    output state_vec_t         x_prior,          // A priori state estimate x_k^-
    output state_mat_t         p_prior,          // A priori covariance P_k^-
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        PRED_IDLE = 2'd0,
        PRED_CALC = 2'd1,
        PRED_DONE = 2'd2
    } pred_state_t;

    pred_state_t state;

    state_vec_t x_reg;
    state_mat_t p_reg;
    state_mat_t p_fpf;
    state_mat_t p_raw;
    state_mat_t p_sym;

    assign x_prior = x_reg;
    assign p_prior = p_reg;

    q16_t fpf_elem, q_elem, p_ij, p_ji, sym_val;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= PRED_IDLE;
            x_reg <= '0;
            p_reg <= '0;
            done  <= 1'b0;
            busy  <= 1'b0;
        end else begin
            case (state)
                PRED_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= PRED_CALC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                PRED_CALC: begin
                    // 1. State vector prediction
                    x_reg <= f_x_pred;

                    // 2. Covariance Sandwich: F * P * F^T
                    p_fpf = mat4_sandwich(f_mat, p_curr, state_dim);

                    // 3. Add Process Noise: P_raw = F P F^T + Q
                    p_raw = '0;
                    for (int i = 0; i < MAX_STATE_DIM; i++) begin
                        for (int j = 0; j < MAX_STATE_DIM; j++) begin
                            if (i < state_dim && j < state_dim) begin
                                fpf_elem = get_smat(p_fpf, 2'(i), 2'(j));
                                q_elem   = get_smat(q_mat, 2'(i), 2'(j));
                                p_raw    = set_smat(p_raw, 2'(i), 2'(j), fpf_elem + q_elem);
                            end else begin
                                p_raw    = set_smat(p_raw, 2'(i), 2'(j), (i == j) ? 32'h0001_0000 : 32'h0000_0000);
                            end
                        end
                    end

                    // 4. Symmetrize P_prior = 0.5 * (P_raw + P_raw^T)
                    p_sym = '0;
                    for (int i = 0; i < MAX_STATE_DIM; i++) begin
                        for (int j = 0; j < MAX_STATE_DIM; j++) begin
                            if (i < state_dim && j < state_dim) begin
                                p_ij    = get_smat(p_raw, 2'(i), 2'(j));
                                p_ji    = get_smat(p_raw, 2'(j), 2'(i));
                                sym_val = q16_mul(32'h0000_8000, p_ij + p_ji);
                                p_sym   = set_smat(p_sym, 2'(i), 2'(j), sym_val);
                            end else begin
                                p_sym   = set_smat(p_sym, 2'(i), 2'(j), (i == j) ? 32'h0001_0000 : 32'h0000_0000);
                            end
                        end
                    end
                    p_reg <= p_sym;

                    state <= PRED_DONE;
                end

                PRED_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= PRED_IDLE;
                end

                default: state <= PRED_IDLE;
            endcase
        end
    end

endmodule
