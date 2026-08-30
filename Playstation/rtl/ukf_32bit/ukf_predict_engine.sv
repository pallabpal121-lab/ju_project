// =============================================================================
// File Name   : ukf_predict_engine.sv
// Module Name : ukf_predict_engine
// Project     : Unscented Kalman Filter (UKF) Accelerator (Solver #25)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Time-Update (Prediction Step) Engine for UKF.
//   1. Evaluates a priori mean:  x_prior = ∑ W_i^(m) * χ_i^x
//   2. Evaluates a priori cov:   P_prior = ∑ W_i^(c) * (χ_i^x - x_prior)(χ_i^x - x_prior)^T + Q
//   3. Symmetric regularization: P_prior = 0.5 * (P_prior + P_prior^T)
// =============================================================================

`timescale 1ns / 1ps

import ukf_types_pkg::*;
`include "ukf_helpers.svh"

module ukf_predict_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [2:0]         state_dim,        // State dimension N (1..4)
    input  sigma_state_arr_t   sigma_points_in,  // Propagated sigma points [36 elements]
    input  weights_arr_t       weights_m,        // Mean weights W_i^(m) [9]
    input  weights_arr_t       weights_c,        // Covariance weights W_i^(c) [9]
    input  state_mat_t         q_mat,            // Process noise covariance Q

    output state_vec_t         x_prior,          // A priori state estimate
    output state_mat_t         p_prior,          // A priori error covariance
    output sigma_state_arr_t   sigma_points_out, // Latched sigma points
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        PRED_IDLE = 2'd0,
        PRED_CALC = 2'd1,
        PRED_DONE = 2'd2
    } pred_state_t;

    pred_state_t state;

    state_vec_t       x_reg;
    state_mat_t       p_reg;
    sigma_state_arr_t sig_reg;

    assign x_prior          = x_reg;
    assign p_prior          = p_reg;
    assign sigma_points_out = sig_reg;

    state_vec_t x_mean_comb;
    q16_t sum_dim, diff_i, diff_j, prod_diff, term_val, p_elem, sig_elem;
    q16_t p_ij, p_ji, sym_val;
    state_mat_t p_accum, p_sym;
    logic [3:0] num_pts;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= PRED_IDLE;
            x_reg   <= '0;
            p_reg   <= '0;
            sig_reg <= '0;
            done    <= 1'b0;
            busy    <= 1'b0;
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
                    sig_reg <= sigma_points_in;
                    num_pts = 4'(2 * state_dim + 1);

                    // 1. Mean State: x_prior = ∑ W_m[p] * χ[p]
                    x_mean_comb = '0;
                    for (int d = 0; d < MAX_STATE_DIM; d++) begin
                        if (d < state_dim) begin
                            sum_dim = 32'sd0;
                            for (int p = 0; p < MAX_SIGMA_POINTS; p++) begin
                                if (p < num_pts) begin
                                    sig_elem = get_sigma_state(sigma_points_in, 4'(p), 2'(d));
                                    sum_dim  = sum_dim + q16_mul(weights_m[p], sig_elem);
                                end
                            end
                            x_mean_comb[d] = sum_dim;
                        end
                    end
                    x_reg <= x_mean_comb;

                    // 2. Covariance: P_raw = Q + ∑ W_c[p] * (χ_p - x_mean)(χ_p - x_mean)^T
                    p_accum = q_mat;
                    for (int i = 0; i < MAX_STATE_DIM; i++) begin
                        for (int j = 0; j < MAX_STATE_DIM; j++) begin
                            if (i < state_dim && j < state_dim) begin
                                p_elem = get_smat(p_accum, 2'(i), 2'(j));
                                for (int p = 0; p < MAX_SIGMA_POINTS; p++) begin
                                    if (p < num_pts) begin
                                        diff_i    = get_sigma_state(sigma_points_in, 4'(p), 2'(i)) - x_mean_comb[i];
                                        diff_j    = get_sigma_state(sigma_points_in, 4'(p), 2'(j)) - x_mean_comb[j];
                                        prod_diff = q16_mul(diff_i, diff_j);
                                        term_val  = q16_mul(weights_c[p], prod_diff);
                                        p_elem    = p_elem + term_val;
                                    end
                                end
                                p_accum = set_smat(p_accum, 2'(i), 2'(j), p_elem);
                            end else begin
                                p_accum = set_smat(p_accum, 2'(i), 2'(j), (i == j) ? 32'h0001_0000 : 32'h0000_0000);
                            end
                        end
                    end

                    // 3. Symmetrize P_prior = 0.5 * (P_accum + P_accum^T)
                    p_sym = '0;
                    for (int i = 0; i < MAX_STATE_DIM; i++) begin
                        for (int j = 0; j < MAX_STATE_DIM; j++) begin
                            if (i < state_dim && j < state_dim) begin
                                p_ij    = get_smat(p_accum, 2'(i), 2'(j));
                                p_ji    = get_smat(p_accum, 2'(j), 2'(i));
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
