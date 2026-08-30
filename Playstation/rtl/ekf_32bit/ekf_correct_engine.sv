// =============================================================================
// File Name   : ekf_correct_engine.sv
// Module Name : ekf_correct_engine
// Project     : Extended Kalman Filter (EKF) Accelerator (Solver #24)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Measurement-Update (Correction Step) Engine for EKF.
//   1. Innovation:            y = z - h(x_prior)
//   2. Innovation Covariance: S = H * P_prior * H^T + R (2x2)
//   3. Invert S:              S_inv = inv(S) via hardware divider
//   4. Kalman Gain:           K = P_prior * H^T * S_inv (4x2)
//   5. A Posteriori State:    x_post = x_prior + K * y (4x1)
//   6. A Posteriori Cov:      P_post = P_prior - K * H * P_prior (4x4)
// =============================================================================

`timescale 1ns / 1ps

import ekf_types_pkg::*;
`include "ekf_helpers.svh"

module ekf_correct_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [2:0]         state_dim,        // State dimension N (1..4)
    input  logic [1:0]         meas_dim,         // Measurement dimension M (1..2)
    input  state_vec_t         x_prior,          // A priori state x_k^-
    input  state_mat_t         p_prior,          // A priori covariance P_k^-
    input  meas_vec_t          z_meas,           // Actual measurement z_k
    input  meas_vec_t          h_x_pred,         // Predicted measurement h(x_k^-)
    input  meas_mat_t          h_mat,            // Measurement Jacobian H_k
    input  innov_mat_t         r_mat,            // Measurement noise covariance R_k

    output state_vec_t         x_post,           // A posteriori state x_k
    output state_mat_t         p_post,           // A posteriori covariance P_k
    output meas_vec_t          innov_y,          // Innovation residual y_k
    output gain_mat_t          k_gain,           // Kalman gain matrix K_k
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        CORR_IDLE     = 3'd0,
        CORR_INNOV    = 3'd1,
        CORR_DIV_WAIT = 3'd2,
        CORR_CALC_K   = 3'd3,
        CORR_DONE     = 3'd4
    } corr_state_t;

    corr_state_t state;

    state_vec_t x_reg;
    state_mat_t p_reg;
    meas_vec_t  y_reg;
    gain_mat_t  k_reg;

    innov_mat_t s_mat_reg;
    innov_mat_t s_inv_reg;
    q16_t       det_s;

    // Hardware Divider Instance
    logic div_start;
    q16_t div_dividend, div_divisor, div_quotient;
    logic div_done, div_by_zero, div_busy;

    q16_divider u_div (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (div_start),
        .dividend   (div_dividend),
        .divisor    (div_divisor),
        .quotient   (div_quotient),
        .done       (div_done),
        .div_by_zero(div_by_zero),
        .busy       (div_busy)
    );

    assign x_post  = x_reg;
    assign p_post  = p_reg;
    assign innov_y = y_reg;
    assign k_gain  = k_reg;

    innov_mat_t hph_comb;
    innov_mat_t s_mat_comb;
    innov_mat_t s_inv_comb;
    gain_mat_t  pht_comb;
    gain_mat_t  k_comb;
    state_mat_t p_khp_comb;
    state_vec_t ky_comb;
    q16_t s00, s01, s10, s11;
    q16_t det_comb;
    q16_t sum_m;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= CORR_IDLE;
            x_reg        <= '0;
            p_reg        <= '0;
            y_reg        <= '0;
            k_reg        <= '0;
            s_mat_reg    <= '0;
            s_inv_reg    <= '0;
            det_s        <= 32'h0001_0000;
            div_start    <= 1'b0;
            div_dividend <= 32'h0001_0000;
            div_divisor  <= 32'h0001_0000;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            div_start <= 1'b0;

            case (state)
                CORR_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= CORR_INNOV;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Innovation y = z - h(x), S = H P H^T + R, det(S)
                CORR_INNOV: begin
                    // Innovation vector y
                    for (int i = 0; i < MAX_MEAS_DIM; i++) begin
                        if (i < meas_dim) begin
                            y_reg[i] <= z_meas[i] - h_x_pred[i];
                        end else begin
                            y_reg[i] <= 32'h0000_0000;
                        end
                    end

                    // S = H P H^T + R
                    hph_comb = mat_hph(h_mat, p_prior, meas_dim, state_dim);
                    s_mat_comb = '0;
                    for (int i = 0; i < MAX_MEAS_DIM; i++) begin
                        for (int j = 0; j < MAX_MEAS_DIM; j++) begin
                            if (i < meas_dim && j < meas_dim) begin
                                s_mat_comb = set_imat(s_mat_comb, 1'(i), 1'(j),
                                                      get_imat(hph_comb, 1'(i), 1'(j)) + get_imat(r_mat, 1'(i), 1'(j)));
                            end else begin
                                s_mat_comb = set_imat(s_mat_comb, 1'(i), 1'(j), (i == j) ? 32'h0001_0000 : 32'h0000_0000);
                            end
                        end
                    end
                    s_mat_reg <= s_mat_comb;

                    // Determinant calculation
                    if (meas_dim == 2'd1) begin
                        det_comb = get_imat(s_mat_comb, 1'b0, 1'b0);
                        if (det_comb < 32'h0000_0004) det_comb = 32'h0000_0004;
                    end else begin
                        s00 = get_imat(s_mat_comb, 1'b0, 1'b0);
                        s01 = get_imat(s_mat_comb, 1'b0, 1'b1);
                        s10 = get_imat(s_mat_comb, 1'b1, 1'b0);
                        s11 = get_imat(s_mat_comb, 1'b1, 1'b1);
                        det_comb = q16_mul(s00, s11) - q16_mul(s01, s10);
                        if (det_comb < 32'h0000_0004) det_comb = 32'h0000_0004;
                    end
                    det_s <= det_comb;

                    // Trigger 1.0 / det(S)
                    div_dividend <= 32'h0001_0000;
                    div_divisor  <= det_comb;
                    div_start    <= 1'b1;
                    state        <= CORR_DIV_WAIT;
                end

                // Step 2: Wait for divider to get inv_det = 1 / det(S)
                CORR_DIV_WAIT: begin
                    if (div_done) begin
                        s_inv_comb = '0;
                        if (meas_dim == 2'd1) begin
                            s_inv_comb = set_imat(s_inv_comb, 1'b0, 1'b0, div_quotient);
                            s_inv_comb = set_imat(s_inv_comb, 1'b1, 1'b1, 32'h0001_0000);
                        end else begin
                            s00 = get_imat(s_mat_reg, 1'b0, 1'b0);
                            s01 = get_imat(s_mat_reg, 1'b0, 1'b1);
                            s10 = get_imat(s_mat_reg, 1'b1, 1'b0);
                            s11 = get_imat(s_mat_reg, 1'b1, 1'b1);

                            s_inv_comb = set_imat(s_inv_comb, 1'b0, 1'b0, q16_mul(s11, div_quotient));
                            s_inv_comb = set_imat(s_inv_comb, 1'b0, 1'b1, q16_mul(-s01, div_quotient));
                            s_inv_comb = set_imat(s_inv_comb, 1'b1, 1'b0, q16_mul(-s10, div_quotient));
                            s_inv_comb = set_imat(s_inv_comb, 1'b1, 1'b1, q16_mul(s00, div_quotient));
                        end
                        s_inv_reg <= s_inv_comb;
                        state     <= CORR_CALC_K;
                    end
                end

                // Step 3: Compute K = (P H^T) S_inv, x_post = x_prior + K y, P_post = P - K H P
                CORR_CALC_K: begin
                    // PH^T: 4x2
                    pht_comb = mat_pht(p_prior, h_mat, state_dim, meas_dim);

                    // K = (PH^T) * S_inv: 4x2
                    k_comb = '0;
                    for (int i = 0; i < MAX_STATE_DIM; i++) begin
                        for (int j = 0; j < MAX_MEAS_DIM; j++) begin
                            if (i < state_dim && j < meas_dim) begin
                                sum_m = 32'h0000_0000;
                                for (int m = 0; m < MAX_MEAS_DIM; m++) begin
                                    if (m < meas_dim) begin
                                        sum_m = sum_m + q16_mul(get_gmat(pht_comb, 2'(i), 1'(m)),
                                                                get_imat(s_inv_reg, 1'(m), 1'(j)));
                                    end
                                end
                                k_comb = set_gmat(k_comb, 2'(i), 1'(j), sum_m);
                            end
                        end
                    end
                    k_reg <= k_comb;

                    // x_post = x_prior + K * y
                    ky_comb = gmat_vec_mul(k_comb, y_reg, state_dim, meas_dim);
                    for (int i = 0; i < MAX_STATE_DIM; i++) begin
                        if (i < state_dim) begin
                            x_reg[i] <= x_prior[i] + ky_comb[i];
                        end else begin
                            x_reg[i] <= 32'h0000_0000;
                        end
                    end

                    // P_post = P_prior - K * H * P_prior
                    p_khp_comb = mat_khp_sub(p_prior, k_comb, h_mat, state_dim, meas_dim);
                    p_reg <= p_khp_comb;

                    state <= CORR_DONE;
                end

                CORR_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= CORR_IDLE;
                end

                default: state <= CORR_IDLE;
            endcase
        end
    end

endmodule
