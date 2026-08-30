// =============================================================================
// File Name   : ukf_correct_engine.sv
// Module Name : ukf_correct_engine
// Project     : Unscented Kalman Filter (UKF) Accelerator (Solver #25)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Measurement-Update (Correction Step) Engine for UKF.
//   1. Predicted Measurement: z_hat = ∑ W_i^(m) * γ_i
//   2. Innovation Covariance:  P_zz  = ∑ W_i^(c) * (γ_i - z_hat)(γ_i - z_hat)^T + R
//   3. Cross Covariance:       P_xz  = ∑ W_i^(c) * (χ_i - x_prior)(γ_i - z_hat)^T
//   4. Invert P_zz:            P_zz_inv = inv(P_zz) via hardware divider
//   5. Kalman Gain Matrix:     K = P_xz * P_zz_inv (4x2)
//   6. A Posteriori Updates:   x_post = x_prior + K * (z - z_hat)
//                              P_post = P_prior - K * P_zz * K^T
// =============================================================================

`timescale 1ns / 1ps

import ukf_types_pkg::*;
`include "ukf_helpers.svh"

module ukf_correct_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [2:0]         state_dim,        // State dimension N (1..4)
    input  logic [1:0]         meas_dim,         // Measurement dimension M (1..2)
    input  state_vec_t         x_prior,          // A priori state estimate
    input  state_mat_t         p_prior,          // A priori error covariance
    input  sigma_state_arr_t   sigma_state_in,   // State sigma points χ_i [36 elements]
    input  sigma_meas_arr_t    sigma_meas_in,    // Measurement sigma points γ_i [18 elements]
    input  meas_vec_t          z_meas,           // Actual measurement z
    input  weights_arr_t       weights_m,        // Mean weights W_i^(m)
    input  weights_arr_t       weights_c,        // Covariance weights W_i^(c)
    input  innov_mat_t         r_mat,            // Measurement noise covariance R

    output state_vec_t         x_post,           // Updated state estimate
    output state_mat_t         p_post,           // Updated error covariance
    output meas_vec_t          innov_y,          // Innovation residual (z - z_hat)
    output cross_mat_t         k_gain,           // Kalman gain matrix K (4x2)
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        CORR_IDLE     = 3'd0,
        CORR_MEAN_Z   = 3'd1,
        CORR_DIV_WAIT = 3'd2,
        CORR_CALC_K   = 3'd3,
        CORR_DONE     = 3'd4
    } corr_state_t;

    corr_state_t state;

    state_vec_t  x_reg;
    state_mat_t  p_reg;
    meas_vec_t   y_reg;
    cross_mat_t  k_reg;

    innov_mat_t  p_zz_reg;
    cross_mat_t  p_xz_reg;
    innov_mat_t  p_zz_inv_reg;
    q16_t        det_pzz;

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

    logic [3:0] num_pts;
    meas_vec_t  z_hat_comb;
    q16_t       sum_m, diff_m1, diff_m2, diff_x, prod_val, term_val, sig_m_elem;
    innov_mat_t p_zz_comb;
    cross_mat_t p_xz_comb;
    innov_mat_t p_zz_inv_comb;
    cross_mat_t k_comb;
    state_vec_t ky_comb;
    state_mat_t p_post_comb;
    q16_t       s00, s01, s10, s11, det_comb;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= CORR_IDLE;
            x_reg        <= '0;
            p_reg        <= '0;
            y_reg        <= '0;
            k_reg        <= '0;
            p_zz_reg     <= '0;
            p_xz_reg     <= '0;
            p_zz_inv_reg <= '0;
            det_pzz      <= 32'h0001_0000;
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
                        state <= CORR_MEAN_Z;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Compute z_hat, y = z - z_hat, P_zz, P_xz, det(P_zz)
                CORR_MEAN_Z: begin
                    num_pts = 4'(2 * state_dim + 1);

                    // 1. Mean measurement: z_hat = ∑ W_m[p] * γ[p]
                    z_hat_comb = '0;
                    for (int m = 0; m < MAX_MEAS_DIM; m++) begin
                        if (m < meas_dim) begin
                            sum_m = 32'sd0;
                            for (int p = 0; p < MAX_SIGMA_POINTS; p++) begin
                                if (p < num_pts) begin
                                    sig_m_elem = get_sigma_meas(sigma_meas_in, 4'(p), 1'(m));
                                    sum_m      = sum_m + q16_mul(weights_m[p], sig_m_elem);
                                end
                            end
                            z_hat_comb[m] = sum_m;
                            y_reg[m]     <= z_meas[m] - sum_m;
                        end else begin
                            y_reg[m] <= 32'sd0;
                        end
                    end

                    // 2. Innovation Covariance P_zz = R + ∑ W_c[p] * (γ_p - z_hat)(γ_p - z_hat)^T
                    p_zz_comb = r_mat;
                    for (int i = 0; i < MAX_MEAS_DIM; i++) begin
                        for (int j = 0; j < MAX_MEAS_DIM; j++) begin
                            if (i < meas_dim && j < meas_dim) begin
                                sum_m = get_imat(p_zz_comb, 1'(i), 1'(j));
                                for (int p = 0; p < MAX_SIGMA_POINTS; p++) begin
                                    if (p < num_pts) begin
                                        diff_m1  = get_sigma_meas(sigma_meas_in, 4'(p), 1'(i)) - z_hat_comb[i];
                                        diff_m2  = get_sigma_meas(sigma_meas_in, 4'(p), 1'(j)) - z_hat_comb[j];
                                        prod_val = q16_mul(diff_m1, diff_m2);
                                        term_val = q16_mul(weights_c[p], prod_val);
                                        sum_m    = sum_m + term_val;
                                    end
                                end
                                p_zz_comb = set_imat(p_zz_comb, 1'(i), 1'(j), sum_m);
                            end else begin
                                p_zz_comb = set_imat(p_zz_comb, 1'(i), 1'(j), (i == j) ? 32'h0001_0000 : 32'h0000_0000);
                            end
                        end
                    end
                    p_zz_reg <= p_zz_comb;

                    // 3. Cross Covariance P_xz = ∑ W_c[p] * (χ_p - x_prior)(γ_p - z_hat)^T (4x2)
                    p_xz_comb = '0;
                    for (int i = 0; i < MAX_STATE_DIM; i++) begin
                        for (int j = 0; j < MAX_MEAS_DIM; j++) begin
                            if (i < state_dim && j < meas_dim) begin
                                sum_m = 32'sd0;
                                for (int p = 0; p < MAX_SIGMA_POINTS; p++) begin
                                    if (p < num_pts) begin
                                        diff_x   = get_sigma_state(sigma_state_in, 4'(p), 2'(i)) - x_prior[i];
                                        diff_m2  = get_sigma_meas(sigma_meas_in, 4'(p), 1'(j)) - z_hat_comb[j];
                                        prod_val = q16_mul(diff_x, diff_m2);
                                        term_val = q16_mul(weights_c[p], prod_val);
                                        sum_m    = sum_m + term_val;
                                    end
                                end
                                p_xz_comb = set_cmat(p_xz_comb, 2'(i), 1'(j), sum_m);
                            end
                        end
                    end
                    p_xz_reg <= p_xz_comb;

                    // Determinant calculation
                    if (meas_dim == 2'd1) begin
                        det_comb = get_imat(p_zz_comb, 1'b0, 1'b0);
                        if (det_comb < 32'h0000_0004) det_comb = 32'h0000_0004;
                    end else begin
                        s00 = get_imat(p_zz_comb, 1'b0, 1'b0);
                        s01 = get_imat(p_zz_comb, 1'b0, 1'b1);
                        s10 = get_imat(p_zz_comb, 1'b1, 1'b0);
                        s11 = get_imat(p_zz_comb, 1'b1, 1'b1);
                        det_comb = q16_mul(s00, s11) - q16_mul(s01, s10);
                        if (det_comb < 32'h0000_0004) det_comb = 32'h0000_0004;
                    end
                    det_pzz <= det_comb;

                    // Trigger 1.0 / det(P_zz)
                    div_dividend <= 32'h0001_0000;
                    div_divisor  <= det_comb;
                    div_start    <= 1'b1;
                    state        <= CORR_DIV_WAIT;
                end

                // Step 2: Wait for divider to get inv_det = 1 / det(P_zz)
                CORR_DIV_WAIT: begin
                    if (div_done) begin
                        p_zz_inv_comb = '0;
                        if (meas_dim == 2'd1) begin
                            p_zz_inv_comb = set_imat(p_zz_inv_comb, 1'b0, 1'b0, div_quotient);
                            p_zz_inv_comb = set_imat(p_zz_inv_comb, 1'b1, 1'b1, 32'h0001_0000);
                        end else begin
                            s00 = get_imat(p_zz_reg, 1'b0, 1'b0);
                            s01 = get_imat(p_zz_reg, 1'b0, 1'b1);
                            s10 = get_imat(p_zz_reg, 1'b1, 1'b0);
                            s11 = get_imat(p_zz_reg, 1'b1, 1'b1);

                            p_zz_inv_comb = set_imat(p_zz_inv_comb, 1'b0, 1'b0, q16_mul(s11, div_quotient));
                            p_zz_inv_comb = set_imat(p_zz_inv_comb, 1'b0, 1'b1, q16_mul(-s01, div_quotient));
                            p_zz_inv_comb = set_imat(p_zz_inv_comb, 1'b1, 1'b0, q16_mul(-s10, div_quotient));
                            p_zz_inv_comb = set_imat(p_zz_inv_comb, 1'b1, 1'b1, q16_mul(s00, div_quotient));
                        end
                        p_zz_inv_reg <= p_zz_inv_comb;
                        state        <= CORR_CALC_K;
                    end
                end

                // Step 3: Compute K = P_xz * P_zz_inv, x_post = x_prior + K y, P_post = P_prior - K P_zz K^T
                CORR_CALC_K: begin
                    // K = P_xz * P_zz_inv: 4x2
                    k_comb = '0;
                    for (int i = 0; i < MAX_STATE_DIM; i++) begin
                        for (int j = 0; j < MAX_MEAS_DIM; j++) begin
                            if (i < state_dim && j < meas_dim) begin
                                sum_m = 32'sd0;
                                for (int m = 0; m < MAX_MEAS_DIM; m++) begin
                                    if (m < meas_dim) begin
                                        sum_m = sum_m + q16_mul(get_cmat(p_xz_reg, 2'(i), 1'(m)),
                                                                get_imat(p_zz_inv_reg, 1'(m), 1'(j)));
                                    end
                                end
                                k_comb = set_cmat(k_comb, 2'(i), 1'(j), sum_m);
                            end
                        end
                    end
                    k_reg <= k_comb;

                    // x_post = x_prior + K * y
                    ky_comb = cmat_vec_mul(k_comb, y_reg, state_dim, meas_dim);
                    for (int i = 0; i < MAX_STATE_DIM; i++) begin
                        if (i < state_dim) begin
                            x_reg[i] <= x_prior[i] + ky_comb[i];
                        end else begin
                            x_reg[i] <= 32'sd0;
                        end
                    end

                    // P_post = P_prior - K * P_zz * K^T
                    p_post_comb = mat_kpzk_sub(p_prior, k_comb, p_zz_reg, state_dim, meas_dim);
                    p_reg <= p_post_comb;

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
