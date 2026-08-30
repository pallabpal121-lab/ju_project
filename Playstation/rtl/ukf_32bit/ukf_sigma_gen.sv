// =============================================================================
// File Name   : ukf_sigma_gen.sv
// Module Name : ukf_sigma_gen
// Project     : Unscented Kalman Filter (UKF) Accelerator (Solver #25)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Sigma-Point Generator for UKF.
//   1. Scales covariance A = γ^2 * P.
//   2. Evaluates Cholesky Factorization L * L^T = A via hardware sqrt & divider.
//   3. Generates 2N+1 deterministic sigma points:
//      χ_0   = x_mean
//      χ_i   = x_mean + L_col(i-1)    (i = 1..N)
//      χ_i+N = x_mean - L_col(i-1)    (i = 1..N)
// =============================================================================

`timescale 1ns / 1ps

import ukf_types_pkg::*;
`include "ukf_helpers.svh"

module ukf_sigma_gen (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [2:0]         state_dim,        // State dimension N (1..4)
    input  state_vec_t         x_mean,           // Current state mean estimate
    input  state_mat_t         p_cov,            // Current covariance matrix P
    input  q16_t               gamma_sq,         // Scale factor γ^2 = N + λ_ukf (Q16.16)

    output sigma_state_arr_t   sigma_points,     // 9 sigma points [36 elements]
    output state_mat_t         l_cholesky,       // Cholesky lower triangular L
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        SIG_IDLE           = 3'd0,
        SIG_INIT           = 3'd1,
        SIG_CHOL_DIAG      = 3'd2,
        SIG_CHOL_SQRT_WAIT = 3'd3,
        SIG_CHOL_OFF_START = 3'd4,
        SIG_CHOL_DIV_WAIT  = 3'd5,
        SIG_BUILD_POINTS   = 3'd6,
        SIG_DONE           = 3'd7
    } sig_state_t;

    sig_state_t state;

    sigma_state_arr_t sig_reg;
    state_mat_t       l_mat;
    state_mat_t       a_mat;
    logic [1:0]       col_j;
    logic [1:0]       row_i;

    // Hardware Square Root Unit
    logic sqrt_start;
    q16_t sqrt_rad, sqrt_root;
    logic sqrt_done, sqrt_busy;

    q16_sqrt u_sqrt (
        .clk  (clk),
        .rst_n(rst_n),
        .start(sqrt_start),
        .rad  (sqrt_rad),
        .root (sqrt_root),
        .done (sqrt_done),
        .busy (sqrt_busy)
    );

    // Hardware Divider Unit
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

    assign sigma_points = sig_reg;
    assign l_cholesky   = l_mat;

    q16_t sum_diag, sum_off, a_elem, l_elem, diag_val;
    state_mat_t a_comb;
    sigma_state_arr_t pts_comb;
    q16_t l_col_val;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= SIG_IDLE;
            sig_reg       <= '0;
            l_mat         <= '0;
            a_mat         <= '0;
            col_j         <= 2'd0;
            row_i         <= 2'd0;
            sqrt_start    <= 1'b0;
            sqrt_rad      <= 32'sd0;
            div_start     <= 1'b0;
            div_dividend  <= 32'sd0;
            div_divisor   <= 32'h0001_0000;
            done          <= 1'b0;
            busy          <= 1'b0;
        end else begin
            sqrt_start <= 1'b0;
            div_start  <= 1'b0;

            case (state)
                SIG_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= SIG_INIT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 0: Scale A = γ^2 * P
                SIG_INIT: begin
                    a_comb = '0;
                    for (int i = 0; i < MAX_STATE_DIM; i++) begin
                        for (int j = 0; j < MAX_STATE_DIM; j++) begin
                            if (i < state_dim && j < state_dim) begin
                                a_elem = get_smat(p_cov, 2'(i), 2'(j));
                                a_comb = set_smat(a_comb, 2'(i), 2'(j), q16_mul(gamma_sq, a_elem));
                            end else begin
                                a_comb = set_smat(a_comb, 2'(i), 2'(j), (i == j) ? 32'h0001_0000 : 32'h0000_0000);
                            end
                        end
                    end
                    a_mat <= a_comb;
                    l_mat <= '0;
                    col_j <= 2'd0;
                    state <= SIG_CHOL_DIAG;
                end

                // Step 1: Diagonal rad = A[j][j] - sum(L[j][k]^2)
                SIG_CHOL_DIAG: begin
                    sum_diag = 32'sd0;
                    for (int k = 0; k < MAX_STATE_DIM; k++) begin
                        if (k < col_j) begin
                            l_elem   = get_smat(l_mat, col_j, 2'(k));
                            sum_diag = sum_diag + q16_mul(l_elem, l_elem);
                        end
                    end
                    a_elem = get_smat(a_mat, col_j, col_j);
                    if (a_elem > sum_diag) begin
                        sqrt_rad <= a_elem - sum_diag;
                    end else begin
                        sqrt_rad <= 32'h0000_0010; // Regularization floor
                    end
                    sqrt_start <= 1'b1;
                    state      <= SIG_CHOL_SQRT_WAIT;
                end

                // Step 2: Latch L[j][j] = sqrt(rad)
                SIG_CHOL_SQRT_WAIT: begin
                    if (sqrt_done) begin
                        diag_val = (sqrt_root > 32'h0000_0004) ? sqrt_root : 32'h0000_0004;
                        l_mat   <= set_smat(l_mat, col_j, col_j, diag_val);
                        if (col_j + 1 < state_dim) begin
                            row_i <= col_j + 1'b1;
                            state <= SIG_CHOL_OFF_START;
                        end else begin
                            state <= SIG_BUILD_POINTS;
                        end
                    end
                end

                // Step 3: Off-diagonal num = A[i][j] - sum(L[i][k]*L[j][k])
                SIG_CHOL_OFF_START: begin
                    sum_off = 32'sd0;
                    for (int k = 0; k < MAX_STATE_DIM; k++) begin
                        if (k < col_j) begin
                            sum_off = sum_off + q16_mul(get_smat(l_mat, row_i, 2'(k)), get_smat(l_mat, col_j, 2'(k)));
                        end
                    end
                    a_elem       = get_smat(a_mat, row_i, col_j);
                    div_dividend <= a_elem - sum_off;
                    div_divisor  <= get_smat(l_mat, col_j, col_j);
                    div_start    <= 1'b1;
                    state        <= SIG_CHOL_DIV_WAIT;
                end

                // Step 4: Latch L[i][j] = quotient
                SIG_CHOL_DIV_WAIT: begin
                    if (div_done) begin
                        l_mat <= set_smat(l_mat, row_i, col_j, div_quotient);
                        if (row_i + 1 < state_dim) begin
                            row_i <= row_i + 1'b1;
                            state <= SIG_CHOL_OFF_START;
                        end else if (col_j + 1 < state_dim) begin
                            col_j <= col_j + 1'b1;
                            state <= SIG_CHOL_DIAG;
                        end else begin
                            state <= SIG_BUILD_POINTS;
                        end
                    end
                end

                // Step 5: Build all 9 Sigma Points
                SIG_BUILD_POINTS: begin
                    pts_comb = '0;

                    // Point 0: Central mean
                    for (int d = 0; d < MAX_STATE_DIM; d++) begin
                        pts_comb = set_sigma_state(pts_comb, 4'd0, 2'(d), x_mean[d]);
                    end

                    // Points 1..N and N+1..2N
                    for (int i = 0; i < MAX_STATE_DIM; i++) begin
                        if (i < state_dim) begin
                            for (int d = 0; d < MAX_STATE_DIM; d++) begin
                                l_col_val = get_smat(l_mat, 2'(d), 2'(i));
                                pts_comb  = set_sigma_state(pts_comb, 4'(i + 1), 2'(d), x_mean[d] + l_col_val);
                                pts_comb  = set_sigma_state(pts_comb, 4'(i + 1 + state_dim), 2'(d), x_mean[d] - l_col_val);
                            end
                        end
                    end

                    // Fill unused points with mean
                    for (int p = 0; p < MAX_SIGMA_POINTS; p++) begin
                        if (p > (2 * state_dim)) begin
                            for (int d = 0; d < MAX_STATE_DIM; d++) begin
                                pts_comb = set_sigma_state(pts_comb, 4'(p), 2'(d), x_mean[d]);
                            end
                        end
                    end

                    sig_reg <= pts_comb;
                    state   <= SIG_DONE;
                end

                SIG_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= SIG_IDLE;
                end

                default: state <= SIG_IDLE;
            endcase
        end
    end

endmodule
