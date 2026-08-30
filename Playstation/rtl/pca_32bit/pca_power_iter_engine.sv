// =============================================================================
// File Name   : pca_power_iter_engine.sv
// Module Name : pca_power_iter_engine
// Project     : Principal Component Analysis (PCA) / Streaming SVD Accelerator
//               (Solver #28)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Power Iteration & Rayleigh Quotient Engine with Gram-Schmidt
//   Deflation and Orthonormal Basis Guarantees.
// =============================================================================

`timescale 1ns / 1ps

import pca_types_pkg::*;
`include "pca_helpers.svh"

module pca_power_iter_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  cov_mat_t           cov_mat,          // Covariance matrix Σ (4x4)
    input  eigen_mat_t         v_mat_prev,       // Previously extracted eigenvectors V
    input  logic [1:0]         comp_idx,         // Current component index (0..3)
    input  logic [2:0]         feat_dim,         // Feature dimension D (1..4)
    input  logic [7:0]         max_iters,        // Maximum power iterations
    input  q16_t               tol_eps,          // Convergence tolerance

    output feature_vec_t       eigen_vec,        // Dominant eigenvector v (4x1)
    output q16_t               eigen_val,        // Corresponding eigenvalue λ
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        PI_IDLE       = 3'd0,
        PI_INIT       = 3'd1,
        PI_INIT_SQRT  = 3'd2,
        PI_INIT_DIV   = 3'd3,
        PI_MAT_MUL    = 3'd4,
        PI_SQRT_WAIT  = 3'd5,
        PI_NORM_DIV   = 3'd6,
        PI_CHECK      = 3'd7
    } pi_state_t;

    pi_state_t state;

    feature_vec_t q_reg;
    feature_vec_t q_new_reg;
    feature_vec_t y_reg;
    q16_t         lambda_reg;
    logic [7:0]   iter_cnt;
    logic [1:0]   norm_idx;

    assign eigen_vec = q_reg;
    assign eigen_val = lambda_reg;

    // Hardware Square Root Instance
    logic sqrt_start;
    q16_t sqrt_rad_in, sqrt_out;
    logic sqrt_done, sqrt_busy;

    q16_sqrt u_sqrt (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (sqrt_start),
        .rad_in  (sqrt_rad_in),
        .sqrt_out(sqrt_out),
        .done    (sqrt_done),
        .busy    (sqrt_busy)
    );

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

    feature_vec_t y_comb;
    q16_t         norm_sq, norm_len;
    q16_t         diff_i, diff_norm_sq;
    q16_t         proj_val, v_elem, sub_val;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= PI_IDLE;
            q_reg        <= '0;
            q_new_reg    <= '0;
            y_reg        <= '0;
            lambda_reg   <= 32'sd0;
            iter_cnt     <= 8'd0;
            norm_idx     <= 2'd0;
            sqrt_start   <= 1'b0;
            sqrt_rad_in  <= 32'sd0;
            div_start    <= 1'b0;
            div_dividend <= 32'sd0;
            div_divisor  <= 32'h0001_0000;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            sqrt_start <= 1'b0;
            div_start  <= 1'b0;

            case (state)
                PI_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy     <= 1'b1;
                        iter_cnt <= 8'd0;
                        state    <= PI_INIT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Initialize basis vector and orthogonalize against previous components
                PI_INIT: begin
                    for (int d = 0; d < MAX_FEATURES; d++) begin
                        if (d == comp_idx) begin
                            y_comb[d] = 32'h0001_0000; // 1.0
                        end else begin
                            y_comb[d] = 32'sd0;
                        end
                    end

                    // Gram-Schmidt orthogonalization on initial vector
                    for (int j = 0; j < MAX_COMPONENTS; j++) begin
                        if (j < comp_idx) begin
                            proj_val = 32'sd0;
                            for (int d = 0; d < MAX_FEATURES; d++) begin
                                if (d < feat_dim) begin
                                    v_elem   = get_eigen_elem(v_mat_prev, 2'(d), 2'(j));
                                    proj_val = proj_val + q16_mul(v_elem, y_comb[d]);
                                end
                            end
                            for (int d = 0; d < MAX_FEATURES; d++) begin
                                if (d < feat_dim) begin
                                    v_elem    = get_eigen_elem(v_mat_prev, 2'(d), 2'(j));
                                    sub_val   = q16_mul(proj_val, v_elem);
                                    y_comb[d] = y_comb[d] - sub_val;
                                end
                            end
                        end
                    end

                    y_reg <= y_comb;

                    norm_sq = 32'sd0;
                    for (int d = 0; d < MAX_FEATURES; d++) begin
                        if (d < feat_dim) begin
                            norm_sq = norm_sq + q16_mul(y_comb[d], y_comb[d]);
                        end
                    end

                    if (norm_sq <= 32'h0000_0004) begin
                        // Try secondary axis if primary was parallel to previous components
                        for (int d = 0; d < MAX_FEATURES; d++) begin
                            y_comb[d] = (d < feat_dim) ? 32'h0000_8000 : 32'sd0;
                        end
                        y_reg <= y_comb;
                        norm_sq = 32'h0001_0000;
                    end

                    sqrt_rad_in <= norm_sq;
                    sqrt_start  <= 1'b1;
                    state       <= PI_INIT_SQRT;
                end

                // Step 1B: Wait for sqrt of initial norm
                PI_INIT_SQRT: begin
                    if (sqrt_done) begin
                        norm_len <= (sqrt_out > 32'h0000_0004) ? sqrt_out : 32'h0001_0000;
                        norm_idx <= 2'd0;

                        div_dividend <= y_reg[0];
                        div_divisor  <= (sqrt_out > 32'h0000_0004) ? sqrt_out : 32'h0001_0000;
                        div_start    <= 1'b1;
                        state        <= PI_INIT_DIV;
                    end
                end

                // Step 1C: Normalize initial vector
                PI_INIT_DIV: begin
                    if (div_done) begin
                        q_reg[norm_idx] <= div_quotient;

                        if (norm_idx + 1'b1 < feat_dim) begin
                            norm_idx     <= norm_idx + 1'b1;
                            div_dividend <= y_reg[norm_idx + 1'b1];
                            div_divisor  <= norm_len;
                            div_start    <= 1'b1;
                        end else begin
                            state <= PI_MAT_MUL;
                        end
                    end
                end

                // Step 2: Compute y = Σ * q, Gram-Schmidt orthogonalize, and ||y||^2
                PI_MAT_MUL: begin
                    y_comb = cov_vec_mul(cov_mat, q_reg, feat_dim);

                    // Gram-Schmidt orthogonalization against previously extracted components
                    for (int j = 0; j < MAX_COMPONENTS; j++) begin
                        if (j < comp_idx) begin
                            proj_val = 32'sd0;
                            for (int d = 0; d < MAX_FEATURES; d++) begin
                                if (d < feat_dim) begin
                                    v_elem   = get_eigen_elem(v_mat_prev, 2'(d), 2'(j));
                                    proj_val = proj_val + q16_mul(v_elem, y_comb[d]);
                                end
                            end
                            for (int d = 0; d < MAX_FEATURES; d++) begin
                                if (d < feat_dim) begin
                                    v_elem    = get_eigen_elem(v_mat_prev, 2'(d), 2'(j));
                                    sub_val   = q16_mul(proj_val, v_elem);
                                    y_comb[d] = y_comb[d] - sub_val;
                                end
                            end
                        end
                    end

                    y_reg <= y_comb;

                    norm_sq = 32'sd0;
                    for (int d = 0; d < MAX_FEATURES; d++) begin
                        if (d < feat_dim) begin
                            norm_sq = norm_sq + q16_mul(y_comb[d], y_comb[d]);
                        end
                    end

                    if (norm_sq <= 32'h0000_0004) begin
                        lambda_reg <= 32'sd0;
                        done       <= 1'b1;
                        busy       <= 1'b0;
                        state      <= PI_IDLE;
                    end else begin
                        sqrt_rad_in <= norm_sq;
                        sqrt_start  <= 1'b1;
                        state       <= PI_SQRT_WAIT;
                    end
                end

                // Step 3: Wait for square root unit
                PI_SQRT_WAIT: begin
                    if (sqrt_done) begin
                        norm_len <= (sqrt_out > 32'h0000_0004) ? sqrt_out : 32'h0001_0000;
                        norm_idx <= 2'd0;

                        // Trigger divider for coordinate 0
                        div_dividend <= y_reg[0];
                        div_divisor  <= (sqrt_out > 32'h0000_0004) ? sqrt_out : 32'h0001_0000;
                        div_start    <= 1'b1;
                        state        <= PI_NORM_DIV;
                    end
                end

                // Step 4: Normalize all coordinates via sequential divider passes
                PI_NORM_DIV: begin
                    if (div_done) begin
                        q_new_reg[norm_idx] <= div_quotient;

                        if (norm_idx + 1'b1 < feat_dim) begin
                            norm_idx     <= norm_idx + 1'b1;
                            div_dividend <= y_reg[norm_idx + 1'b1];
                            div_divisor  <= norm_len;
                            div_start    <= 1'b1;
                        end else begin
                            state <= PI_CHECK;
                        end
                    end
                end

                // Step 5: Check convergence and compute Rayleigh Quotient
                PI_CHECK: begin
                    diff_norm_sq = 32'sd0;
                    for (int d = 0; d < MAX_FEATURES; d++) begin
                        if (d < feat_dim) begin
                            diff_i       = q_new_reg[d] - q_reg[d];
                            diff_norm_sq = diff_norm_sq + q16_mul(diff_i, diff_i);
                        end
                    end

                    q_reg    <= q_new_reg;
                    iter_cnt <= iter_cnt + 1'b1;

                    if ((diff_norm_sq < tol_eps && iter_cnt > 0) || (iter_cnt + 1'b1 >= max_iters)) begin
                        y_comb     = cov_vec_mul(cov_mat, q_new_reg, feat_dim);
                        lambda_reg <= rayleigh_quotient(q_new_reg, y_comb, feat_dim);
                        done       <= 1'b1;
                        busy       <= 1'b0;
                        state      <= PI_IDLE;
                    end else begin
                        state <= PI_MAT_MUL;
                    end
                end

                default: state <= PI_IDLE;
            endcase
        end
    end

endmodule
