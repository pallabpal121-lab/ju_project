// =============================================================================
// File Name   : pca_cov_engine.sv
// Module Name : pca_cov_engine
// Project     : Principal Component Analysis (PCA) / Streaming SVD Accelerator
//               (Solver #28)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Sample Mean & Covariance Matrix Accumulator.
//   Evaluates:
//     1. x_mean_d = (1/M) * ∑ x_i,d
//     2. Σ_r,c = (1/(M-1)) * ∑ (x_i,r - x_mean_r) * (x_i,c - x_mean_c)
// =============================================================================

`timescale 1ns / 1ps

import pca_types_pkg::*;
`include "pca_helpers.svh"

module pca_cov_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [3:0]         num_samples,      // Total samples M (2..8)
    input  logic [2:0]         feat_dim,         // Feature dimension D (1..4)
    input  dataset_arr_t       dataset,          // Training dataset [32]

    output feature_vec_t       mean_vec,         // Mean vector x_mean (4x1)
    output cov_mat_t           cov_matrix,       // Covariance matrix Σ (4x4)
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        COV_IDLE      = 3'd0,
        COV_SUM_MEAN  = 3'd1,
        COV_DIV_MEAN  = 3'd2,
        COV_ACCUM_OUT = 3'd3,
        COV_DIV_COV   = 3'd4,
        COV_DONE      = 3'd5
    } cov_state_t;

    cov_state_t state;

    feature_vec_t mean_reg;
    cov_mat_t     cov_reg;

    assign mean_vec   = mean_reg;
    assign cov_matrix = cov_reg;

    logic [2:0] curr_feat;
    logic [1:0] curr_r, curr_c;

    // Divider interface
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

    q16_t sum_feat, sum_outer, diff_r, diff_c, prod_diff;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= COV_IDLE;
            mean_reg     <= '0;
            cov_reg      <= '0;
            curr_feat    <= 3'd0;
            curr_r       <= 2'd0;
            curr_c       <= 2'd0;
            div_start    <= 1'b0;
            div_dividend <= 32'sd0;
            div_divisor  <= 32'h0001_0000;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            div_start <= 1'b0;

            case (state)
                COV_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        mean_reg  <= '0;
                        cov_reg   <= '0;
                        curr_feat <= 3'd0;
                        state     <= COV_SUM_MEAN;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Compute sum over samples for feature curr_feat
                COV_SUM_MEAN: begin
                    sum_feat = 32'sd0;
                    for (int s = 0; s < MAX_SAMPLES; s++) begin
                        if (s < num_samples) begin
                            sum_feat = sum_feat + get_sample_feat(dataset, 3'(s), 2'(curr_feat));
                        end
                    end

                    // Trigger divider: mean = sum / M
                    div_dividend <= sum_feat;
                    div_divisor  <= {16'd0, num_samples, 16'd0}; // M in Q16.16
                    div_start    <= 1'b1;
                    state        <= COV_DIV_MEAN;
                end

                // Step 2: Latch mean and advance to next feature
                COV_DIV_MEAN: begin
                    if (div_done) begin
                        mean_reg[curr_feat] <= div_quotient;

                        if (curr_feat + 1'b1 < feat_dim) begin
                            curr_feat <= curr_feat + 1'b1;
                            state     <= COV_SUM_MEAN;
                        end else begin
                            curr_r <= 2'd0;
                            curr_c <= 2'd0;
                            state  <= COV_ACCUM_OUT;
                        end
                    end
                end

                // Step 3: Accumulate centered sum for covariance element (curr_r, curr_c)
                COV_ACCUM_OUT: begin
                    sum_outer = 32'sd0;
                    for (int s = 0; s < MAX_SAMPLES; s++) begin
                        if (s < num_samples) begin
                            diff_r    = get_sample_feat(dataset, 3'(s), curr_r) - mean_reg[curr_r];
                            diff_c    = get_sample_feat(dataset, 3'(s), curr_c) - mean_reg[curr_c];
                            prod_diff = q16_mul(diff_r, diff_c);
                            sum_outer = sum_outer + prod_diff;
                        end
                    end

                    // Trigger divider: cov = sum / (M - 1)
                    div_dividend <= sum_outer;
                    div_divisor  <= {16'd0, 4'(num_samples - 1'b1), 16'd0}; // (M-1) in Q16.16
                    div_start    <= 1'b1;
                    state        <= COV_DIV_COV;
                end

                // Step 4: Latch covariance element and advance indices
                COV_DIV_COV: begin
                    if (div_done) begin
                        cov_mat_t c_next;
                        c_next = cov_reg;
                        c_next = set_cov_elem(c_next, curr_r, curr_c, div_quotient);
                        c_next = set_cov_elem(c_next, curr_c, curr_r, div_quotient);
                        cov_reg <= c_next;

                        if (curr_c + 1'b1 < feat_dim) begin
                            curr_c <= curr_c + 1'b1;
                            state  <= COV_ACCUM_OUT;
                        end else if (curr_r + 1'b1 < feat_dim) begin
                            curr_r <= curr_r + 1'b1;
                            curr_c <= curr_r + 1'b1;
                            state  <= COV_ACCUM_OUT;
                        end else begin
                            state <= COV_DONE;
                        end
                    end
                end

                COV_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= COV_IDLE;
                end

                default: state <= COV_IDLE;
            endcase
        end
    end

endmodule
