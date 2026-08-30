// =============================================================================
// File Name   : nes_grad_engine.sv
// Module Name : nes_grad_engine
// Project     : Natural Evolution Strategies (NES) Accelerator (Solver #32)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Stochastic Policy Gradient Estimator:
//     g_d = (1 / (2 * P * σ)) * ∑_{p=0}^{P-1} (R_p^+ - R_p^-) * ε_{p, d}
//   Computes gradient vector g and its squared norm ||g||^2.
// =============================================================================

`timescale 1ns / 1ps

import nes_types_pkg::*;
`include "nes_helpers.svh"

module nes_grad_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  noise_arr_t         noise_arr,        // Perturbations ε
    input  reward_arr_t        pos_rewards,      // Positive rollout rewards R+
    input  reward_arr_t        neg_rewards,      // Negative rollout rewards R-
    input  logic [2:0]         num_pairs,        // Total pairs P (2..4)
    input  logic [2:0]         dim,              // Dimension D (1..4)
    input  q16_t               sigma,            // Exploration standard deviation σ

    output policy_vec_t        grad_vec,         // Estimated gradient g
    output q16_t               grad_norm_sq,     // Squared gradient norm ||g||^2
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        GRD_IDLE     = 3'd0,
        GRD_ACCUM    = 3'd1,
        GRD_DIV_WAIT = 3'd2,
        GRD_NORM     = 3'd3,
        GRD_DONE     = 3'd4
    } grd_state_t;

    grd_state_t state;

    policy_vec_t grad_reg;
    q16_t        norm_reg;

    assign grad_vec     = grad_reg;
    assign grad_norm_sq = norm_reg;

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

    logic [1:0] curr_d;
    q16_t       sum_acc, r_diff, term_prod, two_p_sigma, p_q16;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= GRD_IDLE;
            grad_reg     <= '0;
            norm_reg     <= 32'sd0;
            curr_d       <= 2'd0;
            div_start    <= 1'b0;
            div_dividend <= 32'sd0;
            div_divisor  <= 32'h0001_0000;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            div_start <= 1'b0;

            case (state)
                GRD_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy     <= 1'b1;
                        grad_reg <= '0;
                        norm_reg <= 32'sd0;
                        curr_d   <= 2'd0;
                        state    <= GRD_ACCUM;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Accumulate sum_{p=0}^{P-1} (R_p^+ - R_p^-) * ε_{p, d}
                GRD_ACCUM: begin
                    sum_acc = 32'sd0;
                    for (int p = 0; p < MAX_PAIRS; p++) begin
                        if (p < num_pairs) begin
                            r_diff    = pos_rewards[p] - neg_rewards[p];
                            term_prod = q16_mul(r_diff, get_noise_val(noise_arr, 2'(p), curr_d));
                            sum_acc   = sum_acc + term_prod;
                        end
                    end

                    // Denominator: 2 * P * σ
                    p_q16       = {16'd0, 13'd0, num_pairs, 16'd0};
                    two_p_sigma = q16_mul(32'h0002_0000, q16_mul(p_q16, sigma));

                    div_dividend <= sum_acc;
                    div_divisor  <= (two_p_sigma != 32'sd0) ? two_p_sigma : 32'h0001_0000;
                    div_start    <= 1'b1;
                    state        <= GRD_DIV_WAIT;
                end

                // Step 2: Latch gradient coordinate g_d
                GRD_DIV_WAIT: begin
                    if (div_done) begin
                        grad_reg[curr_d] <= div_quotient;

                        if (curr_d + 1'b1 < dim) begin
                            curr_d <= curr_d + 1'b1;
                            state  <= GRD_ACCUM;
                        end else begin
                            state <= GRD_NORM;
                        end
                    end
                end

                // Step 3: Compute squared gradient norm ||g||^2
                GRD_NORM: begin
                    norm_reg <= 32'sd0;
                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim) begin
                            norm_reg <= norm_reg + q16_mul(grad_reg[d], grad_reg[d]);
                        end
                    end
                    state <= GRD_DONE;
                end

                GRD_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= GRD_IDLE;
                end

                default: state <= GRD_IDLE;
            endcase
        end
    end

endmodule
