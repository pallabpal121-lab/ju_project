// =============================================================================
// File Name   : mppi_weights_engine.sv
// Module Name : mppi_weights_engine
// Project     : Model Predictive Path Integral (MPPI) Accelerator (Solver #33)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Softmax Importance Sampling Weights Engine:
//   1. Finds minimum rollout cost: β = min_k S_k (combinational temporary search)
//   2. Computes exponent: η_k = -(1 / λ) * (S_k - β) <= 0
//   3. Evaluates fixed-point exponential: e_k = exp(η_k)
//   4. Normalizes weights: w_k = e_k / (∑_{j=0}^{K-1} e_j) via hardware divider
// =============================================================================

`timescale 1ns / 1ps

import mppi_types_pkg::*;
`include "mppi_helpers.svh"

module mppi_weights_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  cost_arr_t          rollout_costs,    // Evaluated costs S_k (4x1)
    input  logic [2:0]         num_rollouts,     // Total rollouts K (2..4)
    input  q16_t               lambda_inv,       // 1 / λ temperature parameter

    output weight_arr_t        weights,          // Normalized importance weights w_k (4x1)
    output q16_t               min_cost,         // Minimum trajectory cost β
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        WGT_IDLE     = 3'd0,
        WGT_MIN_COST = 3'd1,
        WGT_EXP_EVAL = 3'd2,
        WGT_DIV_START= 3'd3,
        WGT_DIV_WAIT = 3'd4,
        WGT_DONE     = 3'd5
    } wgt_state_t;

    wgt_state_t state;

    weight_arr_t weights_reg;
    q16_t        min_cost_reg;
    q16_t        sum_exp_reg;

    assign weights  = weights_reg;
    assign min_cost = min_cost_reg;

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

    q16_t exp_vals [0:MAX_ROLLOUTS-1];
    logic [1:0] curr_k;
    q16_t temp_min, temp_sum, diff_cost, eta_val, exp_k;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= WGT_IDLE;
            weights_reg  <= '0;
            min_cost_reg <= Q16_MAX_POS;
            sum_exp_reg  <= 32'h0001_0000;
            curr_k       <= 2'd0;
            div_start    <= 1'b0;
            div_dividend <= 32'sd0;
            div_divisor  <= 32'h0001_0000;
            done         <= 1'b0;
            busy         <= 1'b0;
            for (int k = 0; k < MAX_ROLLOUTS; k++) begin
                exp_vals[k] <= 32'sd0;
            end
        end else begin
            div_start <= 1'b0;

            case (state)
                WGT_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy         <= 1'b1;
                        weights_reg  <= '0;
                        min_cost_reg <= Q16_MAX_POS;
                        state        <= WGT_MIN_COST;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Find min cost β = min_k S_k (combinational search)
                WGT_MIN_COST: begin
                    temp_min = rollout_costs[0];
                    for (int k = 1; k < MAX_ROLLOUTS; k++) begin
                        if (k < num_rollouts) begin
                            if (rollout_costs[k] < temp_min) begin
                                temp_min = rollout_costs[k];
                            end
                        end
                    end
                    min_cost_reg <= temp_min;
                    state        <= WGT_EXP_EVAL;
                end

                // Step 2: Compute e_k = exp(-(1/λ) * (S_k - β)) and sum_exp = ∑ e_k
                WGT_EXP_EVAL: begin
                    temp_sum = 32'sd0;
                    for (int k = 0; k < MAX_ROLLOUTS; k++) begin
                        if (k < num_rollouts) begin
                            diff_cost   = rollout_costs[k] - min_cost_reg;
                            eta_val     = -q16_mul(lambda_inv, diff_cost);
                            exp_k       = fixed_exp(eta_val);
                            exp_vals[k] <= exp_k;
                            temp_sum    = temp_sum + exp_k;
                        end else begin
                            exp_vals[k] <= 32'sd0;
                        end
                    end

                    sum_exp_reg <= (temp_sum != 32'sd0) ? temp_sum : 32'h0001_0000;
                    curr_k      <= 2'd0;
                    state       <= WGT_DIV_START;
                end

                // Step 3: Trigger divider for w_k = e_k / sum_exp
                WGT_DIV_START: begin
                    div_dividend <= exp_vals[curr_k];
                    div_divisor  <= sum_exp_reg;
                    div_start    <= 1'b1;
                    state        <= WGT_DIV_WAIT;
                end

                // Step 4: Latch weight w_k and advance
                WGT_DIV_WAIT: begin
                    if (div_done) begin
                        weights_reg[curr_k] <= div_quotient;

                        if (curr_k + 1'b1 < num_rollouts) begin
                            curr_k <= curr_k + 1'b1;
                            state  <= WGT_DIV_START;
                        end else begin
                            state  <= WGT_DONE;
                        end
                    end
                end

                WGT_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= WGT_IDLE;
                end

                default: state <= WGT_IDLE;
            endcase
        end
    end

endmodule
