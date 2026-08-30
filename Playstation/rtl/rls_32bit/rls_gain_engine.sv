// =============================================================================
// File Name   : rls_gain_engine.sv
// Module Name : rls_gain_engine
// Project     : Recursive Least Squares (RLS) Accelerator (Solver #23)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Kalman Gain Engine for RLS Adaptive Filtering.
//   1. A priori error:    α_t = d_t - w_{t-1}^T * x_t
//   2. Covariance vector: v_t = P_{t-1} * x_t
//   3. Normalization:     β_t = λ + x_t^T * v_t
//   4. Kalman Gain:       k_t = v_t / β_t
// =============================================================================

`timescale 1ns / 1ps

import rls_types_pkg::*;
`include "rls_helpers.svh"

module rls_gain_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [2:0]         num_taps,         // Number of filter taps N (1..4)
    input  vec_t               x_sample,         // Input feature vector x_t
    input  q16_t               d_meas,           // Desired scalar measurement d_t
    input  vec_t               w_curr,           // Current tap weights w_{t-1}
    input  mat_t               p_mat,            // Inverse covariance matrix P_{t-1}
    input  q16_t               lambda_factor,    // Forgetting factor λ

    output vec_t               k_gain,           // Kalman gain vector k_t
    output vec_t               v_vec,            // Intermediate vector v_t = P * x
    output q16_t               alpha_err,        // A priori error α_t
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        GAIN_IDLE      = 3'd0,
        GAIN_CALC_V    = 3'd1,
        GAIN_DIV_START = 3'd2,
        GAIN_DIV_WAIT  = 3'd3,
        GAIN_DONE      = 3'd4
    } gain_state_t;

    gain_state_t state;

    vec_t       v_reg;
    vec_t       k_reg;
    q16_t       alpha_reg;
    q16_t       beta_scalar;
    logic [2:0] div_idx;

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

    assign k_gain    = k_reg;
    assign v_vec     = v_reg;
    assign alpha_err = alpha_reg;

    q16_t wx_dot, xv_dot;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= GAIN_IDLE;
            v_reg        <= '0;
            k_reg        <= '0;
            alpha_reg    <= Q16_ZERO;
            beta_scalar  <= Q16_ONE;
            div_idx      <= 3'd0;
            div_start    <= 1'b0;
            div_dividend <= Q16_ZERO;
            div_divisor  <= Q16_ONE;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            div_start <= 1'b0;

            case (state)
                GAIN_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= GAIN_CALC_V;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Compute v = P * x, alpha = d - w^T * x, beta = λ + x^T * v
                GAIN_CALC_V: begin
                    v_reg     <= mat_vec_mul(p_mat, x_sample, num_taps, num_taps);
                    wx_dot    = vec_dot(w_curr, x_sample, num_taps);
                    alpha_reg <= d_meas - wx_dot;

                    xv_dot      = vec_dot(x_sample, mat_vec_mul(p_mat, x_sample, num_taps, num_taps), num_taps);
                    beta_scalar <= lambda_factor + xv_dot;

                    div_idx <= 3'd0;
                    state   <= GAIN_DIV_START;
                end

                // Step 2: Element-by-element division k_i = v_i / beta
                GAIN_DIV_START: begin
                    if (div_idx < num_taps) begin
                        div_dividend <= v_reg[div_idx];
                        div_divisor  <= (beta_scalar > 32'h0000_0001) ? beta_scalar : 32'h0000_0001;
                        div_start    <= 1'b1;
                        state        <= GAIN_DIV_WAIT;
                    end else begin
                        state <= GAIN_DONE;
                    end
                end

                GAIN_DIV_WAIT: begin
                    if (div_done) begin
                        k_reg[div_idx] <= div_quotient;
                        div_idx        <= div_idx + 1'b1;
                        state          <= GAIN_DIV_START;
                    end
                end

                GAIN_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= GAIN_IDLE;
                end

                default: state <= GAIN_IDLE;
            endcase
        end
    end

endmodule
