// =============================================================================
// File Name   : pdhg_dual_engine.sv
// Module Name : pdhg_dual_engine
// Project     : Primal-Dual Hybrid Gradient (PDHG / Chambolle-Pock) Accelerator (Solver #19)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined hardware dual engine evaluating:
//   1. Forward dual candidate: y_tilde = y + sigma * K * x_bar
//   2. Dual proximal / projection operator:
//      - TV-L2 Mode: y_new = clamp(y_tilde, -lambda, +lambda)
//      - Quadratic / L2 Mode: y_new = (y_tilde - sigma * b) / (1 + sigma)
//   3. Dual infinity-norm residual: max_i |y_new[i] - y_old[i]|.
// =============================================================================

`timescale 1ns / 1ps

import pdhg_types_pkg::*;
`include "pdhg_helpers.svh"

module pdhg_dual_engine (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,

    // Dimensions & Parameters
    input  logic [2:0]  dim_m,         // Dual dimension M (1..4)
    input  logic [2:0]  dim_n,         // Primal dimension N (1..4)
    input  pdhg_mode_t  mode,          // Algorithm mode
    input  mat_t        k_matrix,      // Operator matrix K (M x N)
    input  vec_t        y_curr,        // Current dual vector y
    input  vec_t        x_bar,         // Extrapolated primal vector x_bar
    input  vec_t        b_target,      // Target observation vector b
    input  q16_t        sigma_step,    // Dual step size σ
    input  q16_t        lambda_param,  // Regularization parameter λ

    // Outputs
    output vec_t        y_next,        // Updated dual vector y_{k+1}
    output q16_t        res_dual,      // Dual infinity-norm step change
    output logic        done,
    output logic        busy
);

    typedef enum logic [2:0] {
        DUAL_IDLE  = 3'd0,
        DUAL_PROD  = 3'd1,
        DUAL_STEP  = 3'd2,
        DUAL_DIV   = 3'd3,
        DUAL_RES   = 3'd4,
        DUAL_DONE  = 3'd5
    } dual_state_t;

    dual_state_t state;

    vec_t k_xbar;
    vec_t y_tilde;
    vec_t y_reg;
    q16_t max_diff;
    logic [1:0] div_idx;

    // Divider for Quadratic Dual Modes (1 + sigma)
    logic div_start;
    q16_t div_num, div_den, div_quot;
    logic div_done, div_by_zero, div_busy;

    q16_divider u_div (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (div_start),
        .dividend   (div_num),
        .divisor    (div_den),
        .quotient   (div_quot),
        .done       (div_done),
        .div_by_zero(div_by_zero),
        .busy       (div_busy)
    );

    assign y_next   = y_reg;
    assign res_dual = max_diff;

    q16_t sig_kx, diff_i;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= DUAL_IDLE;
            k_xbar    <= '0;
            y_tilde   <= '0;
            y_reg     <= '0;
            max_diff  <= Q16_ZERO;
            div_idx   <= 2'd0;
            div_start <= 1'b0;
            div_num   <= Q16_ZERO;
            div_den   <= Q16_ONE;
            done      <= 1'b0;
            busy      <= 1'b0;
        end else begin
            div_start <= 1'b0;

            case (state)
                DUAL_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= DUAL_PROD;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Matrix-Vector Product K * x_bar
                DUAL_PROD: begin
                    k_xbar <= mat_vec_mul(k_matrix, x_bar, dim_m, dim_n);
                    state  <= DUAL_STEP;
                end

                // Step 2: Compute y_tilde = y + sigma * K * x_bar and branch on mode
                DUAL_STEP: begin
                    for (int i = 0; i < MAX_DIM; i++) begin
                        if (i < dim_m) begin
                            sig_kx = q16_mul(sigma_step, k_xbar[i]);
                            y_tilde[i] <= y_curr[i] + sig_kx;
                        end else begin
                            y_tilde[i] <= Q16_ZERO;
                        end
                    end

                    if (mode == MODE_TV_L2) begin
                        // TV-L2 Mode: y_{k+1} = clamp(y_tilde, -lambda, +lambda)
                        for (int i = 0; i < MAX_DIM; i++) begin
                            if (i < dim_m) begin
                                sig_kx = q16_mul(sigma_step, k_xbar[i]);
                                y_reg[i] <= clamp_box(y_curr[i] + sig_kx, lambda_param);
                            end else begin
                                y_reg[i] <= Q16_ZERO;
                            end
                        end
                        state <= DUAL_RES;
                    end else begin
                        // Quadratic / L2 Modes: y_{k+1} = (y_tilde - sigma * b) / (1 + sigma)
                        div_idx <= 2'd0;
                        sig_kx   = q16_mul(sigma_step, k_xbar[0]);
                        div_num <= (y_curr[0] + sig_kx) - q16_mul(sigma_step, b_target[0]);
                        div_den <= Q16_ONE + sigma_step;
                        div_start <= 1'b1;
                        state   <= DUAL_DIV;
                    end
                end

                // Step 3: Sequential Divisions for Quadratic Modes
                DUAL_DIV: begin
                    if (div_done) begin
                        y_reg[div_idx] <= div_quot;
                        if ({1'b0, div_idx} + 1'b1 < dim_m) begin
                            div_idx <= div_idx + 1'b1;
                            div_num <= y_tilde[div_idx + 1'b1] - q16_mul(sigma_step, b_target[div_idx + 1'b1]);
                            div_den <= Q16_ONE + sigma_step;
                            div_start <= 1'b1;
                        end else begin
                            state <= DUAL_RES;
                        end
                    end
                end

                // Step 4: Compute Dual Residual max_i |y_new[i] - y_old[i]|
                DUAL_RES: begin
                    max_diff = Q16_ZERO;
                    for (int i = 0; i < MAX_DIM; i++) begin
                        if (i < dim_m) begin
                            diff_i = q16_abs(y_reg[i] - y_curr[i]);
                            if (diff_i > max_diff) begin
                                max_diff = diff_i;
                            end
                        end
                    end
                    state <= DUAL_DONE;
                end

                DUAL_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= DUAL_IDLE;
                end

                default: state <= DUAL_IDLE;
            endcase
        end
    end

endmodule
