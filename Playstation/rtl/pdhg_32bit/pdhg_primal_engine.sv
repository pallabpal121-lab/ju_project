// =============================================================================
// File Name   : pdhg_primal_engine.sv
// Module Name : pdhg_primal_engine
// Project     : Primal-Dual Hybrid Gradient (PDHG / Chambolle-Pock) Accelerator (Solver #19)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined hardware primal engine evaluating:
//   1. Adjoint product: v = K^T * y_{k+1}
//   2. Forward descent step: x_tilde = x_k - tau * K^T * y_{k+1}
//   3. Primal proximal operator:
//      - TV-L2 Mode: x_{k+1} = (x_tilde + tau * b) / (1 + tau)
//      - LASSO-L1 Mode: x_{k+1} = S_{tau * lambda}(x_tilde)
//      - Non-Negative Mode: x_{k+1} = max(x_tilde, 0)
//   4. Over-relaxation extrapolation: x_bar = x_{k+1} + theta * (x_{k+1} - x_k)
//   5. Primal infinity-norm residual: max_i |x_{k+1}[i] - x_k[i]|.
// =============================================================================

`timescale 1ns / 1ps

import pdhg_types_pkg::*;
`include "pdhg_helpers.svh"

module pdhg_primal_engine (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,

    // Dimensions & Parameters
    input  logic [2:0]  dim_m,         // Dual dimension M (1..4)
    input  logic [2:0]  dim_n,         // Primal dimension N (1..4)
    input  pdhg_mode_t  mode,          // Algorithm mode
    input  mat_t        k_matrix,      // Operator matrix K (M x N)
    input  vec_t        x_curr,        // Current primal vector x_k
    input  vec_t        y_next,        // Updated dual vector y_{k+1}
    input  vec_t        b_target,      // Observation vector b
    input  q16_t        tau_step,      // Primal step size τ
    input  q16_t        theta_relax,   // Relaxation parameter θ (e.g. 1.0)
    input  q16_t        lambda_param,  // Regularization parameter λ

    // Outputs
    output vec_t        x_next,        // Updated primal vector x_{k+1}
    output vec_t        x_bar_next,    // Extrapolated primal vector x_bar_{k+1}
    output q16_t        res_primal,    // Primal infinity-norm step change
    output logic        done,
    output logic        busy
);

    typedef enum logic [2:0] {
        PRIMAL_IDLE  = 3'd0,
        PRIMAL_PROD  = 3'd1,
        PRIMAL_STEP  = 3'd2,
        PRIMAL_DIV   = 3'd3,
        PRIMAL_EXTRAP= 3'd4,
        PRIMAL_RES   = 3'd5,
        PRIMAL_DONE  = 3'd6
    } primal_state_t;

    primal_state_t state;

    vec_t kt_y;
    vec_t x_tilde;
    vec_t x_reg;
    vec_t x_bar_reg;
    q16_t max_diff;
    logic [1:0] div_idx;

    // Divider for TV-L2 Mode: (x_tilde + tau * b) / (1 + tau)
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

    assign x_next     = x_reg;
    assign x_bar_next = x_bar_reg;
    assign res_primal = max_diff;

    q16_t tau_v, tau_lam, diff_i, dx_i, x_val;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= PRIMAL_IDLE;
            kt_y      <= '0;
            x_tilde   <= '0;
            x_reg     <= '0;
            x_bar_reg <= '0;
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
                PRIMAL_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= PRIMAL_PROD;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Matrix-Transpose-Vector Product K^T * y_{k+1}
                PRIMAL_PROD: begin
                    kt_y  <= mat_t_vec_mul(k_matrix, y_next, dim_m, dim_n);
                    state <= PRIMAL_STEP;
                end

                // Step 2: Compute x_tilde = x_k - tau * K^T * y_{k+1} and branch
                PRIMAL_STEP: begin
                    for (int i = 0; i < MAX_DIM; i++) begin
                        if (i < dim_n) begin
                            tau_v = q16_mul(tau_step, kt_y[i]);
                            x_tilde[i] <= x_curr[i] - tau_v;
                        end else begin
                            x_tilde[i] <= Q16_ZERO;
                        end
                    end

                    tau_lam = q16_mul(tau_step, lambda_param);

                    if (mode == MODE_TV_L2) begin
                        // TV-L2 Mode: x_{k+1} = (x_tilde + tau * b) / (1 + tau)
                        div_idx <= 2'd0;
                        tau_v    = q16_mul(tau_step, kt_y[0]);
                        div_num <= (x_curr[0] - tau_v) + q16_mul(tau_step, b_target[0]);
                        div_den <= Q16_ONE + tau_step;
                        div_start <= 1'b1;
                        state   <= PRIMAL_DIV;
                    end else if (mode == MODE_LASSO_L1) begin
                        // LASSO-L1 Mode: x_{k+1} = S_{tau * lambda}(x_tilde)
                        for (int i = 0; i < MAX_DIM; i++) begin
                            if (i < dim_n) begin
                                tau_v = q16_mul(tau_step, kt_y[i]);
                                x_reg[i] <= soft_thresh_scalar(x_curr[i] - tau_v, tau_lam);
                            end else begin
                                x_reg[i] <= Q16_ZERO;
                            end
                        end
                        state <= PRIMAL_EXTRAP;
                    end else begin
                        // Non-Negative Mode: x_{k+1} = max(x_tilde, 0)
                        for (int i = 0; i < MAX_DIM; i++) begin
                            if (i < dim_n) begin
                                tau_v = q16_mul(tau_step, kt_y[i]);
                                x_val = x_curr[i] - tau_v;
                                x_reg[i] <= (x_val < Q16_ZERO) ? Q16_ZERO : x_val;
                            end else begin
                                x_reg[i] <= Q16_ZERO;
                            end
                        end
                        state <= PRIMAL_EXTRAP;
                    end
                end

                // Step 3: Sequential Divisions for TV-L2 Mode
                PRIMAL_DIV: begin
                    if (div_done) begin
                        x_reg[div_idx] <= div_quot;
                        if ({1'b0, div_idx} + 1'b1 < dim_n) begin
                            div_idx <= div_idx + 1'b1;
                            div_num <= x_tilde[div_idx + 1'b1] + q16_mul(tau_step, b_target[div_idx + 1'b1]);
                            div_den <= Q16_ONE + tau_step;
                            div_start <= 1'b1;
                        end else begin
                            state <= PRIMAL_EXTRAP;
                        end
                    end
                end

                // Step 4: Over-relaxation extrapolation: x_bar = x_new + theta * (x_new - x_old)
                PRIMAL_EXTRAP: begin
                    for (int i = 0; i < MAX_DIM; i++) begin
                        if (i < dim_n) begin
                            dx_i = x_reg[i] - x_curr[i];
                            x_bar_reg[i] <= x_reg[i] + q16_mul(theta_relax, dx_i);
                        end else begin
                            x_bar_reg[i] <= Q16_ZERO;
                        end
                    end
                    state <= PRIMAL_RES;
                end

                // Step 5: Compute Primal Residual max_i |x_new[i] - x_old[i]|
                PRIMAL_RES: begin
                    max_diff = Q16_ZERO;
                    for (int i = 0; i < MAX_DIM; i++) begin
                        if (i < dim_n) begin
                            diff_i = q16_abs(x_reg[i] - x_curr[i]);
                            if (diff_i > max_diff) begin
                                max_diff = diff_i;
                            end
                        end
                    end
                    state <= PRIMAL_DONE;
                end

                PRIMAL_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= PRIMAL_IDLE;
                end

                default: state <= PRIMAL_IDLE;
            endcase
        end
    end

endmodule
