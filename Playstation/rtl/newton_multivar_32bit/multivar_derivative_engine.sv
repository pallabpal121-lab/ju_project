// =============================================================================
// File Name   : multivar_derivative_engine.sv
// Module Name : multivar_derivative_engine
// Project     : Universal Multivariable Newton 2nd-Order Accelerator
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Computes the N-dimensional Gradient vector g (N x 1) and regularized
//   Hessian matrix A = H + lambda*I (N x N) for any programmed equation.
// =============================================================================

`timescale 1ns / 1ps

import newton_multivar_pkg::*;
`include "multivar_helpers.svh"

module multivar_derivative_engine (
    input  logic               clk,         // System Clock
    input  logic               rst_n,       // Active-Low Reset

    // Programming Interface
    input  logic               prog_en,
    input  logic [4:0]         prog_addr,
    input  instr_t             prog_data,

    // Control & Inputs
    input  logic               start,
    input  logic [2:0]         num_vars,    // Active dimension N (1..4)
    input  vec_t               x_curr,      // Current position vector x
    input  q16_t               lambda_reg,  // Damping factor lambda

    // Outputs
    output q16_t               f_val,       // Function value f(x)
    output vec_t               vec_g,       // Gradient vector g (N x 1)
    output mat_t               mat_a,       // Regularized Hessian A = H + lambda*I (N x N)
    output logic               done,
    output logic               busy
);

    typedef enum logic [4:0] {
        M_IDLE          = 5'd0,
        M_START_F0      = 5'd1,
        M_WAIT_F0       = 5'd2,
        M_START_FP      = 5'd3,
        M_WAIT_FP       = 5'd4,
        M_START_FM      = 5'd5,
        M_WAIT_FM       = 5'd6,
        M_CALC_DIAG     = 5'd7,
        M_START_PP      = 5'd8,
        M_WAIT_PP       = 5'd9,
        M_START_PM      = 5'd10,
        M_WAIT_PM       = 5'd11,
        M_START_MP      = 5'd12,
        M_WAIT_MP       = 5'd13,
        M_START_MM      = 5'd14,
        M_WAIT_MM       = 5'd15,
        M_CALC_CROSS    = 5'd16,
        M_DONE          = 5'd17
    } m_state_t;

    m_state_t state;

    // DFG Evaluator Signals
    logic start_dfg;
    vec_t dfg_x_in;
    q16_t dfg_f_out;
    logic dfg_done, dfg_busy;

    // Sample storage
    q16_t f_0;
    q16_t f_plus, f_minus;
    q16_t f_pp, f_pm, f_mp, f_mm;

    // Loop indices
    logic [1:0] i_idx, j_idx;

    // Internal output registers
    vec_t g_reg;
    mat_t a_reg;

    assign vec_g = g_reg;
    assign mat_a = a_reg;

    // Instantiate Multivariable DFG Engine
    dfg_multivar_engine u_dfg (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_dfg),
        .num_vars   (num_vars),
        .x_vec      (dfg_x_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    vec_t tmp_vec;
    q16_t cross_diff;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= M_IDLE;
            start_dfg <= 1'b0;
            f_val     <= Q16_ZERO;
            f_0       <= Q16_ZERO;
            f_plus    <= Q16_ZERO;
            f_minus   <= Q16_ZERO;
            f_pp      <= Q16_ZERO;
            f_pm      <= Q16_ZERO;
            f_mp      <= Q16_ZERO;
            f_mm      <= Q16_ZERO;
            i_idx     <= '0;
            j_idx     <= '0;
            done      <= 1'b0;
            busy      <= 1'b0;
            dfg_x_in  <= '0;
            g_reg     <= '0;
            a_reg     <= '0;
        end else begin
            start_dfg <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: M_IDLE
                // -------------------------------------------------------------
                M_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= M_START_F0;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // CENTER SAMPLE: f(x)
                // -------------------------------------------------------------
                M_START_F0: begin
                    dfg_x_in  <= x_curr;
                    start_dfg <= 1'b1;
                    state     <= M_WAIT_F0;
                end

                M_WAIT_F0: begin
                    if (dfg_done) begin
                        f_0   <= dfg_f_out;
                        f_val <= dfg_f_out;
                        i_idx <= 2'd0;
                        state <= M_START_FP;
                    end
                end

                // =============================================================
                // PHASE 1: DIAGONAL ELEMENTS & GRADIENT VECTORS
                // =============================================================
                M_START_FP: begin
                    dfg_x_in  <= set_vec(x_curr, i_idx, get_vec(x_curr, i_idx) + Q16_H_STEP);
                    start_dfg <= 1'b1;
                    state     <= M_WAIT_FP;
                end

                M_WAIT_FP: begin
                    if (dfg_done) begin
                        f_plus <= dfg_f_out;
                        state  <= M_START_FM;
                    end
                end

                M_START_FM: begin
                    dfg_x_in  <= set_vec(x_curr, i_idx, get_vec(x_curr, i_idx) - Q16_H_STEP);
                    start_dfg <= 1'b1;
                    state     <= M_WAIT_FM;
                end

                M_WAIT_FM: begin
                    if (dfg_done) begin
                        f_minus <= dfg_f_out;
                        state   <= M_CALC_DIAG;
                    end
                end

                M_CALC_DIAG: begin
                    // 1. Gradient: g[i] = (f_+ - f_-) / (2h) = (f_+ - f_-) <<< 3
                    g_reg <= set_vec(g_reg, i_idx, (f_plus - f_minus) <<< 3);

                    // 2. Diagonal Hessian: H[i][i] = (f_+ - 2*f_0 + f_-) / h^2 = diff_2nd <<< 8
                    a_reg <= set_mat(a_reg, i_idx, i_idx, ((f_plus - (f_0 <<< 1) + f_minus) <<< 8) + lambda_reg);

                    if (i_idx + 1'b1 < num_vars) begin
                        i_idx <= i_idx + 1'b1;
                        state <= M_START_FP;
                    end else begin
                        if (num_vars > 1) begin
                            i_idx <= 2'd0;
                            j_idx <= 2'd1;
                            state <= M_START_PP;
                        end else begin
                            state <= M_DONE;
                        end
                    end
                end

                // =============================================================
                // PHASE 2: OFF-DIAGONAL CROSS-TERMS (4-Point Finite Differences)
                // =============================================================
                M_START_PP: begin
                    tmp_vec   = set_vec(x_curr, i_idx, get_vec(x_curr, i_idx) + Q16_H_STEP);
                    tmp_vec   = set_vec(tmp_vec, j_idx, get_vec(x_curr, j_idx) + Q16_H_STEP);
                    dfg_x_in  <= tmp_vec;
                    start_dfg <= 1'b1;
                    state     <= M_WAIT_PP;
                end

                M_WAIT_PP: begin
                    if (dfg_done) begin
                        f_pp  <= dfg_f_out;
                        state <= M_START_PM;
                    end
                end

                M_START_PM: begin
                    tmp_vec   = set_vec(x_curr, i_idx, get_vec(x_curr, i_idx) + Q16_H_STEP);
                    tmp_vec   = set_vec(tmp_vec, j_idx, get_vec(x_curr, j_idx) - Q16_H_STEP);
                    dfg_x_in  <= tmp_vec;
                    start_dfg <= 1'b1;
                    state     <= M_WAIT_PM;
                end

                M_WAIT_PM: begin
                    if (dfg_done) begin
                        f_pm  <= dfg_f_out;
                        state <= M_START_MP;
                    end
                end

                M_START_MP: begin
                    tmp_vec   = set_vec(x_curr, i_idx, get_vec(x_curr, i_idx) - Q16_H_STEP);
                    tmp_vec   = set_vec(tmp_vec, j_idx, get_vec(x_curr, j_idx) + Q16_H_STEP);
                    dfg_x_in  <= tmp_vec;
                    start_dfg <= 1'b1;
                    state     <= M_WAIT_MP;
                end

                M_WAIT_MP: begin
                    if (dfg_done) begin
                        f_mp  <= dfg_f_out;
                        state <= M_START_MM;
                    end
                end

                M_START_MM: begin
                    tmp_vec   = set_vec(x_curr, i_idx, get_vec(x_curr, i_idx) - Q16_H_STEP);
                    tmp_vec   = set_vec(tmp_vec, j_idx, get_vec(x_curr, j_idx) - Q16_H_STEP);
                    dfg_x_in  <= tmp_vec;
                    start_dfg <= 1'b1;
                    state     <= M_WAIT_MM;
                end

                M_WAIT_MM: begin
                    if (dfg_done) begin
                        f_mm  <= dfg_f_out;
                        state <= M_CALC_CROSS;
                    end
                end

                M_CALC_CROSS: begin
                    // H[i][j] = (f_++ - f_+- - f_-+ + f_--) / (4*h^2) = diff <<< 6
                    cross_diff = (f_pp - f_pm - f_mp + f_mm) <<< 6;
                    a_reg <= set_mat(set_mat(a_reg, i_idx, j_idx, cross_diff), j_idx, i_idx, cross_diff);

                    // Advance pair indices (i, j)
                    if (j_idx + 1'b1 < num_vars) begin
                        j_idx <= j_idx + 1'b1;
                        state <= M_START_PP;
                    end else if (i_idx + 2'd2 < num_vars) begin
                        i_idx <= i_idx + 1'b1;
                        j_idx <= i_idx + 2'd2;
                        state <= M_START_PP;
                    end else begin
                        state <= M_DONE;
                    end
                end

                // -------------------------------------------------------------
                // STATE: M_DONE
                // -------------------------------------------------------------
                M_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= M_IDLE;
                end

                default: state <= M_IDLE;
            endcase
        end
    end

endmodule
