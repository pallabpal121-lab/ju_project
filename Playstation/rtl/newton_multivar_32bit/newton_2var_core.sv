// =============================================================================
// File Name   : newton_2var_core.sv
// Module Name : newton_2var_core
// Project     : Universal Multivariable Newton 2nd-Order Accelerator
// -----------------------------------------------------------------------------
// Description: Dedicated 2-variable Newton Optimization Core.
//              Optimizes a pair of variables (x_i, x_j) within an N-variable
//              state vector X, holding all other N-2 variables frozen.
//              Uses 9-point finite-difference sampling and direct 2x2 solving.
// =============================================================================

`timescale 1ns / 1ps

import newton_multivar_pkg::*;
`include "multivar_helpers.svh"

module newton_2var_core (
    input  logic                          clk,
    input  logic                          rst_n,

    // Programming Interface (forwarded to DFG engine)
    input  logic                          prog_en,
    input  logic [$clog2(PROG_DEPTH)-1:0] prog_addr,
    input  instr_t                        prog_data,

    // Control & Inputs
    input  logic                          start,
    input  logic [NUM_VARS_BITS-1:0]      num_vars,    // Total active variables N
    input  vec_t                          x_full,      // Full N-variable state vector
    input  logic [NUM_VARS_BITS-1:0]      idx_i,       // Index of first variable to optimize
    input  logic [NUM_VARS_BITS-1:0]      idx_j,       // Index of second variable to optimize
    input  q16_t                          lambda_reg,  // Damping factor lambda
    input  q16_t                          step_alpha,  // Step size alpha

    // Outputs
    output q16_t                          delta_i,     // Computed update for x_i: alpha * p_i
    output q16_t                          delta_j,     // Computed update for x_j: alpha * p_j
    output q16_t                          f_val,       // Current function value f(x_full)
    output logic                          done,
    output logic                          busy
);

    // -------------------------------------------------------------------------
    // FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [4:0] {
        C2_IDLE      = 5'd0,
        C2_START_F0  = 5'd1,
        C2_WAIT_F0   = 5'd2,
        C2_START_PI  = 5'd3,
        C2_WAIT_PI   = 5'd4,
        C2_START_MI  = 5'd5,
        C2_WAIT_MI   = 5'd6,
        C2_START_PJ  = 5'd7,
        C2_WAIT_PJ   = 5'd8,
        C2_START_MJ  = 5'd9,
        C2_WAIT_MJ   = 5'd10,
        C2_START_PP  = 5'd11,
        C2_WAIT_PP   = 5'd12,
        C2_START_PM  = 5'd13,
        C2_WAIT_PM   = 5'd14,
        C2_START_MP  = 5'd15,
        C2_WAIT_MP   = 5'd16,
        C2_START_MM  = 5'd17,
        C2_WAIT_MM   = 5'd18,
        C2_CALC_2X2  = 5'd19,
        C2_DIV_I     = 5'd20,
        C2_WAIT_DIVI = 5'd21,
        C2_DIV_J     = 5'd22,
        C2_WAIT_DIVJ = 5'd23,
        C2_DONE      = 5'd24
    } c2_state_t;

    c2_state_t state;

    // DFG Evaluator Signals
    logic        dfg_start;
    vec_t        dfg_x_in;
    q16_t        dfg_f_out;
    logic        dfg_done, dfg_busy;

    // Samples
    q16_t f_0;
    q16_t f_pi, f_mi;
    q16_t f_pj, f_mj;
    q16_t f_pp, f_pm, f_mp, f_mm;

    // 2x2 Gradient and Hessian components
    q16_t g_i, g_j;
    q16_t A_00, A_11, A_01;
    q16_t det;
    q16_t num_i, num_j;
    q16_t p_i_reg, p_j_reg;

    // Divider Signals
    logic        div_start;
    q16_t        div_dividend, div_divisor, div_quotient;
    logic        div_done, div_by_zero, div_busy;

    // Fixed-point multiplier helper
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Submodule: Multivariable DFG Engine
    dfg_multivar_engine u_dfg (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (dfg_start),
        .num_vars   (num_vars),
        .x_vec      (dfg_x_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    // Submodule: Divider for 2x2 Cramer's Rule
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

    vec_t tmp_vec;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= C2_IDLE;
            dfg_start    <= 1'b0;
            div_start    <= 1'b0;
            dfg_x_in     <= '0;
            f_val        <= Q16_ZERO;
            f_0          <= Q16_ZERO;
            f_pi         <= Q16_ZERO;
            f_mi         <= Q16_ZERO;
            f_pj         <= Q16_ZERO;
            f_mj         <= Q16_ZERO;
            f_pp         <= Q16_ZERO;
            f_pm         <= Q16_ZERO;
            f_mp         <= Q16_ZERO;
            f_mm         <= Q16_ZERO;
            g_i          <= Q16_ZERO;
            g_j          <= Q16_ZERO;
            A_00         <= Q16_ZERO;
            A_11         <= Q16_ZERO;
            A_01         <= Q16_ZERO;
            det          <= Q16_ZERO;
            num_i        <= Q16_ZERO;
            num_j        <= Q16_ZERO;
            p_i_reg      <= Q16_ZERO;
            p_j_reg      <= Q16_ZERO;
            delta_i      <= Q16_ZERO;
            delta_j      <= Q16_ZERO;
            div_dividend <= Q16_ZERO;
            div_divisor  <= Q16_ZERO;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            dfg_start <= 1'b0;
            div_start <= 1'b0;

            case (state)
                C2_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= C2_START_F0;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // 1. Center Sample: f(X)
                C2_START_F0: begin
                    dfg_x_in  <= x_full;
                    dfg_start <= 1'b1;
                    state     <= C2_WAIT_F0;
                end

                C2_WAIT_F0: begin
                    if (dfg_done) begin
                        f_0   <= dfg_f_out;
                        f_val <= dfg_f_out;
                        state <= C2_START_PI;
                    end
                end

                // 2. Sample x_i + h
                C2_START_PI: begin
                    dfg_x_in  <= set_vec(x_full, idx_i, get_vec(x_full, idx_i) + Q16_H_STEP);
                    dfg_start <= 1'b1;
                    state     <= C2_WAIT_PI;
                end

                C2_WAIT_PI: begin
                    if (dfg_done) begin
                        f_pi  <= dfg_f_out;
                        state <= C2_START_MI;
                    end
                end

                // 3. Sample x_i - h
                C2_START_MI: begin
                    dfg_x_in  <= set_vec(x_full, idx_i, get_vec(x_full, idx_i) - Q16_H_STEP);
                    dfg_start <= 1'b1;
                    state     <= C2_WAIT_MI;
                end

                C2_WAIT_MI: begin
                    if (dfg_done) begin
                        f_mi  <= dfg_f_out;
                        state <= C2_START_PJ;
                    end
                end

                // 4. Sample x_j + h
                C2_START_PJ: begin
                    dfg_x_in  <= set_vec(x_full, idx_j, get_vec(x_full, idx_j) + Q16_H_STEP);
                    dfg_start <= 1'b1;
                    state     <= C2_WAIT_PJ;
                end

                C2_WAIT_PJ: begin
                    if (dfg_done) begin
                        f_pj  <= dfg_f_out;
                        state <= C2_START_MJ;
                    end
                end

                // 5. Sample x_j - h
                C2_START_MJ: begin
                    dfg_x_in  <= set_vec(x_full, idx_j, get_vec(x_full, idx_j) - Q16_H_STEP);
                    dfg_start <= 1'b1;
                    state     <= C2_WAIT_MJ;
                end

                C2_WAIT_MJ: begin
                    if (dfg_done) begin
                        f_mj  <= dfg_f_out;
                        state <= C2_START_PP;
                    end
                end

                // 6. Sample x_i + h, x_j + h
                C2_START_PP: begin
                    tmp_vec   = set_vec(x_full, idx_i, get_vec(x_full, idx_i) + Q16_H_STEP);
                    tmp_vec   = set_vec(tmp_vec, idx_j, get_vec(x_full, idx_j) + Q16_H_STEP);
                    dfg_x_in  <= tmp_vec;
                    dfg_start <= 1'b1;
                    state     <= C2_WAIT_PP;
                end

                C2_WAIT_PP: begin
                    if (dfg_done) begin
                        f_pp  <= dfg_f_out;
                        state <= C2_START_PM;
                    end
                end

                // 7. Sample x_i + h, x_j - h
                C2_START_PM: begin
                    tmp_vec   = set_vec(x_full, idx_i, get_vec(x_full, idx_i) + Q16_H_STEP);
                    tmp_vec   = set_vec(tmp_vec, idx_j, get_vec(x_full, idx_j) - Q16_H_STEP);
                    dfg_x_in  <= tmp_vec;
                    dfg_start <= 1'b1;
                    state     <= C2_WAIT_PM;
                end

                C2_WAIT_PM: begin
                    if (dfg_done) begin
                        f_pm  <= dfg_f_out;
                        state <= C2_START_MP;
                    end
                end

                // 8. Sample x_i - h, x_j + h
                C2_START_MP: begin
                    tmp_vec   = set_vec(x_full, idx_i, get_vec(x_full, idx_i) - Q16_H_STEP);
                    tmp_vec   = set_vec(tmp_vec, idx_j, get_vec(x_full, idx_j) + Q16_H_STEP);
                    dfg_x_in  <= tmp_vec;
                    dfg_start <= 1'b1;
                    state     <= C2_WAIT_MP;
                end

                C2_WAIT_MP: begin
                    if (dfg_done) begin
                        f_mp  <= dfg_f_out;
                        state <= C2_START_MM;
                    end
                end

                // 9. Sample x_i - h, x_j - h
                C2_START_MM: begin
                    tmp_vec   = set_vec(x_full, idx_i, get_vec(x_full, idx_i) - Q16_H_STEP);
                    tmp_vec   = set_vec(tmp_vec, idx_j, get_vec(x_full, idx_j) - Q16_H_STEP);
                    dfg_x_in  <= tmp_vec;
                    dfg_start <= 1'b1;
                    state     <= C2_WAIT_MM;
                end

                C2_WAIT_MM: begin
                    if (dfg_done) begin
                        f_mm  <= dfg_f_out;
                        state <= C2_CALC_2X2;
                    end
                end

                // -------------------------------------------------------------
                // Compute 2x2 Gradient and Hessian
                // -------------------------------------------------------------
                C2_CALC_2X2: begin
                    // Gradient components
                    g_i <= (f_pi - f_mi) <<< 3;
                    g_j <= (f_pj - f_mj) <<< 3;

                    // Hessian components + damping
                    A_00 <= ((f_pi - (f_0 <<< 1) + f_mi) <<< 8) + lambda_reg;
                    A_11 <= ((f_pj - (f_0 <<< 1) + f_mj) <<< 8) + lambda_reg;
                    A_01 <= (f_pp - f_pm - f_mp + f_mm) <<< 6;

                    // Determinant: det = A_00 * A_11 - A_01^2
                    det  <= q16_mul(((f_pi - (f_0 <<< 1) + f_mi) <<< 8) + lambda_reg,
                                    ((f_pj - (f_0 <<< 1) + f_mj) <<< 8) + lambda_reg) -
                            q16_mul((f_pp - f_pm - f_mp + f_mm) <<< 6,
                                    (f_pp - f_pm - f_mp + f_mm) <<< 6);

                    // Numerators for Cramer's rule:
                    // p_i = (-g_i * A_11 + g_j * A_01) / det
                    // p_j = (-g_j * A_00 + g_i * A_01) / det
                    num_i <= -q16_mul((f_pi - f_mi) <<< 3, ((f_pj - (f_0 <<< 1) + f_mj) <<< 8) + lambda_reg) +
                              q16_mul((f_pj - f_mj) <<< 3, (f_pp - f_pm - f_mp + f_mm) <<< 6);

                    num_j <= -q16_mul((f_pj - f_mj) <<< 3, ((f_pi - (f_0 <<< 1) + f_mi) <<< 8) + lambda_reg) +
                              q16_mul((f_pi - f_mi) <<< 3, (f_pp - f_pm - f_mp + f_mm) <<< 6);

                    state <= C2_DIV_I;
                end

                // Divide p_i = num_i / det
                C2_DIV_I: begin
                    if (det <= 32'sd16) begin
                        // Ill-conditioned fallback: gradient descent step
                        p_i_reg <= -g_i;
                        p_j_reg <= -g_j;
                        delta_i <= -q16_mul(step_alpha, g_i);
                        delta_j <= -q16_mul(step_alpha, g_j);
                        state   <= C2_DONE;
                    end else begin
                        div_dividend <= num_i;
                        div_divisor  <= det;
                        div_start    <= 1'b1;
                        state        <= C2_WAIT_DIVI;
                    end
                end

                C2_WAIT_DIVI: begin
                    if (div_done) begin
                        p_i_reg   <= div_quotient;
                        delta_i   <= q16_mul(step_alpha, div_quotient);
                        state     <= C2_DIV_J;
                    end
                end

                // Divide p_j = num_j / det
                C2_DIV_J: begin
                    div_dividend <= num_j;
                    div_divisor  <= det;
                    div_start    <= 1'b1;
                    state        <= C2_WAIT_DIVJ;
                end

                C2_WAIT_DIVJ: begin
                    if (div_done) begin
                        p_j_reg   <= div_quotient;
                        delta_j   <= q16_mul(step_alpha, div_quotient);
                        state     <= C2_DONE;
                    end
                end

                C2_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= C2_IDLE;
                end

                default: state <= C2_IDLE;
            endcase
        end
    end

endmodule
