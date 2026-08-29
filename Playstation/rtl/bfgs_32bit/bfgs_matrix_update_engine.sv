// =============================================================================
// File Name   : bfgs_matrix_update_engine.sv
// Module Name : bfgs_matrix_update_engine
// Project     : Quasi-Newton BFGS Optimization Accelerator (Solver #4)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Computes the Symmetric Rank-2 Inverse Hessian Matrix update:
//     u = B · y
//     d = yᵀ · s
//     c = yᵀ · u
//     γ1 = (d + c) / d²
//     γ2 = 1 / d
//     B_(k+1) = B_k + γ1(s sᵀ) - γ2(s uᵀ + u sᵀ)
// =============================================================================

`timescale 1ns / 1ps

import bfgs_types_pkg::*;
`include "bfgs_helpers.svh"

module bfgs_matrix_update_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Control & Inputs
    input  logic               start,
    input  logic [2:0]         num_vars,    // Active dimension N (1..4)
    input  mat_t               B_in,        // Current Inverse Hessian B_k
    input  vec_t               vec_s,       // Step displacement s = x_(k+1) - x_k
    input  vec_t               vec_y,       // Gradient change y = g_(k+1) - g_k

    // Outputs
    output mat_t               B_out,       // Updated Inverse Hessian B_(k+1)
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        U_IDLE       = 3'd0,
        U_CALC_U_DOT = 3'd1,
        U_START_DIV1 = 3'd2,
        U_WAIT_DIV1  = 3'd3,
        U_START_DIV2 = 3'd4,
        U_WAIT_DIV2  = 3'd5,
        U_APPLY_B    = 3'd6,
        U_DONE       = 3'd7
    } u_state_t;

    u_state_t state;

    // Vector u = B · y
    vec_t vec_u;
    q16_t d_scalar; // d = yᵀs
    q16_t c_scalar; // c = yᵀu
    q16_t gamma_1;  // γ1 = (d + c) / d²
    q16_t gamma_2;  // γ2 = 1 / d
    q16_t q_inter;  // (d + c) / d

    // Divider Submodule
    logic        div_start;
    q16_t        div_dividend, div_divisor, div_quotient;
    logic        div_done, div_by_zero, div_busy;

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

    // Helper fixed-point multiplier
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    vec_t tmp_u;
    q16_t sum_u_j, sum_d, sum_c;
    mat_t next_B;
    q16_t term_ss, term_su_us;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= U_IDLE;
            B_out        <= '0;
            vec_u        <= '0;
            d_scalar     <= Q16_ZERO;
            c_scalar     <= Q16_ZERO;
            gamma_1      <= Q16_ZERO;
            gamma_2      <= Q16_ZERO;
            q_inter      <= Q16_ZERO;
            div_start    <= 1'b0;
            div_dividend <= Q16_ZERO;
            div_divisor  <= Q16_ZERO;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            div_start <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: U_IDLE
                // -------------------------------------------------------------
                U_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= U_CALC_U_DOT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // COMPUTE: u = B·y,  d = yᵀs,  c = yᵀu
                // -------------------------------------------------------------
                U_CALC_U_DOT: begin
                    // 1. u_j = sum_{k=0}^{N-1} B[j][k] * y[k]
                    tmp_u = '0;
                    for (int j = 0; j < MAX_VARS; j++) begin
                        if (j < num_vars[1:0]) begin
                            sum_u_j = 32'sd0;
                            for (int k = 0; k < MAX_VARS; k++) begin
                                if (k < num_vars[1:0]) begin
                                    sum_u_j = sum_u_j + q16_mul(get_mat(B_in, 2'(j), 2'(k)), get_vec(vec_y, 2'(k)));
                                end
                            end
                            tmp_u = set_vec(tmp_u, 2'(j), sum_u_j);
                        end
                    end
                    vec_u <= tmp_u;

                    // 2. d = yᵀs and c = yᵀu
                    sum_d = 32'sd0;
                    sum_c = 32'sd0;
                    for (int j = 0; j < MAX_VARS; j++) begin
                        if (j < num_vars[1:0]) begin
                            sum_d = sum_d + q16_mul(get_vec(vec_y, 2'(j)), get_vec(vec_s, 2'(j)));
                            sum_c = sum_c + q16_mul(get_vec(vec_y, 2'(j)), get_vec(tmp_u, 2'(j)));
                        end
                    end
                    d_scalar <= sum_d;
                    c_scalar <= sum_c;

                    // Check curvature condition: d = yᵀs > 0
                    if (sum_d <= 32'h0000_0020) begin
                        // Skip update if curvature is non-positive (preserves positive definiteness)
                        B_out <= B_in;
                        state <= U_DONE;
                    end else begin
                        // Launch Division 1: γ2 = 1 / d
                        div_dividend <= Q16_ONE;
                        div_divisor  <= sum_d;
                        div_start    <= 1'b1;
                        state        <= U_WAIT_DIV1;
                    end
                end

                // -------------------------------------------------------------
                // DIVIDER PHASE 1: Wait for γ2 = 1/d, launch q_inter = (d + c) / d
                // -------------------------------------------------------------
                U_WAIT_DIV1: begin
                    if (div_done) begin
                        gamma_2      <= div_quotient;
                        // Launch Division 2: q_inter = (d + c) / d
                        div_dividend <= d_scalar + c_scalar;
                        div_divisor  <= d_scalar;
                        div_start    <= 1'b1;
                        state        <= U_START_DIV2;
                    end
                end

                // -------------------------------------------------------------
                // DIVIDER PHASE 2: Wait for q_inter, launch γ1 = q_inter / d
                // -------------------------------------------------------------
                U_START_DIV2: begin
                    if (div_done) begin
                        q_inter      <= div_quotient;
                        // Launch Division 3: γ1 = ((d+c)/d) / d = (d+c)/d²
                        div_dividend <= div_quotient;
                        div_divisor  <= d_scalar;
                        div_start    <= 1'b1;
                        state        <= U_WAIT_DIV2;
                    end
                end

                U_WAIT_DIV2: begin
                    if (div_done) begin
                        gamma_1 <= div_quotient;
                        state   <= U_APPLY_B;
                    end
                end

                // -------------------------------------------------------------
                // APPLY RANK-2 UPDATE: B_(k+1) = B_k + γ1(s sᵀ) - γ2(s uᵀ + u sᵀ)
                // -------------------------------------------------------------
                U_APPLY_B: begin
                    next_B = B_in;
                    for (int j = 0; j < MAX_VARS; j++) begin
                        if (j < num_vars[1:0]) begin
                            for (int k = 0; k < MAX_VARS; k++) begin
                                if (k < num_vars[1:0]) begin
                                    // term1 = γ1 * s_j * s_k
                                    term_ss = q16_mul(gamma_1, q16_mul(get_vec(vec_s, 2'(j)), get_vec(vec_s, 2'(k))));
                                    // term2 = γ2 * (s_j * u_k + u_j * s_k)
                                    term_su_us = q16_mul(gamma_2, q16_mul(get_vec(vec_s, 2'(j)), get_vec(vec_u, 2'(k))) +
                                                                  q16_mul(get_vec(vec_u, 2'(j)), get_vec(vec_s, 2'(k))));
                                    next_B = set_mat(next_B, 2'(j), 2'(k), get_mat(B_in, 2'(j), 2'(k)) + term_ss - term_su_us);
                                end
                            end
                        end
                    end
                    B_out <= next_B;
                    state <= U_DONE;
                end

                // -------------------------------------------------------------
                // STATE: U_DONE
                // -------------------------------------------------------------
                U_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= U_IDLE;
                end

                default: state <= U_IDLE;
            endcase
        end
    end

endmodule
