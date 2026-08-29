// =============================================================================
// File Name   : fw_step_engine.sv
// Module Name : fw_step_engine
// Project     : Frank-Wolfe / Conditional Gradient Accelerator (Solver #20)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined step calculation engine evaluating:
//   1. Direction d = s - x
//   2. Frank-Wolfe Duality Gap: gap = g^T * (x - s)
//   3. Step size gamma:
//      - Exact Line Search: gamma = clamp(-g^T d / (d^T Q d), 0, 1)
//      - Diminishing Step:  gamma = 2 / (k + 2)
//   4. Convex update: x_{k+1} = (1 - gamma)*x + gamma*s.
// =============================================================================

`timescale 1ns / 1ps

import fw_types_pkg::*;
`include "fw_helpers.svh"

module fw_step_engine (
    input  logic          clk,
    input  logic          rst_n,
    input  logic          start,

    input  logic [2:0]    dim_n,       // Problem dimension N (1..4)
    input  fw_step_mode_t step_mode,   // Line search mode
    input  logic [15:0]   iter_k,      // Current iteration index k
    input  mat_t          q_matrix,    // Quadratic Hessian matrix Q
    input  vec_t          x_curr,      // Current primal point x_k
    input  vec_t          s_lmo,       // LMO extreme point s_k
    input  vec_t          grad_curr,   // Current gradient g_k

    output vec_t          x_next,      // Updated primal point x_{k+1}
    output q16_t          gamma_used,  // Applied step size γ in [0, 1]
    output q16_t          duality_gap, // Exact Frank-Wolfe duality gap
    output logic          done,
    output logic          busy
);

    typedef enum logic [2:0] {
        STEP_IDLE   = 3'd0,
        STEP_DIR    = 3'd1,
        STEP_CURV   = 3'd2,
        STEP_DIV    = 3'd3,
        STEP_UPDATE = 3'd4,
        STEP_DONE   = 3'd5
    } step_state_t;

    step_state_t state;

    vec_t d_dir;
    vec_t q_d;
    q16_t gap_reg;
    q16_t d_q_d;
    q16_t gamma_reg;
    vec_t x_next_reg;

    // Divider
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

    assign x_next      = x_next_reg;
    assign gamma_used  = gamma_reg;
    assign duality_gap = gap_reg;

    q16_t k_plus_2_q16;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= STEP_IDLE;
            d_dir      <= '0;
            q_d        <= '0;
            gap_reg    <= Q16_ZERO;
            d_q_d      <= Q16_ZERO;
            gamma_reg  <= Q16_ZERO;
            x_next_reg <= '0;
            div_start  <= 1'b0;
            div_num    <= Q16_ZERO;
            div_den    <= Q16_ONE;
            done       <= 1'b0;
            busy       <= 1'b0;
        end else begin
            div_start <= 1'b0;

            case (state)
                STEP_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= STEP_DIR;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: d = s - x, and gap = g^T * (x - s)
                STEP_DIR: begin
                    d_dir   <= vec_sub(s_lmo, x_curr, dim_n);
                    gap_reg <= dot_product(grad_curr, vec_sub(x_curr, s_lmo, dim_n), dim_n);
                    state   <= STEP_CURV;
                end

                // Step 2: Compute Q * d and d^T * (Q * d)
                STEP_CURV: begin
                    q_d   <= mat_vec_mul(q_matrix, d_dir, dim_n);
                    d_q_d <= dot_product(d_dir, mat_vec_mul(q_matrix, d_dir, dim_n), dim_n);

                    if (step_mode == STEP_DIMINISHING) begin
                        // Diminishing step: gamma = 2 / (k + 2)
                        k_plus_2_q16 = {16'd0, (iter_k + 16'd2), 16'd0} >> 16; // (k+2) in Q16.16
                        div_num   <= Q16_TWO;
                        div_den   <= {16'd0, (iter_k + 16'd2)};
                        div_start <= 1'b1;
                        state     <= STEP_DIV;
                    end else begin
                        // Exact Line Search: gamma = gap / (d^T Q d)
                        if (dot_product(d_dir, mat_vec_mul(q_matrix, d_dir, dim_n), dim_n) <= Q16_ZERO) begin
                            gamma_reg <= Q16_ONE;
                            state     <= STEP_UPDATE;
                        end else begin
                            div_num   <= (gap_reg < Q16_ZERO) ? Q16_ZERO : gap_reg;
                            div_den   <= dot_product(d_dir, mat_vec_mul(q_matrix, d_dir, dim_n), dim_n);
                            div_start <= 1'b1;
                            state     <= STEP_DIV;
                        end
                    end
                end

                // Step 3: Latch Divider Quotient and Clamp to [0, 1]
                STEP_DIV: begin
                    if (div_done) begin
                        if (div_quot < Q16_ZERO) begin
                            gamma_reg <= Q16_ZERO;
                        end else if (div_quot > Q16_ONE) begin
                            gamma_reg <= Q16_ONE;
                        end else begin
                            gamma_reg <= div_quot;
                        end
                        state <= STEP_UPDATE;
                    end
                end

                // Step 4: Convex combination x_{k+1} = (1 - gamma)*x + gamma*s
                STEP_UPDATE: begin
                    x_next_reg <= convex_comb(x_curr, s_lmo, gamma_reg, dim_n);
                    state      <= STEP_DONE;
                end

                STEP_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= STEP_IDLE;
                end

                default: state <= STEP_IDLE;
            endcase
        end
    end

endmodule
