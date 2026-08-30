// =============================================================================
// File Name   : svm_pair_solver.sv
// Module Name : svm_pair_solver
// Project     : Support Vector Machine Sequential Minimal Optimization (SVM-SMO)
//               Accelerator (Solver #27)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined 2-Variable Analytic SMO Subproblem Solver.
//   Optimizes (α_1, α_2) pair analytically:
//   1. Computes feasible bounds [L, H]
//   2. Computes second derivative curvature η = 2*K_12 - K_11 - K_22
//   3. Evaluates unclipped α_2 = α_2_old - y_2*(E_1 - E_2)/η via divider
//   4. Clips α_2 in [L, H] and updates α_1 = α_1_old + s*(α_2_old - α_2_new)
//   5. Updates threshold bias b
// =============================================================================

`timescale 1ns / 1ps

import svm_types_pkg::*;
`include "svm_helpers.svh"

module svm_pair_solver (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  q16_t               a1_old,           // Current α_1
    input  q16_t               a2_old,           // Current α_2
    input  q16_t               y1,               // Label y_1 (-1.0 or +1.0)
    input  q16_t               y2,               // Label y_2 (-1.0 or +1.0)
    input  q16_t               e1,               // Error E_1 = f(x_1) - y_1
    input  q16_t               e2,               // Error E_2 = f(x_2) - y_2
    input  q16_t               k11,              // K(x_1, x_1)
    input  q16_t               k12,              // K(x_1, x_2)
    input  q16_t               k22,              // K(x_2, x_2)
    input  q16_t               c_bound,          // Box bound C (Q16.16)
    input  q16_t               b_current,        // Current bias b

    output q16_t               a1_new,           // Updated α_1
    output q16_t               a2_new,           // Updated α_2
    output q16_t               b_new,            // Updated bias b
    output logic               changed,          // High if α_2 changed > tol
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        PAIR_IDLE     = 3'd0,
        PAIR_BOUNDS   = 3'd1,
        PAIR_DIV_WAIT = 3'd2,
        PAIR_UPDATE   = 3'd3,
        PAIR_DONE     = 3'd4
    } pair_state_t;

    pair_state_t state;

    q16_t a1_reg, a2_reg, b_reg;
    logic changed_reg;

    assign a1_new  = a1_reg;
    assign a2_new  = a2_reg;
    assign b_new   = b_reg;
    assign changed = changed_reg;

    q16_t l_bound, h_bound;
    q16_t eta_curv;
    q16_t num_val;

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

    q16_t a2_unclipped, a2_clipped, da1, da2, s_sign;
    q16_t b1_cand, b2_cand;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= PAIR_IDLE;
            a1_reg       <= 32'sd0;
            a2_reg       <= 32'sd0;
            b_reg        <= 32'sd0;
            changed_reg  <= 1'b0;
            l_bound      <= 32'sd0;
            h_bound      <= 32'sd0;
            eta_curv     <= -32'h0001_0000;
            num_val      <= 32'sd0;
            div_start    <= 1'b0;
            div_dividend <= 32'sd0;
            div_divisor  <= 32'h0001_0000;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            div_start <= 1'b0;

            case (state)
                PAIR_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy        <= 1'b1;
                        changed_reg <= 1'b0;
                        state       <= PAIR_BOUNDS;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Feasible bounds [L, H] & curvature η
                PAIR_BOUNDS: begin
                    calc_L_H(a1_old, a2_old, y1, y2, c_bound, l_bound, h_bound);

                    if (l_bound >= h_bound) begin
                        // No feasible range for update
                        a1_reg      <= a1_old;
                        a2_reg      <= a2_old;
                        b_reg       <= b_current;
                        changed_reg <= 1'b0;
                        state       <= PAIR_DONE;
                    end else begin
                        // Curvature: η = 2*K12 - K11 - K22
                        eta_curv = q16_mul(32'h0002_0000, k12) - k11 - k22;
                        if (eta_curv >= -32'h0000_0010) begin
                            eta_curv = -32'h0000_0010; // Negative definite floor
                        end

                        // Numerator: num = y2 * (E1 - E2)
                        num_val = q16_mul(y2, e1 - e2);

                        // Trigger divider: quotient = num / eta
                        div_dividend <= num_val;
                        div_divisor  <= eta_curv;
                        div_start    <= 1'b1;
                        state        <= PAIR_DIV_WAIT;
                    end
                end

                // Step 2: Wait for divider to compute delta = num / eta
                PAIR_DIV_WAIT: begin
                    if (div_done) begin
                        // α_2_unclipped = α_2_old - delta
                        a2_unclipped = a2_old - div_quotient;

                        // Box Clipping to [L, H]
                        if (a2_unclipped > h_bound) begin
                            a2_clipped = h_bound;
                        end else if (a2_unclipped < l_bound) begin
                            a2_clipped = l_bound;
                        end else begin
                            a2_clipped = a2_unclipped;
                        end

                        // Check if change is significant (> 1e-4)
                        if (q16_abs(a2_clipped - a2_old) < 32'h0000_0008) begin
                            a1_reg      <= a1_old;
                            a2_reg      <= a2_old;
                            b_reg       <= b_current;
                            changed_reg <= 1'b0;
                            state       <= PAIR_DONE;
                        end else begin
                            // s = y1 * y2
                            s_sign = q16_mul(y1, y2);
                            da2    = a2_old - a2_clipped;
                            da1    = q16_mul(s_sign, da2);

                            // α_1_new = α_1_old + s * (α_2_old - α_2_new)
                            a1_reg <= a1_old + da1;
                            a2_reg <= a2_clipped;

                            // Threshold b candidate updates:
                            // b1 = b - E1 - y1*(a1_new - a1_old)*K11 - y2*(a2_new - a2_old)*K12
                            b1_cand = b_current - e1 - q16_mul(q16_mul(y1, -da1), k11) - q16_mul(q16_mul(y2, -da2), k12);
                            b2_cand = b_current - e2 - q16_mul(q16_mul(y1, -da1), k12) - q16_mul(q16_mul(y2, -da2), k22);

                            if (a1_old + da1 > Q16_ZERO && a1_old + da1 < c_bound) begin
                                b_reg <= b1_cand;
                            end else if (a2_clipped > Q16_ZERO && a2_clipped < c_bound) begin
                                b_reg <= b2_cand;
                            end else begin
                                b_reg <= q16_mul(32'h0000_8000, b1_cand + b2_cand);
                            end

                            changed_reg <= 1'b1;
                            state       <= PAIR_DONE;
                        end
                    end
                end

                PAIR_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= PAIR_IDLE;
                end

                default: state <= PAIR_IDLE;
            endcase
        end
    end

endmodule
