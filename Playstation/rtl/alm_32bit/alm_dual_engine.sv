// =============================================================================
// File Name   : alm_dual_engine.sv
// Module Name : alm_dual_engine
// Project     : Augmented Lagrangian Method (ALM) Accelerator (Solver #21)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined dual multiplier updater evaluating:
//   1. Equality residuals: r_eq = A * x - b
//   2. Inequality residuals: r_ineq = C * x - d
//   3. Multiplier updates:
//      - lambda_{k+1} = lambda_k + rho * r_eq
//      - mu_{k+1, i}  = max(0, mu_{k, i} + rho * r_ineq, i)
//   4. Feasibility violation norm: ||r||_inf
//   5. Penalty adaptation: rho_{k+1} = min(2 * rho_k, rho_max)
// =============================================================================

`timescale 1ns / 1ps

import alm_types_pkg::*;
`include "alm_helpers.svh"

module alm_dual_engine (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,

    input  logic [2:0]  dim_n,         // Primal variables N (1..4)
    input  logic [2:0]  num_eq,        // Equality constraints M_eq (0..4)
    input  logic [2:0]  num_ineq,      // Inequality constraints M_ineq (0..4)

    input  mat_t        a_mat,         // Equality constraint matrix A (M_eq x N)
    input  vec_t        b_vec,         // Equality RHS b
    input  mat_t        c_mat,         // Inequality constraint matrix C (M_ineq x N)
    input  vec_t        d_vec,         // Inequality RHS d

    input  vec_t        x_curr,        // Current primal point x
    input  vec_t        lambda_curr,   // Current equality multipliers λ
    input  vec_t        mu_curr,       // Current inequality multipliers μ
    input  q16_t        rho_curr,      // Current penalty ρ
    input  q16_t        prev_viol,     // Previous constraint violation

    output vec_t        lambda_next,   // Updated equality multipliers λ_{k+1}
    output vec_t        mu_next,       // Updated inequality multipliers μ_{k+1}
    output q16_t        rho_next,      // Updated penalty ρ_{k+1}
    output q16_t        curr_viol,     // Current constraint violation norm ||r||_inf
    output logic        done,
    output logic        busy
);

    typedef enum logic [1:0] {
        DUAL_IDLE = 2'd0,
        DUAL_CALC = 2'd1,
        DUAL_DONE = 2'd2
    } dual_state_t;

    dual_state_t state;

    vec_t ax_prod, cx_prod;
    vec_t r_eq, r_ineq;
    vec_t lam_reg, mu_reg;
    q16_t rho_reg;
    q16_t max_viol;

    assign lambda_next = lam_reg;
    assign mu_next     = mu_reg;
    assign rho_next    = rho_reg;
    assign curr_viol   = max_viol;

    q16_t rho_r, mu_trial, abs_r, ineq_viol;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= DUAL_IDLE;
            lam_reg  <= '0;
            mu_reg   <= '0;
            rho_reg  <= Q16_RHO_INIT;
            max_viol <= Q16_ZERO;
            done     <= 1'b0;
            busy     <= 1'b0;
        end else begin
            case (state)
                DUAL_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= DUAL_CALC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                DUAL_CALC: begin
                    ax_prod = mat_vec_mul(a_mat, x_curr, num_eq, dim_n);
                    cx_prod = mat_vec_mul(c_mat, x_curr, num_ineq, dim_n);

                    r_eq   = vec_sub(ax_prod, b_vec, num_eq);
                    r_ineq = vec_sub(cx_prod, d_vec, num_ineq);

                    max_viol = Q16_ZERO;

                    // 1. Equality Multiplier Updates: λ = λ + ρ * r_eq
                    for (int i = 0; i < MAX_DIM; i++) begin
                        if (i < num_eq) begin
                            rho_r = q16_mul(rho_curr, r_eq[i]);
                            lam_reg[i] <= lambda_curr[i] + rho_r;
                            abs_r = q16_abs(r_eq[i]);
                            if (abs_r > max_viol) begin
                                max_viol = abs_r;
                            end
                        end else begin
                            lam_reg[i] <= Q16_ZERO;
                        end
                    end

                    // 2. Inequality Multiplier Updates: μ = max(0, μ + ρ * r_ineq)
                    for (int j = 0; j < MAX_DIM; j++) begin
                        if (j < num_ineq) begin
                            rho_r    = q16_mul(rho_curr, r_ineq[j]);
                            mu_trial = mu_curr[j] + rho_r;
                            mu_reg[j] <= (mu_trial > Q16_ZERO) ? mu_trial : Q16_ZERO;

                            ineq_viol = (r_ineq[j] > Q16_ZERO) ? r_ineq[j] : Q16_ZERO;
                            if (ineq_viol > max_viol) begin
                                max_viol = ineq_viol;
                            end
                        end else begin
                            mu_reg[j] <= Q16_ZERO;
                        end
                    end

                    // 3. Penalty Adaptation
                    if (max_viol > q16_mul(Q16_HALF, prev_viol) && prev_viol != Q16_ZERO) begin
                        rho_reg <= (rho_curr < Q16_RHO_MAX) ? q16_mul(Q16_TWO, rho_curr) : Q16_RHO_MAX;
                    end else begin
                        rho_reg <= rho_curr;
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
