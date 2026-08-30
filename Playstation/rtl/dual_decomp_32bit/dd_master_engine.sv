// =============================================================================
// File Name   : dd_master_engine.sv
// Module Name : dd_master_engine
// Project     : Dual Decomposition Engine (Solver #22)
// -----------------------------------------------------------------------------
// Description:
//   Master Shadow Price Coordinator for Dual Decomposition.
//   Aggregates total resource consumption: tot = sum_{s=1}^S A_s * x_s
//   Computes coupling residual: r = tot - c
//   Updates shadow price λ:
//     - Equality:   λ_{k+1} = λ_k + α * r
//     - Inequality: λ_{k+1} = max(0, λ_k + α * r)
// =============================================================================

`timescale 1ns / 1ps

import dd_types_pkg::*;
`include "dd_helpers.svh"

module dd_master_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [2:0]         num_agents,       // Number of active agents S (1..4)
    input  logic [1:0]         num_res,          // Number of resources M (1..2)
    input  dd_couple_mode_t    couple_mode,      // Equality or inequality coupling
    input  res_vec_t           c_budget,         // Global resource capacity c (2x1)
    input  res_vec_t           agent_res_usage[MAX_AGENTS], // Usage from each agent

    input  res_vec_t           lambda_curr,      // Current shadow price λ_k
    input  q16_t               step_alpha,       // Dual step size α

    output res_vec_t           lambda_next,      // Updated shadow price λ_{k+1}
    output res_vec_t           res_error,        // Coupling residual r
    output q16_t               viol_norm,        // Feasibility violation ||r||_inf
    output q16_t               price_delta,      // Price change ||λ_{k+1} - λ_k||_inf
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        M_IDLE = 2'd0,
        M_STEP = 2'd1,
        M_DONE = 2'd2
    } master_state_t;

    master_state_t state;

    res_vec_t tot_usage;
    res_vec_t r_vec;
    res_vec_t lam_reg;
    q16_t max_viol;
    q16_t max_pdelta;

    assign lambda_next = lam_reg;
    assign res_error   = r_vec;
    assign viol_norm   = max_viol;
    assign price_delta = max_pdelta;

    q16_t alpha_r, lam_trial, abs_r, ineq_viol, d_lam;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= M_IDLE;
            tot_usage  <= '0;
            r_vec      <= '0;
            lam_reg    <= '0;
            max_viol   <= Q16_ZERO;
            max_pdelta <= Q16_ZERO;
            done       <= 1'b0;
            busy       <= 1'b0;
        end else begin
            case (state)
                M_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= M_STEP;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                M_STEP: begin
                    // 1. Aggregate total resource consumption sum_{s=0}^{S-1} A_s * x_s
                    for (int m = 0; m < MAX_RESOURCES; m++) begin
                        if (m < num_res) begin
                            tot_usage[m] = Q16_ZERO;
                            for (int s = 0; s < MAX_AGENTS; s++) begin
                                if (s < num_agents) begin
                                    tot_usage[m] = tot_usage[m] + agent_res_usage[s][m];
                                end
                            end
                            r_vec[m] <= tot_usage[m] - c_budget[m];
                        end else begin
                            tot_usage[m] = Q16_ZERO;
                            r_vec[m]     <= Q16_ZERO;
                        end
                    end

                    max_viol   = Q16_ZERO;
                    max_pdelta = Q16_ZERO;

                    // 2. Update shadow prices λ_{k+1}
                    for (int m = 0; m < MAX_RESOURCES; m++) begin
                        if (m < num_res) begin
                            alpha_r   = q16_mul(step_alpha, tot_usage[m] - c_budget[m]);
                            lam_trial = lambda_curr[m] + alpha_r;

                            if (couple_mode == COUPLE_EQ) begin
                                lam_reg[m] <= lam_trial;
                                abs_r = q16_abs(tot_usage[m] - c_budget[m]);
                                if (abs_r > max_viol) begin
                                    max_viol = abs_r;
                                end
                                d_lam = q16_abs(alpha_r);
                                if (d_lam > max_pdelta) begin
                                    max_pdelta = d_lam;
                                end
                            end else begin
                                // Inequality: λ = max(0, λ + α * r)
                                if (lam_trial > Q16_ZERO) begin
                                    lam_reg[m] <= lam_trial;
                                end else begin
                                    lam_reg[m] <= Q16_ZERO;
                                end

                                ineq_viol = (tot_usage[m] - c_budget[m] > Q16_ZERO) ?
                                            (tot_usage[m] - c_budget[m]) : Q16_ZERO;
                                if (ineq_viol > max_viol) begin
                                    max_viol = ineq_viol;
                                end

                                d_lam = q16_abs(((lam_trial > Q16_ZERO) ? lam_trial : Q16_ZERO) - lambda_curr[m]);
                                if (d_lam > max_pdelta) begin
                                    max_pdelta = d_lam;
                                end
                            end
                        end else begin
                            lam_reg[m] <= Q16_ZERO;
                        end
                    end

                    state <= M_DONE;
                end

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
