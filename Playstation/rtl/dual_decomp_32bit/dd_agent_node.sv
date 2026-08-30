// =============================================================================
// File Name   : dd_agent_node.sv
// Module Name : dd_agent_node
// Project     : Dual Decomposition Engine (Solver #22)
// -----------------------------------------------------------------------------
// Description:
//   Parallel Local Agent Core for Dual Decomposition.
//   Solves local subproblem:
//   x_s^*(λ) = Q_s^{-1} * (p_s - A_s^T * λ)
//   and computes local resource usage A_s * x_s.
// =============================================================================

`timescale 1ns / 1ps

import dd_types_pkg::*;
`include "dd_helpers.svh"

module dd_agent_node #(
    parameter int AGENT_ID = 0
)(
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [1:0]         local_dim,    // Local dimension N_s (1..2)
    input  logic [1:0]         num_res,      // Number of coupled resources M (1..2)
    input  agent_mat_t         q_inv_mat,    // Local inverse Hessian Q_s^{-1} (2x2)
    input  agent_vec_t         p_vec,        // Local cost/profit vector p_s (2x1)
    input  couple_mat_t        a_mat,        // Local coupling matrix A_s (M x N_s)
    input  res_vec_t           lambda_bus,   // Global shadow price broadcast λ (2x1)

    output agent_vec_t         x_opt,        // Local optimal decision x_s
    output res_vec_t           res_usage,    // Local resource usage A_s * x_s
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        NODE_IDLE = 2'd0,
        NODE_STEP = 2'd1,
        NODE_DONE = 2'd2
    } node_state_t;

    node_state_t state;

    agent_vec_t at_lam;
    agent_vec_t eff_p;
    agent_vec_t x_reg;
    res_vec_t   ax_reg;

    assign x_opt     = x_reg;
    assign res_usage = ax_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= NODE_IDLE;
            at_lam <= '0;
            eff_p  <= '0;
            x_reg  <= '0;
            ax_reg <= '0;
            done   <= 1'b0;
            busy   <= 1'b0;
        end else begin
            case (state)
                NODE_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= NODE_STEP;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: at_lam = A_s^T * λ, eff_p = p_s - at_lam, x_s = Q_s^{-1} * eff_p
                NODE_STEP: begin
                    at_lam = mat2_t_vec_mul(a_mat, lambda_bus, num_res, local_dim);
                    eff_p  = vec2_sub(p_vec, at_lam, local_dim);
                    x_reg  <= mat2_vec_mul(q_inv_mat, eff_p, local_dim, local_dim);
                    ax_reg <= mat2_vec_mul(a_mat, mat2_vec_mul(q_inv_mat, eff_p, local_dim, local_dim), num_res, local_dim);
                    state  <= NODE_DONE;
                end

                NODE_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= NODE_IDLE;
                end

                default: state <= NODE_IDLE;
            endcase
        end
    end

endmodule
