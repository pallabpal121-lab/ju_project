// =============================================================================
// File Name   : mpc_condense_engine.sv
// Module Name : mpc_condense_engine
// Project     : Model Predictive Control (MPC) Accelerator (Solver #26)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Gradient & Condensing Engine for MPC.
//   Evaluates linear gradient vector:
//     g_mpc = M_x * x_curr - M_ref * x_ref
// =============================================================================

`timescale 1ns / 1ps

import mpc_types_pkg::*;
`include "mpc_helpers.svh"

module mpc_condense_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [2:0]         state_dim,        // State dimension Nx (1..4)
    input  logic [2:0]         stacked_dim,      // Stacked control horizon dim (1..4)
    input  state_vec_t         x_curr,           // Current measured state x_0
    input  state_vec_t         x_ref,            // Reference target state x_ref
    input  grad_mat_t          m_x_mat,          // State mapping matrix M_x (4x4)
    input  grad_mat_t          m_ref_mat,        // Reference mapping matrix M_ref (4x4)

    output stacked_vec_t       g_mpc,            // Linear gradient vector g (4x1)
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        COND_IDLE = 2'd0,
        COND_CALC = 2'd1,
        COND_DONE = 2'd2
    } cond_state_t;

    cond_state_t state;

    stacked_vec_t g_reg;
    assign g_mpc = g_reg;

    stacked_vec_t mx_comb;
    stacked_vec_t mref_comb;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= COND_IDLE;
            g_reg <= '0;
            done  <= 1'b0;
            busy  <= 1'b0;
        end else begin
            case (state)
                COND_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= COND_CALC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                COND_CALC: begin
                    mx_comb   = map_vec_mul(m_x_mat, x_curr, stacked_dim, state_dim);
                    mref_comb = map_vec_mul(m_ref_mat, x_ref, stacked_dim, state_dim);

                    for (int i = 0; i < MAX_STACKED_DIM; i++) begin
                        if (i < stacked_dim) begin
                            g_reg[i] <= mx_comb[i] - mref_comb[i];
                        end else begin
                            g_reg[i] <= 32'sd0;
                        end
                    end

                    state <= COND_DONE;
                end

                COND_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= COND_IDLE;
                end

                default: state <= COND_IDLE;
            endcase
        end
    end

endmodule
