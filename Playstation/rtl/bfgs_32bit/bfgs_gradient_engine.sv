// =============================================================================
// File Name   : bfgs_gradient_engine.sv
// Module Name : bfgs_gradient_engine
// Project     : Quasi-Newton BFGS Optimization Accelerator (Solver #4)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Evaluates the scalar function value f(x) and the N-dimensional Gradient
//   vector g = ∇f(x) using finite differences with step h = 2^-4 (shift <<< 3).
//   Requires only 2N + 1 function evaluations (no 2nd derivatives needed!).
// =============================================================================

`timescale 1ns / 1ps

import bfgs_types_pkg::*;
`include "bfgs_helpers.svh"

module bfgs_gradient_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Programming Interface
    input  logic               prog_en,
    input  logic [4:0]         prog_addr,
    input  instr_t             prog_data,

    // Control & Inputs
    input  logic               start,
    input  logic [2:0]         num_vars,    // Active dimension N (1..4)
    input  vec_t               x_curr,      // Position vector x

    // Outputs
    output q16_t               f_val,       // Function value f(x)
    output vec_t               vec_g,       // Gradient vector g = ∇f(x)
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        G_IDLE      = 3'd0,
        G_START_F0  = 3'd1,
        G_WAIT_F0   = 3'd2,
        G_START_FP  = 3'd3,
        G_WAIT_FP   = 3'd4,
        G_START_FM  = 3'd5,
        G_WAIT_FM   = 3'd6,
        G_CALC_ELEM = 3'd7
    } g_state_t;

    g_state_t state;

    // DFG Submodule Interconnect
    logic start_dfg;
    vec_t dfg_x_in;
    q16_t dfg_f_out;
    logic dfg_done, dfg_busy;

    logic [1:0] i_idx;
    q16_t f_0, f_plus, f_minus;
    vec_t g_reg;

    assign vec_g = g_reg;

    // Instantiate DFG Engine
    dfg_bfgs_engine u_dfg (
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

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= G_IDLE;
            start_dfg <= 1'b0;
            dfg_x_in  <= '0;
            f_val     <= Q16_ZERO;
            f_0       <= Q16_ZERO;
            f_plus    <= Q16_ZERO;
            f_minus   <= Q16_ZERO;
            i_idx     <= '0;
            g_reg     <= '0;
            done      <= 1'b0;
            busy      <= 1'b0;
        end else begin
            start_dfg <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: G_IDLE
                // -------------------------------------------------------------
                G_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= G_START_F0;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE CENTER: f(x)
                // -------------------------------------------------------------
                G_START_F0: begin
                    dfg_x_in  <= x_curr;
                    start_dfg <= 1'b1;
                    state     <= G_WAIT_F0;
                end

                G_WAIT_F0: begin
                    if (dfg_done) begin
                        f_0   <= dfg_f_out;
                        f_val <= dfg_f_out;
                        i_idx <= 2'd0;
                        state <= G_START_FP;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE FORWARD PERTURBATION: f(x + h*e_i)
                // -------------------------------------------------------------
                G_START_FP: begin
                    dfg_x_in  <= set_vec(x_curr, i_idx, get_vec(x_curr, i_idx) + Q16_H_STEP);
                    start_dfg <= 1'b1;
                    state     <= G_WAIT_FP;
                end

                G_WAIT_FP: begin
                    if (dfg_done) begin
                        f_plus <= dfg_f_out;
                        state  <= G_START_FM;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE BACKWARD PERTURBATION: f(x - h*e_i)
                // -------------------------------------------------------------
                G_START_FM: begin
                    dfg_x_in  <= set_vec(x_curr, i_idx, get_vec(x_curr, i_idx) - Q16_H_STEP);
                    start_dfg <= 1'b1;
                    state     <= G_WAIT_FM;
                end

                G_WAIT_FM: begin
                    if (dfg_done) begin
                        f_minus <= dfg_f_out;
                        state   <= G_CALC_ELEM;
                    end
                end

                // -------------------------------------------------------------
                // CALCULATE GRADIENT COMPONENT: g[i] = (f_+ - f_-) / (2h)
                // -------------------------------------------------------------
                G_CALC_ELEM: begin
                    g_reg <= set_vec(g_reg, i_idx, (f_plus - f_minus) <<< 3);

                    if (i_idx + 1'b1 < num_vars[1:0]) begin
                        i_idx <= i_idx + 1'b1;
                        state <= G_START_FP;
                    end else begin
                        done  <= 1'b1;
                        busy  <= 1'b0;
                        state <= G_IDLE;
                    end
                end

                default: state <= G_IDLE;
            endcase
        end
    end

endmodule
