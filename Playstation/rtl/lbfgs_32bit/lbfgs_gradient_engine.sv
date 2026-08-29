// =============================================================================
// File Name   : lbfgs_gradient_engine.sv
// Module Name : lbfgs_gradient_engine
// Project     : Limited-Memory BFGS (L-BFGS) Hardware Accelerator (Solver #9)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Finite-difference numerical gradient sweeper for L-BFGS optimization.
//   Evaluates baseline cost f0 = f(x) and sweeps all N partial derivatives:
//     g_j = (f(x + h*e_j) - f(x - h*e_j)) / (2*h)
//   Using h = 2^-4 = 0.0625, division by 2h is a single-cycle shift: (f+ - f-) <<< 3.
// =============================================================================

`timescale 1ns / 1ps

import lbfgs_types_pkg::*;
`include "lbfgs_helpers.svh"

module lbfgs_gradient_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Programming Interface (for DFG evaluator)
    input  logic               prog_en,
    input  logic [4:0]         prog_addr,
    input  instr_t             prog_data,

    // Control & Inputs
    input  logic               start_grad,
    input  logic [2:0]         num_params,
    input  vec_t               x_current,

    // Outputs
    output q16_t               f0_out,        // Baseline cost f(x)
    output vec_t               grad_out,      // Gradient vector g = \nabla f(x)
    output logic               grad_done,
    output logic               busy
);

    typedef enum logic [2:0] {
        GRAD_IDLE       = 3'd0,
        GRAD_BASE_START = 3'd1,
        GRAD_BASE_WAIT  = 3'd2,
        GRAD_POS_START  = 3'd3,
        GRAD_POS_WAIT   = 3'd4,
        GRAD_NEG_START  = 3'd5,
        GRAD_NEG_WAIT   = 3'd6,
        GRAD_DONE       = 3'd7
    } grad_state_t;

    grad_state_t state;

    logic [1:0] param_idx;
    q16_t       f_pos_val, f_neg_val;
    vec_t       g_accum;

    // DFG Interconnect
    logic start_dfg;
    vec_t dfg_x_in;
    q16_t dfg_f_out;
    logic dfg_done, dfg_busy;

    dfg_lbfgs_engine u_dfg (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_dfg),
        .num_params (num_params),
        .x_vec      (dfg_x_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    vec_t perturbed_vec;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= GRAD_IDLE;
            param_idx   <= 2'd0;
            f0_out      <= Q16_ZERO;
            f_pos_val   <= Q16_ZERO;
            f_neg_val   <= Q16_ZERO;
            g_accum     <= '0;
            grad_out    <= '0;
            grad_done   <= 1'b0;
            busy        <= 1'b0;
            start_dfg   <= 1'b0;
            dfg_x_in    <= '0;
        end else begin
            start_dfg <= 1'b0;

            case (state)
                GRAD_IDLE: begin
                    grad_done <= 1'b0;
                    if (start_grad) begin
                        busy      <= 1'b1;
                        param_idx <= 2'd0;
                        g_accum   <= '0;
                        state     <= GRAD_BASE_START;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // Step 1: Evaluate Baseline f0 = f(x)
                // -------------------------------------------------------------
                GRAD_BASE_START: begin
                    dfg_x_in  <= x_current;
                    start_dfg <= 1'b1;
                    state     <= GRAD_BASE_WAIT;
                end

                GRAD_BASE_WAIT: begin
                    if (dfg_done) begin
                        f0_out    <= dfg_f_out;
                        param_idx <= 2'd0;
                        state     <= GRAD_POS_START;
                    end
                end

                // -------------------------------------------------------------
                // Step 2: Positive Perturbation f(x + h*e_j)
                // -------------------------------------------------------------
                GRAD_POS_START: begin
                    perturbed_vec = x_current;
                    perturbed_vec = set_vec(perturbed_vec, param_idx, get_vec(x_current, param_idx) + Q16_STEP_H);
                    dfg_x_in      <= perturbed_vec;
                    start_dfg     <= 1'b1;
                    state         <= GRAD_POS_WAIT;
                end

                GRAD_POS_WAIT: begin
                    if (dfg_done) begin
                        f_pos_val <= dfg_f_out;
                        state     <= GRAD_NEG_START;
                    end
                end

                // -------------------------------------------------------------
                // Step 3: Negative Perturbation f(x - h*e_j)
                // -------------------------------------------------------------
                GRAD_NEG_START: begin
                    perturbed_vec = x_current;
                    perturbed_vec = set_vec(perturbed_vec, param_idx, get_vec(x_current, param_idx) - Q16_STEP_H);
                    dfg_x_in      <= perturbed_vec;
                    start_dfg     <= 1'b1;
                    state         <= GRAD_NEG_WAIT;
                end

                GRAD_NEG_WAIT: begin
                    if (dfg_done) begin
                        f_neg_val <= dfg_f_out;
                        // g_j = (f+ - f-) <<< 3
                        g_accum   <= set_vec(g_accum, param_idx, (f_pos_val - dfg_f_out) <<< 3);

                        if (param_idx + 1'b1 < num_params) begin
                            param_idx <= param_idx + 1'b1;
                            state     <= GRAD_POS_START;
                        end else begin
                            state <= GRAD_DONE;
                        end
                    end
                end

                // -------------------------------------------------------------
                // Step 4: Finished Gradient Evaluation
                // -------------------------------------------------------------
                GRAD_DONE: begin
                    grad_out  <= g_accum;
                    grad_done <= 1'b1;
                    busy      <= 1'b0;
                    state     <= GRAD_IDLE;
                end

                default: state <= GRAD_IDLE;
            endcase
        end
    end

endmodule
