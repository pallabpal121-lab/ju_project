// =============================================================================
// File Name   : pgd_gradient_engine.sv
// Module Name : pgd_gradient_engine
// Project     : Projected Gradient Descent (PGD) Accelerator (Solver #12)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Zero-cost bit-shift numerical gradient engine:
//     g_i = (f(x + h*e_i) - f(x)) / h
//   Using h = 2^-8 (Q16.16 = 32'h0000_0100), division by h is exact (diff <<< 8).
// =============================================================================

`timescale 1ns / 1ps

import pgd_types_pkg::*;
`include "pgd_helpers.svh"

module pgd_gradient_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Programming Interface (passed to internal DFG)
    input  logic               prog_en,
    input  logic [4:0]         prog_addr,
    input  instr_t             prog_data,

    // Evaluation Controls
    input  logic               start,
    input  logic [2:0]         num_params,
    input  vec_t               param_in,

    output vec_t               grad_out,
    output q16_t               f0_out,
    output logic               done,
    output logic               busy
);

    localparam q16_t H_STEP = 32'h0000_0100; // h = 2^-8

    typedef enum logic [2:0] {
        GRAD_IDLE      = 3'd0,
        GRAD_EVAL_F0   = 3'd1,
        GRAD_WAIT_F0   = 3'd2,
        GRAD_EVAL_PERT = 3'd3,
        GRAD_WAIT_PERT = 3'd4,
        GRAD_DONE      = 3'd5
    } grad_state_t;

    grad_state_t state;

    logic [1:0] cur_dim;
    q16_t       f0_val;
    vec_t       grad_accum;

    // DFG Submodule Interconnect
    logic         dfg_start;
    vec_t         dfg_param;
    q16_t         dfg_cost;
    logic         dfg_done, dfg_busy;

    dfg_pgd_engine u_dfg (
        .clk       (clk),
        .rst_n     (rst_n),
        .prog_en   (prog_en),
        .prog_addr (prog_addr),
        .prog_data (prog_data),
        .start_eval(dfg_start),
        .num_params(num_params),
        .x_vec     (dfg_param),
        .f_out     (dfg_cost),
        .eval_done (dfg_done),
        .busy      (dfg_busy)
    );

    q16_t pert_coord;
    vec_t pert_vec;
    q16_t diff_val;

    assign f0_out   = f0_val;
    assign grad_out = grad_accum;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= GRAD_IDLE;
            cur_dim    <= 2'd0;
            f0_val     <= Q16_ZERO;
            grad_accum <= '0;
            dfg_start  <= 1'b0;
            dfg_param  <= '0;
            done       <= 1'b0;
            busy       <= 1'b0;
        end else begin
            dfg_start <= 1'b0;

            case (state)
                GRAD_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy       <= 1'b1;
                        dfg_param  <= param_in;
                        dfg_start  <= 1'b1;
                        state      <= GRAD_WAIT_F0;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                GRAD_WAIT_F0: begin
                    if (dfg_done) begin
                        f0_val  <= dfg_cost;
                        cur_dim <= 2'd0;
                        state   <= GRAD_EVAL_PERT;
                    end
                end

                GRAD_EVAL_PERT: begin
                    if (cur_dim < num_params[1:0]) begin
                        pert_coord = get_vec(param_in, cur_dim) + H_STEP;
                        pert_vec   = set_vec(param_in, cur_dim, pert_coord);
                        dfg_param  <= pert_vec;
                        dfg_start  <= 1'b1;
                        state      <= GRAD_WAIT_PERT;
                    end else begin
                        state <= GRAD_DONE;
                    end
                end

                GRAD_WAIT_PERT: begin
                    if (dfg_done) begin
                        diff_val   = dfg_cost - f0_val;
                        grad_accum <= set_vec(grad_accum, cur_dim, q16_t'(diff_val <<< 8));
                        cur_dim    <= cur_dim + 1'b1;
                        state      <= GRAD_EVAL_PERT;
                    end
                end

                GRAD_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= GRAD_IDLE;
                end

                default: state <= GRAD_IDLE;
            endcase
        end
    end

endmodule
