// =============================================================================
// File Name   : adam_gradient_engine.sv
// Module Name : adam_gradient_engine
// Project     : Adaptive Moment Estimation (Adam) Accelerator (Solver #17)
// -----------------------------------------------------------------------------
// Description:
//   Evaluates the numerical gradient g = ∇f(θ) at current parameters θ
//   using central finite differences:
//   g_i = (f(θ + h*e_i) - f(θ - h*e_i)) / (2*h)
//   With h = 2^-4 = 0.0625, 1/(2h) = 2^3 = 8 (single-cycle arithmetic left shift <<< 3).
//   Also returns base loss f(θ).
// =============================================================================

`timescale 1ns / 1ps

import adam_types_pkg::*;
`include "adam_helpers.svh"

module adam_gradient_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Programming Interface (for sub-DFG engine)
    input  logic               prog_en,
    input  logic [4:0]         prog_addr,
    input  instr_t             prog_data,

    // Control & Inputs
    input  logic               start,
    input  logic [2:0]         num_dims,      // Number of parameters N (1..4)
    input  vec_t               theta_in,      // Current parameter vector θ

    // Outputs
    output vec_t               vec_g_out,     // Gradient vector g = ∇f(θ)
    output q16_t               f_base_out,    // Loss f(θ)
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        G_IDLE       = 3'd0,
        G_START_BASE = 3'd1,
        G_WAIT_BASE  = 3'd2,
        G_START_POS  = 3'd3,
        G_WAIT_POS   = 3'd4,
        G_START_NEG  = 3'd5,
        G_WAIT_NEG   = 3'd6,
        G_DONE       = 3'd7
    } g_state_t;

    g_state_t state;

    logic [2:0] curr_dim;
    vec_t       g_acc;
    q16_t       f_base_reg;
    q16_t       f_pos_reg;
    q16_t       f_diff;

    // Sub-DFG Instance
    logic start_dfg;
    vec_t dfg_x_in;
    q16_t dfg_f_out;
    logic dfg_done, dfg_busy;

    dfg_adam_engine u_dfg (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_dfg),
        .num_dims   (num_dims),
        .x_vec      (dfg_x_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    assign vec_g_out  = g_acc;
    assign f_base_out = f_base_reg;

    q16_t orig_val;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= G_IDLE;
            curr_dim   <= 3'd0;
            g_acc      <= '0;
            f_base_reg <= Q16_ZERO;
            f_pos_reg  <= Q16_ZERO;
            start_dfg  <= 1'b0;
            dfg_x_in   <= '0;
            done       <= 1'b0;
            busy       <= 1'b0;
        end else begin
            start_dfg <= 1'b0;

            case (state)
                G_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy     <= 1'b1;
                        curr_dim <= 3'd0;
                        g_acc    <= '0;
                        state    <= G_START_BASE;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // 1. Evaluate baseline loss f(θ)
                G_START_BASE: begin
                    dfg_x_in  <= theta_in;
                    start_dfg <= 1'b1;
                    state     <= G_WAIT_BASE;
                end

                G_WAIT_BASE: begin
                    if (dfg_done) begin
                        f_base_reg <= dfg_f_out;
                        curr_dim   <= 3'd0;
                        state      <= G_START_POS;
                    end
                end

                // 2. Evaluate f(θ + h*e_i)
                G_START_POS: begin
                    orig_val = get_vec(theta_in, 2'(curr_dim));
                    dfg_x_in <= set_vec(theta_in, 2'(curr_dim), orig_val + Q16_H_STEP);
                    start_dfg <= 1'b1;
                    state     <= G_WAIT_POS;
                end

                G_WAIT_POS: begin
                    if (dfg_done) begin
                        f_pos_reg <= dfg_f_out;
                        state     <= G_START_NEG;
                    end
                end

                // 3. Evaluate f(θ - h*e_i)
                G_START_NEG: begin
                    orig_val = get_vec(theta_in, 2'(curr_dim));
                    dfg_x_in <= set_vec(theta_in, 2'(curr_dim), orig_val - Q16_H_STEP);
                    start_dfg <= 1'b1;
                    state     <= G_WAIT_NEG;
                end

                G_WAIT_NEG: begin
                    if (dfg_done) begin
                        // g_i = (f_pos - f_neg) / (2 * h) = (f_pos - f_neg) <<< 3
                        f_diff = f_pos_reg - dfg_f_out;
                        g_acc  <= set_vec(g_acc, 2'(curr_dim), f_diff <<< (H_SHIFT - 1));

                        if (curr_dim + 1'b1 < num_dims) begin
                            curr_dim <= curr_dim + 1'b1;
                            state    <= G_START_POS;
                        end else begin
                            state <= G_DONE;
                        end
                    end
                end

                G_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= G_IDLE;
                end

                default: state <= G_IDLE;
            endcase
        end
    end

endmodule
