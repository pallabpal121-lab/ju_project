// =============================================================================
// File Name   : lbfgs_two_loop_engine.sv
// Module Name : lbfgs_two_loop_engine
// Project     : Limited-Memory BFGS (L-BFGS) Hardware Accelerator (Solver #9)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Hardware implementation of the L-BFGS Two-Loop Recursion Algorithm.
//   Computes the Quasi-Newton search direction p = -H_k * g_k in O(mN) time
//   using only circular vector displacement history buffers:
//     1. Backward Loop : alpha_i = rho_i * (s_i^T * q),  q = q - alpha_i * y_i
//     2. Scaling       : gamma = (s_{k-1}^T * y_{k-1}) / (y_{k-1}^T * y_{k-1}), r = gamma * q
//     3. Forward Loop  : beta = rho_i * (y_i^T * r),      r = r + (alpha_i - beta) * s_i
//     4. Search Dir    : p = -r
// =============================================================================

`timescale 1ns / 1ps

import lbfgs_types_pkg::*;
`include "lbfgs_helpers.svh"

module lbfgs_two_loop_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Control & Inputs
    input  logic               start_loop,
    input  logic [2:0]         num_params,     // Dimension N (1..4)
    input  vec_t               grad_in,        // Current gradient g_k
    input  history_vec_t       history_s,      // Circular history of s_i
    input  history_vec_t       history_y,      // Circular history of y_i
    input  history_scalar_t    history_rho,    // Circular history of rho_i
    input  logic [2:0]         hist_count,     // Number of valid history entries (0..4)
    input  logic [1:0]         head_ptr,       // Circular buffer head pointer (0..3)

    // Outputs
    output vec_t               p_search_out,   // L-BFGS search direction p = -r
    output logic               loop_done,
    output logic               busy
);

    typedef enum logic [3:0] {
        LOOP_IDLE       = 4'd0,
        LOOP_BACK_DOT   = 4'd1,
        LOOP_BACK_SUB   = 4'd2,
        LOOP_SCALE_DIV  = 4'd3,
        LOOP_SCALE_WAIT = 4'd4,
        LOOP_SCALE_MUL  = 4'd5,
        LOOP_FWD_DOT    = 4'd6,
        LOOP_FWD_ADD    = 4'd7,
        LOOP_DONE       = 4'd8
    } loop_state_t;

    loop_state_t state;

    vec_t q_vec;
    vec_t r_vec;

    history_scalar_t alpha_store; // Holds alpha[0..3]
    logic signed [3:0] loop_i;    // Signed loop index (-1..4)

    // Divider for gamma = (s_last^T * y_last) / (y_last^T * y_last)
    logic               div_start;
    q16_t               div_dividend, div_divisor, div_quotient;
    logic               div_done, div_by_zero, div_busy;

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

    logic [1:0] buf_idx;
    logic [1:0] last_idx;
    assign last_idx = 2'((int'(head_ptr) - 1 + MEM_DEPTH) % MEM_DEPTH);

    q16_t dot_s_q, cur_alpha;
    q16_t dot_y_r, cur_beta, diff_alpha_beta;
    vec_t cur_s, cur_y;
    q16_t cur_rho;
    vec_t tmp_q, tmp_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= LOOP_IDLE;
            q_vec        <= '0;
            r_vec        <= '0;
            alpha_store  <= '0;
            loop_i       <= 4'sd0;
            p_search_out <= '0;
            loop_done    <= 1'b0;
            busy         <= 1'b0;
            div_start    <= 1'b0;
            div_dividend <= Q16_ZERO;
            div_divisor  <= Q16_ZERO;
        end else begin
            div_start <= 1'b0;

            case (state)
                LOOP_IDLE: begin
                    loop_done <= 1'b0;
                    if (start_loop) begin
                        busy <= 1'b1;
                        if (hist_count == 3'd0) begin
                            // No history -> Fallback to steepest descent p = -g
                            for (int k = 0; k < MAX_PARAMS; k++) begin
                                p_search_out = set_vec(p_search_out, 2'(k), -get_vec(grad_in, 2'(k)));
                            end
                            loop_done <= 1'b1;
                            busy      <= 1'b0;
                            state     <= LOOP_IDLE;
                        end else begin
                            q_vec  <= grad_in;
                            loop_i <= 4'sh0 + 4'(hist_count - 1'b1); // i = hist_count - 1
                            state  <= LOOP_BACK_DOT;
                        end
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // 1. FIRST LOOP (BACKWARD): i = hist_count - 1 down to 0
                // -------------------------------------------------------------
                LOOP_BACK_DOT: begin
                    if (loop_i >= 4'sd0) begin
                        // buf_idx = (head_ptr - 1 - loop_i + MEM_DEPTH) % MEM_DEPTH
                        buf_idx = 2'((int'(head_ptr) - 1 - int'(loop_i) + (2 * MEM_DEPTH)) % MEM_DEPTH);
                        cur_s   = get_hist_vec(history_s, buf_idx);
                        cur_rho = get_hist_scalar(history_rho, buf_idx);

                        // alpha_i = rho_idx * (s_idx^T * q)
                        dot_s_q   = q16_dot(cur_s, q_vec, num_params);
                        cur_alpha = q16_t'((64'(cur_rho) * 64'(dot_s_q)) >>> 16);

                        alpha_store <= set_hist_scalar(alpha_store, 2'(loop_i[1:0]), cur_alpha);
                        state       <= LOOP_BACK_SUB;
                    end else begin
                        // Backward loop completed -> Scale base direction
                        state <= LOOP_SCALE_DIV;
                    end
                end

                LOOP_BACK_SUB: begin
                    buf_idx = 2'((int'(head_ptr) - 1 - int'(loop_i) + (2 * MEM_DEPTH)) % MEM_DEPTH);
                    cur_y     = get_hist_vec(history_y, buf_idx);
                    cur_alpha = get_hist_scalar(alpha_store, 2'(loop_i[1:0]));

                    // q = q - alpha_i * y_idx
                    tmp_q = q_vec;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params) begin
                            tmp_q = set_vec(tmp_q, 2'(k), get_vec(tmp_q, 2'(k)) - q16_t'((64'(cur_alpha) * 64'(get_vec(cur_y, 2'(k)))) >>> 16));
                        end
                    end
                    q_vec  <= tmp_q;
                    loop_i <= loop_i - 1'b1;
                    state  <= LOOP_BACK_DOT;
                end

                // -------------------------------------------------------------
                // 2. BASE SCALING: gamma = (s_last^T * y_last) / (y_last^T * y_last)
                // -------------------------------------------------------------
                LOOP_SCALE_DIV: begin
                    cur_s = get_hist_vec(history_s, last_idx);
                    cur_y = get_hist_vec(history_y, last_idx);

                    div_dividend <= q16_dot(cur_s, cur_y, num_params); // s_last^T * y_last
                    div_divisor  <= q16_dot(cur_y, cur_y, num_params); // y_last^T * y_last
                    div_start    <= 1'b1;
                    state        <= LOOP_SCALE_WAIT;
                end

                LOOP_SCALE_WAIT: begin
                    if (div_done) begin
                        state <= LOOP_SCALE_MUL;
                    end
                end

                LOOP_SCALE_MUL: begin
                    // r = gamma * q
                    tmp_r = '0;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params) begin
                            if (div_by_zero || div_quotient <= Q16_ZERO) begin
                                tmp_r = set_vec(tmp_r, 2'(k), get_vec(q_vec, 2'(k))); // gamma = 1.0
                            end else begin
                                tmp_r = set_vec(tmp_r, 2'(k), q16_t'((64'(div_quotient) * 64'(get_vec(q_vec, 2'(k)))) >>> 16));
                            end
                        end
                    end
                    r_vec  <= tmp_r;
                    loop_i <= 4'sd0; // i = 0
                    state  <= LOOP_FWD_DOT;
                end

                // -------------------------------------------------------------
                // 3. SECOND LOOP (FORWARD): i = 0 up to hist_count - 1
                // -------------------------------------------------------------
                LOOP_FWD_DOT: begin
                    if (loop_i < 4'(hist_count)) begin
                        buf_idx = 2'((int'(head_ptr) - 1 - int'(loop_i) + (2 * MEM_DEPTH)) % MEM_DEPTH);
                        cur_y   = get_hist_vec(history_y, buf_idx);
                        cur_rho = get_hist_scalar(history_rho, buf_idx);

                        // beta = rho_idx * (y_idx^T * r)
                        dot_y_r  = q16_dot(cur_y, r_vec, num_params);
                        cur_beta = q16_t'((64'(cur_rho) * 64'(dot_y_r)) >>> 16);

                        cur_alpha       = get_hist_scalar(alpha_store, 2'(loop_i[1:0]));
                        diff_alpha_beta = cur_alpha - cur_beta;

                        // r = r + (alpha_i - beta) * s_idx
                        cur_s = get_hist_vec(history_s, buf_idx);
                        tmp_r = r_vec;
                        for (int k = 0; k < MAX_PARAMS; k++) begin
                            if (k < num_params) begin
                                tmp_r = set_vec(tmp_r, 2'(k), get_vec(tmp_r, 2'(k)) + q16_t'((64'(diff_alpha_beta) * 64'(get_vec(cur_s, 2'(k)))) >>> 16));
                            end
                        end
                        r_vec  <= tmp_r;
                        loop_i <= loop_i + 1'b1;
                        state  <= LOOP_FWD_DOT;
                    end else begin
                        state <= LOOP_DONE;
                    end
                end

                // -------------------------------------------------------------
                // 4. FINISHED: p = -r
                // -------------------------------------------------------------
                LOOP_DONE: begin
                    tmp_r = '0;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params) begin
                            tmp_r = set_vec(tmp_r, 2'(k), -get_vec(r_vec, 2'(k)));
                        end
                    end
                    p_search_out <= tmp_r;
                    loop_done    <= 1'b1;
                    busy         <= 1'b0;
                    state        <= LOOP_IDLE;
                end

                default: state <= LOOP_IDLE;
            endcase
        end
    end

endmodule
