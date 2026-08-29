// =============================================================================
// File Name   : lbfgs_top.sv
// Module Name : lbfgs_top
// Project     : Limited-Memory BFGS (L-BFGS) Hardware Accelerator (Solver #9)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Top-level SoC Controller for the Limited-Memory BFGS Accelerator.
//   Orchestrates numerical gradient sweeping, O(mN) circular history buffer
//   maintenance (s, y, rho), Two-Loop Quasi-Newton search direction evaluation,
//   directional curvature line search, and parameter updating.
// =============================================================================

`timescale 1ns / 1ps

import lbfgs_types_pkg::*;
`include "lbfgs_helpers.svh"

module lbfgs_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Model Microcode Programming Port
    // -------------------------------------------------------------------------
    input  logic               prog_en,       // Microcode Write Enable
    input  logic [4:0]         prog_addr,     // Microcode Address (0..31)
    input  instr_t             prog_data,     // 32-bit Microcode Instruction

    // -------------------------------------------------------------------------
    // Interface 2: Optimization Controls & Parameters
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start pulse
    input  logic [2:0]         num_params,    // Parameter Dimension N (1..4)
    input  vec_t               x_init,        // Initial Starting Guess x0
    input  q16_t               tolerance,     // Gradient infinity norm threshold ||g||_inf
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 3: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,     // Converged parameter vector x*
    output q16_t               f_optimal,     // Final objective function value f(x*)
    output q16_t               g_norm_inf,    // Final gradient infinity norm ||g||_inf
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while solving
);

    // -------------------------------------------------------------------------
    // Master FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        TOP_IDLE         = 4'd0,
        TOP_START_GRAD   = 4'd1,
        TOP_WAIT_GRAD    = 4'd2,
        TOP_CHECK_CONV   = 4'd3,
        TOP_DIV_RHO_WAIT = 4'd4,
        TOP_START_LOOP   = 4'd5,
        TOP_WAIT_LOOP    = 4'd6,
        TOP_LS_POS_START = 4'd7,
        TOP_LS_POS_WAIT  = 4'd8,
        TOP_LS_NEG_START = 4'd9,
        TOP_LS_NEG_WAIT  = 4'd10,
        TOP_LS_DIV_START = 4'd11,
        TOP_LS_DIV_WAIT  = 4'd12,
        TOP_UPDATE_STEP  = 4'd13,
        TOP_DONE         = 4'd14
    } top_state_t;

    top_state_t state;

    // Registers
    logic [2:0] num_params_reg;
    q16_t       tol_reg;
    logic [7:0] max_iters_reg;

    vec_t       x_cur, x_prev;
    vec_t       g_cur, g_prev;
    q16_t       f_cur;

    // Circular History Buffers
    history_vec_t    history_s;
    history_vec_t    history_y;
    history_scalar_t history_rho;
    logic [2:0]      hist_count;
    logic [1:0]      head_ptr;

    // Gradient Engine Interconnect
    logic grad_start;
    q16_t grad_f0;
    vec_t grad_g;
    logic grad_done, grad_busy;

    lbfgs_gradient_engine u_grad (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_grad (grad_start),
        .num_params (num_params_reg),
        .x_current  (x_cur),
        .f0_out     (grad_f0),
        .grad_out   (grad_g),
        .grad_done  (grad_done),
        .busy       (grad_busy)
    );

    // Two-Loop Engine Interconnect
    logic loop_start;
    vec_t loop_p;
    logic loop_done, loop_busy;

    lbfgs_two_loop_engine u_two_loop (
        .clk         (clk),
        .rst_n       (rst_n),
        .start_loop  (loop_start),
        .num_params  (num_params_reg),
        .grad_in     (g_cur),
        .history_s   (history_s),
        .history_y   (history_y),
        .history_rho (history_rho),
        .hist_count  (hist_count),
        .head_ptr    (head_ptr),
        .p_search_out(loop_p),
        .loop_done   (loop_done),
        .busy        (loop_busy)
    );

    // Dedicated Divider for rho and line search alpha
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

    // DFG Evaluator for Line Search
    logic dfg_ls_start;
    vec_t dfg_ls_x_in;
    q16_t dfg_ls_f_out;
    logic dfg_ls_done, dfg_ls_busy;

    dfg_lbfgs_engine u_dfg_ls (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (dfg_ls_start),
        .num_params (num_params_reg),
        .x_vec      (dfg_ls_x_in),
        .f_out      (dfg_ls_f_out),
        .eval_done  (dfg_ls_done),
        .busy       (dfg_ls_busy)
    );

    vec_t p_dir;
    vec_t s_disp, y_disp;
    q16_t f_pos, f_neg, kappa, alpha_step;
    q16_t neg_g_dot_p;
    vec_t tmp_ls_x;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= TOP_IDLE;
            num_params_reg <= 3'd2;
            tol_reg        <= Q16_EPS_DEF;
            max_iters_reg  <= 8'd30;
            iter_count     <= 8'd0;
            status         <= STATUS_IDLE;
            done           <= 1'b0;
            busy           <= 1'b0;
            x_optimal      <= '0;
            f_optimal      <= Q16_ZERO;
            g_norm_inf     <= Q16_ZERO;
            x_cur          <= '0;
            x_prev         <= '0;
            g_cur          <= '0;
            g_prev         <= '0;
            f_cur          <= Q16_ZERO;
            history_s      <= '0;
            history_y      <= '0;
            history_rho    <= '0;
            hist_count     <= 3'd0;
            head_ptr       <= 2'd0;
            p_dir          <= '0;
            s_disp         <= '0;
            y_disp         <= '0;
            f_pos          <= Q16_ZERO;
            f_neg          <= Q16_ZERO;
            kappa          <= Q16_ZERO;
            alpha_step     <= Q16_ONE;
            neg_g_dot_p    <= Q16_ZERO;
            grad_start     <= 1'b0;
            loop_start     <= 1'b0;
            div_start      <= 1'b0;
            div_dividend   <= Q16_ZERO;
            div_divisor    <= Q16_ZERO;
            dfg_ls_start   <= 1'b0;
            dfg_ls_x_in    <= '0;
        end else begin
            grad_start   <= 1'b0;
            loop_start   <= 1'b0;
            div_start    <= 1'b0;
            dfg_ls_start <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: TOP_IDLE
                // -------------------------------------------------------------
                TOP_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy           <= 1'b1;
                        num_params_reg <= (num_params != 3'd0)    ? num_params : 3'd2;
                        tol_reg        <= (tolerance != Q16_ZERO) ? tolerance  : Q16_EPS_DEF;
                        max_iters_reg  <= (max_iters != 8'd0)     ? max_iters  : 8'd30;
                        iter_count     <= 8'd0;
                        status         <= STATUS_RUNNING;
                        x_cur          <= x_init;
                        x_prev         <= x_init;
                        history_s      <= '0;
                        history_y      <= '0;
                        history_rho    <= '0;
                        hist_count     <= 3'd0;
                        head_ptr       <= 2'd0;
                        state          <= TOP_START_GRAD;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // 1. EVALUATE GRADIENT g_k = \nabla f(x_k)
                // -------------------------------------------------------------
                TOP_START_GRAD: begin
                    grad_start <= 1'b1;
                    state      <= TOP_WAIT_GRAD;
                end

                TOP_WAIT_GRAD: begin
                    if (grad_done) begin
                        f_cur <= grad_f0;
                        g_cur <= grad_g;
                        state <= TOP_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // 2. CHECK CONVERGENCE & UPDATE HISTORY BUFFER
                // -------------------------------------------------------------
                TOP_CHECK_CONV: begin
                    g_norm_inf <= q16_norm_inf(g_cur, num_params_reg);
                    f_optimal  <= f_cur;
                    x_optimal  <= x_cur;

                    // Condition 1: Converged if ||g||_inf <= tolerance
                    if (q16_norm_inf(g_cur, num_params_reg) <= tol_reg) begin
                        status <= STATUS_CONVERGED;
                        state  <= TOP_DONE;

                    // Condition 2: Max iterations reached
                    end else if (iter_count >= max_iters_reg) begin
                        status <= STATUS_MAX_ITERS;
                        state  <= TOP_DONE;

                    // Condition 3: Update history buffer (if iter_count > 0)
                    end else begin
                        if (iter_count > 8'd0) begin
                            // s = x_cur - x_prev, y = g_cur - g_prev
                            s_disp = '0;
                            y_disp = '0;
                            for (int k = 0; k < MAX_PARAMS; k++) begin
                                if (k < num_params_reg) begin
                                    s_disp = set_vec(s_disp, 2'(k), get_vec(x_cur, 2'(k)) - get_vec(x_prev, 2'(k)));
                                    y_disp = set_vec(y_disp, 2'(k), get_vec(g_cur, 2'(k)) - get_vec(g_prev, 2'(k)));
                                end
                            end

                            // rho = 1.0 / (y^T * s)
                            if (q16_dot(y_disp, s_disp, num_params_reg) > 32'sd10) begin // Strict positive curvature
                                div_dividend <= Q16_ONE;
                                div_divisor  <= q16_dot(y_disp, s_disp, num_params_reg);
                                div_start    <= 1'b1;
                                state        <= TOP_DIV_RHO_WAIT;
                            end else begin
                                // Non-positive curvature -> Skip storing into history
                                state <= TOP_START_LOOP;
                            end
                        end else begin
                            state <= TOP_START_LOOP;
                        end
                    end
                end

                TOP_DIV_RHO_WAIT: begin
                    if (div_done) begin
                        history_s   <= set_hist_vec(history_s, head_ptr, s_disp);
                        history_y   <= set_hist_vec(history_y, head_ptr, y_disp);
                        history_rho <= set_hist_scalar(history_rho, head_ptr, div_quotient);

                        head_ptr <= 2'((head_ptr + 1'b1) % MEM_DEPTH);
                        if (hist_count < 3'(MEM_DEPTH)) begin
                            hist_count <= hist_count + 1'b1;
                        end

                        state <= TOP_START_LOOP;
                    end
                end

                // -------------------------------------------------------------
                // 3. RUN TWO-LOOP RECURSION TO FIND p_search
                // -------------------------------------------------------------
                TOP_START_LOOP: begin
                    loop_start <= 1'b1;
                    state      <= TOP_WAIT_LOOP;
                end

                TOP_WAIT_LOOP: begin
                    if (loop_done) begin
                        p_dir <= loop_p;
                        state <= TOP_LS_POS_START;
                    end
                end

                // -------------------------------------------------------------
                // 4. DIRECTIONAL CURVATURE LINE SEARCH: kappa = p^T * H * p
                // -------------------------------------------------------------
                TOP_LS_POS_START: begin
                    // x_pos = x + h * p
                    tmp_ls_x = x_cur;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params_reg) begin
                            tmp_ls_x = set_vec(tmp_ls_x, 2'(k), get_vec(x_cur, 2'(k)) + q16_t'((64'(Q16_STEP_H) * 64'(get_vec(p_dir, 2'(k)))) >>> 16));
                        end
                    end
                    dfg_ls_x_in  <= tmp_ls_x;
                    dfg_ls_start <= 1'b1;
                    state        <= TOP_LS_POS_WAIT;
                end

                TOP_LS_POS_WAIT: begin
                    if (dfg_ls_done) begin
                        f_pos <= dfg_ls_f_out;
                        state <= TOP_LS_NEG_START;
                    end
                end

                TOP_LS_NEG_START: begin
                    // x_neg = x - h * p
                    tmp_ls_x = x_cur;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params_reg) begin
                            tmp_ls_x = set_vec(tmp_ls_x, 2'(k), get_vec(x_cur, 2'(k)) - q16_t'((64'(Q16_STEP_H) * 64'(get_vec(p_dir, 2'(k)))) >>> 16));
                        end
                    end
                    dfg_ls_x_in  <= tmp_ls_x;
                    dfg_ls_start <= 1'b1;
                    state        <= TOP_LS_NEG_WAIT;
                end

                TOP_LS_NEG_WAIT: begin
                    if (dfg_ls_done) begin
                        f_neg <= dfg_ls_f_out;
                        // kappa = (f+ - 2*f0 + f-) <<< 8
                        kappa <= (f_pos - (f_cur <<< 1) + dfg_ls_f_out) <<< 8;
                        state <= TOP_LS_DIV_START;
                    end
                end

                TOP_LS_DIV_START: begin
                    // neg_g_dot_p = -g^T * p
                    neg_g_dot_p = -q16_dot(g_cur, p_dir, num_params_reg);

                    if (kappa > 32'sd10) begin
                        div_dividend <= neg_g_dot_p;
                        div_divisor  <= kappa;
                        div_start    <= 1'b1;
                        state        <= TOP_LS_DIV_WAIT;
                    end else begin
                        alpha_step <= Q16_ONE; // Fallback step size = 1.0
                        state      <= TOP_UPDATE_STEP;
                    end
                end

                TOP_LS_DIV_WAIT: begin
                    if (div_done) begin
                        // Clamp step size in [0.05, 1.0]
                        if (div_quotient > Q16_ONE) begin
                            alpha_step <= Q16_ONE;
                        end else if (div_quotient < 32'h0000_0CCD) begin // 0.05
                            alpha_step <= 32'h0000_0CCD;
                        end else begin
                            alpha_step <= div_quotient;
                        end
                        state <= TOP_UPDATE_STEP;
                    end
                end

                // -------------------------------------------------------------
                // 5. UPDATE PARAMETER: x_{k+1} = x_k + alpha * p
                // -------------------------------------------------------------
                TOP_UPDATE_STEP: begin
                    x_prev <= x_cur;
                    g_prev <= g_cur;

                    tmp_ls_x = x_cur;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params_reg) begin
                            tmp_ls_x = set_vec(tmp_ls_x, 2'(k), get_vec(x_cur, 2'(k)) + q16_t'((64'(alpha_step) * 64'(get_vec(p_dir, 2'(k)))) >>> 16));
                        end
                    end
                    x_cur      <= tmp_ls_x;
                    iter_count <= iter_count + 1'b1;
                    state      <= TOP_START_GRAD;
                end

                // -------------------------------------------------------------
                // STATE: TOP_DONE
                // -------------------------------------------------------------
                TOP_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= TOP_IDLE;
                end

                default: state <= TOP_IDLE;
            endcase
        end
    end

endmodule
