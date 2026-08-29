// =============================================================================
// File Name   : cg_top.sv
// Module Name : cg_top
// Project     : Non-Linear Conjugate Gradient (CG) Accelerator (Solver #5)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Top-level SoC Module for the Non-Linear Conjugate Gradient (CG) Accelerator.
//   Executes Polak-Ribière conjugate direction updates with Powell restarts
//   and analytical directional curvature step sizing in O(N) vector memory.
// =============================================================================

`timescale 1ns / 1ps

import cg_types_pkg::*;
`include "cg_helpers.svh"

module cg_top (
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
    input  logic [2:0]         num_vars,      // Number of variables N (1..4)
    input  vec_t               x_init,        // Initial parameter guess [x3, x2, x1, x0]
    input  q16_t               tolerance,     // Convergence threshold on gradient infinity norm
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 3: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,     // Converged parameter vector x*
    output q16_t               f_optimal,     // Final function value f(x*)
    output q16_t               g_norm_inf,    // Final max gradient component max_j(|g_j|)
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while solving
);

    // -------------------------------------------------------------------------
    // Master FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        CG_IDLE             = 4'd0,
        CG_START_INIT_GRAD  = 4'd1,
        CG_WAIT_INIT_GRAD   = 4'd2,
        CG_CHECK_CONV       = 4'd3,
        CG_START_LS         = 4'd4,
        CG_WAIT_LS          = 4'd5,
        CG_START_NEW_GRAD   = 4'd6,
        CG_WAIT_NEW_GRAD    = 4'd7,
        CG_WAIT_BETA        = 4'd8,
        CG_UPDATE_DIR       = 4'd9,
        CG_DONE             = 4'd10
    } cg_state_t;

    cg_state_t state;

    // Vector Register File (Matrix-Free O(N) Storage!)
    vec_t       x_curr;
    vec_t       x_next;
    vec_t       g_curr;
    vec_t       g_next;
    vec_t       d_dir;
    q16_t       f_curr_val;
    q16_t       f_next_val;
    q16_t       alpha_val;
    q16_t       beta_val;
    logic [2:0] num_vars_reg;
    q16_t       tol_reg;
    logic [7:0] max_iters_reg;

    // Gradient Engine Interconnect
    logic grad_start;
    vec_t grad_x_in;
    q16_t grad_f_val;
    vec_t grad_vec_g;
    logic grad_done, grad_busy;

    // Line Search Engine Interconnect
    logic ls_start;
    q16_t ls_alpha_out;
    logic ls_done, ls_busy;

    // Beta Divider
    logic div_start;
    q16_t div_dividend, div_divisor, div_quotient;
    logic div_done, div_by_zero, div_busy;

    // Helper fixed-point multiplier
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Submodule 1: Gradient Vector Sweeper Engine
    cg_gradient_engine u_grad_engine (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (grad_start),
        .num_vars   (num_vars_reg),
        .x_curr     (grad_x_in),
        .f_val      (grad_f_val),
        .vec_g      (grad_vec_g),
        .done       (grad_done),
        .busy       (grad_busy)
    );

    // Submodule 2: Directional Curvature & Optimal Step Engine
    cg_line_search_engine u_ls_engine (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (ls_start),
        .num_vars   (num_vars_reg),
        .x_curr     (x_curr),
        .f_curr     (f_curr_val),
        .vec_g      (g_curr),
        .vec_d      (d_dir),
        .step_alpha (ls_alpha_out),
        .done       (ls_done),
        .busy       (ls_busy)
    );

    // Submodule 3: Polak-Ribière Beta Divider
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

    vec_t tmp_x_next;
    vec_t tmp_d_next;
    q16_t sum_num, sum_den;
    q16_t max_grad_calc;
    q16_t cur_g_val;

    // Master Conjugate Gradient Optimization FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= CG_IDLE;
            num_vars_reg  <= 3'd2;
            tol_reg       <= Q16_EPS_DEF;
            max_iters_reg <= 8'd50;
            iter_count    <= 8'd0;
            status        <= STATUS_IDLE;
            done          <= 1'b0;
            busy          <= 1'b0;
            f_optimal     <= Q16_ZERO;
            g_norm_inf    <= Q16_ZERO;
            x_optimal     <= '0;
            x_curr        <= '0;
            x_next        <= '0;
            g_curr        <= '0;
            g_next        <= '0;
            d_dir         <= '0;
            f_curr_val    <= Q16_ZERO;
            f_next_val    <= Q16_ZERO;
            alpha_val     <= Q16_ONE;
            beta_val      <= Q16_ZERO;
            grad_start    <= 1'b0;
            grad_x_in     <= '0;
            ls_start      <= 1'b0;
            div_start     <= 1'b0;
            div_dividend  <= Q16_ZERO;
            div_divisor   <= Q16_ZERO;
        end else begin
            grad_start <= 1'b0;
            ls_start   <= 1'b0;
            div_start  <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: CG_IDLE
                // -------------------------------------------------------------
                CG_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy          <= 1'b1;
                        num_vars_reg  <= (num_vars != 3'd0)      ? num_vars  : 3'd2;
                        tol_reg       <= (tolerance != Q16_ZERO) ? tolerance : Q16_EPS_DEF;
                        max_iters_reg <= (max_iters != 8'd0)     ? max_iters : 8'd50;
                        iter_count    <= 8'd0;
                        status        <= STATUS_RUNNING;
                        x_curr        <= x_init;
                        state         <= CG_START_INIT_GRAD;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE INITIAL GRADIENT g_0 = ∇f(x_0)
                // -------------------------------------------------------------
                CG_START_INIT_GRAD: begin
                    grad_x_in  <= x_curr;
                    grad_start <= 1'b1;
                    state      <= CG_WAIT_INIT_GRAD;
                end

                CG_WAIT_INIT_GRAD: begin
                    if (grad_done) begin
                        f_curr_val <= grad_f_val;
                        g_curr     <= grad_vec_g;
                        // Initial direction: d_0 = -g_0
                        tmp_d_next = '0;
                        for (int j = 0; j < MAX_VARS; j++) begin
                            if (j < num_vars_reg) begin
                                tmp_d_next = set_vec(tmp_d_next, 2'(j), -get_vec(grad_vec_g, 2'(j)));
                            end
                        end
                        d_dir <= tmp_d_next;
                        state <= CG_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // CHECK CONVERGENCE (||g||_inf <= tolerance or max iterations)
                // -------------------------------------------------------------
                CG_CHECK_CONV: begin
                    max_grad_calc = 32'sd0;
                    for (int j = 0; j < MAX_VARS; j++) begin
                        if (j < num_vars_reg) begin
                            cur_g_val = get_vec(g_curr, 2'(j));
                            if (cur_g_val < 32'sd0) cur_g_val = -cur_g_val;
                            if (cur_g_val > max_grad_calc) max_grad_calc = cur_g_val;
                        end
                    end

                    // Condition 1: Converged
                    if (max_grad_calc <= tol_reg) begin
                        status     <= STATUS_CONVERGED;
                        x_optimal  <= x_curr;
                        f_optimal  <= f_curr_val;
                        g_norm_inf <= max_grad_calc;
                        state      <= CG_DONE;

                    // Condition 2: Max iterations reached
                    end else if (iter_count >= max_iters_reg) begin
                        status     <= STATUS_MAX_ITERS;
                        x_optimal  <= x_curr;
                        f_optimal  <= f_curr_val;
                        g_norm_inf <= max_grad_calc;
                        state      <= CG_DONE;

                    // Condition 3: Launch Directional Curvature Step Engine
                    end else begin
                        ls_start <= 1'b1;
                        state    <= CG_WAIT_LS;
                    end
                end

                // -------------------------------------------------------------
                // WAIT FOR OPTIMAL STEP α_k
                // -------------------------------------------------------------
                CG_WAIT_LS: begin
                    if (ls_done) begin
                        alpha_val <= ls_alpha_out;

                        // Position Update: x_(k+1) = x_k + α · d_k
                        tmp_x_next = x_curr;
                        for (int j = 0; j < MAX_VARS; j++) begin
                            if (j < num_vars_reg) begin
                                tmp_x_next = set_vec(tmp_x_next, 2'(j), get_vec(x_curr, 2'(j)) + q16_mul(ls_alpha_out, get_vec(d_dir, 2'(j))));
                            end
                        end
                        x_next <= tmp_x_next;

                        // Trigger Gradient Engine at new position x_(k+1)
                        grad_x_in  <= tmp_x_next;
                        grad_start <= 1'b1;
                        state      <= CG_WAIT_NEW_GRAD;
                    end
                end

                // -------------------------------------------------------------
                // WAIT FOR NEW GRADIENT g_(k+1) = ∇f(x_(k+1))
                // -------------------------------------------------------------
                CG_WAIT_NEW_GRAD: begin
                    if (grad_done) begin
                        f_next_val <= grad_f_val;
                        g_next     <= grad_vec_g;

                        // Polak-Ribière: numerator n = g_(k+1)ᵀ (g_(k+1) - g_k), denominator d = g_kᵀ g_k
                        sum_num = 32'sd0;
                        sum_den = 32'sd0;
                        for (int j = 0; j < MAX_VARS; j++) begin
                            if (j < num_vars_reg) begin
                                sum_num = sum_num + q16_mul(get_vec(grad_vec_g, 2'(j)), get_vec(grad_vec_g, 2'(j)) - get_vec(g_curr, 2'(j)));
                                sum_den = sum_den + q16_mul(get_vec(g_curr, 2'(j)), get_vec(g_curr, 2'(j)));
                            end
                        end

                        // Powell Restart: if n <= 0 or (k+1 mod N == 0), reset β = 0
                        if (sum_num <= 32'sd0 || sum_den <= 32'h0000_0010 || ((iter_count + 1'b1) % num_vars_reg == 0)) begin
                            beta_val <= Q16_ZERO;
                            state    <= CG_UPDATE_DIR;
                        end else begin
                            div_dividend <= sum_num;
                            div_divisor  <= sum_den;
                            div_start    <= 1'b1;
                            state        <= CG_WAIT_BETA;
                        end
                    end
                end

                // -------------------------------------------------------------
                // WAIT FOR BETA DIVISION: β = n / d
                // -------------------------------------------------------------
                CG_WAIT_BETA: begin
                    if (div_done) begin
                        if (div_quotient <= 32'sd0) begin
                            beta_val <= Q16_ZERO;
                        end else begin
                            beta_val <= div_quotient;
                        end
                        state <= CG_UPDATE_DIR;
                    end
                end

                // -------------------------------------------------------------
                // UPDATE CONJUGATE DIRECTION: d_(k+1) = -g_(k+1) + β · d_k
                // -------------------------------------------------------------
                CG_UPDATE_DIR: begin
                    tmp_d_next = '0;
                    for (int j = 0; j < MAX_VARS; j++) begin
                        if (j < num_vars_reg) begin
                            tmp_d_next = set_vec(tmp_d_next, 2'(j), -get_vec(g_next, 2'(j)) + q16_mul(beta_val, get_vec(d_dir, 2'(j))));
                        end
                    end
                    d_dir      <= tmp_d_next;
                    x_curr     <= x_next;
                    g_curr     <= g_next;
                    f_curr_val <= f_next_val;
                    iter_count <= iter_count + 1'b1;
                    state      <= CG_CHECK_CONV;
                end

                // -------------------------------------------------------------
                // STATE: CG_DONE
                // -------------------------------------------------------------
                CG_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= CG_IDLE;
                end

                default: state <= CG_IDLE;
            endcase
        end
    end

endmodule
