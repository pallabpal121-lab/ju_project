// =============================================================================
// File Name   : newton_multivar_top.sv
// Module Name : newton_multivar_top
// Project     : Universal Multivariable Newton 2nd-Order Accelerator
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Top-level SoC Module for the N-Dimensional Newton Optimization Accelerator.
// =============================================================================

`timescale 1ns / 1ps

import newton_multivar_pkg::*;
`include "multivar_helpers.svh"

module newton_multivar_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Equation Microcode Programming Port
    // -------------------------------------------------------------------------
    input  logic               prog_en,       // Microcode Write Enable
    input  logic [4:0]         prog_addr,     // Microcode Memory Address (0..31)
    input  instr_t             prog_data,     // 32-bit Microcode Instruction Word

    // -------------------------------------------------------------------------
    // Interface 2: Optimization Parameters & Controls
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start pulse
    input  logic [2:0]         num_vars,      // Number of active variables (1..4)
    input  vec_t               x_init,        // Initial guess vector [x3, x2, x1, x0]
    input  q16_t               tolerance,     // Convergence threshold (infinity-norm of g)
    input  q16_t               step_alpha,    // Step size / learning rate α
    input  q16_t               lambda_reg,    // Damping factor λ
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 3: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,     // Converged optimal vector x*
    output q16_t               f_optimal,     // Final function value f(x*)
    output q16_t               g_norm_inf,    // Final max gradient component max_i(|g_i|)
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion pulse
    output logic               busy           // High while solver is running
);

    // -------------------------------------------------------------------------
    // Master FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        TOP_IDLE        = 3'd0,
        TOP_START_DERIV = 3'd1,
        TOP_WAIT_DERIV  = 3'd2,
        TOP_CHECK_CONV  = 3'd3,
        TOP_START_SOLVE = 3'd4,
        TOP_WAIT_SOLVE  = 3'd5,
        TOP_UPDATE_X    = 3'd6,
        TOP_DONE        = 3'd7
    } top_state_t;

    top_state_t state;

    // Configuration & State Registers
    vec_t       x_reg;
    logic [2:0] num_vars_reg;
    q16_t       tol_reg;
    q16_t       alpha_reg;
    q16_t       lambda_reg_in;
    logic [7:0] max_iters_reg;

    // Derivative Engine Interconnect
    logic deriv_start;
    q16_t deriv_f_val;
    vec_t deriv_vec_g;
    mat_t deriv_mat_a;
    logic deriv_done;
    logic deriv_busy;

    // Cholesky Solver Interconnect
    logic chol_start;
    vec_t chol_vec_p;
    logic chol_done;
    logic chol_singular;
    logic chol_busy;

    // Helper fixed-point multiplier
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Submodule 1: Multivariable Derivative & Curvature Engine
    multivar_derivative_engine u_deriv_engine (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (deriv_start),
        .num_vars   (num_vars_reg),
        .x_curr     (x_reg),
        .lambda_reg (lambda_reg_in),
        .f_val      (deriv_f_val),
        .vec_g      (deriv_vec_g),
        .mat_a      (deriv_mat_a),
        .done       (deriv_done),
        .busy       (deriv_busy)
    );

    // Submodule 2: Hardware Cholesky Linear System Solver
    cholesky_solver_engine u_chol_solver (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (chol_start),
        .num_vars   (num_vars_reg),
        .mat_a      (deriv_mat_a),
        .vec_g      (deriv_vec_g),
        .vec_p      (chol_vec_p),
        .done       (chol_done),
        .singular   (chol_singular),
        .busy       (chol_busy)
    );

    vec_t tmp_next_x;
    q16_t max_grad_calc;
    q16_t cur_g_val;

    // Master Optimization Loop FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= TOP_IDLE;
            num_vars_reg  <= 3'd2;
            tol_reg       <= Q16_EPS_DEF;
            alpha_reg     <= Q16_ONE;
            lambda_reg_in <= Q16_LAMBDA_DEF;
            max_iters_reg <= 8'd50;
            iter_count    <= 8'd0;
            status        <= STATUS_IDLE;
            done          <= 1'b0;
            busy          <= 1'b0;
            f_optimal     <= Q16_ZERO;
            g_norm_inf    <= Q16_ZERO;
            deriv_start   <= 1'b0;
            chol_start    <= 1'b0;
            x_reg         <= '0;
            x_optimal     <= '0;
        end else begin
            deriv_start <= 1'b0;
            chol_start  <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: TOP_IDLE
                // -------------------------------------------------------------
                TOP_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy          <= 1'b1;
                        num_vars_reg  <= (num_vars != 3'd0) ? num_vars : 3'd2;
                        tol_reg       <= (tolerance != Q16_ZERO)  ? tolerance  : Q16_EPS_DEF;
                        alpha_reg     <= (step_alpha != Q16_ZERO) ? step_alpha : Q16_ONE;
                        lambda_reg_in <= (lambda_reg != Q16_ZERO) ? lambda_reg : Q16_LAMBDA_DEF;
                        max_iters_reg <= (max_iters != 8'd0)      ? max_iters  : 8'd50;
                        iter_count    <= 8'd0;
                        status        <= STATUS_RUNNING;
                        x_reg         <= x_init;
                        state         <= TOP_START_DERIV;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE: TOP_START_DERIV
                // -------------------------------------------------------------
                TOP_START_DERIV: begin
                    deriv_start <= 1'b1;
                    state       <= TOP_WAIT_DERIV;
                end

                // -------------------------------------------------------------
                // STATE: TOP_WAIT_DERIV
                // -------------------------------------------------------------
                TOP_WAIT_DERIV: begin
                    if (deriv_done) begin
                        state <= TOP_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // STATE: TOP_CHECK_CONV
                // -------------------------------------------------------------
                TOP_CHECK_CONV: begin
                    max_grad_calc = 32'sd0;
                    for (int i = 0; i < MAX_VARS; i++) begin
                        if (i < num_vars_reg) begin
                            cur_g_val = get_vec(deriv_vec_g, 2'(i));
                            if (cur_g_val < 32'sd0) cur_g_val = -cur_g_val;
                            if (cur_g_val > max_grad_calc) max_grad_calc = cur_g_val;
                        end
                    end

                    // Condition 1: Converged (max |g_i| <= tolerance)
                    if (max_grad_calc <= tol_reg) begin
                        status     <= STATUS_CONVERGED;
                        x_optimal  <= x_reg;
                        f_optimal  <= deriv_f_val;
                        g_norm_inf <= max_grad_calc;
                        state      <= TOP_DONE;

                    // Condition 2: Maximum iterations reached
                    end else if (iter_count >= max_iters_reg) begin
                        status     <= STATUS_MAX_ITERS;
                        x_optimal  <= x_reg;
                        f_optimal  <= deriv_f_val;
                        g_norm_inf <= max_grad_calc;
                        state      <= TOP_DONE;

                    // Condition 3: Continue -> solve (H + lambda*I) · p = -g
                    end else begin
                        chol_start <= 1'b1;
                        state      <= TOP_WAIT_SOLVE;
                    end
                end

                // -------------------------------------------------------------
                // STATE: TOP_WAIT_SOLVE
                // -------------------------------------------------------------
                TOP_WAIT_SOLVE: begin
                    if (chol_done) begin
                        if (chol_singular) begin
                            status     <= STATUS_SINGULAR;
                            x_optimal  <= x_reg;
                            f_optimal  <= deriv_f_val;
                            g_norm_inf <= max_grad_calc;
                            state      <= TOP_DONE;
                        end else begin
                            state <= TOP_UPDATE_X;
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE: TOP_UPDATE_X (Parallel Vector Update: x = x + α·p)
                // -------------------------------------------------------------
                TOP_UPDATE_X: begin
                    tmp_next_x = x_reg;
                    for (int i = 0; i < MAX_VARS; i++) begin
                        if (i < num_vars_reg) begin
                            tmp_next_x = set_vec(tmp_next_x, 2'(i), get_vec(x_reg, 2'(i)) + q16_mul(alpha_reg, get_vec(chol_vec_p, 2'(i))));
                        end
                    end
                    x_reg      <= tmp_next_x;
                    iter_count <= iter_count + 1'b1;
                    state      <= TOP_START_DERIV;
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
