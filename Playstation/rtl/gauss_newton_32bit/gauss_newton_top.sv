// =============================================================================
// File Name   : gauss_newton_top.sv
// Module Name : gauss_newton_top
// Project     : Gauss-Newton Non-Linear Least Squares Accelerator (Solver #6)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Top-level SoC Module for the Gauss-Newton Accelerator.
//   Solves Non-Linear Least Squares parameter estimation problems:
//     (Jᵀ · J + λ_eps · I) · p = -Jᵀ · r
//     x_(k+1) = x_k + α · p
// =============================================================================

`timescale 1ns / 1ps

import gn_types_pkg::*;
`include "gn_helpers.svh"

module gauss_newton_top (
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
    // Interface 2: Observation Dataset Memory Port
    // -------------------------------------------------------------------------
    input  logic               obs_we,        // Observation Write Enable
    input  logic [2:0]         obs_addr,      // Observation Table Index (0..7)
    input  q16_t               obs_t_in,      // Input data point t_m
    input  q16_t               obs_y_in,      // Measured output y_m

    // -------------------------------------------------------------------------
    // Interface 3: Optimization Controls & Parameters
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start pulse
    input  logic [2:0]         num_params,    // Number of parameters N (1..4)
    input  logic [3:0]         num_obs,       // Number of observations M (1..8)
    input  vec_t               x_init,        // Initial parameter guess [x3, x2, x1, x0]
    input  q16_t               step_alpha,    // Step size scale α
    input  q16_t               tolerance,     // Convergence threshold on gradient infinity norm
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 4: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,     // Converged parameter vector x*
    output q16_t               cost_optimal,  // Final least squares cost S(x*)
    output q16_t               g_norm_inf,    // Final max gradient component max_j(|g_j|)
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while solving
);

    // -------------------------------------------------------------------------
    // Master FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        GN_IDLE        = 3'd0,
        GN_START_EVAL  = 3'd1,
        GN_WAIT_EVAL   = 3'd2,
        GN_CHECK_CONV  = 3'd3,
        GN_START_SOLVE = 3'd4,
        GN_WAIT_SOLVE  = 3'd5,
        GN_UPDATE_X    = 3'd6,
        GN_DONE        = 3'd7
    } gn_state_t;

    gn_state_t state;

    // Internal Observation Memory (Packed 256-bit vectors)
    res_vec_t obs_t_reg;
    res_vec_t obs_y_reg;

    // Configuration & State Registers
    vec_t       x_reg;
    logic [2:0] num_params_reg;
    logic [3:0] num_obs_reg;
    q16_t       alpha_reg;
    q16_t       tol_reg;
    logic [7:0] max_iters_reg;

    // Jacobian Engine Interconnect
    logic eval_start;
    q16_t eval_cost_out;
    vec_t eval_vec_g;
    mat_t eval_mat_a;
    logic eval_done, eval_busy;

    // Cholesky Solver Interconnect
    logic chol_start;
    vec_t chol_vec_p;
    logic chol_done, chol_singular, chol_busy;

    // Helper fixed-point multiplier
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Submodule 1: Jacobian & Normal Matrix Accumulator Engine
    gn_jacobian_engine u_jac_engine (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (eval_start),
        .num_params (num_params_reg),
        .num_obs    (num_obs_reg),
        .x_curr     (x_reg),
        .obs_t_vec  (obs_t_reg),
        .obs_y_vec  (obs_y_reg),
        .cost_s     (eval_cost_out),
        .vec_g      (eval_vec_g),
        .mat_a      (eval_mat_a),
        .done       (eval_done),
        .busy       (eval_busy)
    );

    // Submodule 2: Hardware Cholesky Linear Solver: A · p = -g
    cholesky_solver_engine u_chol_solver (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (chol_start),
        .num_vars   (num_params_reg),
        .mat_a      (eval_mat_a),
        .vec_g      (eval_vec_g),
        .vec_p      (chol_vec_p),
        .done       (chol_done),
        .singular   (chol_singular),
        .busy       (chol_busy)
    );

    vec_t tmp_next_x;
    q16_t max_grad_calc;
    q16_t cur_g_val;

    // Master Gauss-Newton Optimization Loop FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= GN_IDLE;
            num_params_reg <= 3'd2;
            num_obs_reg    <= 4'd4;
            alpha_reg      <= Q16_ONE;
            tol_reg        <= Q16_EPS_DEF;
            max_iters_reg  <= 8'd50;
            iter_count     <= 8'd0;
            status         <= STATUS_IDLE;
            done           <= 1'b0;
            busy           <= 1'b0;
            cost_optimal   <= Q16_ZERO;
            g_norm_inf     <= Q16_ZERO;
            x_optimal      <= '0;
            x_reg          <= '0;
            obs_t_reg      <= '0;
            obs_y_reg      <= '0;
            eval_start     <= 1'b0;
            chol_start     <= 1'b0;
        end else begin
            eval_start <= 1'b0;
            chol_start <= 1'b0;

            // Latch observation writes
            if (obs_we) begin
                obs_t_reg <= set_res(obs_t_reg, obs_addr, obs_t_in);
                obs_y_reg <= set_res(obs_y_reg, obs_addr, obs_y_in);
            end

            case (state)
                // -------------------------------------------------------------
                // STATE: GN_IDLE
                // -------------------------------------------------------------
                GN_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy           <= 1'b1;
                        num_params_reg <= (num_params != 3'd0)     ? num_params : 3'd2;
                        num_obs_reg    <= (num_obs != 4'd0)        ? num_obs    : 4'd4;
                        alpha_reg      <= (step_alpha != Q16_ZERO) ? step_alpha : Q16_ONE;
                        tol_reg        <= (tolerance != Q16_ZERO)  ? tolerance  : Q16_EPS_DEF;
                        max_iters_reg  <= (max_iters != 8'd0)      ? max_iters  : 8'd50;
                        iter_count     <= 8'd0;
                        status         <= STATUS_RUNNING;
                        x_reg          <= x_init;
                        state          <= GN_START_EVAL;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE RESIDUALS, JACOBIAN, JᵀJ, AND GRADIENT Jᵀr
                // -------------------------------------------------------------
                GN_START_EVAL: begin
                    eval_start <= 1'b1;
                    state      <= GN_WAIT_EVAL;
                end

                GN_WAIT_EVAL: begin
                    if (eval_done) begin
                        state <= GN_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // CHECK CONVERGENCE (||g||_inf <= tolerance or max iterations)
                // -------------------------------------------------------------
                GN_CHECK_CONV: begin
                    max_grad_calc = 32'sd0;
                    for (int j = 0; j < MAX_PARAMS; j++) begin
                        if (j < num_params_reg) begin
                            cur_g_val = get_vec(eval_vec_g, 2'(j));
                            if (cur_g_val < 32'sd0) cur_g_val = -cur_g_val;
                            if (cur_g_val > max_grad_calc) max_grad_calc = cur_g_val;
                        end
                    end

                    // Condition 1: Gradient converged
                    if (max_grad_calc <= tol_reg) begin
                        status       <= STATUS_CONVERGED;
                        x_optimal    <= x_reg;
                        cost_optimal <= eval_cost_out;
                        g_norm_inf   <= max_grad_calc;
                        state        <= GN_DONE;

                    // Condition 2: Max iterations reached
                    end else if (iter_count >= max_iters_reg) begin
                        status       <= STATUS_MAX_ITERS;
                        x_optimal    <= x_reg;
                        cost_optimal <= eval_cost_out;
                        g_norm_inf   <= max_grad_calc;
                        state        <= GN_DONE;

                    // Condition 3: Solve (JᵀJ + λ_eps·I) p = -Jᵀr via Cholesky
                    end else begin
                        chol_start <= 1'b1;
                        state      <= GN_WAIT_SOLVE;
                    end
                end

                // -------------------------------------------------------------
                // WAIT FOR CHOLESKY LINEAR SOLVER
                // -------------------------------------------------------------
                GN_WAIT_SOLVE: begin
                    if (chol_done) begin
                        if (chol_singular) begin
                            status       <= STATUS_SINGULAR;
                            x_optimal    <= x_reg;
                            cost_optimal <= eval_cost_out;
                            g_norm_inf   <= max_grad_calc;
                            state        <= GN_DONE;
                        end else begin
                            state <= GN_UPDATE_X;
                        end
                    end
                end

                // -------------------------------------------------------------
                // UPDATE PARAMETER VECTOR: x = x + α · p
                // -------------------------------------------------------------
                GN_UPDATE_X: begin
                    tmp_next_x = x_reg;
                    for (int j = 0; j < MAX_PARAMS; j++) begin
                        if (j < num_params_reg) begin
                            tmp_next_x = set_vec(tmp_next_x, 2'(j), get_vec(x_reg, 2'(j)) + q16_mul(alpha_reg, get_vec(chol_vec_p, 2'(j))));
                        end
                    end
                    x_reg      <= tmp_next_x;
                    iter_count <= iter_count + 1'b1;
                    state      <= GN_START_EVAL;
                end

                // -------------------------------------------------------------
                // STATE: GN_DONE
                // -------------------------------------------------------------
                GN_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= GN_IDLE;
                end

                default: state <= GN_IDLE;
            endcase
        end
    end

endmodule
