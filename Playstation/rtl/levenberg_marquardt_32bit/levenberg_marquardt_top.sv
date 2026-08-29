// =============================================================================
// File Name   : levenberg_marquardt_top.sv
// Module Name : levenberg_marquardt_top
// Project     : Levenberg-Marquardt (LM) Non-Linear Least Squares Accelerator (Solver #2)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Top-level SoC Module for the Levenberg-Marquardt Non-Linear Least Squares
//   Optimization Accelerator. Features an adaptive Marquardt damping controller
//   that dynamically tunes λ between gradient descent and Gauss-Newton.
// =============================================================================

`timescale 1ns / 1ps

import lm_types_pkg::*;
`include "lm_helpers.svh"

module levenberg_marquardt_top (
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
    input  q16_t               tolerance,     // Convergence threshold on gradient infinity norm
    input  q16_t               lambda_init,   // Initial damping factor λ_0
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 4: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,     // Converged parameter vector x*
    output q16_t               cost_optimal,  // Final least-squares cost S(x*)
    output q16_t               g_norm_inf,    // Final max gradient component max_j(|g_j|)
    output q16_t               lambda_final,  // Final adaptive damping factor λ
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while solving
);

    // -------------------------------------------------------------------------
    // Master FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        LM_IDLE         = 4'd0,
        LM_START_BASE   = 4'd1,
        LM_WAIT_BASE    = 4'd2,
        LM_CHECK_CONV   = 4'd3,
        LM_START_SOLVE  = 4'd4,
        LM_WAIT_SOLVE   = 4'd5,
        LM_START_TRIAL  = 4'd6,
        LM_WAIT_TRIAL   = 4'd7,
        LM_ADAPT_LAMBDA = 4'd8,
        LM_DONE         = 4'd9
    } lm_state_t;

    lm_state_t state;

    // Packed Observation Dataset Registers
    res_vec_t obs_t_reg;
    res_vec_t obs_y_reg;

    // Configuration & State Registers
    vec_t       x_reg;
    vec_t       x_trial_reg;
    logic [2:0] num_params_reg;
    logic [3:0] num_obs_reg;
    q16_t       tol_reg;
    q16_t       lambda_curr;
    logic [7:0] max_iters_reg;

    q16_t       cost_base;
    q16_t       cost_trial;

    // Jacobian Engine Interconnect
    logic start_jac;
    vec_t jac_x_in;
    q16_t jac_lambda_in;
    q16_t jac_cost_out;
    vec_t jac_vec_g;
    mat_t jac_mat_a;
    logic jac_done, jac_busy;

    // Cholesky Solver Interconnect
    logic chol_start;
    vec_t chol_vec_p;
    logic chol_done, chol_singular, chol_busy;

    // Submodule 1: Jacobian & Hessian Accumulator Engine
    lm_jacobian_engine u_jac_engine (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (start_jac),
        .num_params (num_params_reg),
        .num_obs    (num_obs_reg),
        .x_curr     (jac_x_in),
        .obs_t_vec  (obs_t_reg),
        .obs_y_vec  (obs_y_reg),
        .lambda_reg (jac_lambda_in),
        .cost_s     (jac_cost_out),
        .vec_g      (jac_vec_g),
        .mat_a      (jac_mat_a),
        .done       (jac_done),
        .busy       (jac_busy)
    );

    // Submodule 2: Hardware Cholesky Linear System Solver
    cholesky_solver_engine u_chol_solver (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (chol_start),
        .num_vars   (num_params_reg),
        .mat_a      (jac_mat_a),
        .vec_g      (jac_vec_g),
        .vec_p      (chol_vec_p),
        .done       (chol_done),
        .singular   (chol_singular),
        .busy       (chol_busy)
    );

    vec_t tmp_step_x;
    q16_t max_grad_calc;
    q16_t cur_g_val;

    // Master Levenberg-Marquardt FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= LM_IDLE;
            num_params_reg <= 3'd2;
            num_obs_reg    <= 4'd4;
            tol_reg        <= Q16_EPS_DEF;
            lambda_curr    <= Q16_LAMBDA_DEF;
            max_iters_reg  <= 8'd50;
            iter_count     <= 8'd0;
            status         <= STATUS_IDLE;
            done           <= 1'b0;
            busy           <= 1'b0;
            cost_base      <= Q16_ZERO;
            cost_trial     <= Q16_ZERO;
            cost_optimal   <= Q16_ZERO;
            g_norm_inf     <= Q16_ZERO;
            lambda_final   <= Q16_ZERO;
            start_jac      <= 1'b0;
            chol_start     <= 1'b0;
            jac_x_in       <= '0;
            jac_lambda_in  <= Q16_ZERO;
            x_reg          <= '0;
            x_trial_reg    <= '0;
            x_optimal      <= '0;
            obs_t_reg      <= '0;
            obs_y_reg      <= '0;
        end else begin
            start_jac  <= 1'b0;
            chol_start <= 1'b0;

            // Load observation points asynchronously to optimization runs
            if (obs_we) begin
                obs_t_reg <= set_res(obs_t_reg, obs_addr, obs_t_in);
                obs_y_reg <= set_res(obs_y_reg, obs_addr, obs_y_in);
            end

            case (state)
                // -------------------------------------------------------------
                // STATE: LM_IDLE
                // -------------------------------------------------------------
                LM_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy           <= 1'b1;
                        num_params_reg <= (num_params != 3'd0)  ? num_params  : 3'd2;
                        num_obs_reg    <= (num_obs != 4'd0)     ? num_obs     : 4'd4;
                        tol_reg        <= (tolerance != Q16_ZERO) ? tolerance : Q16_EPS_DEF;
                        lambda_curr    <= (lambda_init != Q16_ZERO) ? lambda_init : Q16_LAMBDA_DEF;
                        max_iters_reg  <= (max_iters != 8'd0)   ? max_iters   : 8'd50;
                        iter_count     <= 8'd0;
                        status         <= STATUS_RUNNING;
                        x_reg          <= x_init;
                        state          <= LM_START_BASE;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE BASE RESIDUALS & AUGMENTED NORMAL EQUATIONS
                // -------------------------------------------------------------
                LM_START_BASE: begin
                    jac_x_in      <= x_reg;
                    jac_lambda_in <= lambda_curr;
                    start_jac     <= 1'b1;
                    state         <= LM_WAIT_BASE;
                end

                LM_WAIT_BASE: begin
                    if (jac_done) begin
                        cost_base <= jac_cost_out;
                        state     <= LM_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // CHECK CONVERGENCE (||g||_inf <= tolerance or max iterations)
                // -------------------------------------------------------------
                LM_CHECK_CONV: begin
                    max_grad_calc = 32'sd0;
                    for (int j = 0; j < MAX_PARAMS; j++) begin
                        if (j < num_params_reg) begin
                            cur_g_val = get_vec(jac_vec_g, 2'(j));
                            if (cur_g_val < 32'sd0) cur_g_val = -cur_g_val;
                            if (cur_g_val > max_grad_calc) max_grad_calc = cur_g_val;
                        end
                    end

                    // Condition 1: Gradient norm converged
                    if (max_grad_calc <= tol_reg || cost_base <= tol_reg) begin
                        status       <= STATUS_CONVERGED;
                        x_optimal    <= x_reg;
                        cost_optimal <= cost_base;
                        g_norm_inf   <= max_grad_calc;
                        lambda_final <= lambda_curr;
                        state        <= LM_DONE;

                    // Condition 2: Max iterations reached
                    end else if (iter_count >= max_iters_reg) begin
                        status       <= STATUS_MAX_ITERS;
                        x_optimal    <= x_reg;
                        cost_optimal <= cost_base;
                        g_norm_inf   <= max_grad_calc;
                        lambda_final <= lambda_curr;
                        state        <= LM_DONE;

                    // Condition 3: Solve (JᵀJ + λI)p = -Jᵀr via Cholesky
                    end else begin
                        chol_start <= 1'b1;
                        state      <= LM_WAIT_SOLVE;
                    end
                end

                // -------------------------------------------------------------
                // WAIT FOR CHOLESKY LINEAR SOLVER
                // -------------------------------------------------------------
                LM_WAIT_SOLVE: begin
                    if (chol_done) begin
                        if (chol_singular) begin
                            // If matrix is non-positive definite, increase lambda and retry
                            lambda_curr <= (lambda_curr < Q16_LAMBDA_MAX) ? (lambda_curr <<< 2) : Q16_LAMBDA_MAX;
                            iter_count  <= iter_count + 1'b1;
                            state       <= LM_START_BASE;
                        end else begin
                            // Candidate trial step: x_trial = x + p
                            tmp_step_x = x_reg;
                            for (int j = 0; j < MAX_PARAMS; j++) begin
                                if (j < num_params_reg) begin
                                    tmp_step_x = set_vec(tmp_step_x, 2'(j), get_vec(x_reg, 2'(j)) + get_vec(chol_vec_p, 2'(j)));
                                end
                            end
                            x_trial_reg <= tmp_step_x;
                            state       <= LM_START_TRIAL;
                        end
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE TRIAL CANDIDATE COST: S(x_trial)
                // -------------------------------------------------------------
                LM_START_TRIAL: begin
                    jac_x_in      <= x_trial_reg;
                    jac_lambda_in <= lambda_curr;
                    start_jac     <= 1'b1;
                    state         <= LM_WAIT_TRIAL;
                end

                LM_WAIT_TRIAL: begin
                    if (jac_done) begin
                        cost_trial <= jac_cost_out;
                        state      <= LM_ADAPT_LAMBDA;
                    end
                end

                // -------------------------------------------------------------
                // ADAPTIVE MARQUARDT DAMPING CONTROLLER
                // -------------------------------------------------------------
                LM_ADAPT_LAMBDA: begin
                    iter_count <= iter_count + 1'b1;

                    // CASE 1: SUCCESSFUL STEP -> S(x_trial) < S(x_curr)
                    if (cost_trial < cost_base) begin
                        x_reg       <= x_trial_reg;  // Accept candidate parameter update
                        cost_base   <= cost_trial;
                        // Reduce damping: λ = max(λ / 2, λ_min) to accelerate towards Gauss-Newton
                        lambda_curr <= (lambda_curr > Q16_LAMBDA_MIN) ? (lambda_curr >>> 1) : Q16_LAMBDA_MIN;
                        state       <= LM_START_BASE;

                    // CASE 2: UNSUCCESSFUL STEP -> S(x_trial) >= S(x_curr)
                    end else begin
                        // Reject candidate update, retain x_reg
                        // Increase damping: λ = min(λ * 2, λ_max) to revert towards safe gradient descent
                        lambda_curr <= (lambda_curr < Q16_LAMBDA_MAX) ? (lambda_curr <<< 1) : Q16_LAMBDA_MAX;
                        state       <= LM_START_BASE;
                    end
                end

                // -------------------------------------------------------------
                // STATE: LM_DONE
                // -------------------------------------------------------------
                LM_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= LM_IDLE;
                end

                default: state <= LM_IDLE;
            endcase
        end
    end

endmodule
