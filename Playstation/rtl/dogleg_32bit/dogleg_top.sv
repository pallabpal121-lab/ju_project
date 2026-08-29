// =============================================================================
// File Name   : dogleg_top.sv
// Module Name : dogleg_top
// Project     : Trust-Region Dogleg Non-Linear Optimizer Accelerator (Solver #13)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Module for the Trust-Region Dogleg Non-Linear Optimization
//   Hardware Accelerator. Features:
//   - Programmable DFG Microcode Engine for arbitrary nonlinear models
//   - High-precision Central Difference Jacobian and Hessian Gram Matrix Accumulator
//   - Hardware Cholesky Decomposition Matrix Solver for Normal Equations
//   - Dedicated Powell Dogleg Step Engine (Gauss-Newton, Truncated Cauchy, Quadratic Interp)
//   - Adaptive Trust-Region Radius Management (dynamic expansion and contraction)
// =============================================================================

`timescale 1ns / 1ps

import dogleg_types_pkg::*;
`include "dogleg_helpers.svh"

module dogleg_top (
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
    input  logic               start,         // 1-cycle start trigger
    input  logic [2:0]         num_params,    // Number of parameters N (1..4)
    input  logic [3:0]         num_obs,       // Number of observations M (1..8)
    input  vec_t               x_init,        // Initial parameter guess [x3, x2, x1, x0]
    input  q16_t               tolerance,     // Convergence threshold on gradient infinity norm
    input  q16_t               delta_init,    // Initial trust region radius Δ_0
    input  logic [7:0]         max_iters,     // Maximum iteration budget

    // -------------------------------------------------------------------------
    // Interface 4: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,     // Converged parameter vector x*
    output q16_t               cost_optimal,  // Final least-squares cost F(x*)
    output q16_t               g_norm_inf,    // Final max gradient component max_j(|g_j|)
    output q16_t               delta_final,   // Final trust region radius Δ
    output step_type_t         step_type_last,// Last chosen step classification
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while solving
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        TR_IDLE         = 4'd0,
        TR_START_BASE   = 4'd1,
        TR_WAIT_BASE    = 4'd2,
        TR_CHECK_CONV   = 4'd3,
        TR_START_CHOL   = 4'd4,
        TR_WAIT_CHOL    = 4'd5,
        TR_START_DOGLEG = 4'd6,
        TR_WAIT_DOGLEG  = 4'd7,
        TR_START_TRIAL  = 4'd8,
        TR_WAIT_TRIAL   = 4'd9,
        TR_DIV_RHO      = 4'd10,
        TR_WAIT_RHO     = 4'd11,
        TR_ADAPT_DELTA  = 4'd12,
        TR_DONE         = 4'd13
    } tr_state_t;

    tr_state_t state;

    // Packed Observation Dataset Registers
    res_vec_t obs_t_reg;
    res_vec_t obs_y_reg;

    // Configuration & State Registers
    vec_t       x_curr;
    vec_t       x_trial;
    logic [2:0] num_params_reg;
    logic [3:0] num_obs_reg;
    q16_t       tol_reg;
    q16_t       delta_curr;
    logic [7:0] max_iters_reg;
    logic [7:0] iter_cnt;
    status_t    status_reg;

    q16_t       cost_base;
    q16_t       cost_trial;
    vec_t       vec_g_base;
    mat_t       mat_b_base;
    q16_t       g_inf_norm_reg;

    vec_t       vec_p_gn;
    vec_t       vec_p_dl;
    q16_t       pred_reduct_val;
    q16_t       p_norm_l2_val;
    step_type_t step_type_reg;
    step_type_t step_type_out;

    q16_t       delta_f_act;
    q16_t       rho_val;

    // Submodule 1: Jacobian & Hessian Accumulator Engine Interconnect
    logic start_jac;
    vec_t jac_x_in;
    q16_t jac_cost_out;
    vec_t jac_vec_g;
    mat_t jac_mat_a;
    logic jac_done, jac_busy;

    dogleg_jacobian_engine u_jac_engine (
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
        .cost_s     (jac_cost_out),
        .vec_g      (jac_vec_g),
        .mat_a      (jac_mat_a),
        .done       (jac_done),
        .busy       (jac_busy)
    );

    // Submodule 2: Cholesky Solver Engine Interconnect
    logic chol_start;
    vec_t chol_vec_p;
    logic chol_done, chol_singular, chol_busy;

    cholesky_solver_engine u_chol_engine (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (chol_start),
        .num_vars   (num_params_reg),
        .mat_a      (mat_b_base),
        .vec_g      (vec_g_base),
        .vec_p      (chol_vec_p),
        .done       (chol_done),
        .singular   (chol_singular),
        .busy       (chol_busy)
    );

    // Submodule 3: Dogleg Step Engine Interconnect
    logic start_dl;
    vec_t dl_vec_p;
    q16_t dl_pred_reduct;
    q16_t dl_p_norm_l2;
    step_type_t dl_step_type;
    logic dl_done, dl_busy;

    dogleg_step_engine u_dl_engine (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start_dl),
        .num_params   (num_params_reg),
        .vec_g        (vec_g_base),
        .mat_b        (mat_b_base),
        .vec_p_gn     (vec_p_gn),
        .delta_radius (delta_curr),
        .vec_p_dl     (dl_vec_p),
        .pred_reduct  (dl_pred_reduct),
        .p_norm_l2    (dl_p_norm_l2),
        .step_type    (dl_step_type),
        .done         (dl_done),
        .busy         (dl_busy)
    );

    // Submodule 4: Divider for Gain Ratio rho = delta_f_act / pred_reduct
    logic        div_start;
    q16_t        div_dividend, div_divisor, div_quotient;
    logic        div_done, div_by_zero, div_busy;

    q16_divider u_div_rho (
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

    // Output assignments
    assign x_optimal      = x_curr;
    assign cost_optimal   = cost_base;
    assign g_norm_inf     = g_inf_norm_reg;
    assign delta_final    = delta_curr;
    assign step_type_last = step_type_out;
    assign iter_count     = iter_cnt;
    assign status         = status_reg;

    // Temporary variables
    q16_t max_g;
    q16_t elem_g;
    vec_t temp_trial;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= TR_IDLE;
            obs_t_reg       <= '0;
            obs_y_reg       <= '0;
            x_curr          <= '0;
            x_trial         <= '0;
            num_params_reg  <= 3'd0;
            num_obs_reg     <= 4'd0;
            tol_reg         <= Q16_EPS_DEF;
            delta_curr      <= Q16_DELTA_INIT;
            max_iters_reg   <= 8'd0;
            iter_cnt        <= 8'd0;
            status_reg      <= STATUS_IDLE;
            cost_base       <= Q16_ZERO;
            cost_trial      <= Q16_ZERO;
            vec_g_base      <= '0;
            mat_b_base      <= '0;
            g_inf_norm_reg  <= Q16_ZERO;
            vec_p_gn        <= '0;
            vec_p_dl        <= '0;
            pred_reduct_val <= Q16_ZERO;
            p_norm_l2_val   <= Q16_ZERO;
            step_type_reg   <= STEP_NONE;
            step_type_out   <= STEP_NONE;
            delta_f_act     <= Q16_ZERO;
            rho_val         <= Q16_ZERO;
            start_jac       <= 1'b0;
            jac_x_in        <= '0;
            chol_start      <= 1'b0;
            start_dl        <= 1'b0;
            div_start       <= 1'b0;
            div_dividend    <= Q16_ZERO;
            div_divisor     <= Q16_ZERO;
            done            <= 1'b0;
            busy            <= 1'b0;
        end else begin
            // Observation Memory Writing
            if (obs_we) begin
                obs_t_reg <= set_res(obs_t_reg, obs_addr, obs_t_in);
                obs_y_reg <= set_res(obs_y_reg, obs_addr, obs_y_in);
            end

            start_jac  <= 1'b0;
            chol_start <= 1'b0;
            start_dl   <= 1'b0;
            div_start  <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: TR_IDLE - Latch User Configuration & Start
                // -------------------------------------------------------------
                TR_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy           <= 1'b1;
                        num_params_reg <= num_params;
                        num_obs_reg    <= num_obs;
                        x_curr         <= x_init;
                        tol_reg        <= (tolerance != Q16_ZERO) ? tolerance : Q16_EPS_DEF;
                        delta_curr     <= (delta_init != Q16_ZERO) ? delta_init : Q16_DELTA_INIT;
                        max_iters_reg  <= max_iters;
                        iter_cnt       <= 8'd0;
                        status_reg     <= STATUS_RUNNING;
                        state          <= TR_START_BASE;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: TR_START_BASE - Launch Baseline Evaluation
                // -------------------------------------------------------------
                TR_START_BASE: begin
                    jac_x_in  <= x_curr;
                    start_jac <= 1'b1;
                    state     <= TR_WAIT_BASE;
                end

                // -------------------------------------------------------------
                // STATE 2: TR_WAIT_BASE - Latch Baseline Cost, g, and Hessian B
                // -------------------------------------------------------------
                TR_WAIT_BASE: begin
                    if (jac_done) begin
                        cost_base  <= jac_cost_out;
                        vec_g_base <= jac_vec_g;
                        mat_b_base <= jac_mat_a;
                        state      <= TR_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // STATE 3: TR_CHECK_CONV - Check Gradient Norm & Iteration Budget
                // -------------------------------------------------------------
                TR_CHECK_CONV: begin
                    // Compute gradient infinity norm ||g||_inf = max_j |g_j|
                    max_g = Q16_ZERO;
                    for (int j = 0; j < MAX_PARAMS; j++) begin
                        if (j < num_params_reg[1:0]) begin
                            elem_g = q16_abs(get_vec(vec_g_base, 2'(j)));
                            if (elem_g > max_g) begin
                                max_g = elem_g;
                            end
                        end
                    end
                    g_inf_norm_reg <= max_g;

                    if (max_g <= tol_reg || cost_base <= tol_reg) begin
                        status_reg <= STATUS_CONVERGED;
                        state      <= TR_DONE;
                    end else if (iter_cnt >= max_iters_reg) begin
                        status_reg <= STATUS_MAX_ITERS;
                        state      <= TR_DONE;
                    end else if (delta_curr <= Q16_DELTA_MIN) begin
                        status_reg <= (cost_base <= (tol_reg <<< 2)) ? STATUS_CONVERGED : STATUS_RADIUS_MIN;
                        state      <= TR_DONE;
                    end else begin
                        state <= TR_START_CHOL;
                    end
                end

                // -------------------------------------------------------------
                // STATE 4: TR_START_CHOL - Solve B · p_gn = -g
                // -------------------------------------------------------------
                TR_START_CHOL: begin
                    chol_start <= 1'b1;
                    state      <= TR_WAIT_CHOL;
                end

                // -------------------------------------------------------------
                // STATE 5: TR_WAIT_CHOL - Latch Gauss-Newton Step p_gn
                // -------------------------------------------------------------
                TR_WAIT_CHOL: begin
                    if (chol_done) begin
                        if (chol_singular) begin
                            status_reg <= STATUS_SINGULAR;
                            state      <= TR_DONE;
                        end else begin
                            vec_p_gn <= chol_vec_p;
                            state    <= TR_START_DOGLEG;
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE 6: TR_START_DOGLEG - Launch Powell Dogleg Step Engine
                // -------------------------------------------------------------
                TR_START_DOGLEG: begin
                    start_dl <= 1'b1;
                    state    <= TR_WAIT_DOGLEG;
                end

                // -------------------------------------------------------------
                // STATE 7: TR_WAIT_DOGLEG - Latch Dogleg Step p_dl & Form Trial Point
                // -------------------------------------------------------------
                TR_WAIT_DOGLEG: begin
                    if (dl_done) begin
                        vec_p_dl        <= dl_vec_p;
                        pred_reduct_val <= dl_pred_reduct;
                        p_norm_l2_val   <= dl_p_norm_l2;
                        step_type_reg   <= dl_step_type;
                        step_type_out   <= dl_step_type;

                        // Candidate trial point x_trial = x_curr + p_dl
                        temp_trial = '0;
                        for (int i = 0; i < MAX_PARAMS; i++) begin
                            if (i < num_params_reg[1:0]) begin
                                temp_trial = set_vec(temp_trial, 2'(i), get_vec(x_curr, 2'(i)) + get_vec(dl_vec_p, 2'(i)));
                            end
                        end
                        x_trial <= temp_trial;

                        state <= TR_START_TRIAL;
                    end
                end

                // -------------------------------------------------------------
                // STATE 8: TR_START_TRIAL - Evaluate Cost at Trial Point F(x_trial)
                // -------------------------------------------------------------
                TR_START_TRIAL: begin
                    jac_x_in  <= x_trial;
                    start_jac <= 1'b1;
                    state     <= TR_WAIT_TRIAL;
                end

                // -------------------------------------------------------------
                // STATE 9: TR_WAIT_TRIAL - Latch Trial Cost & Calculate Actual Reduction
                // -------------------------------------------------------------
                TR_WAIT_TRIAL: begin
                    if (jac_done) begin
                        cost_trial  <= jac_cost_out;
                        delta_f_act <= cost_base - jac_cost_out;
                        state       <= TR_DIV_RHO;
                    end
                end

                // -------------------------------------------------------------
                // STATE 10: TR_DIV_RHO - Calculate Gain Ratio rho = delta_f_act / pred_reduct
                // -------------------------------------------------------------
                TR_DIV_RHO: begin
                    if (pred_reduct_val <= Q16_ZERO) begin
                        rho_val <= (delta_f_act > Q16_ZERO) ? Q16_ONE : Q16_NEG_ONE;
                        state   <= TR_ADAPT_DELTA;
                    end else begin
                        div_dividend <= delta_f_act;
                        div_divisor  <= pred_reduct_val;
                        div_start    <= 1'b1;
                        state        <= TR_WAIT_RHO;
                    end
                end

                TR_WAIT_RHO: begin
                    if (div_done || div_by_zero) begin
                        rho_val <= (div_by_zero) ? ((delta_f_act > Q16_ZERO) ? Q16_ONE : Q16_NEG_ONE) : div_quotient;
                        state   <= TR_ADAPT_DELTA;
                    end
                end

                // -------------------------------------------------------------
                // STATE 12: TR_ADAPT_DELTA - Step Acceptance & Trust Radius Adaptation
                // -------------------------------------------------------------
                TR_ADAPT_DELTA: begin
                    iter_cnt <= iter_cnt + 1'b1;

                    // 1. Step Acceptance Decision
                    if (rho_val > Q16_ETA_DEF && delta_f_act > Q16_ZERO) begin
                        // Accept step: update x_curr and cost_base
                        x_curr    <= x_trial;
                        cost_base <= cost_trial;

                        // 2. Radius Expansion if very successful and step hit boundary
                        if (rho_val > Q16_THREE_QUART && p_norm_l2_val >= q16_mul(Q16_THREE_QUART, delta_curr)) begin
                            // Expand trust region: min(2 * Δ, Δ_max)
                            if (delta_curr < (Q16_DELTA_MAX >>> 1)) begin
                                delta_curr <= (delta_curr <<< 1);
                            end else begin
                                delta_curr <= Q16_DELTA_MAX;
                            end
                        end
                        // Else keep radius unchanged

                        state <= TR_START_BASE; // Evaluate new Jacobian & Hessian at x_{k+1}
                    end else begin
                        // Reject step: keep x_curr unchanged, shrink trust region
                        // Shrink radius: max(0.5 * Δ, Δ_min)
                        if ((delta_curr >>> 1) > Q16_DELTA_MIN) begin
                            delta_curr <= (delta_curr >>> 1);
                        end else begin
                            delta_curr <= Q16_DELTA_MIN;
                        end

                        // If radius collapsed to minimum, terminate
                        if ((delta_curr >>> 1) <= Q16_DELTA_MIN) begin
                            status_reg <= STATUS_RADIUS_MIN;
                            state      <= TR_DONE;
                        end else begin
                            // Retry step engine with smaller trust region radius
                            state <= TR_START_DOGLEG;
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE 13: TR_DONE - Completion & Handshake
                // -------------------------------------------------------------
                TR_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= TR_IDLE;
                end

                default: state <= TR_IDLE;
            endcase
        end
    end

endmodule
