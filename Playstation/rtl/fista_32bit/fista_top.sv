// =============================================================================
// File Name   : fista_top.sv
// Module Name : fista_top
// Project     : Fast Iterative Shrinkage-Thresholding Algorithm (FISTA) Accelerator (Solver #15)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Module for the FISTA / Accelerated Proximal Gradient Optimizer.
//   Executes Nesterov momentum acceleration y_k, zero-cost numerical gradient
//   evaluations ∇f(y_k), hardware soft-thresholding proximal steps S_{γλ}(·),
//   convergence detection, and exact sparsity tracking.
// =============================================================================

`timescale 1ns / 1ps

import fista_types_pkg::*;
`include "fista_helpers.svh"

module fista_top (
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
    // Interface 2: Algorithm Parameters & Controls
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start trigger
    input  logic [2:0]         num_dims,      // Number of dimensions N (1..4)
    input  q16_t               gamma_step,    // Gradient step size γ = 1/L
    input  q16_t               lambda_reg,    // L1 Regularization penalty λ
    input  vec_t               x_init,        // Initial parameter estimate x_0
    input  q16_t               tolerance,     // Convergence threshold ε_tol
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 3: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,     // Converged parameter vector x*
    output q16_t               f_optimal,     // Final smooth cost f(x*)
    output q16_t               f_composite,   // Composite cost F(x*) = f(x*) + λ||x*||_1
    output logic [2:0]         sparsity_count,// Number of exact zero coefficients (x_i == 0)
    output logic [7:0]         iter_count,    // Total iterations executed
    output q16_t               dx_norm_inf,   // Final infinity norm ||x_k - x_{k-1}||_inf
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        FISTA_IDLE            = 3'd0,
        FISTA_START_GRAD      = 3'd1,
        FISTA_WAIT_GRAD       = 3'd2,
        FISTA_CHECK_CONV      = 3'd3,
        FISTA_WAIT_NESTEROV   = 3'd4,
        FISTA_EVAL_FINAL_COST = 3'd5,
        FISTA_WAIT_FINAL_COST = 3'd6,
        FISTA_DONE            = 3'd7
    } fista_state_t;

    fista_state_t state;

    // Iterate Vectors & State Registers
    vec_t       x_curr;      // x_k
    vec_t       x_prev;      // x_{k-1}
    vec_t       y_curr;      // Extrapolated momentum point y_k
    q16_t       t_curr;      // Nesterov scalar t_k
    logic [7:0] iter_cnt;
    status_t    status_reg;
    q16_t       dx_inf_reg;
    q16_t       f_smooth_reg;
    q16_t       f_comp_reg;
    logic [2:0] zeros_reg;

    // Latched Parameters
    logic [2:0] num_dims_reg;
    q16_t       gamma_reg;
    q16_t       lambda_val_reg;
    q16_t       tol_reg;
    logic [7:0] max_iters_reg;

    // Gradient Engine Submodule Interconnect
    logic start_grad;
    vec_t grad_vec_out;
    q16_t grad_f_base;
    logic grad_done, grad_busy;

    fista_gradient_engine u_grad (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (start_grad),
        .num_dims   (num_dims_reg),
        .y_in       (y_curr),
        .vec_g_out  (grad_vec_out),
        .f_base_out (grad_f_base),
        .done       (grad_done),
        .busy       (grad_busy)
    );

    // Nesterov Momentum Engine Submodule Interconnect
    logic start_nest;
    q16_t nest_t_next;
    q16_t nest_beta_out;
    vec_t nest_y_next;
    logic nest_done, nest_busy;

    fista_nesterov_engine u_nest (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (start_nest),
        .num_dims (num_dims_reg),
        .t_curr   (t_curr),
        .x_curr   (x_curr),
        .x_prev   (x_prev),
        .t_next   (nest_t_next),
        .beta_out (nest_beta_out),
        .y_next   (nest_y_next),
        .done     (nest_done),
        .busy     (nest_busy)
    );

    // DFG Submodule for Final Cost Evaluation
    logic start_dfg;
    vec_t dfg_x_in;
    q16_t dfg_f_out;
    logic dfg_done, dfg_busy;

    dfg_fista_engine u_dfg_final (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_dfg),
        .num_dims   (num_dims_reg),
        .x_vec      (dfg_x_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    // Output assignments
    assign x_optimal      = x_curr;
    assign f_optimal      = f_smooth_reg;
    assign f_composite    = f_comp_reg;
    assign sparsity_count = zeros_reg;
    assign iter_count     = iter_cnt;
    assign dx_norm_inf    = dx_inf_reg;
    assign status         = status_reg;

    // Temporary variables for calculations
    q16_t y_elem, g_elem, z_elem, x_new_elem, tau_val;
    q16_t max_dx, dx_elem;
    vec_t temp_x_new;
    q16_t l1_acc, elem_abs;
    logic [2:0] zero_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= FISTA_IDLE;
            x_curr         <= '0;
            x_prev         <= '0;
            y_curr         <= '0;
            t_curr         <= Q16_ONE;
            iter_cnt       <= 8'd0;
            status_reg     <= STATUS_IDLE;
            dx_inf_reg     <= Q16_ZERO;
            f_smooth_reg   <= Q16_ZERO;
            f_comp_reg     <= Q16_ZERO;
            zeros_reg      <= 3'd0;
            num_dims_reg   <= 3'd2;
            gamma_reg      <= Q16_GAMMA_DEF;
            lambda_val_reg <= Q16_LAMBDA_DEF;
            tol_reg        <= Q16_EPS_DEF;
            max_iters_reg  <= 8'd30;
            start_grad     <= 1'b0;
            start_nest     <= 1'b0;
            start_dfg      <= 1'b0;
            dfg_x_in       <= '0;
            done           <= 1'b0;
            busy           <= 1'b0;
        end else begin
            start_grad <= 1'b0;
            start_nest <= 1'b0;
            start_dfg  <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: FISTA_IDLE - Latch User Configuration & Initialization
                // -------------------------------------------------------------
                FISTA_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy           <= 1'b1;
                        num_dims_reg   <= num_dims;
                        gamma_reg      <= (gamma_step != Q16_ZERO) ? gamma_step : Q16_GAMMA_DEF;
                        lambda_val_reg <= lambda_reg;
                        tol_reg        <= (tolerance != Q16_ZERO)  ? tolerance  : Q16_EPS_DEF;
                        max_iters_reg  <= max_iters;
                        x_curr         <= x_init;
                        x_prev         <= x_init;
                        y_curr         <= x_init;
                        t_curr         <= Q16_ONE;
                        iter_cnt       <= 8'd0;
                        dx_inf_reg     <= Q16_ZERO;
                        status_reg     <= STATUS_RUNNING;
                        state          <= FISTA_START_GRAD;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: FISTA_START_GRAD - Trigger Gradient Evaluation at y_k
                // -------------------------------------------------------------
                FISTA_START_GRAD: begin
                    start_grad <= 1'b1;
                    state      <= FISTA_WAIT_GRAD;
                end

                // -------------------------------------------------------------
                // STATE 2: FISTA_WAIT_GRAD - Proximal Step & Soft-Thresholding
                // -------------------------------------------------------------
                FISTA_WAIT_GRAD: begin
                    if (grad_done) begin
                        // Threshold parameter tau = gamma * lambda
                        tau_val = q16_mul(gamma_reg, lambda_val_reg);

                        temp_x_new = '0;
                        max_dx     = Q16_ZERO;

                        for (int d = 0; d < MAX_PARAMS; d++) begin
                            if (d < num_dims_reg) begin
                                y_elem     = get_vec(y_curr, 2'(d));
                                g_elem     = get_vec(grad_vec_out, 2'(d));

                                // 1. Forward gradient step: z_k = y_k - γ ∇f(y_k)
                                z_elem     = y_elem - q16_mul(gamma_reg, g_elem);

                                // 2. Backward proximal step: x_k = S_{γλ}(z_k)
                                x_new_elem = q16_soft_thresh(z_elem, tau_val);
                                temp_x_new = set_vec(temp_x_new, 2'(d), x_new_elem);

                                // 3. Track infinity norm ||x_k - x_{k-1}||_inf
                                dx_elem    = q16_abs(x_new_elem - get_vec(x_curr, 2'(d)));
                                if (dx_elem > max_dx) begin
                                    max_dx = dx_elem;
                                end
                            end
                        end

                        x_prev     <= x_curr;
                        x_curr     <= temp_x_new;
                        dx_inf_reg <= max_dx;
                        iter_cnt   <= iter_cnt + 1'b1;

                        state      <= FISTA_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // STATE 3: FISTA_CHECK_CONV - Check Convergence & Nesterov Step
                // -------------------------------------------------------------
                FISTA_CHECK_CONV: begin
                    if (dx_inf_reg <= tol_reg) begin
                        status_reg <= STATUS_CONVERGED;
                        state      <= FISTA_EVAL_FINAL_COST;
                    end else if (iter_cnt >= max_iters_reg) begin
                        status_reg <= STATUS_MAX_ITERS;
                        state      <= FISTA_EVAL_FINAL_COST;
                    end else begin
                        // Trigger Nesterov Momentum Extrapolation Engine
                        start_nest <= 1'b1;
                        state      <= FISTA_WAIT_NESTEROV;
                    end
                end

                // -------------------------------------------------------------
                // STATE 4: FISTA_WAIT_NESTEROV - Update Momentum Point y_{k+1}
                // -------------------------------------------------------------
                FISTA_WAIT_NESTEROV: begin
                    if (nest_done) begin
                        t_curr <= nest_t_next;
                        y_curr <= nest_y_next;
                        state  <= FISTA_START_GRAD;
                    end
                end

                // -------------------------------------------------------------
                // STATE 5: FISTA_EVAL_FINAL_COST - Evaluate Final Objective f(x*)
                // -------------------------------------------------------------
                FISTA_EVAL_FINAL_COST: begin
                    dfg_x_in  <= x_curr;
                    start_dfg <= 1'b1;
                    state     <= FISTA_WAIT_FINAL_COST;
                end

                // -------------------------------------------------------------
                // STATE 6: FISTA_WAIT_FINAL_COST - Calculate Composite Cost & Sparsity
                // -------------------------------------------------------------
                FISTA_WAIT_FINAL_COST: begin
                    if (dfg_done) begin
                        f_smooth_reg <= dfg_f_out;

                        // Calculate L1 norm ||x*||_1 and count exact zeros
                        l1_acc   = Q16_ZERO;
                        zero_cnt = 3'd0;

                        for (int d = 0; d < MAX_PARAMS; d++) begin
                            if (d < num_dims_reg) begin
                                elem_abs = q16_abs(get_vec(x_curr, 2'(d)));
                                l1_acc   = l1_acc + elem_abs;
                                if (elem_abs == Q16_ZERO) begin
                                    zero_cnt = zero_cnt + 1'b1;
                                end
                            end
                        end

                        zeros_reg  <= zero_cnt;
                        f_comp_reg <= dfg_f_out + q16_mul(lambda_val_reg, l1_acc);
                        state      <= FISTA_DONE;
                    end
                end

                // -------------------------------------------------------------
                // STATE 7: FISTA_DONE - Assert Completion Strobe
                // -------------------------------------------------------------
                FISTA_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= FISTA_IDLE;
                end

                default: state <= FISTA_IDLE;
            endcase
        end
    end

endmodule
