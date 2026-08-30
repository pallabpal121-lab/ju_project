// =============================================================================
// File Name   : nes_top.sv
// Module Name : nes_top
// Project     : Natural Evolution Strategies (NES) Accelerator (Solver #32)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Controller for Natural Evolution Strategies (NES) Optimizer.
//   Manages policy generation loops, antithetic mirrored sampling, positive/negative
//   rollout evaluations, stochastic search gradient estimation, momentum updates,
//   and standard deviation annealing.
// =============================================================================

`timescale 1ns / 1ps

import nes_types_pkg::*;
`include "nes_helpers.svh"

module nes_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Initialization & Hyperparameters
    // -------------------------------------------------------------------------
    input  logic               init_nes,         // 1-cycle initialization strobe
    input  reward_fn_t         fn_type,          // Objective reward function
    input  logic [2:0]         num_pairs,        // Total antithetic pairs P (2..4)
    input  logic [2:0]         dim,              // Dimension D (1..4)
    input  policy_vec_t        init_policy,      // Initial policy vector θ_0
    input  q16_t               init_sigma,       // Initial exploration std dev σ_0
    input  policy_vec_t        lb_vec,           // Lower parameter bounds
    input  policy_vec_t        ub_vec,           // Upper parameter bounds
    input  logic [7:0]         max_iters,        // Maximum policy iterations
    input  q16_t               target_tol,       // Stopping gradient norm tolerance
    input  q16_t               learning_rate,    // Policy gradient step size α
    input  q16_t               momentum_rate,    // Momentum smoothing factor β
    input  q16_t               sigma_decay,      // Multiplicative decay factor γ
    input  q16_t               sigma_min,        // Minimum exploration std dev

    // -------------------------------------------------------------------------
    // Interface 2: Optimization Execution & Results
    // -------------------------------------------------------------------------
    input  logic               opt_valid,        // Start NES optimization loop
    output policy_vec_t        best_policy,      // Champion policy vector θ* (4x1)
    output q16_t               best_reward,      // Champion maximum reward R*
    output q16_t               final_sigma,      // Final exploration std dev σ
    output logic [7:0]         iter_count,       // Completed policy iterations
    output status_t            status,           // Status code
    output logic               opt_done,         // Optimization complete strobe
    output logic               busy              // High while engine is active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        NES_IDLE         = 4'd0,
        NES_SAMPLE_START = 4'd1,
        NES_SAMPLE_WAIT  = 4'd2,
        NES_EVAL_POS     = 4'd3,
        NES_EVAL_NEG     = 4'd4,
        NES_GRAD_START   = 4'd5,
        NES_GRAD_WAIT    = 4'd6,
        NES_UPDATE       = 4'd7,
        NES_CHECK        = 4'd8,
        NES_DONE         = 4'd9
    } nes_state_t;

    nes_state_t state;

    // Internal Configuration Registers
    reward_fn_t  fn_type_reg;
    logic [2:0]  p_pairs_reg;
    logic [2:0]  dim_reg;
    policy_vec_t theta_reg;
    policy_vec_t vel_reg;
    q16_t        sigma_reg;
    policy_vec_t lb_reg;
    policy_vec_t ub_reg;
    logic [7:0]  max_i_reg;
    q16_t        tol_reg;
    q16_t        lr_reg;
    q16_t        mom_reg;
    q16_t        decay_reg;
    q16_t        sig_min_reg;
    status_t     status_reg;
    logic [7:0]  i_cnt;
    logic [1:0]  curr_p;

    // Champion Registers
    policy_vec_t best_th_reg;
    q16_t        best_r_reg;

    noise_arr_t  noise_arr_reg;
    reward_arr_t r_pos_reg;
    reward_arr_t r_neg_reg;

    assign best_policy = best_th_reg;
    assign best_reward = best_r_reg;
    assign final_sigma = sigma_reg;
    assign iter_count  = i_cnt;
    assign status      = status_reg;

    // Sub-engine 1: Perturbation Sampling Instance
    logic       start_smp;
    noise_arr_t smp_noise_out;
    logic       smp_done;
    logic       smp_busy;

    nes_sample_engine u_smp (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (start_smp),
        .num_pairs(p_pairs_reg),
        .dim      (dim_reg),
        .noise_arr(smp_noise_out),
        .done     (smp_done),
        .busy     (smp_busy)
    );

    // Sub-engine 2: Gradient Estimator Instance
    logic        start_grd;
    policy_vec_t grd_vec_out;
    q16_t        grd_norm_out;
    logic        grd_done;
    logic        grd_busy;

    nes_grad_engine u_grd (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (start_grd),
        .noise_arr   (noise_arr_reg),
        .pos_rewards (r_pos_reg),
        .neg_rewards (r_neg_reg),
        .num_pairs   (p_pairs_reg),
        .dim         (dim_reg),
        .sigma       (sigma_reg),
        .grad_vec    (grd_vec_out),
        .grad_norm_sq(grd_norm_out),
        .done        (grd_done),
        .busy        (grd_busy)
    );

    policy_vec_t eval_th;
    q16_t        eval_r;
    q16_t        r_baseline;
    q16_t        new_vel, new_th, new_sig;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= NES_IDLE;
            fn_type_reg   <= FN_QUADRATIC;
            p_pairs_reg   <= 3'd4;
            dim_reg       <= 3'd2;
            theta_reg     <= '0;
            vel_reg       <= '0;
            sigma_reg     <= 32'h0000_8000; // σ = 0.50
            lb_reg        <= '0;
            ub_reg        <= '0;
            max_i_reg     <= 8'd40;
            tol_reg       <= 32'h0000_0010;
            lr_reg        <= 32'h0000_1999; // α = 0.10
            mom_reg       <= 32'h0000_E666; // β = 0.90
            decay_reg     <= 32'h0000_FA64; // γ = 0.98
            sig_min_reg   <= 32'h0000_0CCC; // σ_min = 0.05
            status_reg    <= STATUS_IDLE;
            i_cnt         <= 8'd0;
            curr_p        <= 2'd0;
            best_th_reg   <= '0;
            best_r_reg    <= Q16_MIN_NEG;
            noise_arr_reg <= '0;
            r_pos_reg     <= '0;
            r_neg_reg     <= '0;
            start_smp     <= 1'b0;
            start_grd     <= 1'b0;
            opt_done      <= 1'b0;
            busy          <= 1'b0;
        end else begin
            start_smp <= 1'b0;
            start_grd <= 1'b0;
            opt_done  <= 1'b0;

            case (state)
                NES_IDLE: begin
                    if (init_nes) begin
                        fn_type_reg <= fn_type;
                        p_pairs_reg <= num_pairs;
                        dim_reg     <= dim;
                        theta_reg   <= init_policy;
                        vel_reg     <= '0;
                        sigma_reg   <= (init_sigma != 32'sd0) ? init_sigma : 32'h0000_8000;
                        lb_reg      <= lb_vec;
                        ub_reg      <= ub_vec;
                        max_i_reg   <= (max_iters != 8'd0) ? max_iters : 8'd40;
                        tol_reg     <= (target_tol != 32'sd0) ? target_tol : 32'h0000_0010;
                        lr_reg      <= (learning_rate != 32'sd0) ? learning_rate : 32'h0000_1999;
                        mom_reg     <= (momentum_rate != 32'sd0) ? momentum_rate : 32'h0000_E666;
                        decay_reg   <= (sigma_decay != 32'sd0) ? sigma_decay : 32'h0000_FA64;
                        sig_min_reg <= (sigma_min != 32'sd0) ? sigma_min : 32'h0000_0CCC;
                        status_reg  <= STATUS_IDLE;
                        i_cnt       <= 8'd0;
                        curr_p      <= 2'd0;
                        best_th_reg <= init_policy;
                        best_r_reg  <= evaluate_reward(fn_type, init_policy, dim);
                    end else if (opt_valid) begin
                        busy       <= 1'b1;
                        i_cnt      <= 8'd0;
                        status_reg <= STATUS_SAMPLING;
                        start_smp  <= 1'b1;
                        state      <= NES_SAMPLE_WAIT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Wait for antithetic sampling engine
                NES_SAMPLE_WAIT: begin
                    if (smp_done) begin
                        noise_arr_reg <= smp_noise_out;
                        curr_p        <= 2'd0;
                        status_reg    <= STATUS_EVALUATING;
                        state         <= NES_EVAL_POS;
                    end
                end

                // Step 2: Evaluate Positive Rollout θ + σ * ε_p
                NES_EVAL_POS: begin
                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim_reg) begin
                            eval_th[d] = clamp_param(
                                theta_reg[d] + q16_mul(sigma_reg, get_noise_val(noise_arr_reg, curr_p, 2'(d))),
                                lb_reg[d], ub_reg[d]
                            );
                        end else begin
                            eval_th[d] = 32'sd0;
                        end
                    end

                    eval_r = evaluate_reward(fn_type_reg, eval_th, dim_reg);
                    r_pos_reg[curr_p] <= eval_r;

                    if (eval_r > best_r_reg) begin
                        best_r_reg  <= eval_r;
                        best_th_reg <= eval_th;
                    end

                    state <= NES_EVAL_NEG;
                end

                // Step 3: Evaluate Negative Mirrored Rollout θ - σ * ε_p
                NES_EVAL_NEG: begin
                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim_reg) begin
                            eval_th[d] = clamp_param(
                                theta_reg[d] - q16_mul(sigma_reg, get_noise_val(noise_arr_reg, curr_p, 2'(d))),
                                lb_reg[d], ub_reg[d]
                            );
                        end else begin
                            eval_th[d] = 32'sd0;
                        end
                    end

                    eval_r = evaluate_reward(fn_type_reg, eval_th, dim_reg);
                    r_neg_reg[curr_p] <= eval_r;

                    if (eval_r > best_r_reg) begin
                        best_r_reg  <= eval_r;
                        best_th_reg <= eval_th;
                    end

                    if (curr_p + 1'b1 < p_pairs_reg) begin
                        curr_p <= curr_p + 1'b1;
                        state  <= NES_EVAL_POS;
                    end else begin
                        status_reg <= STATUS_GRAD_EST;
                        start_grd  <= 1'b1;
                        state      <= NES_GRAD_WAIT;
                    end
                end

                // Step 4: Wait for gradient estimator
                NES_GRAD_WAIT: begin
                    if (grd_done) begin
                        status_reg <= STATUS_UPDATING;
                        state      <= NES_UPDATE;
                    end
                end

                // Step 5: Update policy parameter with momentum and decay sigma
                NES_UPDATE: begin
                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim_reg) begin
                            // v_{t+1} = β * v_t + α * g_t
                            new_vel = q16_mul(mom_reg, vel_reg[d]) + q16_mul(lr_reg, grd_vec_out[d]);
                            new_th  = clamp_param(theta_reg[d] + new_vel, lb_reg[d], ub_reg[d]);

                            vel_reg[d]   <= new_vel;
                            theta_reg[d] <= new_th;
                        end
                    end

                    // Baseline reward check at updated θ
                    r_baseline = evaluate_reward(fn_type_reg, theta_reg, dim_reg);
                    if (r_baseline > best_r_reg) begin
                        best_r_reg  <= r_baseline;
                        best_th_reg <= theta_reg;
                    end

                    // Anneal exploration standard deviation: σ = max(σ * γ, σ_min)
                    new_sig = q16_mul(sigma_reg, decay_reg);
                    if (new_sig < sig_min_reg) begin
                        sigma_reg <= sig_min_reg;
                    end else begin
                        sigma_reg <= new_sig;
                    end

                    state <= NES_CHECK;
                end

                // Step 6: Check convergence and iteration count
                NES_CHECK: begin
                    if (grd_norm_out <= tol_reg || best_r_reg >= -tol_reg) begin
                        status_reg <= STATUS_CONVERGED;
                        state      <= NES_DONE;
                    end else if (i_cnt + 1'b1 >= max_i_reg) begin
                        status_reg <= STATUS_MAX_ITERS;
                        state      <= NES_DONE;
                    end else begin
                        i_cnt      <= i_cnt + 1'b1;
                        status_reg <= STATUS_SAMPLING;
                        start_smp  <= 1'b1;
                        state      <= NES_SAMPLE_WAIT;
                    end
                end

                NES_DONE: begin
                    opt_done <= 1'b1;
                    busy     <= 1'b0;
                    state    <= NES_IDLE;
                end

                default: state <= NES_IDLE;
            endcase
        end
    end

endmodule
