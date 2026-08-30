// =============================================================================
// File Name   : cem_top.sv
// Module Name : cem_top
// Project     : Cross-Entropy Method (CEM) Accelerator (Solver #31)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Controller for Cross-Entropy Method (CEM) Optimizer.
//   Manages generation loops, Gaussian sampling, candidate fitness evaluation,
//   elite parameter updates, Polyak smoothing, and convergence tracking.
// =============================================================================

`timescale 1ns / 1ps

import cem_types_pkg::*;
`include "cem_helpers.svh"

module cem_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Initialization & Hyperparameters
    // -------------------------------------------------------------------------
    input  logic               init_cem,         // 1-cycle initialization strobe
    input  fitness_fn_t        fn_type,          // Objective fitness function
    input  logic [3:0]         num_samples,      // Total candidate samples S (4..8)
    input  logic [2:0]         num_elites,       // Total elite samples K (2..4)
    input  logic [2:0]         dim,              // Dimension D (1..4)
    input  param_vec_t         init_mean,        // Initial distribution mean μ_0
    input  param_vec_t         init_sigma,       // Initial distribution std dev σ_0
    input  param_vec_t         lb_vec,           // Lower bounds
    input  param_vec_t         ub_vec,           // Upper bounds
    input  logic [7:0]         max_iters,        // Maximum generations / iterations
    input  q16_t               target_tol,       // Stopping fitness / variance tolerance
    input  q16_t               alpha_smooth,     // Polyak smoothing factor α
    input  q16_t               sigma_min,        // Minimum std dev clamp

    // -------------------------------------------------------------------------
    // Interface 2: Optimization Execution & Results
    // -------------------------------------------------------------------------
    input  logic               opt_valid,        // Start CEM optimization loop
    output param_vec_t         best_solution,    // Champion parameter vector x* (4x1)
    output q16_t               best_fitness,     // Champion minimum fitness F*
    output param_vec_t         final_mean,       // Final distribution mean μ
    output param_vec_t         final_sigma,      // Final distribution std dev σ
    output logic [7:0]         iter_count,       // Completed generations
    output status_t            status,           // Status code
    output logic               opt_done,         // Optimization complete strobe
    output logic               busy              // High while engine is active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        CEM_IDLE         = 3'd0,
        CEM_SAMPLE_START = 3'd1,
        CEM_SAMPLE_WAIT  = 3'd2,
        CEM_FIT_EVAL     = 3'd3,
        CEM_UPDATE_START = 3'd4,
        CEM_UPDATE_WAIT  = 3'd5,
        CEM_CHECK        = 3'd6,
        CEM_DONE         = 3'd7
    } cem_state_t;

    cem_state_t state;

    // Internal Configuration Registers
    fitness_fn_t  fn_type_reg;
    logic [3:0]   s_samples_reg;
    logic [2:0]   k_elites_reg;
    logic [2:0]   dim_reg;
    param_vec_t   mu_reg;
    param_vec_t   sig_reg;
    param_vec_t   lb_reg;
    param_vec_t   ub_reg;
    logic [7:0]   max_i_reg;
    q16_t         tol_reg;
    q16_t         alpha_reg;
    q16_t         sig_min_reg;
    status_t      status_reg;
    logic [7:0]   i_cnt;
    logic [2:0]   curr_s;

    // Champion Registers
    param_vec_t   champ_sol_reg;
    q16_t         champ_fit_reg;
    sample_arr_t  curr_samples_reg;
    fitness_arr_t curr_fit_reg;

    assign best_solution = champ_sol_reg;
    assign best_fitness  = champ_fit_reg;
    assign final_mean    = mu_reg;
    assign final_sigma   = sig_reg;
    assign iter_count    = i_cnt;
    assign status        = status_reg;

    // Sub-engine 1: Sampling Instance
    logic        start_smp;
    sample_arr_t smp_arr_out;
    logic        smp_done;
    logic        smp_busy;

    cem_sample_engine u_smp (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start_smp),
        .mean_vec   (mu_reg),
        .sigma_vec  (sig_reg),
        .num_samples(s_samples_reg),
        .dim        (dim_reg),
        .lb_vec     (lb_reg),
        .ub_vec     (ub_reg),
        .sample_arr (smp_arr_out),
        .done       (smp_done),
        .busy       (smp_busy)
    );

    // Sub-engine 2: Elite Update Instance
    logic       start_upd;
    param_vec_t upd_mean_out;
    param_vec_t upd_sigma_out;
    param_vec_t upd_champ_sol;
    q16_t       upd_champ_fit;
    logic       upd_done;
    logic       upd_busy;

    cem_elite_update_engine u_upd (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (start_upd),
        .sample_arr      (curr_samples_reg),
        .fitness_arr     (curr_fit_reg),
        .num_samples     (s_samples_reg),
        .num_elites      (k_elites_reg),
        .dim             (dim_reg),
        .current_mean    (mu_reg),
        .current_sigma   (sig_reg),
        .alpha_smooth    (alpha_reg),
        .sigma_min       (sig_min_reg),
        .next_mean       (upd_mean_out),
        .next_sigma      (upd_sigma_out),
        .champion_sol    (upd_champ_sol),
        .champion_fitness(upd_champ_fit),
        .done            (upd_done),
        .busy            (upd_busy)
    );

    param_vec_t eval_x_in;
    q16_t       eval_fit_out;
    q16_t       max_sig_val;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= CEM_IDLE;
            fn_type_reg      <= FN_QUADRATIC;
            s_samples_reg    <= 4'd6;
            k_elites_reg     <= 3'd2;
            dim_reg          <= 3'd2;
            mu_reg           <= '0;
            sig_reg          <= '0;
            lb_reg           <= '0;
            ub_reg           <= '0;
            max_i_reg        <= 8'd30;
            tol_reg          <= 32'h0000_0010;
            alpha_reg        <= 32'h0000_B333; // α = 0.70
            sig_min_reg      <= 32'h0000_0CCC; // σ_min = 0.05
            status_reg       <= STATUS_IDLE;
            i_cnt            <= 8'd0;
            curr_s           <= 3'd0;
            champ_sol_reg    <= '0;
            champ_fit_reg    <= Q16_MAX_POS;
            curr_samples_reg <= '0;
            curr_fit_reg     <= '0;
            start_smp        <= 1'b0;
            start_upd        <= 1'b0;
            opt_done         <= 1'b0;
            busy             <= 1'b0;
        end else begin
            start_smp <= 1'b0;
            start_upd <= 1'b0;
            opt_done  <= 1'b0;

            case (state)
                CEM_IDLE: begin
                    if (init_cem) begin
                        fn_type_reg   <= fn_type;
                        s_samples_reg <= num_samples;
                        k_elites_reg  <= num_elites;
                        dim_reg       <= dim;
                        mu_reg        <= init_mean;
                        sig_reg       <= init_sigma;
                        lb_reg        <= lb_vec;
                        ub_reg        <= ub_vec;
                        max_i_reg     <= (max_iters != 8'd0) ? max_iters : 8'd30;
                        tol_reg       <= (target_tol != 32'sd0) ? target_tol : 32'h0000_0010;
                        alpha_reg     <= (alpha_smooth != 32'sd0) ? alpha_smooth : 32'h0000_B333;
                        sig_min_reg   <= (sigma_min != 32'sd0) ? sigma_min : 32'h0000_0CCC;
                        status_reg    <= STATUS_IDLE;
                        i_cnt         <= 8'd0;
                        curr_s        <= 3'd0;
                        champ_fit_reg <= Q16_MAX_POS;
                    end else if (opt_valid) begin
                        busy       <= 1'b1;
                        i_cnt      <= 8'd0;
                        status_reg <= STATUS_SAMPLING;
                        start_smp  <= 1'b1;
                        state      <= CEM_SAMPLE_WAIT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Wait for sampling engine
                CEM_SAMPLE_WAIT: begin
                    if (smp_done) begin
                        curr_samples_reg <= smp_arr_out;
                        curr_s           <= 3'd0;
                        status_reg       <= STATUS_EVALUATING;
                        state            <= CEM_FIT_EVAL;
                    end
                end

                // Step 2: Sequential sample fitness evaluation
                CEM_FIT_EVAL: begin
                    eval_x_in    = get_sample_vec(curr_samples_reg, curr_s);
                    eval_fit_out = evaluate_fitness(fn_type_reg, eval_x_in, dim_reg);

                    curr_fit_reg[curr_s] <= eval_fit_out;

                    if (curr_s + 1'b1 < s_samples_reg) begin
                        curr_s <= curr_s + 1'b1;
                    end else begin
                        status_reg <= STATUS_UPDATING;
                        start_upd  <= 1'b1;
                        state      <= CEM_UPDATE_WAIT;
                    end
                end

                // Step 3: Wait for elite selection & distribution update
                CEM_UPDATE_WAIT: begin
                    if (upd_done) begin
                        mu_reg  <= upd_mean_out;
                        sig_reg <= upd_sigma_out;

                        if (upd_champ_fit < champ_fit_reg || i_cnt == 8'd0) begin
                            champ_fit_reg <= upd_champ_fit;
                            champ_sol_reg <= upd_champ_sol;
                        end

                        state <= CEM_CHECK;
                    end
                end

                // Step 4: Check Convergence & Max Iterations
                CEM_CHECK: begin
                    max_sig_val = 32'sd0;
                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim_reg && sig_reg[d] > max_sig_val) begin
                            max_sig_val = sig_reg[d];
                        end
                    end

                    if (champ_fit_reg <= tol_reg || max_sig_val <= tol_reg) begin
                        status_reg <= STATUS_CONVERGED;
                        state      <= CEM_DONE;
                    end else if (i_cnt + 1'b1 >= max_i_reg) begin
                        status_reg <= STATUS_MAX_GENS;
                        state      <= CEM_DONE;
                    end else begin
                        i_cnt      <= i_cnt + 1'b1;
                        status_reg <= STATUS_SAMPLING;
                        start_smp  <= 1'b1;
                        state      <= CEM_SAMPLE_WAIT;
                    end
                end

                CEM_DONE: begin
                    opt_done <= 1'b1;
                    busy     <= 1'b0;
                    state    <= CEM_IDLE;
                end

                default: state <= CEM_IDLE;
            endcase
        end
    end

endmodule
