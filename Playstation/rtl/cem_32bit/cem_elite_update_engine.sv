// =============================================================================
// File Name   : cem_elite_update_engine.sv
// Module Name : cem_elite_update_engine
// Project     : Cross-Entropy Method (CEM) Accelerator (Solver #31)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Elite Selection & Distribution Parameter Updater:
//   1. Ranks top K elite samples by fitness F_s
//   2. Computes elite sample mean: μ_elite = (1/K) * ∑ x_elite
//   3. Computes elite standard deviation: σ_elite = sqrt( (1/K) * ∑ (x - μ)^2 + σ_min^2 )
//   4. Applies Polyak smoothing: μ_new = α μ_elite + (1-α) μ_old
// =============================================================================

`timescale 1ns / 1ps

import cem_types_pkg::*;
`include "cem_helpers.svh"

module cem_elite_update_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  sample_arr_t        sample_arr,       // Candidate samples
    input  fitness_arr_t       fitness_arr,      // Candidate fitness values
    input  logic [3:0]         num_samples,      // Total samples S (4..8)
    input  logic [2:0]         num_elites,       // Total elites K (2..4)
    input  logic [2:0]         dim,              // Dimension D (1..4)
    input  param_vec_t         current_mean,     // Current distribution mean μ
    input  param_vec_t         current_sigma,    // Current distribution std dev σ
    input  q16_t               alpha_smooth,     // Polyak smoothing factor α
    input  q16_t               sigma_min,        // Minimum std dev clamp

    output param_vec_t         next_mean,        // Updated mean vector μ_new
    output param_vec_t         next_sigma,       // Updated std dev vector σ_new
    output param_vec_t         champion_sol,     // Best sample x*
    output q16_t               champion_fitness, // Minimum fitness F*
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        ELT_IDLE     = 3'd0,
        ELT_RANK     = 3'd1,
        ELT_MEAN_SUM = 3'd2,
        ELT_VAR_SUM  = 3'd3,
        ELT_SQRT_WAIT= 3'd4,
        ELT_SMOOTH   = 3'd5,
        ELT_DONE     = 3'd6
    } elt_state_t;

    elt_state_t state;

    param_vec_t next_mean_reg;
    param_vec_t next_sigma_reg;
    param_vec_t champ_sol_reg;
    q16_t       champ_fit_reg;

    assign next_mean        = next_mean_reg;
    assign next_sigma       = next_sigma_reg;
    assign champion_sol     = champ_sol_reg;
    assign champion_fitness = champ_fit_reg;

    // Hardware Square Root Unit
    logic sqrt_start;
    q16_t sqrt_rad_in, sqrt_out;
    logic sqrt_done, sqrt_busy;

    q16_sqrt u_sqrt (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (sqrt_start),
        .rad_in  (sqrt_rad_in),
        .sqrt_out(sqrt_out),
        .done    (sqrt_done),
        .busy    (sqrt_busy)
    );

    // Hardware Divider Unit
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

    logic [2:0] elite_indices [0:MAX_ELITES-1];
    logic [1:0] curr_d;
    param_vec_t elite_mean_raw;
    param_vec_t elite_sigma_raw;

    // Temporary variables
    logic [2:0]   sorted_idx [0:MAX_SAMPLES-1];
    fitness_arr_t sorted_fit;
    logic [2:0]   temp_idx;
    q16_t         temp_fit, sum_dim, diff_val, prod_diff, sum_var;
    q16_t         blend_mu, blend_sig, sig_clamped, alpha_sig;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ELT_IDLE;
            next_mean_reg   <= '0;
            next_sigma_reg  <= '0;
            champ_sol_reg   <= '0;
            champ_fit_reg   <= Q16_MAX_POS;
            elite_mean_raw  <= '0;
            elite_sigma_raw <= '0;
            curr_d          <= 2'd0;
            sqrt_start      <= 1'b0;
            sqrt_rad_in     <= 32'sd0;
            div_start       <= 1'b0;
            div_dividend    <= 32'sd0;
            div_divisor     <= 32'h0001_0000;
            done            <= 1'b0;
            busy            <= 1'b0;
        end else begin
            sqrt_start <= 1'b0;
            div_start  <= 1'b0;

            case (state)
                ELT_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= ELT_RANK;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Rank samples by fitness via insertion sort
                ELT_RANK: begin
                    for (int s = 0; s < MAX_SAMPLES; s++) begin
                        sorted_idx[s] = 3'(s);
                        sorted_fit[s] = fitness_arr[s];
                    end

                    // Simple bubble sort with constant loop initializers
                    for (int i = 0; i < MAX_SAMPLES; i++) begin
                        for (int j = 0; j < MAX_SAMPLES; j++) begin
                            if (j > i && j < num_samples && sorted_fit[j] < sorted_fit[i]) begin
                                temp_fit      = sorted_fit[i];
                                sorted_fit[i] = sorted_fit[j];
                                sorted_fit[j] = temp_fit;

                                temp_idx      = sorted_idx[i];
                                sorted_idx[i] = sorted_idx[j];
                                sorted_idx[j] = temp_idx;
                            end
                        end
                    end

                    for (int k = 0; k < MAX_ELITES; k++) begin
                        elite_indices[k] <= sorted_idx[k];
                    end

                    champ_fit_reg <= sorted_fit[0];
                    champ_sol_reg <= get_sample_vec(sample_arr, sorted_idx[0]);

                    curr_d <= 2'd0;
                    state  <= ELT_MEAN_SUM;
                end

                // Step 2: Compute elite sample mean for dimension curr_d
                ELT_MEAN_SUM: begin
                    sum_dim = 32'sd0;
                    for (int k = 0; k < MAX_ELITES; k++) begin
                        if (k < num_elites) begin
                            sum_dim = sum_dim + get_sample_param(sample_arr, elite_indices[k], curr_d);
                        end
                    end

                    // Trigger divider: mean = sum / K
                    div_dividend <= sum_dim;
                    div_divisor  <= {16'd0, 13'd0, num_elites, 16'd0}; // K in Q16.16
                    div_start    <= 1'b1;
                    state        <= ELT_VAR_SUM;
                end

                // Step 3: Latch elite mean, compute elite variance, and trigger divider
                ELT_VAR_SUM: begin
                    if (div_done) begin
                        elite_mean_raw[curr_d] <= div_quotient;

                        sum_var = 32'sd0;
                        for (int k = 0; k < MAX_ELITES; k++) begin
                            if (k < num_elites) begin
                                diff_val  = get_sample_param(sample_arr, elite_indices[k], curr_d) - div_quotient;
                                prod_diff = q16_mul(diff_val, diff_val);
                                sum_var   = sum_var + prod_diff;
                            end
                        end

                        // Trigger divider: var = sum_var / K
                        div_dividend <= sum_var;
                        div_divisor  <= {16'd0, 13'd0, num_elites, 16'd0};
                        div_start    <= 1'b1;
                        state        <= ELT_SQRT_WAIT;
                    end
                end

                // Step 4: Latch variance, add sigma_min^2, and trigger square root
                ELT_SQRT_WAIT: begin
                    if (div_done) begin
                        // var_total = var + sigma_min^2
                        sqrt_rad_in <= div_quotient + q16_mul(sigma_min, sigma_min);
                        sqrt_start  <= 1'b1;
                    end

                    if (sqrt_done) begin
                        elite_sigma_raw[curr_d] <= sqrt_out;

                        if (curr_d + 1'b1 < dim) begin
                            curr_d <= curr_d + 1'b1;
                            state  <= ELT_MEAN_SUM;
                        end else begin
                            state <= ELT_SMOOTH;
                        end
                    end
                end

                // Step 5: Polyak Exponential Smoothing & Update
                ELT_SMOOTH: begin
                    // Alpha for sigma is tempered to prevent rapid variance collapse
                    alpha_sig = alpha_smooth >>> 2; // α_sigma = α / 4

                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim) begin
                            // μ_new = α μ_elite + (1-α) μ_curr
                            blend_mu = q16_mul(alpha_smooth, elite_mean_raw[d]) +
                                       q16_mul(32'h0001_0000 - alpha_smooth, current_mean[d]);

                            // σ_new = α_sig σ_elite + (1-α_sig) σ_curr
                            blend_sig = q16_mul(alpha_sig, elite_sigma_raw[d]) +
                                        q16_mul(32'h0001_0000 - alpha_sig, current_sigma[d]);

                            sig_clamped = (blend_sig < sigma_min) ? sigma_min : blend_sig;

                            next_mean_reg[d]  <= blend_mu;
                            next_sigma_reg[d] <= sig_clamped;
                        end else begin
                            next_mean_reg[d]  <= 32'sd0;
                            next_sigma_reg[d] <= 32'h0001_0000;
                        end
                    end

                    state <= ELT_DONE;
                end

                ELT_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= ELT_IDLE;
                end

                default: state <= ELT_IDLE;
            endcase
        end
    end

endmodule
