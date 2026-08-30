// =============================================================================
// File Name   : de_top.sv
// Module Name : de_top
// Project     : Differential Evolution (DE) Accelerator (Solver #30)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Controller for Differential Evolution (DE) Optimizer.
//   Manages generation loops, DE/rand/1/bin mutation, binomial crossover,
//   one-to-one greedy selection, and champion solution convergence tracking.
// =============================================================================

`timescale 1ns / 1ps

import de_types_pkg::*;
`include "de_helpers.svh"

module de_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Initialization & Hyperparameters
    // -------------------------------------------------------------------------
    input  logic               init_de,          // 1-cycle initialization strobe
    input  fitness_fn_t        fn_type,          // Objective fitness function
    input  logic [3:0]         pop_size,         // Population size Np (4..8)
    input  logic [2:0]         dim,              // Dimension D (1..4)
    input  pop_arr_t           init_population,  // Initial seed population [32]
    input  gene_vec_t          lb_vec,           // Lower bounds
    input  gene_vec_t          ub_vec,           // Upper bounds
    input  logic [7:0]         max_gens,         // Maximum generations
    input  q16_t               target_tol,       // Stopping fitness tolerance
    input  q16_t               f_scale,          // Differential mutation factor F
    input  q16_t               cr_rate,          // Crossover probability CR

    // -------------------------------------------------------------------------
    // Interface 2: Optimization Execution & Results
    // -------------------------------------------------------------------------
    input  logic               opt_valid,        // Start DE optimization loop
    output gene_vec_t          best_solution,    // Champion individual x* (4x1)
    output q16_t               best_fitness,     // Champion minimum fitness F*
    output logic [7:0]         gen_count,        // Completed generations
    output status_t            status,           // Status code
    output logic               opt_done,         // Optimization complete strobe
    output logic               busy              // High while engine is active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        DE_IDLE         = 3'd0,
        DE_INIT_EVAL    = 3'd1,
        DE_TGT_START    = 3'd2,
        DE_MC_WAIT      = 3'd3,
        DE_FIT_WAIT     = 3'd4,
        DE_CHECK_GEN    = 3'd5,
        DE_DONE         = 3'd6
    } de_state_t;

    de_state_t state;

    // Internal Configuration Registers
    fitness_fn_t  fn_type_reg;
    logic [3:0]   p_size_reg;
    logic [2:0]   dim_reg;
    pop_arr_t     pop_reg;
    fitness_arr_t fit_arr_reg;
    gene_vec_t    lb_reg;
    gene_vec_t    ub_reg;
    logic [7:0]   max_g_reg;
    q16_t         tol_reg;
    q16_t         f_scale_reg;
    q16_t         cr_rate_reg;
    status_t      status_reg;
    logic [7:0]   g_cnt;
    logic [2:0]   curr_i;

    // Champion Registers
    gene_vec_t    best_sol_reg;
    q16_t         best_fit_reg;

    assign best_solution = best_sol_reg;
    assign best_fitness  = best_fit_reg;
    assign gen_count     = g_cnt;
    assign status        = status_reg;

    // Sub-engine 1: Mutation & Crossover Instance
    logic      start_mc;
    gene_vec_t mc_trial_out;
    logic      mc_done;
    logic      mc_busy;

    de_mutate_cross_engine u_mc (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start_mc),
        .target_idx(curr_i),
        .population(pop_reg),
        .pop_size  (p_size_reg),
        .dim       (dim_reg),
        .f_scale   (f_scale_reg),
        .cr_rate   (cr_rate_reg),
        .lb_vec    (lb_reg),
        .ub_vec    (ub_reg),
        .trial_vec (mc_trial_out),
        .done      (mc_done),
        .busy      (mc_busy)
    );

    // Sub-engine 2: Fitness Evaluator Instance
    logic      start_fit;
    gene_vec_t fit_x_in;
    q16_t      fit_val_out;
    logic      fit_done;
    logic      fit_busy;

    de_fitness_engine u_fit (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start_fit),
        .fn_type    (fn_type_reg),
        .dim        (dim_reg),
        .x_vec      (fit_x_in),
        .fitness_val(fit_val_out),
        .done       (fit_done),
        .busy       (fit_busy)
    );

    gene_vec_t latched_trial;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= DE_IDLE;
            fn_type_reg   <= FN_QUADRATIC;
            p_size_reg    <= 4'd6;
            dim_reg       <= 3'd2;
            pop_reg       <= '0;
            fit_arr_reg   <= '0;
            lb_reg        <= '0;
            ub_reg        <= '0;
            max_g_reg     <= 8'd30;
            tol_reg       <= 32'h0000_0010;
            f_scale_reg   <= 32'h0000_8000; // F = 0.50
            cr_rate_reg   <= 32'h0000_CCCC; // CR = 0.80
            status_reg    <= STATUS_IDLE;
            g_cnt         <= 8'd0;
            curr_i        <= 3'd0;
            best_sol_reg  <= '0;
            best_fit_reg  <= Q16_MAX_POS;
            latched_trial <= '0;
            start_mc      <= 1'b0;
            start_fit     <= 1'b0;
            fit_x_in      <= '0;
            opt_done      <= 1'b0;
            busy          <= 1'b0;
        end else begin
            start_mc  <= 1'b0;
            start_fit <= 1'b0;
            opt_done  <= 1'b0;

            case (state)
                DE_IDLE: begin
                    if (init_de) begin
                        fn_type_reg  <= fn_type;
                        p_size_reg   <= pop_size;
                        dim_reg      <= dim;
                        pop_reg      <= init_population;
                        lb_reg       <= lb_vec;
                        ub_reg       <= ub_vec;
                        max_g_reg    <= (max_gens != 8'd0) ? max_gens : 8'd30;
                        tol_reg      <= (target_tol != 32'sd0) ? target_tol : 32'h0000_0010;
                        f_scale_reg  <= (f_scale != 32'sd0) ? f_scale : 32'h0000_8000;
                        cr_rate_reg  <= (cr_rate != 32'sd0) ? cr_rate : 32'h0000_CCCC;
                        status_reg   <= STATUS_IDLE;
                        g_cnt        <= 8'd0;
                        curr_i       <= 3'd0;
                        best_fit_reg <= Q16_MAX_POS;
                    end else if (opt_valid) begin
                        busy       <= 1'b1;
                        g_cnt      <= 8'd0;
                        curr_i     <= 3'd0;
                        status_reg <= STATUS_EVAL_INIT;
                        fit_x_in   <= get_individual(pop_reg, 3'd0);
                        start_fit  <= 1'b1;
                        state      <= DE_INIT_EVAL;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Initial population fitness evaluation
                DE_INIT_EVAL: begin
                    if (fit_done) begin
                        fit_arr_reg[curr_i] <= fit_val_out;

                        if (fit_val_out < best_fit_reg || curr_i == 3'd0) begin
                            best_fit_reg <= fit_val_out;
                            best_sol_reg <= get_individual(pop_reg, curr_i);
                        end

                        if (curr_i + 1'b1 < p_size_reg) begin
                            curr_i    <= curr_i + 1'b1;
                            fit_x_in  <= get_individual(pop_reg, curr_i + 1'b1);
                            start_fit <= 1'b1;
                        end else begin
                            curr_i     <= 3'd0;
                            status_reg <= STATUS_EVOLVING;
                            state      <= DE_TGT_START;
                        end
                    end
                end

                // Step 2: Start Mutation & Crossover for individual curr_i
                DE_TGT_START: begin
                    start_mc <= 1'b1;
                    state    <= DE_MC_WAIT;
                end

                // Step 3: Wait for trial vector u_i and trigger fitness evaluation
                DE_MC_WAIT: begin
                    if (mc_done) begin
                        latched_trial <= mc_trial_out;
                        fit_x_in      <= mc_trial_out;
                        start_fit     <= 1'b1;
                        state         <= DE_FIT_WAIT;
                    end
                end

                // Step 4: Greedy Selection: compare f(u_i) vs f(x_i)
                DE_FIT_WAIT: begin
                    if (fit_done) begin
                        if (fit_val_out <= fit_arr_reg[curr_i]) begin
                            pop_reg             <= set_individual(pop_reg, curr_i, latched_trial);
                            fit_arr_reg[curr_i] <= fit_val_out;

                            if (fit_val_out < best_fit_reg) begin
                                best_fit_reg <= fit_val_out;
                                best_sol_reg <= latched_trial;
                            end
                        end

                        if (curr_i + 1'b1 < p_size_reg) begin
                            curr_i <= curr_i + 1'b1;
                            state  <= DE_TGT_START;
                        end else begin
                            state  <= DE_CHECK_GEN;
                        end
                    end
                end

                // Step 5: Check Generation Termination
                DE_CHECK_GEN: begin
                    if (best_fit_reg <= tol_reg) begin
                        status_reg <= STATUS_CONVERGED;
                        state      <= DE_DONE;
                    end else if (g_cnt + 1'b1 >= max_g_reg) begin
                        status_reg <= STATUS_MAX_GENS;
                        state      <= DE_DONE;
                    end else begin
                        g_cnt  <= g_cnt + 1'b1;
                        curr_i <= 3'd0;
                        state  <= DE_TGT_START;
                    end
                end

                DE_DONE: begin
                    opt_done <= 1'b1;
                    busy     <= 1'b0;
                    state    <= DE_IDLE;
                end

                default: state <= DE_IDLE;
            endcase
        end
    end

endmodule
