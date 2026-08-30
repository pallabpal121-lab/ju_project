// =============================================================================
// File Name   : ga_top.sv
// Module Name : ga_top
// Project     : Genetic Algorithm (GA) Global Search Accelerator (Solver #29)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Controller for Genetic Algorithm (GA) Global Optimizer.
//   Manages generation loops, fitness evaluation, elitism tracking, tournament
//   selection, crossover, and mutation across non-convex/multi-modal fitness landscapes.
// =============================================================================

`timescale 1ns / 1ps

import ga_types_pkg::*;
`include "ga_helpers.svh"

module ga_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Initialization & Hyperparameters
    // -------------------------------------------------------------------------
    input  logic               init_ga,          // 1-cycle initialization strobe
    input  fitness_fn_t        fn_type,          // Objective fitness function
    input  logic [3:0]         pop_size,         // Population size P (2..8)
    input  logic [2:0]         dim,              // Dimension D (1..4)
    input  pop_arr_t           init_population,  // Initial seed population [32]
    input  gene_vec_t          lb_vec,           // Lower bounds
    input  gene_vec_t          ub_vec,           // Upper bounds
    input  logic [7:0]         max_gens,         // Maximum generations
    input  q16_t               target_tol,       // Stopping fitness tolerance
    input  q16_t               mut_prob,         // Mutation probability (Q16.16)
    input  q16_t               mut_scale,        // Mutation scale σ (Q16.16)
    input  q16_t               crossover_prob,   // Crossover probability (Q16.16)

    // -------------------------------------------------------------------------
    // Interface 2: Optimization Execution & Results
    // -------------------------------------------------------------------------
    input  logic               opt_valid,        // Start GA optimization loop
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
        GA_IDLE      = 3'd0,
        GA_FIT_START = 3'd1,
        GA_FIT_WAIT  = 3'd2,
        GA_CHECK     = 3'd3,
        GA_REP_START = 3'd4,
        GA_REP_WAIT  = 3'd5,
        GA_DONE      = 3'd6
    } ga_state_t;

    ga_state_t state;

    // Internal Configuration Registers
    fitness_fn_t fn_type_reg;
    logic [3:0]  p_size_reg;
    logic [2:0]  dim_reg;
    pop_arr_t    pop_reg;
    gene_vec_t   lb_reg;
    gene_vec_t   ub_reg;
    logic [7:0]  max_g_reg;
    q16_t        tol_reg;
    q16_t        m_prob_reg;
    q16_t        m_scale_reg;
    q16_t        c_prob_reg;
    status_t     status_reg;
    logic [7:0]  g_cnt;

    // Champion Registers
    gene_vec_t   best_sol_reg;
    q16_t        best_fit_reg;

    assign best_solution = best_sol_reg;
    assign best_fitness  = best_fit_reg;
    assign gen_count     = g_cnt;
    assign status        = status_reg;

    // Sub-engine 1: Fitness Engine Instance
    logic         start_fit;
    fitness_arr_t fit_arr_out;
    logic [2:0]   fit_best_idx;
    q16_t         fit_best_val;
    logic         fit_done;
    logic         fit_busy;

    ga_fitness_engine u_fit (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (start_fit),
        .fn_type     (fn_type_reg),
        .pop_size    (p_size_reg),
        .dim         (dim_reg),
        .population  (pop_reg),
        .fitness_arr (fit_arr_out),
        .best_ind_idx(fit_best_idx),
        .best_fitness(fit_best_val),
        .done        (fit_done),
        .busy        (fit_busy)
    );

    // Sub-engine 2: Reproduction Engine Instance
    logic     start_rep;
    pop_arr_t next_pop_out;
    logic     rep_done;
    logic     rep_busy;

    ga_reproduce_engine u_rep (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start_rep),
        .population     (pop_reg),
        .fitness_arr    (fit_arr_out),
        .best_ind_idx   (fit_best_idx),
        .pop_size       (p_size_reg),
        .dim            (dim_reg),
        .lb_vec         (lb_reg),
        .ub_vec         (ub_reg),
        .mut_prob       (m_prob_reg),
        .mut_scale      (m_scale_reg),
        .crossover_prob (c_prob_reg),
        .next_population(next_pop_out),
        .done           (rep_done),
        .busy           (rep_busy)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= GA_IDLE;
            fn_type_reg  <= FN_QUADRATIC;
            p_size_reg   <= 4'd6;
            dim_reg      <= 3'd2;
            pop_reg      <= '0;
            lb_reg       <= '0;
            ub_reg       <= '0;
            max_g_reg    <= 8'd30;
            tol_reg      <= 32'h0000_0010;
            m_prob_reg   <= 32'h0000_3333; // 0.20
            m_scale_reg  <= 32'h0000_4000; // 0.25
            c_prob_reg   <= 32'h0000_CCCC; // 0.80
            status_reg   <= STATUS_IDLE;
            g_cnt        <= 8'd0;
            best_sol_reg <= '0;
            best_fit_reg <= Q16_MAX_POS;
            start_fit    <= 1'b0;
            start_rep    <= 1'b0;
            opt_done     <= 1'b0;
            busy         <= 1'b0;
        end else begin
            start_fit <= 1'b0;
            start_rep <= 1'b0;
            opt_done  <= 1'b0;

            case (state)
                GA_IDLE: begin
                    if (init_ga) begin
                        fn_type_reg  <= fn_type;
                        p_size_reg   <= pop_size;
                        dim_reg      <= dim;
                        pop_reg      <= init_population;
                        lb_reg       <= lb_vec;
                        ub_reg       <= ub_vec;
                        max_g_reg    <= (max_gens != 8'd0) ? max_gens : 8'd30;
                        tol_reg      <= (target_tol != 32'sd0) ? target_tol : 32'h0000_0010;
                        m_prob_reg   <= (mut_prob != 32'sd0) ? mut_prob : 32'h0000_3333;
                        m_scale_reg  <= (mut_scale != 32'sd0) ? mut_scale : 32'h0000_4000;
                        c_prob_reg   <= (crossover_prob != 32'sd0) ? crossover_prob : 32'h0000_CCCC;
                        status_reg   <= STATUS_IDLE;
                        g_cnt        <= 8'd0;
                        best_fit_reg <= Q16_MAX_POS;
                    end else if (opt_valid) begin
                        busy       <= 1'b1;
                        g_cnt      <= 8'd0;
                        status_reg <= STATUS_EVALUATING;
                        state      <= GA_FIT_START;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Start fitness evaluation of current generation
                GA_FIT_START: begin
                    start_fit <= 1'b1;
                    state     <= GA_FIT_WAIT;
                end

                // Step 2: Wait for fitness evaluator
                GA_FIT_WAIT: begin
                    if (fit_done) begin
                        // Update champion if improved
                        if (fit_best_val < best_fit_reg || g_cnt == 8'd0) begin
                            best_fit_reg <= fit_best_val;
                            best_sol_reg <= get_individual(pop_reg, fit_best_idx);
                        end

                        state <= GA_CHECK;
                    end
                end

                // Step 3: Check termination conditions (convergence or max generations)
                GA_CHECK: begin
                    if (best_fit_reg <= tol_reg) begin
                        status_reg <= STATUS_CONVERGED;
                        state      <= GA_DONE;
                    end else if (g_cnt + 1'b1 >= max_g_reg) begin
                        status_reg <= STATUS_MAX_GENS;
                        state      <= GA_DONE;
                    end else begin
                        g_cnt      <= g_cnt + 1'b1;
                        status_reg <= STATUS_REPRODUCING;
                        state      <= GA_REP_START;
                    end
                end

                // Step 4: Start reproduction (Selection, Crossover, Mutation)
                GA_REP_START: begin
                    start_rep <= 1'b1;
                    state     <= GA_REP_WAIT;
                end

                // Step 5: Wait for next generation population
                GA_REP_WAIT: begin
                    if (rep_done) begin
                        pop_reg    <= next_pop_out;
                        status_reg <= STATUS_EVALUATING;
                        state      <= GA_FIT_START;
                    end
                end

                GA_DONE: begin
                    opt_done <= 1'b1;
                    busy     <= 1'b0;
                    state    <= GA_IDLE;
                end

                default: state <= GA_IDLE;
            endcase
        end
    end

endmodule
