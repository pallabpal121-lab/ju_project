// =============================================================================
// File Name   : ga_fitness_engine.sv
// Module Name : ga_fitness_engine
// Project     : Genetic Algorithm (GA) Global Search Accelerator (Solver #29)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Population Fitness Evaluator.
//   Evaluates fitness F_p = f(x_p) for all individuals in the population,
//   identifies the generation champion p* = argmin F_p and best fitness.
// =============================================================================

`timescale 1ns / 1ps

import ga_types_pkg::*;
`include "ga_helpers.svh"

module ga_fitness_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  fitness_fn_t        fn_type,          // Objective function type
    input  logic [3:0]         pop_size,         // Population size P (2..8)
    input  logic [2:0]         dim,              // Dimension D (1..4)
    input  pop_arr_t           population,       // Current population array

    output fitness_arr_t       fitness_arr,      // Fitness of all individuals
    output logic [2:0]         best_ind_idx,     // Index of best individual
    output q16_t               best_fitness,     // Minimum fitness value
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        FIT_IDLE = 2'd0,
        FIT_EVAL = 2'd1,
        FIT_DONE = 2'd2
    } fit_state_t;

    fit_state_t state;

    logic [2:0]   curr_p;
    fitness_arr_t fit_reg;
    logic [2:0]   best_idx_reg;
    q16_t         best_fit_reg;

    assign fitness_arr  = fit_reg;
    assign best_ind_idx = best_idx_reg;
    assign best_fitness = best_fit_reg;

    gene_vec_t ind_x;
    q16_t      ind_fit;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= FIT_IDLE;
            curr_p       <= 3'd0;
            fit_reg      <= '0;
            best_idx_reg <= 3'd0;
            best_fit_reg <= Q16_MAX_POS;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            case (state)
                FIT_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy         <= 1'b1;
                        curr_p       <= 3'd0;
                        best_fit_reg <= Q16_MAX_POS;
                        best_idx_reg <= 3'd0;
                        state        <= FIT_EVAL;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                FIT_EVAL: begin
                    ind_x   = get_individual(population, curr_p);
                    ind_fit = evaluate_fitness(fn_type, ind_x, dim);

                    fit_reg[curr_p] <= ind_fit;

                    // Track minimum fitness
                    if (ind_fit < best_fit_reg || curr_p == 3'd0) begin
                        best_fit_reg <= ind_fit;
                        best_idx_reg <= curr_p;
                    end

                    if (curr_p + 1'b1 < pop_size) begin
                        curr_p <= curr_p + 1'b1;
                    end else begin
                        state <= FIT_DONE;
                    end
                end

                FIT_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= FIT_IDLE;
                end

                default: state <= FIT_IDLE;
            endcase
        end
    end

endmodule
