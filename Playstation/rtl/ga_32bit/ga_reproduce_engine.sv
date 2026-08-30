// =============================================================================
// File Name   : ga_reproduce_engine.sv
// Module Name : ga_reproduce_engine
// Project     : Genetic Algorithm (GA) Global Search Accelerator (Solver #29)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Reproduction Engine:
//   1. Elitism: Preserves champion individual at slot 0
//   2. Tournament Selection: Selects parents via binary tournaments
//   3. Arithmetic Crossover: Blends parent genomes with random weights
//   4. Stochastic Mutation: Perturbs offspring genes within hyperbox bounds
// =============================================================================

`timescale 1ns / 1ps

import ga_types_pkg::*;
`include "ga_helpers.svh"

module ga_reproduce_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  pop_arr_t           population,       // Current population
    input  fitness_arr_t       fitness_arr,      // Current fitness array
    input  logic [2:0]         best_ind_idx,     // Champion index
    input  logic [3:0]         pop_size,         // Population size P (2..8)
    input  logic [2:0]         dim,              // Dimension D (1..4)
    input  gene_vec_t          lb_vec,           // Lower bounds
    input  gene_vec_t          ub_vec,           // Upper bounds
    input  q16_t               mut_prob,         // Mutation probability (Q16.16)
    input  q16_t               mut_scale,        // Mutation scale σ (Q16.16)
    input  q16_t               crossover_prob,   // Crossover probability (Q16.16)

    output pop_arr_t           next_population,  // Next generation population
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        REP_IDLE     = 3'd0,
        REP_ELITE    = 3'd1,
        REP_SELECT   = 3'd2,
        REP_CROSS    = 3'd3,
        REP_MUTATE   = 3'd4,
        REP_LATCH    = 3'd5,
        REP_DONE     = 3'd6
    } rep_state_t;

    rep_state_t state;

    pop_arr_t next_pop_reg;
    logic [2:0] target_p;
    logic [2:0] parent_a_idx, parent_b_idx;
    gene_vec_t  child_x;

    assign next_population = next_pop_reg;

    // PRNG Instance
    logic        prng_next;
    logic [31:0] prng_rand_out;
    q16_t        prng_q16;

    xorshift32_prng u_prng (
        .clk      (clk),
        .rst_n    (rst_n),
        .next_rand(prng_next),
        .seed     (32'h8765_4321),
        .rand_out (prng_rand_out),
        .rand_q16 (prng_q16)
    );

    logic [2:0] r1, r2, r3, r4;
    gene_vec_t  parent_a_x, parent_b_x;
    q16_t       alpha_q16, blend_a, blend_b, delta_mut;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= REP_IDLE;
            next_pop_reg   <= '0;
            target_p       <= 3'd0;
            parent_a_idx   <= 3'd0;
            parent_b_idx   <= 3'd0;
            child_x        <= '0;
            prng_next      <= 1'b0;
            done           <= 1'b0;
            busy           <= 1'b0;
        end else begin
            prng_next <= 1'b0;

            case (state)
                REP_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy     <= 1'b1;
                        target_p <= 3'd0;
                        state    <= REP_ELITE;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Elitism - preserve champion at slot 0
                REP_ELITE: begin
                    next_pop_reg <= set_individual(next_pop_reg, 3'd0, get_individual(population, best_ind_idx));
                    target_p     <= 3'd1;
                    prng_next    <= 1'b1;
                    state        <= REP_SELECT;
                end

                // Step 2: Binary Tournament Selection for Parents A and B
                REP_SELECT: begin
                    r1 = 3'(prng_rand_out[2:0] % pop_size);
                    r2 = 3'(prng_rand_out[5:3] % pop_size);
                    r3 = 3'(prng_rand_out[8:6] % pop_size);
                    r4 = 3'(prng_rand_out[11:9] % pop_size);

                    parent_a_idx <= (fitness_arr[r1] <= fitness_arr[r2]) ? r1 : r2;
                    parent_b_idx <= (fitness_arr[r3] <= fitness_arr[r4]) ? r3 : r4;

                    prng_next <= 1'b1;
                    state     <= REP_CROSS;
                end

                // Step 3: Arithmetic Crossover
                REP_CROSS: begin
                    parent_a_x = get_individual(population, parent_a_idx);
                    parent_b_x = get_individual(population, parent_b_idx);

                    if (prng_q16 <= crossover_prob) begin
                        // Blend coordinates: child = alpha * pA + (1-alpha) * pB
                        alpha_q16 = prng_q16;
                        for (int d = 0; d < MAX_DIM; d++) begin
                            if (d < dim) begin
                                blend_a    = q16_mul(alpha_q16, parent_a_x[d]);
                                blend_b    = q16_mul(32'h0001_0000 - alpha_q16, parent_b_x[d]);
                                child_x[d] = blend_a + blend_b;
                            end else begin
                                child_x[d] = 32'sd0;
                            end
                        end
                    end else begin
                        child_x <= parent_a_x;
                    end

                    prng_next <= 1'b1;
                    state     <= REP_MUTATE;
                end

                // Step 4: Stochastic Mutation & Clamping
                REP_MUTATE: begin
                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim) begin
                            // Random perturbation: delta = mut_scale * (2*rand - 1)
                            delta_mut  = q16_mul(mut_scale, (2 * prng_q16) - 32'h0001_0000);
                            child_x[d] = clamp_gene(child_x[d] + delta_mut, lb_vec[d], ub_vec[d]);
                        end
                    end

                    state <= REP_LATCH;
                end

                // Step 5: Latch child into next population
                REP_LATCH: begin
                    next_pop_reg <= set_individual(next_pop_reg, target_p, child_x);

                    if (target_p + 1'b1 < pop_size) begin
                        target_p  <= target_p + 1'b1;
                        prng_next <= 1'b1;
                        state     <= REP_SELECT;
                    end else begin
                        state <= REP_DONE;
                    end
                end

                REP_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= REP_IDLE;
                end

                default: state <= REP_IDLE;
            endcase
        end
    end

endmodule
