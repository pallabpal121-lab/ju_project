// =============================================================================
// File Name   : de_mutate_cross_engine.sv
// Module Name : de_mutate_cross_engine
// Project     : Differential Evolution (DE) Accelerator (Solver #30)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined DE/rand/1/bin Mutation & Binomial Crossover Engine:
//   1. Generates mutant vector: v_i = x_r1 + F * (x_r2 - x_r3)
//   2. Performs binomial crossover: u_i,d = v_i,d if rand <= CR or d == j_rand else x_i,d
//   3. Enforces hyperbox bounding: u_i,d = clamp(u_i,d, lb_d, ub_d)
// =============================================================================

`timescale 1ns / 1ps

import de_types_pkg::*;
`include "de_helpers.svh"

module de_mutate_cross_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [2:0]         target_idx,       // Target individual index i
    input  pop_arr_t           population,       // Current population array
    input  logic [3:0]         pop_size,         // Population size Np (4..8)
    input  logic [2:0]         dim,              // Dimension D (1..4)
    input  q16_t               f_scale,          // Differential mutation factor F
    input  q16_t               cr_rate,          // Crossover probability CR
    input  gene_vec_t          lb_vec,           // Lower bounds
    input  gene_vec_t          ub_vec,           // Upper bounds

    output gene_vec_t          trial_vec,        // Output trial vector u_i
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        MC_IDLE     = 3'd0,
        MC_PICK_R1  = 3'd1,
        MC_PICK_R2  = 3'd2,
        MC_PICK_R3  = 3'd3,
        MC_CROSSOVER= 3'd4,
        MC_DONE     = 3'd5
    } mc_state_t;

    mc_state_t state;

    gene_vec_t trial_reg;
    assign trial_vec = trial_reg;

    // PRNG Instance
    logic        prng_next;
    logic [31:0] prng_rand_out;
    q16_t        prng_q16;

    xorshift32_prng u_prng (
        .clk      (clk),
        .rst_n    (rst_n),
        .next_rand(prng_next),
        .seed     (32'h1357_9BDF),
        .rand_out (prng_rand_out),
        .rand_q16 (prng_q16)
    );

    logic [2:0] r1_idx, r2_idx, r3_idx, j_rand;
    logic [2:0] cand_r;
    gene_vec_t  x_target, x_r1, x_r2, x_r3;
    q16_t       diff_gene, f_diff, v_gene, u_gene;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= MC_IDLE;
            trial_reg <= '0;
            r1_idx    <= 3'd0;
            r2_idx    <= 3'd0;
            r3_idx    <= 3'd0;
            j_rand    <= 3'd0;
            prng_next <= 1'b0;
            done      <= 1'b0;
            busy      <= 1'b0;
        end else begin
            prng_next <= 1'b0;

            case (state)
                MC_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        prng_next <= 1'b1;
                        state     <= MC_PICK_R1;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Pick random index r1 != target_idx
                MC_PICK_R1: begin
                    cand_r = 3'(prng_rand_out[2:0] % pop_size);
                    if (cand_r == target_idx) begin
                        r1_idx <= 3'((cand_r + 1'b1) % pop_size);
                    end else begin
                        r1_idx <= cand_r;
                    end

                    prng_next <= 1'b1;
                    state     <= MC_PICK_R2;
                end

                // Step 2: Pick random index r2 != r1 and r2 != target_idx
                MC_PICK_R2: begin
                    cand_r = 3'(prng_rand_out[4:2] % pop_size);
                    if (cand_r == target_idx || cand_r == r1_idx) begin
                        r2_idx <= 3'((cand_r + 2'd2) % pop_size);
                    end else begin
                        r2_idx <= cand_r;
                    end

                    prng_next <= 1'b1;
                    state     <= MC_PICK_R3;
                end

                // Step 3: Pick random index r3 != r1, r2, target_idx and j_rand
                MC_PICK_R3: begin
                    cand_r = 3'(prng_rand_out[6:4] % pop_size);
                    if (cand_r == target_idx || cand_r == r1_idx || cand_r == r2_idx) begin
                        r3_idx <= 3'((cand_r + 2'd3) % pop_size);
                    end else begin
                        r3_idx <= cand_r;
                    end

                    j_rand    <= 3'(prng_rand_out[9:7] % dim);
                    prng_next <= 1'b1;
                    state     <= MC_CROSSOVER;
                end

                // Step 4: Differential Mutation & Binomial Crossover
                MC_CROSSOVER: begin
                    x_target = get_individual(population, target_idx);
                    x_r1     = get_individual(population, r1_idx);
                    x_r2     = get_individual(population, r2_idx);
                    x_r3     = get_individual(population, r3_idx);

                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim) begin
                            // Mutant: v_d = x_r1,d + F * (x_r2,d - x_r3,d)
                            diff_gene = x_r2[d] - x_r3[d];
                            f_diff    = q16_mul(f_scale, diff_gene);
                            v_gene    = x_r1[d] + f_diff;

                            // Binomial Crossover: u_d = (rand <= CR || d == j_rand) ? v_d : x_target_d
                            if (d == j_rand || prng_q16 <= cr_rate) begin
                                u_gene = clamp_gene(v_gene, lb_vec[d], ub_vec[d]);
                            end else begin
                                u_gene = x_target[d];
                            end

                            trial_reg[d] <= u_gene;
                        end else begin
                            trial_reg[d] <= 32'sd0;
                        end
                    end

                    state <= MC_DONE;
                end

                MC_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= MC_IDLE;
                end

                default: state <= MC_IDLE;
            endcase
        end
    end

endmodule
