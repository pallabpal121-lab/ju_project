// =============================================================================
// File Name   : qubo_lfsr_prng.sv
// Module Name : qubo_lfsr_prng
// Project     : QUBO / Simulated Annealing Ising Accelerator (Solver #18)
// -----------------------------------------------------------------------------
// Description:
//   32-bit Xorshift Pseudo-Random Number Generator producing single-cycle
//   uniform fractional random numbers in Q16.16: rand_q16 in [0.0, 1.0).
// =============================================================================

`timescale 1ns / 1ps

import qubo_types_pkg::*;

module qubo_lfsr_prng (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        next_rand,   // Enable next random draw
    input  logic [31:0] seed,        // Initial seed (non-zero)
    input  logic        load_seed,   // Strobe to load seed
    output q16_t        rand_q16     // Uniform random number in [0, 1) Q16.16
);

    logic [31:0] state;
    logic [31:0] x1, x2, x3;

    // Xorshift-32 Algorithm:
    // x ^= x << 13;
    // x ^= x >> 17;
    // x ^= x << 5;
    always_comb begin
        x1 = state ^ (state << 13);
        x2 = x1 ^ (x1 >> 17);
        x3 = x2 ^ (x2 << 5);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= 32'hACE1_794B; // Default non-zero seed
        end else if (load_seed) begin
            state <= (seed != 32'd0) ? seed : 32'hACE1_794B;
        end else if (next_rand) begin
            state <= x3;
        end
    end

    // Map top 16 bits of pseudo-random state to [0, 1) in Q16.16
    assign rand_q16 = {16'h0000, state[31:16]};

endmodule
