// =============================================================================
// File Name   : pso_lfsr_prng.sv
// Module Name : pso_lfsr_prng
// Project     : Particle Swarm Optimization (PSO) Accelerator (Solver #14)
// -----------------------------------------------------------------------------
// Description:
//   32-bit Hardware Xorshift Pseudo-Random Number Generator.
//   Generates uniform pseudo-random fractional scalars r in [0, 1) formatted
//   in Q16.16 fixed-point (with integer bits = 0, fractional bits = 16 bits).
// =============================================================================

`timescale 1ns / 1ps

import pso_types_pkg::*;

module pso_lfsr_prng (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        seed_load,
    input  logic [31:0] seed_val,
    input  logic        next_rand,
    output q16_t        rand_q16
);

    logic [31:0] lfsr_state;

    // Xorshift32 PRNG Algorithm:
    // y ^= y << 13; y ^= y >> 17; y ^= y << 5;
    logic [31:0] s1, s2, s3;
    assign s1 = lfsr_state ^ (lfsr_state << 13);
    assign s2 = s1 ^ (s1 >> 17);
    assign s3 = s2 ^ (s2 << 5);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lfsr_state <= 32'hACE1_54A9; // Non-zero default seed
        end else if (seed_load) begin
            lfsr_state <= (seed_val != 32'd0) ? seed_val : 32'hACE1_54A9;
        end else if (next_rand) begin
            lfsr_state <= (s3 != 32'd0) ? s3 : 32'h1357_9BDF;
        end
    end

    // Map top 16 bits to fraction in Q16.16: [0.0, 1.0)
    // Value = {16'h0000, lfsr_state[31:16]}
    assign rand_q16 = {16'h0000, lfsr_state[31:16]};

endmodule
