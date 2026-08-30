// =============================================================================
// File Name   : xorshift32_prng.sv
// Module Name : xorshift32_prng
// Project     : Cross-Entropy Method (CEM) Accelerator (Solver #31)
// -----------------------------------------------------------------------------
// Description:
//   32-bit hardware Galois/Xorshift pseudo-random number generator.
//   Provides uniform 32-bit random numbers and Q0.16 fractional values in [0, 1].
// =============================================================================

`timescale 1ns / 1ps

import cem_types_pkg::*;

module xorshift32_prng (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        next_rand,
    input  logic [31:0] seed,
    output logic [31:0] rand_out,
    output q16_t        rand_q16 // Fractional random value in [0.0, 1.0)
);

    logic [31:0] state;

    logic [31:0] s1, s2, s3;
    always_comb begin
        s1 = state ^ (state << 13);
        s2 = s1 ^ (s1 >> 17);
        s3 = s2 ^ (s2 << 5);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= (seed != 32'd0) ? seed : 32'hBEEF_CAFE;
        end else begin
            if (next_rand) begin
                state <= (s3 != 32'd0) ? s3 : 32'h9876_5432;
            end
        end
    end

    assign rand_out = state;
    assign rand_q16 = {16'd0, state[15:0]};

endmodule
