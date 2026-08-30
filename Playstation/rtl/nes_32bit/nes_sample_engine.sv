// =============================================================================
// File Name   : nes_sample_engine.sv
// Module Name : nes_sample_engine
// Project     : Natural Evolution Strategies (NES) Accelerator (Solver #32)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Antithetic Perturbation Generator for Natural Evolution Strategies.
//   Generates P independent perturbation vectors ε_p ~ Uniform[-1, 1] / N(0, I).
// =============================================================================

`timescale 1ns / 1ps

import nes_types_pkg::*;
`include "nes_helpers.svh"

module nes_sample_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [2:0]         num_pairs,        // Total antithetic pairs P (2..4)
    input  logic [2:0]         dim,              // Dimension D (1..4)

    output noise_arr_t         noise_arr,        // Generated perturbations ε
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        SMP_IDLE = 2'd0,
        SMP_GEN  = 2'd1,
        SMP_NEXT = 2'd2,
        SMP_DONE = 2'd3
    } smp_state_t;

    smp_state_t state;

    noise_arr_t narr_reg;
    logic [1:0] curr_p;
    logic [1:0] curr_d;

    assign noise_arr = narr_reg;

    // PRNG Instance
    logic        prng_next;
    logic [31:0] prng_rand_out;
    q16_t        prng_q16;

    xorshift32_prng u_prng (
        .clk      (clk),
        .rst_n    (rst_n),
        .next_rand(prng_next),
        .seed     (32'hDEAD_BEEF),
        .rand_out (prng_rand_out),
        .rand_q16 (prng_q16)
    );

    q16_t z_noise;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= SMP_IDLE;
            narr_reg  <= '0;
            curr_p    <= 2'd0;
            curr_d    <= 2'd0;
            prng_next <= 1'b0;
            done      <= 1'b0;
            busy      <= 1'b0;
        end else begin
            prng_next <= 1'b0;

            case (state)
                SMP_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        narr_reg  <= '0;
                        curr_p    <= 2'd0;
                        curr_d    <= 2'd0;
                        prng_next <= 1'b1;
                        state     <= SMP_GEN;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Generate noise component ε_p,d in [-1.0, 1.0]
                SMP_GEN: begin
                    z_noise = (2 * prng_q16) - 32'h0001_0000; // [-1.0, 1.0]

                    narr_reg <= set_noise_val(narr_reg, curr_p, curr_d, z_noise);

                    if (curr_d + 1'b1 < dim) begin
                        curr_d    <= curr_d + 1'b1;
                        prng_next <= 1'b1;
                    end else begin
                        curr_d <= 2'd0;
                        state  <= SMP_NEXT;
                    end
                end

                // Step 2: Advance to next pair or finish
                SMP_NEXT: begin
                    if (curr_p + 1'b1 < num_pairs) begin
                        curr_p    <= curr_p + 1'b1;
                        prng_next <= 1'b1;
                        state     <= SMP_GEN;
                    end else begin
                        state <= SMP_DONE;
                    end
                end

                SMP_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= SMP_IDLE;
                end

                default: state <= SMP_IDLE;
            endcase
        end
    end

endmodule
