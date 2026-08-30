// =============================================================================
// File Name   : cem_sample_engine.sv
// Module Name : cem_sample_engine
// Project     : Cross-Entropy Method (CEM) Accelerator (Solver #31)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Gaussian Trajectory / Parameter Sampling Engine.
//   Generates S candidate samples:
//     x_s,d = clamp(μ_d + σ_d * (4*rand - 2.0), lb_d, ub_d)
//   Sample 0 is anchored to the deterministic mean μ.
// =============================================================================

`timescale 1ns / 1ps

import cem_types_pkg::*;
`include "cem_helpers.svh"

module cem_sample_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  param_vec_t         mean_vec,         // Distribution mean μ (4x1)
    input  param_vec_t         sigma_vec,        // Distribution standard deviation σ (4x1)
    input  logic [3:0]         num_samples,      // Total samples S (4..8)
    input  logic [2:0]         dim,              // Dimension D (1..4)
    input  param_vec_t         lb_vec,           // Lower bounds
    input  param_vec_t         ub_vec,           // Upper bounds

    output sample_arr_t        sample_arr,       // Output candidate samples
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        SMP_IDLE    = 3'd0,
        SMP_ANCHOR  = 3'd1,
        SMP_GEN     = 3'd2,
        SMP_NEXT    = 3'd3,
        SMP_DONE    = 3'd4
    } smp_state_t;

    smp_state_t state;

    sample_arr_t sarr_reg;
    logic [2:0]  curr_s;
    logic [1:0]  curr_d;

    assign sample_arr = sarr_reg;

    // PRNG Instance
    logic        prng_next;
    logic [31:0] prng_rand_out;
    q16_t        prng_q16;

    xorshift32_prng u_prng (
        .clk      (clk),
        .rst_n    (rst_n),
        .next_rand(prng_next),
        .seed     (32'hBEEF_1234),
        .rand_out (prng_rand_out),
        .rand_q16 (prng_q16)
    );

    q16_t z_noise, delta_val, raw_val, clamped_val;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= SMP_IDLE;
            sarr_reg  <= '0;
            curr_s    <= 3'd0;
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
                        busy     <= 1'b1;
                        sarr_reg <= '0;
                        curr_s   <= 3'd0;
                        state    <= SMP_ANCHOR;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Anchor sample 0 to deterministic mean μ
                SMP_ANCHOR: begin
                    sarr_reg <= set_sample_vec(sarr_reg, 3'd0, mean_vec);
                    curr_s   <= 3'd1;
                    curr_d   <= 2'd0;
                    prng_next<= 1'b1;
                    state    <= SMP_GEN;
                end

                // Step 2: Sample coordinates with 2-sigma perturbation z in [-2.0, 2.0]
                SMP_GEN: begin
                    z_noise     = (4 * prng_q16) - 32'h0002_0000; // [-2.0, 2.0]
                    delta_val   = q16_mul(sigma_vec[curr_d], z_noise);
                    raw_val     = mean_vec[curr_d] + delta_val;
                    clamped_val = clamp_param(raw_val, lb_vec[curr_d], ub_vec[curr_d]);

                    sarr_reg <= set_sample_param(sarr_reg, curr_s, curr_d, clamped_val);

                    if (curr_d + 1'b1 < dim) begin
                        curr_d    <= curr_d + 1'b1;
                        prng_next <= 1'b1;
                    end else begin
                        curr_d <= 2'd0;
                        state  <= SMP_NEXT;
                    end
                end

                // Step 3: Advance to next sample or finish
                SMP_NEXT: begin
                    if (curr_s + 1'b1 < num_samples) begin
                        curr_s    <= curr_s + 1'b1;
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
