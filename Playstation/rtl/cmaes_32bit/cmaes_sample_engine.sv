// =============================================================================
// File Name   : cmaes_sample_engine.sv
// Module Name : cmaes_sample_engine
// Project     : Covariance Matrix Adaptation Evolution Strategy (CMA-ES) (Solver #34)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Anisotropic Candidate Generator for CMA-ES:
//   Generates dual antithetic mirrored pairs:
//     - Candidate 0: +z_A
//     - Candidate 1: -z_A (Antithetic mirror of 0)
//     - Candidate 2: +z_B
//     - Candidate 3: -z_B (Antithetic mirror of 2)
//   Transforms: y_k = A * z_k
//   Candidate:  x_k = clamp(m + σ * y_k, lb, ub)
// =============================================================================

`timescale 1ns / 1ps

import cmaes_types_pkg::*;
`include "cmaes_helpers.svh"

module cmaes_sample_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  param_vec_t         mean_vec,     // Distribution mean m (4x1)
    input  q16_t               sigma,        // Step size σ
    input  cov_mat_t           A_mat,        // Coordinate transformation matrix A (4x4)
    input  logic [2:0]         pop_size,     // Population size λ (2..4)
    input  logic [2:0]         dim,          // Dimension D (1..4)
    input  param_vec_t         lb_vec,       // Lower bounds
    input  param_vec_t         ub_vec,       // Upper bounds

    output cand_arr_t          cand_samples, // Candidate vectors x_k (16x1 packed)
    output cand_arr_t          cand_y_vecs,  // Anisotropic steps y_k (16x1 packed)
    output cand_arr_t          cand_z_vecs,  // Isotropic perturbations z_k (16x1 packed)
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        SMP_IDLE    = 3'd0,
        SMP_GEN_Z   = 3'd1,
        SMP_TRANS_Y = 3'd2,
        SMP_CALC_X  = 3'd3,
        SMP_NEXT_K  = 3'd4,
        SMP_DONE    = 3'd5
    } smp_state_t;

    smp_state_t state;

    cand_arr_t x_reg;
    cand_arr_t y_reg;
    cand_arr_t z_reg;

    assign cand_samples = x_reg;
    assign cand_y_vecs  = y_reg;
    assign cand_z_vecs  = z_reg;

    // PRNG Instance
    logic        prng_next;
    logic [31:0] prng_rand_out;
    q16_t        prng_q16;

    xorshift32_prng u_prng (
        .clk      (clk),
        .rst_n    (rst_n),
        .next_rand(prng_next),
        .seed     (32'hA55A_5AA5),
        .rand_out (prng_rand_out),
        .rand_q16 (prng_q16)
    );

    logic [1:0] curr_k;
    param_vec_t temp_z, temp_y;
    param_vec_t temp_y_comb;
    q16_t       z_val, x_val;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= SMP_IDLE;
            x_reg     <= '0;
            y_reg     <= '0;
            z_reg     <= '0;
            curr_k    <= 2'd0;
            temp_z    <= '0;
            temp_y    <= '0;
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
                        x_reg     <= '0;
                        y_reg     <= '0;
                        z_reg     <= '0;
                        curr_k    <= 2'd0;
                        prng_next <= 1'b1;
                        state     <= SMP_GEN_Z;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Generate dual antithetic mirrored pairs
                SMP_GEN_Z: begin
                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim) begin
                            if (curr_k == 2'd0) begin
                                // Pair A, positive: +z_A
                                if (d == 0)      z_val = q16_t'((prng_rand_out[15:0]  << 1) - 32'h0001_0000);
                                else if (d == 1) z_val = q16_t'((prng_rand_out[31:16] << 1) - 32'h0001_0000);
                                else if (d == 2) z_val = q16_t'(((prng_rand_out[15:0] ^ 16'h5555) << 1) - 32'h0001_0000);
                                else             z_val = q16_t'(((prng_rand_out[31:16] ^ 16'hAAAA) << 1) - 32'h0001_0000);
                            end else if (curr_k == 2'd1) begin
                                // Pair A, mirrored: -z_A
                                z_val = -z_reg[{2'd0, 2'(d)}];
                            end else if (curr_k == 2'd2) begin
                                // Pair B, positive: +z_B
                                if (d == 0)      z_val = q16_t'((prng_rand_out[31:16] << 1) - 32'h0001_0000);
                                else if (d == 1) z_val = q16_t'((prng_rand_out[15:0]  << 1) - 32'h0001_0000);
                                else if (d == 2) z_val = q16_t'(((prng_rand_out[31:16] ^ 16'h3333) << 1) - 32'h0001_0000);
                                else             z_val = q16_t'(((prng_rand_out[15:0]  ^ 16'hCCCC) << 1) - 32'h0001_0000);
                            end else begin
                                // Pair B, mirrored: -z_B
                                z_val = -z_reg[{2'd2, 2'(d)}];
                            end

                            temp_z[d] <= z_val;
                            z_reg[{curr_k, 2'(d)}] <= z_val;
                        end else begin
                            temp_z[d] <= 32'sd0;
                            z_reg[{curr_k, 2'(d)}] <= 32'sd0;
                        end
                    end

                    prng_next <= 1'b1;
                    state     <= SMP_TRANS_Y;
                end

                // Step 2: Transform y_k = A * z_k
                SMP_TRANS_Y: begin
                    temp_y_comb = mat_vec_mul(A_mat, temp_z, dim);
                    temp_y <= temp_y_comb;
                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim) begin
                            y_reg[{curr_k, 2'(d)}] <= temp_y_comb[d];
                        end else begin
                            y_reg[{curr_k, 2'(d)}] <= 32'sd0;
                        end
                    end
                    state <= SMP_CALC_X;
                end

                // Step 3: Compute candidate vector x_k = clamp(m + σ * y_k, lb, ub)
                SMP_CALC_X: begin
                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim) begin
                            x_val = clamp_param(
                                mean_vec[d] + q16_mul(sigma, temp_y[d]),
                                lb_vec[d], ub_vec[d]
                            );
                            x_reg[{curr_k, 2'(d)}] <= x_val;
                        end else begin
                            x_reg[{curr_k, 2'(d)}] <= 32'sd0;
                        end
                    end
                    state <= SMP_NEXT_K;
                end

                // Step 4: Advance to next candidate
                SMP_NEXT_K: begin
                    if (curr_k + 1'b1 < pop_size) begin
                        curr_k <= curr_k + 1'b1;
                        state  <= SMP_GEN_Z;
                    end else begin
                        state  <= SMP_DONE;
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
