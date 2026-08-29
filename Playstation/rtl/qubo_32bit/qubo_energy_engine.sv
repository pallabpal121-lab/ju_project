// =============================================================================
// File Name   : qubo_energy_engine.sv
// Module Name : qubo_energy_engine
// Project     : QUBO / Simulated Annealing Ising Accelerator (Solver #18)
// -----------------------------------------------------------------------------
// Description:
//   Evaluates total QUBO Hamiltonian energy E(q) = q^T Q q and single-spin
//   flip energy change ΔE_k = (1 - 2*q_k) * (Q_kk + sum_{j != k} (Q_kj + Q_jk)*q_j).
// =============================================================================

`timescale 1ns / 1ps

import qubo_types_pkg::*;
`include "qubo_helpers.svh"

module qubo_energy_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Control & Inputs
    input  logic               start_total,   // Compute full energy E(q)
    input  logic               start_delta,   // Compute single-spin ΔE_k
    input  logic [3:0]         num_spins,     // Number of spins N (1..8)
    input  qubo_mat_t          q_matrix,      // 8x8 QUBO Matrix Q
    input  spin_vec_t          spin_state,    // Current 8-bit spin state q
    input  logic [2:0]         flip_idx,      // Index of spin to flip (k in 0..7)

    // Outputs
    output q16_t               energy_out,    // Total energy E(q) or ΔE_k
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        ENG_IDLE  = 2'd0,
        ENG_TOTAL = 2'd1,
        ENG_DELTA = 2'd2,
        ENG_DONE  = 2'd3
    } eng_state_t;

    eng_state_t state;

    q16_t energy_reg;
    assign energy_out = energy_reg;

    q16_t sum_e, local_field;
    logic q_i, q_j, q_k;
    q16_t q_ij, q_ji, q_kj, q_kk;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ENG_IDLE;
            energy_reg <= Q16_ZERO;
            done       <= 1'b0;
            busy       <= 1'b0;
        end else begin
            case (state)
                ENG_IDLE: begin
                    done <= 1'b0;
                    if (start_total) begin
                        busy  <= 1'b1;
                        state <= ENG_TOTAL;
                    end else if (start_delta) begin
                        busy  <= 1'b1;
                        state <= ENG_DELTA;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Full Hamiltonian Energy: E(q) = sum_i sum_j Q_ij * q_i * q_j
                ENG_TOTAL: begin
                    sum_e = Q16_ZERO;
                    for (int i = 0; i < MAX_SPINS; i++) begin
                        if (i < num_spins) begin
                            q_i = get_spin(spin_state, 3'(i));
                            if (q_i) begin
                                for (int j = 0; j < MAX_SPINS; j++) begin
                                    if (j < num_spins) begin
                                        q_j = get_spin(spin_state, 3'(j));
                                        if (q_j) begin
                                            q_ij  = get_qmat(q_matrix, 3'(i), 3'(j));
                                            sum_e = sum_e + q_ij;
                                        end
                                    end
                                end
                            end
                        end
                    end
                    energy_reg <= sum_e;
                    state      <= ENG_DONE;
                end

                // Single Spin Flip Energy Change:
                // ΔE_k = (1 - 2*q_k) * (Q_kk + sum_{j != k} (Q_kj + Q_jk)*q_j)
                ENG_DELTA: begin
                    q_k  = get_spin(spin_state, flip_idx);
                    q_kk = get_qmat(q_matrix, flip_idx, flip_idx);

                    local_field = q_kk;
                    for (int j = 0; j < MAX_SPINS; j++) begin
                        if (j < num_spins && 3'(j) != flip_idx) begin
                            q_j = get_spin(spin_state, 3'(j));
                            if (q_j) begin
                                q_kj = get_qmat(q_matrix, flip_idx, 3'(j));
                                q_ji = get_qmat(q_matrix, 3'(j), flip_idx);
                                local_field = local_field + q_kj + q_ji;
                            end
                        end
                    end

                    // If q_k == 0 (0 -> 1 flip): ΔE = +local_field
                    // If q_k == 1 (1 -> 0 flip): ΔE = -local_field
                    if (q_k == 1'b0) begin
                        energy_reg <= local_field;
                    end else begin
                        energy_reg <= -local_field;
                    end
                    state <= ENG_DONE;
                end

                ENG_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= ENG_IDLE;
                end

                default: state <= ENG_IDLE;
            endcase
        end
    end

endmodule
