// =============================================================================
// File Name   : qubo_exp_unit.sv
// Module Name : qubo_exp_unit
// Project     : QUBO / Simulated Annealing Ising Accelerator (Solver #18)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Boltzmann Exponential Probability Evaluator P = exp(-u) in Q16.16.
//   Uses base-2 transformation: exp(-u) = 2^(-u * log2(e)) = (2^-k) * (2^-f)
//   where k = int_part, f = frac_part.
//   Evaluates 2^-f via second-order Taylor polynomial: 1 - ln(2)*f + 0.5*ln(2)^2*f^2.
// =============================================================================

`timescale 1ns / 1ps

import qubo_types_pkg::*;
`include "qubo_helpers.svh"

module qubo_exp_unit (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  q16_t        u_val,       // Exponent u = ΔE / T (must be >= 0)
    output q16_t        prob_out,    // P = exp(-u) in [0.0, 1.0] Q16.16
    output logic        done,
    output logic        busy
);

    typedef enum logic [1:0] {
        EXP_IDLE  = 2'd0,
        EXP_CALC1 = 2'd1,
        EXP_CALC2 = 2'd2,
        EXP_DONE  = 2'd3
    } exp_state_t;

    exp_state_t state;

    q16_t w_val;
    logic [4:0] int_k;
    q16_t frac_f;
    q16_t p_frac;
    q16_t f_sq;
    q16_t prob_reg;

    assign prob_out = prob_reg;

    localparam q16_t C_LN2     = 32'h0000_B172; // ln(2) ≈ 0.693147
    localparam q16_t C_HALF_L2 = 32'h0000_3D80; // 0.5 * ln(2)^2 ≈ 0.240226

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= EXP_IDLE;
            w_val    <= Q16_ZERO;
            int_k    <= 5'd0;
            frac_f   <= Q16_ZERO;
            p_frac   <= Q16_ZERO;
            f_sq     <= Q16_ZERO;
            prob_reg <= Q16_ZERO;
            done     <= 1'b0;
            busy     <= 1'b0;
        end else begin
            case (state)
                EXP_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        if (u_val <= Q16_ZERO) begin
                            // exp(0) = 1.0
                            prob_reg <= Q16_ONE;
                            done     <= 1'b1;
                            busy     <= 1'b0;
                            state    <= EXP_IDLE;
                        end else if (u_val >= 32'h0010_0000) begin
                            // u >= 16.0 -> exp(-16) ≈ 10^-7 ≈ 0
                            prob_reg <= Q16_ZERO;
                            done     <= 1'b1;
                            busy     <= 1'b0;
                            state    <= EXP_IDLE;
                        end else begin
                            // w = u * log2(e)
                            w_val  <= q16_mul(u_val, Q16_LOG2_E);
                            state  <= EXP_CALC1;
                        end
                    end else begin
                        busy <= 1'b0;
                    end
                end

                EXP_CALC1: begin
                    int_k  <= (w_val[31:16] > 16) ? 5'd16 : w_val[20:16];
                    frac_f <= {16'h0000, w_val[15:0]};
                    f_sq   <= q16_mul({16'h0000, w_val[15:0]}, {16'h0000, w_val[15:0]});
                    state  <= EXP_CALC2;
                end

                EXP_CALC2: begin
                    // 2^-f ≈ 1.0 - ln(2)*f + 0.240226*f^2
                    p_frac   = Q16_ONE - q16_mul(C_LN2, frac_f) + q16_mul(C_HALF_L2, f_sq);
                    prob_reg <= (int_k >= 16) ? Q16_ZERO : (p_frac >> int_k);
                    state    <= EXP_DONE;
                end

                EXP_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= EXP_IDLE;
                end

                default: state <= EXP_IDLE;
            endcase
        end
    end

endmodule
