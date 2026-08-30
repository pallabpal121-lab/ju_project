// =============================================================================
// File Name   : rls_update_engine.sv
// Module Name : rls_update_engine
// Project     : Recursive Least Squares (RLS) Accelerator (Solver #23)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Weight & Covariance Update Engine for RLS.
//   1. Weight Update:     w_t = w_{t-1} + k_t * α_t
//   2. Covariance Update: P_t = (1/λ) * (P_{t-1} - k_t * v_t^T)
//   3. Symmetric regularization: P_t = 0.5 * (P_t + P_t^T)
// =============================================================================

`timescale 1ns / 1ps

import rls_types_pkg::*;
`include "rls_helpers.svh"

module rls_update_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [2:0]         num_taps,         // Number of filter taps N (1..4)
    input  vec_t               w_curr,           // Current tap weights w_{t-1}
    input  mat_t               p_curr,           // Current inverse covariance P_{t-1}
    input  vec_t               k_gain,           // Kalman gain vector k_t
    input  vec_t               v_vec,            // Intermediate vector v_t
    input  q16_t               alpha_err,        // A priori error α_t
    input  q16_t               inv_lambda,       // Pre-calculated 1.0 / λ (Q16.16)

    output vec_t               w_next,           // Updated tap weights w_t
    output mat_t               p_next,           // Updated inverse covariance P_t
    output q16_t               w_delta,          // Weight change norm ||k * α||_inf
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        UPD_IDLE = 2'd0,
        UPD_CALC = 2'd1,
        UPD_DONE = 2'd2
    } upd_state_t;

    upd_state_t state;

    vec_t w_reg;
    mat_t p_reg;
    mat_t p_raw;
    mat_t p_comb;
    q16_t max_dw;

    assign w_next  = w_reg;
    assign p_next  = p_reg;
    assign w_delta = max_dw;

    q16_t k_alpha, abs_dw;
    q16_t p_elem, kv_elem, diff_elem, raw_val;
    q16_t p_ij, p_ji, sym_val;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= UPD_IDLE;
            w_reg  <= '0;
            p_reg  <= '0;
            p_raw  <= '0;
            p_comb <= '0;
            max_dw <= Q16_ZERO;
            done   <= 1'b0;
            busy   <= 1'b0;
        end else begin
            case (state)
                UPD_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= UPD_CALC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                UPD_CALC: begin
                    // 1. Weight Update: w_i = w_i + k_i * α
                    max_dw = Q16_ZERO;
                    for (int i = 0; i < MAX_TAPS; i++) begin
                        if (i < num_taps) begin
                            k_alpha  = q16_mul(k_gain[i], alpha_err);
                            w_reg[i] <= w_curr[i] + k_alpha;
                            abs_dw   = q16_abs(k_alpha);
                            if (abs_dw > max_dw) begin
                                max_dw = abs_dw;
                            end
                        end else begin
                            w_reg[i] <= Q16_ZERO;
                        end
                    end

                    // 2. Covariance Update: P_raw = (1/λ) * (P - k * v^T)
                    p_raw = '0;
                    for (int i = 0; i < MAX_TAPS; i++) begin
                        for (int j = 0; j < MAX_TAPS; j++) begin
                            if (i < num_taps && j < num_taps) begin
                                p_elem    = get_mat(p_curr, 2'(i), 2'(j));
                                kv_elem   = q16_mul(k_gain[i], v_vec[j]);
                                diff_elem = p_elem - kv_elem;
                                raw_val   = q16_mul(diff_elem, inv_lambda);
                                p_raw     = set_mat(p_raw, 2'(i), 2'(j), raw_val);
                            end else begin
                                p_raw     = set_mat(p_raw, 2'(i), 2'(j), (i == j) ? 32'h0001_0000 : 32'h0000_0000);
                            end
                        end
                    end

                    // 3. Symmetrize P = 0.5 * (P_raw + P_raw^T)
                    p_comb = '0;
                    for (int i = 0; i < MAX_TAPS; i++) begin
                        for (int j = 0; j < MAX_TAPS; j++) begin
                            if (i < num_taps && j < num_taps) begin
                                p_ij    = get_mat(p_raw, 2'(i), 2'(j));
                                p_ji    = get_mat(p_raw, 2'(j), 2'(i));
                                sym_val = q16_mul(32'h0000_8000, p_ij + p_ji);
                                p_comb  = set_mat(p_comb, 2'(i), 2'(j), sym_val);
                            end else begin
                                p_comb  = set_mat(p_comb, 2'(i), 2'(j), (i == j) ? 32'h0001_0000 : 32'h0000_0000);
                            end
                        end
                    end
                    p_reg <= p_comb;

                    state <= UPD_DONE;
                end

                UPD_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= UPD_IDLE;
                end

                default: state <= UPD_IDLE;
            endcase
        end
    end

endmodule
