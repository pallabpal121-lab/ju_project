// =============================================================================
// File Name   : rls_top.sv
// Module Name : rls_top
// Project     : Recursive Least Squares (RLS) Accelerator (Solver #23)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Controller for Real-Time Streaming RLS Adaptive Filtering.
//   Supports streaming sample updates:
//   1. Initialization: P_0 = δ * I, w_0 = w_init, precompute 1/λ
//   2. On sample_valid:
//      - Step A: rls_gain_engine computes α_t, v_t = P*x, β_t, k_t = v / β
//      - Step B: rls_update_engine updates w_t = w_{t-1} + k*α and P_t = (1/λ)(P - k*v^T)
//      - Step C: sample_done strobed, updated weights w_t available
// =============================================================================

`timescale 1ns / 1ps

import rls_types_pkg::*;
`include "rls_helpers.svh"

module rls_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Configuration & Initialization
    // -------------------------------------------------------------------------
    input  logic               init_rls,         // 1-cycle reset & initialization strobe
    input  logic [2:0]         num_taps,         // Number of filter taps N (1..4)
    input  q16_t               lambda_factor,    // Forgetting factor λ (e.g. 0.99)
    input  q16_t               delta_init,       // Initial covariance scalar P_0 = δ * I
    input  vec_t               w_init,           // Initial tap weight guess w_0

    // -------------------------------------------------------------------------
    // Interface 2: Streaming Sample Input
    // -------------------------------------------------------------------------
    input  logic               sample_valid,     // 1-cycle strobe for new streaming sample
    input  vec_t               x_sample,         // Input feature vector x_t
    input  q16_t               d_meas,           // Desired scalar measurement d_t

    // -------------------------------------------------------------------------
    // Interface 3: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               w_optimal,        // Current estimated tap weights w_t
    output mat_t               p_matrix,         // Current inverse covariance matrix P_t
    output q16_t               a_priori_err,     // A priori estimation error α_t
    output vec_t               k_gain_out,       // Current Kalman gain vector k_t
    output status_t            status,           // Status code
    output logic               sample_done,      // 1-cycle sample processed strobe
    output logic               busy              // High while processing sample
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        RLS_IDLE      = 3'd0,
        RLS_INIT_DIV  = 3'd1,
        RLS_WAIT_GAIN = 3'd2,
        RLS_WAIT_UPD  = 3'd3
    } rls_state_t;

    rls_state_t state;

    // Internal Registers
    vec_t        w_reg;
    mat_t        p_reg;
    q16_t        inv_lambda_reg;
    logic [2:0]  num_taps_reg;
    q16_t        lambda_reg;
    status_t     status_reg;
    q16_t        alpha_latched;
    vec_t        k_latched;

    // Sample Input Latch
    vec_t        x_latched;
    q16_t        d_latched;

    // Initial Divider for 1.0 / λ
    logic div_init_start;
    q16_t div_init_q;
    logic div_init_done;

    q16_divider u_init_div (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (div_init_start),
        .dividend   (32'h0001_0000),
        .divisor    (lambda_factor),
        .quotient   (div_init_q),
        .done       (div_init_done),
        .div_by_zero(),
        .busy       ()
    );

    // Sub-engine 1: rls_gain_engine
    logic start_gain;
    vec_t gain_k_out;
    vec_t gain_v_out;
    q16_t gain_alpha_out;
    logic gain_done;
    logic gain_busy;

    rls_gain_engine u_gain (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start_gain),
        .num_taps     (num_taps_reg),
        .x_sample     (x_latched),
        .d_meas       (d_latched),
        .w_curr       (w_reg),
        .p_mat        (p_reg),
        .lambda_factor(lambda_reg),
        .k_gain       (gain_k_out),
        .v_vec        (gain_v_out),
        .alpha_err    (gain_alpha_out),
        .done         (gain_done),
        .busy         (gain_busy)
    );

    // Sub-engine 2: rls_update_engine
    logic start_upd;
    vec_t upd_w_out;
    mat_t upd_p_out;
    q16_t upd_w_delta;
    logic upd_done;
    logic upd_busy;

    rls_update_engine u_upd (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start_upd),
        .num_taps  (num_taps_reg),
        .w_curr    (w_reg),
        .p_curr    (p_reg),
        .k_gain    (gain_k_out),
        .v_vec     (gain_v_out),
        .alpha_err (gain_alpha_out),
        .inv_lambda(inv_lambda_reg),
        .w_next    (upd_w_out),
        .p_next    (upd_p_out),
        .w_delta   (upd_w_delta),
        .done      (upd_done),
        .busy      (upd_busy)
    );

    // Outputs
    assign w_optimal    = w_reg;
    assign p_matrix     = p_reg;
    assign a_priori_err = alpha_latched;
    assign k_gain_out   = k_latched;
    assign status       = status_reg;

    mat_t p_init_comb;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= RLS_IDLE;
            w_reg          <= '0;
            p_reg          <= '0;
            inv_lambda_reg <= 32'h0001_0000;
            num_taps_reg   <= 3'd2;
            lambda_reg     <= 32'h0000_FE00;
            status_reg     <= STATUS_IDLE;
            alpha_latched  <= 32'h0000_0000;
            k_latched      <= '0;
            x_latched      <= '0;
            d_latched      <= 32'h0000_0000;
            div_init_start <= 1'b0;
            start_gain     <= 1'b0;
            start_upd      <= 1'b0;
            sample_done    <= 1'b0;
            busy           <= 1'b0;
        end else begin
            div_init_start <= 1'b0;
            start_gain     <= 1'b0;
            start_upd      <= 1'b0;
            sample_done    <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: RLS_IDLE - Handle Init Strobe or Streaming Sample
                // -------------------------------------------------------------
                RLS_IDLE: begin
                    if (init_rls) begin
                        busy         <= 1'b1;
                        num_taps_reg <= num_taps;
                        lambda_reg   <= (lambda_factor != 32'h0000_0000) ? lambda_factor : 32'h0000_FE00;
                        w_reg        <= w_init;

                        // Form initial diagonal covariance P_0 = δ * I
                        p_init_comb = '0;
                        for (int i = 0; i < MAX_TAPS; i++) begin
                            for (int j = 0; j < MAX_TAPS; j++) begin
                                if (i == j && i < num_taps) begin
                                    p_init_comb = set_mat(p_init_comb, 2'(i), 2'(j), (delta_init != 32'h0000_0000) ? delta_init : 32'h0064_0000);
                                end else begin
                                    p_init_comb = set_mat(p_init_comb, 2'(i), 2'(j), (i == j) ? 32'h0001_0000 : 32'h0000_0000);
                                end
                            end
                        end
                        p_reg <= p_init_comb;

                        // Start 1.0 / λ calculation
                        div_init_start <= 1'b1;
                        status_reg     <= STATUS_BUSY;
                        state          <= RLS_INIT_DIV;
                    end else if (sample_valid) begin
                        busy          <= 1'b1;
                        x_latched     <= x_sample;
                        d_latched     <= d_meas;
                        start_gain    <= 1'b1;
                        status_reg    <= STATUS_BUSY;
                        state         <= RLS_WAIT_GAIN;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: RLS_INIT_DIV - Latch 1.0 / λ
                // -------------------------------------------------------------
                RLS_INIT_DIV: begin
                    if (div_init_done) begin
                        inv_lambda_reg <= div_init_q;
                        status_reg     <= STATUS_READY;
                        busy           <= 1'b0;
                        state          <= RLS_IDLE;
                    end
                end

                // -------------------------------------------------------------
                // STATE 2: RLS_WAIT_GAIN - Wait for Kalman Gain Engine
                // -------------------------------------------------------------
                RLS_WAIT_GAIN: begin
                    if (gain_done) begin
                        alpha_latched <= gain_alpha_out;
                        k_latched     <= gain_k_out;
                        start_upd     <= 1'b1;
                        state         <= RLS_WAIT_UPD;
                    end
                end

                // -------------------------------------------------------------
                // STATE 3: RLS_WAIT_UPD - Latch Weight & Matrix Updates
                // -------------------------------------------------------------
                RLS_WAIT_UPD: begin
                    if (upd_done) begin
                        w_reg       <= upd_w_out;
                        p_reg       <= upd_p_out;
                        sample_done <= 1'b1;
                        status_reg  <= STATUS_UPDATED;
                        busy        <= 1'b0;
                        state       <= RLS_IDLE;
                    end
                end

                default: state <= RLS_IDLE;
            endcase
        end
    end

endmodule
