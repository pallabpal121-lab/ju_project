// =============================================================================
// File Name   : lasso_coordinate_engine.sv
// Module Name : lasso_coordinate_engine
// Project     : Coordinate Descent / LASSO L1 Sparsity Accelerator (Solver #10)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Executes a single Coordinate Descent step on feature coordinate j:
//   1. Computes partial residuals: r_m^{(j)} = r_m + X_{m, j} * w_j
//   2. Accumulates correlation z_j = sum_{m=1}^M X_{m, j} * r_m^{(j)}
//   3. Accumulates column energy c_j = sum_{m=1}^M X_{m, j}^2
//   4. Applies Hardware Soft-Thresholding: S = S_lambda(z_j)
//   5. Solves new coordinate weight: w_j^{new} = S / c_j
//   6. Updates full residual: r_m^{new} = r_m - X_{m, j} * (w_j^{new} - w_j)
// =============================================================================

`timescale 1ns / 1ps

import lasso_types_pkg::*;
`include "lasso_helpers.svh"

module lasso_coordinate_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Control & Inputs
    input  logic               start_coord,
    input  logic [1:0]         coord_idx,     // Coordinate j (0..3)
    input  logic [3:0]         num_obs,       // Number of samples M (1..8)
    input  dataset_mat_t       x_matrix,      // Dataset matrix X (M x N)
    input  obs_vec_t           r_in,          // Current residual vector r
    input  q16_t               w_j_in,        // Current weight w_j
    input  q16_t               lambda_reg,    // Regularization parameter lambda

    // Outputs
    output q16_t               w_j_out,       // Updated weight w_j^{new}
    output obs_vec_t           r_out,         // Updated residual vector r^{new}
    output q16_t               delta_w_abs,   // Absolute coordinate shift |w_j^{new} - w_j|
    output logic               coord_done,
    output logic               busy
);

    typedef enum logic [2:0] {
        COORD_IDLE      = 3'd0,
        COORD_ACCUM     = 3'd1,
        COORD_THRESHOLD = 3'd2,
        COORD_DIV_START = 3'd3,
        COORD_DIV_WAIT  = 3'd4,
        COORD_RESIDUAL  = 3'd5,
        COORD_DONE      = 3'd6
    } coord_state_t;

    coord_state_t state;

    logic [3:0] obs_idx;
    logic signed [63:0] z_accum_64;
    logic signed [63:0] c_accum_64;

    q16_t z_val;
    q16_t c_val;
    q16_t s_thresh_val;
    logic is_exact_zero;

    q16_t w_new_val;
    q16_t delta_w_val;
    obs_vec_t r_accum;

    // Soft-Thresholding Unit
    q16_soft_threshold u_thresh (
        .z_in     (z_val),
        .lambda_in(lambda_reg),
        .s_out    (s_thresh_val),
        .is_zero  (is_exact_zero)
    );

    // Divider for w_j = S / c_j
    logic div_start;
    q16_t div_dividend, div_divisor, div_quotient;
    logic div_done, div_by_zero, div_busy;

    q16_divider u_div (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (div_start),
        .dividend   (div_dividend),
        .divisor    (div_divisor),
        .quotient   (div_quotient),
        .done       (div_done),
        .div_by_zero(div_by_zero),
        .busy       (div_busy)
    );

    q16_t x_mj;
    q16_t r_m;
    q16_t r_partial_m;
    logic signed [63:0] prod_zr, prod_c;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= COORD_IDLE;
            obs_idx      <= 4'd0;
            z_accum_64   <= 64'sd0;
            c_accum_64   <= 64'sd0;
            z_val        <= Q16_ZERO;
            c_val        <= Q16_ZERO;
            w_new_val    <= Q16_ZERO;
            delta_w_val  <= Q16_ZERO;
            r_accum      <= '0;
            w_j_out      <= Q16_ZERO;
            r_out        <= '0;
            delta_w_abs  <= Q16_ZERO;
            coord_done   <= 1'b0;
            busy         <= 1'b0;
            div_start    <= 1'b0;
            div_dividend <= Q16_ZERO;
            div_divisor  <= Q16_ZERO;
        end else begin
            div_start <= 1'b0;

            case (state)
                COORD_IDLE: begin
                    coord_done <= 1'b0;
                    if (start_coord) begin
                        busy       <= 1'b1;
                        obs_idx    <= 4'd0;
                        z_accum_64 <= 64'sd0;
                        c_accum_64 <= 64'sd0;
                        r_accum    <= r_in;
                        state      <= COORD_ACCUM;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // 1. ACCUMULATE z_j = sum X_{m,j} * r_m^{(j)} AND c_j = sum X_{m,j}^2
                // -------------------------------------------------------------
                COORD_ACCUM: begin
                    if (obs_idx < num_obs) begin
                        x_mj = get_mat_elem(x_matrix, 3'(obs_idx[2:0]), coord_idx);
                        r_m  = get_obs(r_in, 3'(obs_idx[2:0]));

                        // r_partial_m = r_m + X_{m,j} * w_j
                        r_partial_m = r_m + q16_t'((64'(x_mj) * 64'(w_j_in)) >>> 16);

                        // Accumulate z_j
                        prod_zr    = 64'(x_mj) * 64'(r_partial_m);
                        z_accum_64 <= z_accum_64 + prod_zr;

                        // Accumulate c_j
                        prod_c     = 64'(x_mj) * 64'(x_mj);
                        c_accum_64 <= c_accum_64 + prod_c;

                        obs_idx <= obs_idx + 1'b1;
                    end else begin
                        z_val <= q16_t'(z_accum_64 >>> 16);
                        c_val <= q16_t'(c_accum_64 >>> 16);
                        state <= COORD_THRESHOLD;
                    end
                end

                // -------------------------------------------------------------
                // 2. SOFT-THRESHOLDING S_lambda(z_j)
                // -------------------------------------------------------------
                COORD_THRESHOLD: begin
                    if (is_exact_zero || c_val == Q16_ZERO) begin
                        // Sparse zero coefficient!
                        w_new_val   <= Q16_ZERO;
                        delta_w_val <= Q16_ZERO - w_j_in;
                        obs_idx     <= 4'd0;
                        state       <= COORD_RESIDUAL;
                    end else begin
                        // Proceed to division w_new = S / c_j
                        div_dividend <= s_thresh_val;
                        div_divisor  <= c_val;
                        div_start    <= 1'b1;
                        state        <= COORD_DIV_WAIT;
                    end
                end

                COORD_DIV_WAIT: begin
                    if (div_done) begin
                        w_new_val   <= div_quotient;
                        delta_w_val <= div_quotient - w_j_in;
                        obs_idx     <= 4'd0;
                        state       <= COORD_RESIDUAL;
                    end
                end

                // -------------------------------------------------------------
                // 3. UPDATE FULL RESIDUAL: r_m^{new} = r_m - X_{m,j} * delta_w
                // -------------------------------------------------------------
                COORD_RESIDUAL: begin
                    if (obs_idx < num_obs) begin
                        x_mj = get_mat_elem(x_matrix, 3'(obs_idx[2:0]), coord_idx);
                        r_m  = get_obs(r_accum, 3'(obs_idx[2:0]));

                        // r_m^{new} = r_m - X_{m,j} * delta_w
                        r_accum <= set_obs(r_accum, 3'(obs_idx[2:0]), r_m - q16_t'((64'(x_mj) * 64'(delta_w_val)) >>> 16));
                        obs_idx <= obs_idx + 1'b1;
                    end else begin
                        state <= COORD_DONE;
                    end
                end

                // -------------------------------------------------------------
                // 4. DONE
                // -------------------------------------------------------------
                COORD_DONE: begin
                    w_j_out     <= w_new_val;
                    r_out       <= r_accum;
                    delta_w_abs <= (delta_w_val < 32'sd0) ? -delta_w_val : delta_w_val;
                    coord_done  <= 1'b1;
                    busy        <= 1'b0;
                    state       <= COORD_IDLE;
                end

                default: state <= COORD_IDLE;
            endcase
        end
    end

endmodule
