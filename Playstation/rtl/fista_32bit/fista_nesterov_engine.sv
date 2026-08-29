// =============================================================================
// File Name   : fista_nesterov_engine.sv
// Module Name : fista_nesterov_engine
// Project     : Fast Iterative Shrinkage-Thresholding Algorithm (FISTA) Accelerator (Solver #15)
// -----------------------------------------------------------------------------
// Description:
//   Evaluates the Nesterov acceleration momentum scalar update and vector
//   extrapolation:
//   1. t_{k+1} = (1 + sqrt(1 + 4 * t_k^2)) / 2
//   2. β_k = (t_k - 1) / t_{k+1}
//   3. y_{k+1} = x_k + β_k * (x_k - x_{k-1})
// =============================================================================

`timescale 1ns / 1ps

import fista_types_pkg::*;
`include "fista_helpers.svh"

module fista_nesterov_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Control & Inputs
    input  logic               start,
    input  logic [2:0]         num_dims,      // Number of dimensions N (1..4)
    input  q16_t               t_curr,        // Current Nesterov scalar t_k
    input  vec_t               x_curr,        // Current iterate x_k
    input  vec_t               x_prev,        // Previous iterate x_{k-1}

    // Outputs
    output q16_t               t_next,        // Updated scalar t_{k+1}
    output q16_t               beta_out,      // Extrapolation weight β_k
    output vec_t               y_next,        // Extrapolated momentum vector y_{k+1}
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        NEST_IDLE        = 3'd0,
        NEST_START_SQRT  = 3'd1,
        NEST_WAIT_SQRT   = 3'd2,
        NEST_START_DIV   = 3'd3,
        NEST_WAIT_DIV    = 3'd4,
        NEST_CALC_EXTRAP = 3'd5,
        NEST_DONE        = 3'd6
    } nest_state_t;

    nest_state_t state;

    q16_t t_next_reg;
    q16_t beta_reg;
    vec_t y_next_reg;

    assign t_next   = t_next_reg;
    assign beta_out = beta_reg;
    assign y_next   = y_next_reg;

    // Hardware Sqrt Unit
    logic sqrt_start;
    q16_t sqrt_val, sqrt_root;
    logic sqrt_done, sqrt_busy;

    q16_sqrt u_sqrt (
        .clk   (clk),
        .rst_n (rst_n),
        .start (sqrt_start),
        .val   (sqrt_val),
        .root  (sqrt_root),
        .done  (sqrt_done),
        .busy  (sqrt_busy)
    );

    // Hardware Divider Unit
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

    q16_t t_sq_4;
    q16_t x_k, x_km1, diff_x, extrap_val;
    vec_t temp_y;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= NEST_IDLE;
            t_next_reg   <= Q16_ONE;
            beta_reg     <= Q16_ZERO;
            y_next_reg   <= '0;
            sqrt_start   <= 1'b0;
            sqrt_val     <= Q16_ZERO;
            div_start    <= 1'b0;
            div_dividend <= Q16_ZERO;
            div_divisor  <= Q16_ZERO;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            sqrt_start <= 1'b0;
            div_start  <= 1'b0;

            case (state)
                NEST_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        // Radicand = 1.0 + 4 * t_k^2
                        t_sq_4     = q16_mul(t_curr, t_curr) <<< 2;
                        sqrt_val   <= Q16_ONE + t_sq_4;
                        sqrt_start <= 1'b1;
                        state      <= NEST_WAIT_SQRT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                NEST_WAIT_SQRT: begin
                    if (sqrt_done) begin
                        // t_{k+1} = (1.0 + sqrt_root) / 2
                        t_next_reg <= (Q16_ONE + sqrt_root) >>> 1;

                        // β_k = (t_k - 1.0) / t_{k+1}
                        if (t_curr <= Q16_ONE) begin
                            beta_reg <= Q16_ZERO;
                            state    <= NEST_CALC_EXTRAP;
                        end else begin
                            div_dividend <= t_curr - Q16_ONE;
                            div_divisor  <= (Q16_ONE + sqrt_root) >>> 1;
                            div_start    <= 1'b1;
                            state        <= NEST_WAIT_DIV;
                        end
                    end
                end

                NEST_WAIT_DIV: begin
                    if (div_done) begin
                        beta_reg <= (div_by_zero) ? Q16_ZERO : div_quotient;
                        state    <= NEST_CALC_EXTRAP;
                    end
                end

                NEST_CALC_EXTRAP: begin
                    temp_y = '0;
                    for (int d = 0; d < MAX_PARAMS; d++) begin
                        if (d < num_dims) begin
                            x_k        = get_vec(x_curr, 2'(d));
                            x_km1      = get_vec(x_prev, 2'(d));
                            diff_x     = x_k - x_km1;
                            extrap_val = x_k + q16_mul(beta_reg, diff_x);
                            temp_y     = set_vec(temp_y, 2'(d), extrap_val);
                        end
                    end
                    y_next_reg <= temp_y;
                    state      <= NEST_DONE;
                end

                NEST_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= NEST_IDLE;
                end

                default: state <= NEST_IDLE;
            endcase
        end
    end

endmodule
