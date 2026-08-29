// =============================================================================
// File Name   : cg_line_search_engine.sv
// Module Name : cg_line_search_engine
// Project     : Non-Linear Conjugate Gradient (CG) Accelerator (Solver #5)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Evaluates Directional Curvature κ = dᵀ H d and computes the optimal
//   analytical step size α = -(gᵀ d) / κ along conjugate direction d.
// =============================================================================

`timescale 1ns / 1ps

import cg_types_pkg::*;
`include "cg_helpers.svh"

module cg_line_search_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Programming Interface
    input  logic               prog_en,
    input  logic [4:0]         prog_addr,
    input  instr_t             prog_data,

    // Control & Inputs
    input  logic               start,
    input  logic [2:0]         num_vars,    // Active dimension N (1..4)
    input  vec_t               x_curr,      // Current point x
    input  q16_t               f_curr,      // Center value f_0
    input  vec_t               vec_g,       // Gradient vector g
    input  vec_t               vec_d,       // Search direction d

    // Outputs
    output q16_t               step_alpha,  // Optimal step size α
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        LS_IDLE       = 3'd0,
        LS_START_FP   = 3'd1,
        LS_WAIT_FP    = 3'd2,
        LS_START_FM   = 3'd3,
        LS_WAIT_FM    = 3'd4,
        LS_CALC_KAPPA = 3'd5,
        LS_WAIT_DIV   = 3'd6,
        LS_DONE       = 3'd7
    } ls_state_t;

    ls_state_t state;

    // DFG Submodule Interconnect
    logic start_dfg;
    vec_t dfg_x_in;
    q16_t dfg_f_out;
    logic dfg_done, dfg_busy;

    // Divider Submodule
    logic div_start;
    q16_t div_dividend, div_divisor, div_quotient;
    logic div_done, div_by_zero, div_busy;

    q16_t f_plus_d, f_minus_d;
    q16_t g_proj;
    q16_t kappa;

    // Helper fixed-point multiplier
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Instantiate DFG Engine
    dfg_cg_engine u_dfg (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_dfg),
        .num_vars   (num_vars),
        .x_vec      (dfg_x_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    // Instantiate Divider
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

    vec_t tmp_x_p, tmp_x_m;
    q16_t sum_g_proj;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= LS_IDLE;
            start_dfg    <= 1'b0;
            dfg_x_in     <= '0;
            div_start    <= 1'b0;
            div_dividend <= Q16_ZERO;
            div_divisor  <= Q16_ZERO;
            f_plus_d     <= Q16_ZERO;
            f_minus_d    <= Q16_ZERO;
            g_proj       <= Q16_ZERO;
            kappa        <= Q16_ZERO;
            step_alpha   <= Q16_ONE;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            start_dfg <= 1'b0;
            div_start <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: LS_IDLE
                // -------------------------------------------------------------
                LS_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;

                        // Directional projection: g_proj = gᵀ d
                        sum_g_proj = 32'sd0;
                        for (int j = 0; j < MAX_VARS; j++) begin
                            if (j < num_vars[1:0]) begin
                                sum_g_proj = sum_g_proj + q16_mul(get_vec(vec_g, 2'(j)), get_vec(vec_d, 2'(j)));
                            end
                        end
                        g_proj <= sum_g_proj;

                        state <= LS_START_FP;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE FORWARD DIRECTIONAL PERTURBATION: f(x + h*d)
                // -------------------------------------------------------------
                LS_START_FP: begin
                    tmp_x_p = x_curr;
                    for (int j = 0; j < MAX_VARS; j++) begin
                        if (j < num_vars[1:0]) begin
                            // h*d_j = d_j >>> 4 (since h = 2^-4)
                            tmp_x_p = set_vec(tmp_x_p, 2'(j), get_vec(x_curr, 2'(j)) + (get_vec(vec_d, 2'(j)) >>> 4));
                        end
                    end
                    dfg_x_in  <= tmp_x_p;
                    start_dfg <= 1'b1;
                    state     <= LS_WAIT_FP;
                end

                LS_WAIT_FP: begin
                    if (dfg_done) begin
                        f_plus_d <= dfg_f_out;
                        state    <= LS_START_FM;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE BACKWARD DIRECTIONAL PERTURBATION: f(x - h*d)
                // -------------------------------------------------------------
                LS_START_FM: begin
                    tmp_x_m = x_curr;
                    for (int j = 0; j < MAX_VARS; j++) begin
                        if (j < num_vars[1:0]) begin
                            tmp_x_m = set_vec(tmp_x_m, 2'(j), get_vec(x_curr, 2'(j)) - (get_vec(vec_d, 2'(j)) >>> 4));
                        end
                    end
                    dfg_x_in  <= tmp_x_m;
                    start_dfg <= 1'b1;
                    state     <= LS_WAIT_FM;
                end

                LS_WAIT_FM: begin
                    if (dfg_done) begin
                        f_minus_d <= dfg_f_out;
                        state     <= LS_CALC_KAPPA;
                    end
                end

                // -------------------------------------------------------------
                // COMPUTE CURVATURE κ = (f_+d - 2f_0 + f_-d) << 8 AND LAUNCH DIV
                // -------------------------------------------------------------
                LS_CALC_KAPPA: begin
                    // κ = (f_+d - 2*f_0 + f_-d) * (1/h^2) = (f_+d - (f_0 << 1) + f_-d) <<< 8
                    kappa <= (f_plus_d - (f_curr <<< 1) + f_minus_d) <<< 8;

                    if ((f_plus_d - (f_curr <<< 1) + f_minus_d) <= 32'sd0) begin
                        // Negative or zero curvature -> fallback to standard step 0.5
                        step_alpha <= Q16_HALF;
                        state      <= LS_DONE;
                    end else begin
                        // Launch Division: α = -g_proj / κ
                        div_dividend <= -g_proj;
                        div_divisor  <= (f_plus_d - (f_curr <<< 1) + f_minus_d) <<< 8;
                        div_start    <= 1'b1;
                        state        <= LS_WAIT_DIV;
                    end
                end

                // -------------------------------------------------------------
                // WAIT FOR DIVISION: α = -(gᵀ d) / κ
                // -------------------------------------------------------------
                LS_WAIT_DIV: begin
                    if (div_done) begin
                        if (div_quotient <= 32'sd0) begin
                            step_alpha <= Q16_HALF;
                        end else begin
                            step_alpha <= div_quotient;
                        end
                        state <= LS_DONE;
                    end
                end

                // -------------------------------------------------------------
                // STATE: LS_DONE
                // -------------------------------------------------------------
                LS_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= LS_IDLE;
                end

                default: state <= LS_IDLE;
            endcase
        end
    end

endmodule
