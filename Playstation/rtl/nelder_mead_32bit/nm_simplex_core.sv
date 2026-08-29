// =============================================================================
// File Name   : nm_simplex_core.sv
// Module Name : nm_simplex_core
// Project     : Nelder-Mead Simplex Direct Search Accelerator (Solver #8)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Core Geometric State Machine for the Nelder-Mead Simplex Algorithm:
//   1. Bubble-sorts N+1 simplex vertices by fitness f_0 <= f_1 <= ... <= f_N.
//   2. Computes centroid x_c = (1/N) * sum_{i=0}^{N-1} x_i.
//   3. Reflection   : x_r  = 2*x_c - x_N
//   4. Expansion    : x_e  = x_c + 2*(x_r - x_c) = 2*x_r - x_c
//   5. Contraction  : x_oc = 0.5*(x_c + x_r) OR x_ic = 0.5*(x_c + x_N)
//   6. Shrink       : x_i  = 0.5*(x_0 + x_i) for i = 1..N
// =============================================================================

`timescale 1ns / 1ps

import nm_types_pkg::*;
`include "nm_helpers.svh"

module nm_simplex_core (
    input  logic               clk,
    input  logic               rst_n,

    // Programming Interface (for DFG evaluator)
    input  logic               prog_en,
    input  logic [4:0]         prog_addr,
    input  instr_t             prog_data,

    // Control & Inputs
    input  logic               start_iter,
    input  logic [2:0]         num_params,    // Dimension N (1..4)
    input  simplex_vec_t       simplex_in,    // Current N+1 vertices
    input  simplex_f_t         fitness_in,    // Current N+1 function values

    // Outputs
    output simplex_vec_t       simplex_out,   // Updated N+1 vertices
    output simplex_f_t         fitness_out,   // Updated N+1 function values
    output q16_t               f_best,        // Best objective value f_0
    output vec_t               x_best,        // Best parameter vector x_0
    output q16_t               simplex_radius,// Max coordinate distance ||x_i - x_0||_inf
    output q16_t               f_span,        // Function spread f_N - f_0
    output logic               iter_done,
    output logic               busy
);

    typedef enum logic [4:0] {
        CORE_IDLE        = 5'd0,
        CORE_SORT        = 5'd1,
        CORE_CENTROID    = 5'd2,
        CORE_START_REFL  = 5'd3,
        CORE_WAIT_REFL   = 5'd4,
        CORE_EVAL_REFL   = 5'd5,
        CORE_START_EXP   = 5'd6,
        CORE_WAIT_EXP    = 5'd7,
        CORE_START_CONT  = 5'd8,
        CORE_WAIT_CONT   = 5'd9,
        CORE_START_SHRINK= 5'd10,
        CORE_WAIT_SHRINK = 5'd11,
        CORE_METRICS     = 5'd12,
        CORE_DONE        = 5'd13
    } core_state_t;

    core_state_t state;

    // Simplex Registers
    simplex_vec_t s_vec;
    simplex_f_t   s_f;

    // DFG Interconnect
    logic start_dfg;
    vec_t dfg_x_in;
    q16_t dfg_f_out;
    logic dfg_done, dfg_busy;

    // Instantiate DFG Objective Evaluator
    dfg_nm_engine u_dfg (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_dfg),
        .num_params (num_params),
        .x_vec      (dfg_x_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    vec_t x_c;    // Centroid vector
    vec_t x_r;    // Reflection vector
    q16_t f_r;    // Reflection fitness
    vec_t x_e;    // Expansion vector
    vec_t x_cont; // Contraction vector
    logic [2:0] shrink_idx;

    logic [2:0] num_vert;
    assign num_vert = num_params + 1'b1; // M = N + 1

    logic [2:0] sort_i, sort_j;
    logic       inside_cont;

    assign simplex_out = s_vec;
    assign fitness_out = s_f;
    assign x_best      = get_s_vec(s_vec, 3'd0);
    assign f_best      = get_s_f(s_f, 3'd0);

    q16_t sum_coord;
    vec_t vi, vj, v0;
    q16_t fi, fj;
    q16_t max_rad, cur_dist, cur_diff;
    vec_t tmp_xc, tmp_xr, tmp_xe, tmp_xcont, tmp_v;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= CORE_IDLE;
            s_vec          <= '0;
            s_f            <= '0;
            x_c            <= '0;
            x_r            <= '0;
            f_r            <= Q16_ZERO;
            x_e            <= '0;
            x_cont         <= '0;
            inside_cont    <= 1'b0;
            shrink_idx     <= '0;
            sort_i         <= '0;
            sort_j         <= '0;
            start_dfg      <= 1'b0;
            dfg_x_in       <= '0;
            simplex_radius <= Q16_ZERO;
            f_span         <= Q16_ZERO;
            iter_done      <= 1'b0;
            busy           <= 1'b0;
        end else begin
            start_dfg <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: CORE_IDLE
                // -------------------------------------------------------------
                CORE_IDLE: begin
                    iter_done <= 1'b0;
                    if (start_iter) begin
                        busy   <= 1'b1;
                        s_vec  <= simplex_in;
                        s_f    <= fitness_in;
                        sort_i <= 3'd0;
                        sort_j <= 3'd0;
                        state  <= CORE_SORT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // 1. SORT VERTICES BY FITNESS (f_0 <= f_1 <= ... <= f_N)
                // -------------------------------------------------------------
                CORE_SORT: begin
                    if (sort_i < num_vert) begin
                        if (sort_j + 1'b1 < num_vert) begin
                            fi = get_s_f(s_f, sort_j);
                            fj = get_s_f(s_f, sort_j + 1'b1);
                            if (fi > fj) begin
                                // Swap fitness
                                s_f <= set_s_f(set_s_f(s_f, sort_j, fj), sort_j + 1'b1, fi);
                                // Swap vertex coordinates
                                vi  = get_s_vec(s_vec, sort_j);
                                vj  = get_s_vec(s_vec, sort_j + 1'b1);
                                s_vec <= set_s_vec(set_s_vec(s_vec, sort_j, vj), sort_j + 1'b1, vi);
                            end
                            sort_j <= sort_j + 1'b1;
                        end else begin
                            sort_j <= 3'd0;
                            sort_i <= sort_i + 1'b1;
                        end
                    end else begin
                        state <= CORE_CENTROID;
                    end
                end

                // -------------------------------------------------------------
                // 2. COMPUTE CENTROID: x_c = (1/N) * sum_{i=0}^{N-1} x_i
                // -------------------------------------------------------------
                CORE_CENTROID: begin
                    tmp_xc = x_c;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params) begin
                            sum_coord = 32'sd0;
                            for (int i = 0; i < MAX_VERTICES; i++) begin
                                if (i < num_params) begin // Excludes worst vertex x_N
                                    sum_coord = sum_coord + get_vec(get_s_vec(s_vec, 3'(i)), 2'(k));
                                end
                            end
                            // Division by N
                            case (num_params)
                                3'd1: tmp_xc = set_vec(tmp_xc, 2'(k), sum_coord);
                                3'd2: tmp_xc = set_vec(tmp_xc, 2'(k), sum_coord >>> 1); // /2
                                3'd3: tmp_xc = set_vec(tmp_xc, 2'(k), q16_t'((64'(sum_coord) * 64'(32'h0000_5555)) >>> 16)); // /3 ≈ * 0.33333
                                3'd4: tmp_xc = set_vec(tmp_xc, 2'(k), sum_coord >>> 2); // /4
                                default: tmp_xc = set_vec(tmp_xc, 2'(k), sum_coord >>> 1);
                            endcase
                        end
                    end
                    x_c   <= tmp_xc;
                    state <= CORE_START_REFL;
                end

                // -------------------------------------------------------------
                // 3. REFLECTION: x_r = 2*x_c - x_N
                // -------------------------------------------------------------
                CORE_START_REFL: begin
                    tmp_xr = x_r;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params) begin
                            tmp_xr = set_vec(tmp_xr, 2'(k), (get_vec(x_c, 2'(k)) <<< 1) - get_vec(get_s_vec(s_vec, num_params), 2'(k)));
                        end
                    end
                    x_r   <= tmp_xr;
                    state <= CORE_WAIT_REFL;
                end

                CORE_WAIT_REFL: begin
                    dfg_x_in  <= x_r;
                    start_dfg <= 1'b1;
                    state     <= CORE_EVAL_REFL;
                end

                CORE_EVAL_REFL: begin
                    if (dfg_done) begin
                        f_r <= dfg_f_out;

                        // Case A: Reflection is better than best (f_r < f_0) -> Try Expansion
                        if (dfg_f_out < get_s_f(s_f, 3'd0)) begin
                            tmp_xe = x_e;
                            for (int k = 0; k < MAX_PARAMS; k++) begin
                                if (k < num_params) begin
                                    // x_e = x_c + 2*(x_r - x_c) = 2*x_r - x_c
                                    tmp_xe = set_vec(tmp_xe, 2'(k), (get_vec(x_r, 2'(k)) <<< 1) - get_vec(x_c, 2'(k)));
                                end
                            end
                            x_e   <= tmp_xe;
                            state <= CORE_START_EXP;

                        // Case B: f_0 <= f_r < f_{N-1} (Standard Step) -> Accept Reflection
                        end else if (dfg_f_out < get_s_f(s_f, num_params - 1'b1)) begin
                            s_vec <= set_s_vec(s_vec, num_params, x_r);
                            s_f   <= set_s_f(s_f, num_params, dfg_f_out);
                            state <= CORE_METRICS;

                        // Case C: f_r >= f_{N-1} -> Contraction
                        end else begin
                            tmp_xcont = x_cont;
                            if (dfg_f_out < get_s_f(s_f, num_params)) begin
                                // Outside Contraction: x_oc = 0.5*(x_c + x_r)
                                inside_cont <= 1'b0;
                                for (int k = 0; k < MAX_PARAMS; k++) begin
                                    if (k < num_params) begin
                                        tmp_xcont = set_vec(tmp_xcont, 2'(k), (get_vec(x_c, 2'(k)) + get_vec(x_r, 2'(k))) >>> 1);
                                    end
                                end
                            end else begin
                                // Inside Contraction: x_ic = 0.5*(x_c + x_N)
                                inside_cont <= 1'b1;
                                for (int k = 0; k < MAX_PARAMS; k++) begin
                                    if (k < num_params) begin
                                        tmp_xcont = set_vec(tmp_xcont, 2'(k), (get_vec(x_c, 2'(k)) + get_vec(get_s_vec(s_vec, num_params), 2'(k))) >>> 1);
                                    end
                                end
                            end
                            x_cont <= tmp_xcont;
                            state  <= CORE_START_CONT;
                        end
                    end
                end

                // -------------------------------------------------------------
                // 4. EXPANSION: x_e
                // -------------------------------------------------------------
                CORE_START_EXP: begin
                    dfg_x_in  <= x_e;
                    start_dfg <= 1'b1;
                    state     <= CORE_WAIT_EXP;
                end

                CORE_WAIT_EXP: begin
                    if (dfg_done) begin
                        if (dfg_f_out < f_r) begin
                            // Accept expansion
                            s_vec <= set_s_vec(s_vec, num_params, x_e);
                            s_f   <= set_s_f(s_f, num_params, dfg_f_out);
                        end else begin
                            // Accept reflection
                            s_vec <= set_s_vec(s_vec, num_params, x_r);
                            s_f   <= set_s_f(s_f, num_params, f_r);
                        end
                        state <= CORE_METRICS;
                    end
                end

                // -------------------------------------------------------------
                // 5. CONTRACTION: x_oc / x_ic
                // -------------------------------------------------------------
                CORE_START_CONT: begin
                    dfg_x_in  <= x_cont;
                    start_dfg <= 1'b1;
                    state     <= CORE_WAIT_CONT;
                end

                CORE_WAIT_CONT: begin
                    if (dfg_done) begin
                        if (!inside_cont && dfg_f_out <= f_r) begin
                            // Accept Outside Contraction
                            s_vec <= set_s_vec(s_vec, num_params, x_cont);
                            s_f   <= set_s_f(s_f, num_params, dfg_f_out);
                            state <= CORE_METRICS;
                        end else if (inside_cont && dfg_f_out < get_s_f(s_f, num_params)) begin
                            // Accept Inside Contraction
                            s_vec <= set_s_vec(s_vec, num_params, x_cont);
                            s_f   <= set_s_f(s_f, num_params, dfg_f_out);
                            state <= CORE_METRICS;
                        end else begin
                            // Contraction failed -> Shrink
                            shrink_idx <= 3'd1;
                            state      <= CORE_START_SHRINK;
                        end
                    end
                end

                // -------------------------------------------------------------
                // 6. SHRINK: x_i = 0.5*(x_0 + x_i) for i = 1..N
                // -------------------------------------------------------------
                CORE_START_SHRINK: begin
                    if (shrink_idx < num_vert) begin
                        tmp_v = get_s_vec(s_vec, shrink_idx);
                        v0    = get_s_vec(s_vec, 3'd0);
                        for (int k = 0; k < MAX_PARAMS; k++) begin
                            if (k < num_params) begin
                                tmp_v = set_vec(tmp_v, 2'(k), (get_vec(v0, 2'(k)) + get_vec(tmp_v, 2'(k))) >>> 1);
                            end
                        end
                        s_vec     <= set_s_vec(s_vec, shrink_idx, tmp_v);
                        dfg_x_in  <= tmp_v;
                        start_dfg <= 1'b1;
                        state     <= CORE_WAIT_SHRINK;
                    end else begin
                        state <= CORE_METRICS;
                    end
                end

                CORE_WAIT_SHRINK: begin
                    if (dfg_done) begin
                        s_f        <= set_s_f(s_f, shrink_idx, dfg_f_out);
                        shrink_idx <= shrink_idx + 1'b1;
                        state      <= CORE_START_SHRINK;
                    end
                end

                // -------------------------------------------------------------
                // 7. COMPUTE CONVERGENCE METRICS
                // -------------------------------------------------------------
                CORE_METRICS: begin
                    // Simplex radius: max_{i=1..N} max_k |x_{i, k} - x_{0, k}|
                    max_rad = 32'sd0;
                    v0      = get_s_vec(s_vec, 3'd0);

                    for (int i = 1; i < MAX_VERTICES; i++) begin
                        if (i < num_vert) begin
                            vi = get_s_vec(s_vec, 3'(i));
                            for (int k = 0; k < MAX_PARAMS; k++) begin
                                if (k < num_params) begin
                                    cur_diff = get_vec(vi, 2'(k)) - get_vec(v0, 2'(k));
                                    cur_dist = (cur_diff < 32'sd0) ? -cur_diff : cur_diff;
                                    if (cur_dist > max_rad) max_rad = cur_dist;
                                end
                            end
                        end
                    end
                    simplex_radius <= max_rad;

                    // Function spread: |f_N - f_0|
                    cur_diff = get_s_f(s_f, num_params) - get_s_f(s_f, 3'd0);
                    f_span   <= (cur_diff < 32'sd0) ? -cur_diff : cur_diff;

                    state <= CORE_DONE;
                end

                // -------------------------------------------------------------
                // STATE: CORE_DONE
                // -------------------------------------------------------------
                CORE_DONE: begin
                    iter_done <= 1'b1;
                    busy      <= 1'b0;
                    state     <= CORE_IDLE;
                end

                default: state <= CORE_IDLE;
            endcase
        end
    end

endmodule
