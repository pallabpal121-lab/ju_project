// =============================================================================
// File Name   : nelder_mead_top.sv
// Module Name : nelder_mead_top
// Project     : Nelder-Mead Simplex Direct Search Accelerator (Solver #8)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Top-level SoC Module for the Nelder-Mead Derivative-Free Accelerator.
//   Constructs the initial geometric simplex around initial guess x_init,
//   evaluates vertex fitnesses, and executes the simplex transformation loop
//   until the simplex contracts to the global minimum.
// =============================================================================

`timescale 1ns / 1ps

import nm_types_pkg::*;
`include "nm_helpers.svh"

module nelder_mead_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Model Microcode Programming Port
    // -------------------------------------------------------------------------
    input  logic               prog_en,       // Microcode Write Enable
    input  logic [4:0]         prog_addr,     // Microcode Address (0..31)
    input  instr_t             prog_data,     // 32-bit Microcode Instruction

    // -------------------------------------------------------------------------
    // Interface 2: Optimization Controls & Parameters
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start pulse
    input  logic [2:0]         num_params,    // Dimension N (1..4)
    input  vec_t               x_init,        // Starting initial guess x_0
    input  q16_t               init_step,     // Initial simplex spread step size
    input  q16_t               tolerance,     // Convergence threshold on simplex radius
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 3: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,     // Converged optimal parameter vector x*
    output q16_t               f_optimal,     // Minimum objective function value f(x*)
    output q16_t               simplex_radius,// Final simplex diameter
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while solving
);

    // -------------------------------------------------------------------------
    // Master FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        NM_IDLE        = 3'd0,
        NM_INIT_EVAL   = 3'd1,
        NM_WAIT_EVAL   = 3'd2,
        NM_START_ITER  = 3'd3,
        NM_WAIT_ITER   = 3'd4,
        NM_CHECK_CONV  = 3'd5,
        NM_DONE        = 3'd6
    } nm_state_t;

    nm_state_t state;

    // Configuration & Simplex State
    logic [2:0]   num_params_reg;
    logic [2:0]   num_vert_reg;
    q16_t         step_reg;
    q16_t         tol_reg;
    logic [7:0]   max_iters_reg;

    simplex_vec_t cur_simplex;
    simplex_f_t   cur_fitness;
    logic [2:0]   init_eval_idx;

    // Simplex Core Interconnect
    logic         core_start;
    simplex_vec_t core_simplex_out;
    simplex_f_t   core_fitness_out;
    q16_t         core_f_best;
    vec_t         core_x_best;
    q16_t         core_radius;
    q16_t         core_f_span;
    logic         core_done, core_busy;

    // Initial Evaluator DFG Interconnect
    logic start_init_dfg;
    vec_t init_dfg_x_in;
    q16_t init_dfg_f_out;
    logic init_dfg_done, init_dfg_busy;

    // Instantiate Simplex Geometric Engine Core
    nm_simplex_core u_core (
        .clk            (clk),
        .rst_n          (rst_n),
        .prog_en        (prog_en),
        .prog_addr      (prog_addr),
        .prog_data      (prog_data),
        .start_iter     (core_start),
        .num_params     (num_params_reg),
        .simplex_in     (cur_simplex),
        .fitness_in     (cur_fitness),
        .simplex_out    (core_simplex_out),
        .fitness_out    (core_fitness_out),
        .f_best         (core_f_best),
        .x_best         (core_x_best),
        .simplex_radius (core_radius),
        .f_span         (core_f_span),
        .iter_done      (core_done),
        .busy           (core_busy)
    );

    // Instantiate Initial DFG Evaluator
    dfg_nm_engine u_init_dfg (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_init_dfg),
        .num_params (num_params_reg),
        .x_vec      (init_dfg_x_in),
        .f_out      (init_dfg_f_out),
        .eval_done  (init_dfg_done),
        .busy       (init_dfg_busy)
    );

    vec_t         tmp_vertex;
    simplex_vec_t s_init;

    // Master Nelder-Mead Loop FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= NM_IDLE;
            num_params_reg <= 3'd2;
            num_vert_reg   <= 3'd3;
            step_reg       <= Q16_INIT_STEP;
            tol_reg        <= Q16_EPS_DEF;
            max_iters_reg  <= 8'd50;
            iter_count     <= 8'd0;
            status         <= STATUS_IDLE;
            done           <= 1'b0;
            busy           <= 1'b0;
            x_optimal      <= '0;
            f_optimal      <= Q16_ZERO;
            simplex_radius <= Q16_ZERO;
            cur_simplex    <= '0;
            cur_fitness    <= '0;
            init_eval_idx  <= '0;
            core_start     <= 1'b0;
            start_init_dfg <= 1'b0;
            init_dfg_x_in  <= '0;
        end else begin
            core_start     <= 1'b0;
            start_init_dfg <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: NM_IDLE
                // -------------------------------------------------------------
                NM_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy           <= 1'b1;
                        num_params_reg <= (num_params != 3'd0)     ? num_params : 3'd2;
                        num_vert_reg   <= (num_params != 3'd0)     ? num_params + 1'b1 : 3'd3;
                        step_reg       <= (init_step != Q16_ZERO)  ? init_step  : Q16_INIT_STEP;
                        tol_reg        <= (tolerance != Q16_ZERO)  ? tolerance  : Q16_EPS_DEF;
                        max_iters_reg  <= (max_iters != 8'd0)      ? max_iters  : 8'd50;
                        iter_count     <= 8'd0;
                        status         <= STATUS_RUNNING;

                        // 1. Construct Initial Simplex Vertices: V_0 = x_init, V_{i+1} = x_init + step*e_i
                        s_init = '0;
                        s_init = set_s_vec(s_init, 3'd0, x_init);
                        for (int i = 0; i < MAX_PARAMS; i++) begin
                            tmp_vertex = x_init;
                            tmp_vertex = set_vec(tmp_vertex, 2'(i), get_vec(x_init, 2'(i)) + ((init_step != Q16_ZERO) ? init_step : Q16_INIT_STEP));
                            s_init = set_s_vec(s_init, 3'(i + 1), tmp_vertex);
                        end
                        cur_simplex   <= s_init;
                        init_eval_idx <= 3'd0;
                        state         <= NM_INIT_EVAL;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE INITIAL FITNESSES f_0..f_N
                // -------------------------------------------------------------
                NM_INIT_EVAL: begin
                    if (init_eval_idx < num_vert_reg) begin
                        init_dfg_x_in  <= get_s_vec(cur_simplex, init_eval_idx);
                        start_init_dfg <= 1'b1;
                        state          <= NM_WAIT_EVAL;
                    end else begin
                        state <= NM_START_ITER;
                    end
                end

                NM_WAIT_EVAL: begin
                    if (init_dfg_done) begin
                        cur_fitness   <= set_s_f(cur_fitness, init_eval_idx, init_dfg_f_out);
                        init_eval_idx <= init_eval_idx + 1'b1;
                        state         <= NM_INIT_EVAL;
                    end
                end

                // -------------------------------------------------------------
                // RUN SIMPLEX ITERATION
                // -------------------------------------------------------------
                NM_START_ITER: begin
                    core_start <= 1'b1;
                    state      <= NM_WAIT_ITER;
                end

                NM_WAIT_ITER: begin
                    if (core_done) begin
                        cur_simplex <= core_simplex_out;
                        cur_fitness <= core_fitness_out;
                        state       <= NM_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // CHECK CONVERGENCE
                // -------------------------------------------------------------
                NM_CHECK_CONV: begin
                    // Condition 1: Simplex diameter contracted to tolerance
                    if (core_radius <= tol_reg) begin
                        status         <= STATUS_CONVERGED;
                        x_optimal      <= core_x_best;
                        f_optimal      <= core_f_best;
                        simplex_radius <= core_radius;
                        state          <= NM_DONE;

                    // Condition 2: Max iterations reached
                    end else if (iter_count >= max_iters_reg) begin
                        status         <= STATUS_MAX_ITERS;
                        x_optimal      <= core_x_best;
                        f_optimal      <= core_f_best;
                        simplex_radius <= core_radius;
                        state          <= NM_DONE;

                    // Condition 3: Continue simplex transformation loop
                    end else begin
                        iter_count <= iter_count + 1'b1;
                        state      <= NM_START_ITER;
                    end
                end

                // -------------------------------------------------------------
                // STATE: NM_DONE
                // -------------------------------------------------------------
                NM_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= NM_IDLE;
                end

                default: state <= NM_IDLE;
            endcase
        end
    end

endmodule
