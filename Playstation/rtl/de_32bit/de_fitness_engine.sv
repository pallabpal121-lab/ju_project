// =============================================================================
// File Name   : de_fitness_engine.sv
// Module Name : de_fitness_engine
// Project     : Differential Evolution (DE) Accelerator (Solver #30)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Single-Vector Objective Fitness Evaluator.
//   Evaluates fitness F = f(x) for candidate target and trial vectors.
// =============================================================================

`timescale 1ns / 1ps

import de_types_pkg::*;
`include "de_helpers.svh"

module de_fitness_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  fitness_fn_t        fn_type,          // Objective function type
    input  logic [2:0]         dim,              // Dimension D (1..4)
    input  gene_vec_t          x_vec,            // Candidate parameter vector

    output q16_t               fitness_val,      // Evaluated fitness f(x)
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        FIT_IDLE = 2'd0,
        FIT_EVAL = 2'd1,
        FIT_DONE = 2'd2
    } fit_state_t;

    fit_state_t state;

    q16_t fit_reg;
    assign fitness_val = fit_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= FIT_IDLE;
            fit_reg <= Q16_MAX_POS;
            done    <= 1'b0;
            busy    <= 1'b0;
        end else begin
            case (state)
                FIT_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= FIT_EVAL;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                FIT_EVAL: begin
                    fit_reg <= evaluate_fitness(fn_type, x_vec, dim);
                    state   <= FIT_DONE;
                end

                FIT_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= FIT_IDLE;
                end

                default: state <= FIT_IDLE;
            endcase
        end
    end

endmodule
