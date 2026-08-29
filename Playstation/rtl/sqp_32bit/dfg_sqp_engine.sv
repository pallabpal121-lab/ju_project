// =============================================================================
// File Name   : dfg_sqp_engine.sv
// Module Name : dfg_sqp_engine
// Project     : Sequential Quadratic Programming (SQP) Accelerator (Solver #7)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Programmable DFG Model Evaluator for SQP Optimization.
//   Evaluates arbitrary non-linear objective functions f(x0, x1, x2, x3).
//   Output scalar objective is returned in r15.
// =============================================================================

`timescale 1ns / 1ps

import sqp_types_pkg::*;
`include "sqp_helpers.svh"

module dfg_sqp_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Programming Interface
    input  logic               prog_en,
    input  logic [4:0]         prog_addr,
    input  instr_t             prog_data,

    // Evaluation Interface
    input  logic               start_eval,
    input  logic [2:0]         num_params,
    input  vec_t               x_vec,
    output q16_t               f_out,
    output logic               eval_done,
    output logic               busy
);

    instr_t prog_mem [0:PROG_DEPTH-1];
    q16_t   reg_file [0:NUM_REGS-1];
    logic [4:0] pc;

    typedef enum logic [1:0] {
        ENG_IDLE       = 2'd0,
        ENG_FETCH_EXEC = 2'd1,
        ENG_DIV_WAIT   = 2'd2,
        ENG_DONE       = 2'd3
    } eng_state_t;

    eng_state_t state;

    q16_t               alu_src_a, alu_src_b;
    logic signed [15:0] alu_imm;
    opcode_t            alu_op;
    q16_t               alu_result;
    logic               alu_overflow;

    logic               div_start;
    q16_t               div_dividend, div_divisor, div_quotient;
    logic               div_done, div_by_zero, div_busy;

    q16_alu u_alu (
        .src_a   (alu_src_a),
        .src_b   (alu_src_b),
        .imm     (alu_imm),
        .op      (alu_op),
        .result  (alu_result),
        .overflow(alu_overflow)
    );

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

    always_ff @(posedge clk) begin
        if (prog_en) begin
            prog_mem[prog_addr] <= prog_data;
        end
    end

    instr_t current_instr;
    assign current_instr = prog_mem[pc];

    assign alu_src_a     = reg_file[current_instr.src_a];
    assign alu_src_b     = reg_file[current_instr.src_b];
    assign alu_imm       = current_instr.imm;
    assign alu_op        = current_instr.op;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ENG_IDLE;
            pc           <= '0;
            eval_done    <= 1'b0;
            busy         <= 1'b0;
            f_out        <= Q16_ZERO;
            div_start    <= 1'b0;
            div_dividend <= Q16_ZERO;
            div_divisor  <= Q16_ZERO;
            for (int i = 0; i < NUM_REGS; i++) begin
                reg_file[i] <= Q16_ZERO;
            end
        end else begin
            div_start <= 1'b0;

            case (state)
                ENG_IDLE: begin
                    eval_done <= 1'b0;
                    if (start_eval) begin
                        busy <= 1'b1;
                        pc   <= '0;
                        for (int i = 0; i < MAX_PARAMS; i++) begin
                            if (i < num_params) begin
                                reg_file[i] <= get_vec(x_vec, 2'(i));
                            end else begin
                                reg_file[i] <= Q16_ZERO;
                            end
                        end
                        state <= ENG_FETCH_EXEC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                ENG_FETCH_EXEC: begin
                    if (current_instr.op == OP_END) begin
                        f_out     <= reg_file[REG_RESULT];
                        eval_done <= 1'b1;
                        busy      <= 1'b0;
                        state     <= ENG_IDLE;
                    end else if (current_instr.op == OP_DIV) begin
                        div_dividend <= reg_file[current_instr.src_a];
                        div_divisor  <= reg_file[current_instr.src_b];
                        div_start    <= 1'b1;
                        state        <= ENG_DIV_WAIT;
                    end else begin
                        reg_file[current_instr.dst] <= alu_result;
                        pc <= pc + 1'b1;
                    end
                end

                ENG_DIV_WAIT: begin
                    if (div_done) begin
                        reg_file[current_instr.dst] <= div_quotient;
                        pc    <= pc + 1'b1;
                        state <= ENG_FETCH_EXEC;
                    end
                end

                default: state <= ENG_IDLE;
            endcase
        end
    end

endmodule
