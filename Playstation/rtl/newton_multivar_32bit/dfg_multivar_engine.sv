// =============================================================================
// File Name   : dfg_multivar_engine.sv
// Module Name : dfg_multivar_engine
// Project     : Universal Multivariable Newton 2nd-Order Accelerator
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Programmable Multivariable DFG Equation Processor.
//   Evaluates scalar loss f(x0, x1, ...) for any N-dimensional vector input x.
// =============================================================================

`timescale 1ns / 1ps

import newton_multivar_pkg::*;
`include "multivar_helpers.svh"

module dfg_multivar_engine (
    input  logic               clk,         // System Clock
    input  logic               rst_n,       // Active-Low Reset

    // Programming Interface
    input  logic                                  prog_en,     // Program Write Enable
    input  logic [$clog2(PROG_DEPTH)-1:0]         prog_addr,   // Program Memory Address
    input  instr_t                                prog_data,   // 32-bit Microcode Instruction Word

    // Evaluation Interface
    input  logic                          start_eval,  // 1-cycle strobe to begin evaluating f(x)
    input  logic [NUM_VARS_BITS-1:0]      num_vars,    // Active dimension N (1..MAX_VARS)
    input  vec_t                          x_vec,       // Input vector [x0, x1, ...]
    output q16_t                          f_out,       // Evaluated scalar result f(x)
    output logic                          eval_done,   // 1-cycle completion strobe
    output logic                          busy         // High while equation is executing
);

    // -------------------------------------------------------------------------
    // Internal Memories & Registers
    // -------------------------------------------------------------------------
    instr_t prog_mem [0:PROG_DEPTH-1];
    q16_t   reg_file [0:NUM_REGS-1];
    logic [$clog2(PROG_DEPTH)-1:0] pc;

    typedef enum logic [1:0] {
        ENG_IDLE       = 2'd0,
        ENG_FETCH_EXEC = 2'd1,
        ENG_DIV_WAIT   = 2'd2,
        ENG_DONE       = 2'd3
    } eng_state_t;

    eng_state_t state;

    // ALU Signals
    q16_t               alu_src_a, alu_src_b;
    logic signed [12:0] alu_imm;
    opcode_t            alu_op;
    q16_t               alu_result;
    logic               alu_overflow;

    // Divider Signals
    logic               div_start;
    q16_t               div_dividend, div_divisor, div_quotient;
    logic               div_done, div_by_zero, div_busy;

    // Submodules
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

    // Microcode Program Memory
    always_ff @(posedge clk) begin
        if (prog_en) begin
            prog_mem[prog_addr] <= prog_data;
        end
    end

    // Instruction Decoding
    instr_t current_instr;
    assign current_instr = prog_mem[pc];

    assign alu_src_a     = reg_file[current_instr.src_a];
    assign alu_src_b     = reg_file[current_instr.src_b];
    assign alu_imm       = current_instr.imm;
    assign alu_op        = current_instr.op;

    // Execution FSM
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
                // -------------------------------------------------------------
                // STATE: ENG_IDLE
                // -------------------------------------------------------------
                ENG_IDLE: begin
                    eval_done <= 1'b0;
                    if (start_eval) begin
                        busy <= 1'b1;
                        pc   <= '0;
                        for (int i = 0; i < MAX_VARS; i++) begin
                            if (i < num_vars) begin
                                reg_file[i] <= get_vec(x_vec, i);
                            end
                        end
                        state <= ENG_FETCH_EXEC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE: ENG_FETCH_EXEC
                // -------------------------------------------------------------
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

                // -------------------------------------------------------------
                // STATE: ENG_DIV_WAIT
                // -------------------------------------------------------------
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
