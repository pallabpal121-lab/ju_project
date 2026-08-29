// =============================================================================
// File Name   : dfg_pgd_engine.sv
// Module Name : dfg_pgd_engine
// Project     : Projected Gradient Descent (PGD) Accelerator (Solver #12)
// -----------------------------------------------------------------------------
// Description: Programmable Data-Flow Graph (DFG) Microcode Objective Evaluator f(x).
// =============================================================================

`timescale 1ns / 1ps

import pgd_types_pkg::*;
`include "pgd_helpers.svh"

module dfg_pgd_engine (
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
        ENG_DONE       = 2'd2
    } eng_state_t;

    eng_state_t state;

    q16_t               alu_src_a, alu_src_b;
    logic signed [15:0] alu_imm;
    opcode_t            alu_op;
    q16_t               alu_result;
    logic               alu_overflow;

    q16_alu u_alu (
        .src_a   (alu_src_a),
        .src_b   (alu_src_b),
        .imm     (alu_imm),
        .op      (alu_op),
        .result  (alu_result),
        .overflow(alu_overflow)
    );

    // Programming Port
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
            state     <= ENG_IDLE;
            pc        <= 5'd0;
            f_out     <= Q16_ZERO;
            eval_done <= 1'b0;
            busy      <= 1'b0;
            for (int r = 0; r < NUM_REGS; r++) reg_file[r] <= Q16_ZERO;
        end else begin
            case (state)
                ENG_IDLE: begin
                    eval_done <= 1'b0;
                    if (start_eval) begin
                        busy <= 1'b1;
                        pc   <= 5'd0;

                        // Load x vector coordinates into R0..R3
                        reg_file[0] <= get_vec(x_vec, 2'd0);
                        reg_file[1] <= (num_params > 3'd1) ? get_vec(x_vec, 2'd1) : Q16_ZERO;
                        reg_file[2] <= (num_params > 3'd2) ? get_vec(x_vec, 2'd2) : Q16_ZERO;
                        reg_file[3] <= (num_params > 3'd3) ? get_vec(x_vec, 2'd3) : Q16_ZERO;

                        // Zero temporary registers
                        for (int r = 4; r < NUM_REGS; r++) reg_file[r] <= Q16_ZERO;

                        state <= ENG_FETCH_EXEC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                ENG_FETCH_EXEC: begin
                    if (current_instr.op == OP_NOP) begin
                        state <= ENG_DONE;
                    end else begin
                        reg_file[current_instr.dst] <= alu_result;
                        if (pc == PROG_DEPTH - 1) begin
                            state <= ENG_DONE;
                        end else begin
                            pc <= pc + 1'b1;
                        end
                    end
                end

                ENG_DONE: begin
                    f_out     <= reg_file[0]; // Output result in R0
                    eval_done <= 1'b1;
                    busy      <= 1'b0;
                    state     <= ENG_IDLE;
                end

                default: state <= ENG_IDLE;
            endcase
        end
    end

endmodule
