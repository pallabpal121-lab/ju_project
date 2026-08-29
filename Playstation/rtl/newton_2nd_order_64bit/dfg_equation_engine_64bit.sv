// =============================================================================
// File Name   : dfg_equation_engine_64bit.sv
// Module Name : dfg_equation_engine_64bit
// Project     : Universal Newton 2nd-Order Optimization Accelerator (64-Bit)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This module is the 64-bit programmable Data-Flow Graph (DFG) processor.
//   It features a 64-word 64-bit micro-instruction memory and a 64-register
//   register file (r0 to r63).
//
// Operation:
//   - Programming: Loads up to 64 custom 64-bit micro-instructions into prog_mem.
//   - Evaluation : Loads input variable 'x' into r0 (REG_X_64), executes instructions
//                  using q32_alu (1 cycle) and q32_divider (96 cycles), and returns
//                  r63 (REG_RESULT_64) as f(x) when OP_END is encountered.
// =============================================================================

`timescale 1ns / 1ps

import newton_types_64bit_pkg::*;

module dfg_equation_engine_64bit (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,         // System Clock
    input  logic               rst_n,       // Active-Low Reset

    // -------------------------------------------------------------------------
    // Interface 1: 64-Bit Equation Programming Port
    // -------------------------------------------------------------------------
    input  logic               prog_en,     // Program Write Enable
    input  logic [5:0]         prog_addr,   // Program Memory Address (0..63)
    input  instr64_t           prog_data,   // 64-bit Microcode Instruction Word

    // -------------------------------------------------------------------------
    // Interface 2: Function Evaluation Port
    // -------------------------------------------------------------------------
    input  logic               start_eval,  // 1-cycle strobe to begin evaluating f(x)
    input  q32_t               x_in,        // Input argument 'x' (Q32.32 signed)
    output q32_t               f_out,       // Evaluated result f(x) (Q32.32 signed)
    output logic               eval_done,   // 1-cycle completion pulse
    output logic               busy         // High while equation is executing
);

    // -------------------------------------------------------------------------
    // INTERNAL REGISTERS & MEMORY ARRAYS
    // -------------------------------------------------------------------------
    instr64_t   prog_mem [0:PROG_DEPTH_64-1]; // 64 instructions
    q32_t       reg_file [0:NUM_REGS_64-1];   // 64 registers (r0..r63)
    logic [5:0] pc;                           // 6-bit Program Counter

    // Engine State Definitions
    typedef enum logic [1:0] {
        ENG_IDLE       = 2'd0,
        ENG_FETCH_EXEC = 2'd1,
        ENG_DIV_WAIT   = 2'd2,
        ENG_DONE       = 2'd3
    } eng_state_t;

    eng_state_t state;

    // -------------------------------------------------------------------------
    // Sub-Module Interconnect Signals
    // -------------------------------------------------------------------------
    q32_t               alu_src_a, alu_src_b;
    logic signed [31:0] alu_imm;
    opcode_t            alu_op;
    q32_t               alu_result;
    logic               alu_overflow;

    logic               div_start;
    q32_t               div_dividend, div_divisor, div_quotient;
    logic               div_done, div_by_zero, div_busy;

    // -------------------------------------------------------------------------
    // Submodule Instantiations
    // -------------------------------------------------------------------------
    // 1. 64-Bit Single-Cycle Combinational ALU
    q32_alu u_alu (
        .src_a   (alu_src_a),
        .src_b   (alu_src_b),
        .imm     (alu_imm),
        .op      (alu_op),
        .result  (alu_result),
        .overflow(alu_overflow)
    );

    // 2. 64-Bit Multi-Cycle Iterative Divider (for OP_DIV instructions)
    q32_divider u_div (
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

    // -------------------------------------------------------------------------
    // SECTION 1: PROGRAM MEMORY WRITE LOGIC
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (prog_en) begin
            prog_mem[prog_addr] <= prog_data;
        end
    end

    // -------------------------------------------------------------------------
    // SECTION 2: COMBINATIONAL INSTRUCTION DECODING
    // -------------------------------------------------------------------------
    instr64_t current_instr;
    assign current_instr = prog_mem[pc];

    assign alu_src_a     = reg_file[current_instr.src_a];
    assign alu_src_b     = reg_file[current_instr.src_b];
    assign alu_imm       = current_instr.imm;
    assign alu_op        = current_instr.op;

    // -------------------------------------------------------------------------
    // SECTION 3: SEQUENTIAL EXECUTION FSM
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ENG_IDLE;
            pc           <= '0;
            eval_done    <= 1'b0;
            busy         <= 1'b0;
            f_out        <= Q32_ZERO;
            div_start    <= 1'b0;
            div_dividend <= Q32_ZERO;
            div_divisor  <= Q32_ZERO;
            for (int i = 0; i < NUM_REGS_64; i++) begin
                reg_file[i] <= Q32_ZERO;
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
                        busy               <= 1'b1;
                        pc                 <= '0;
                        reg_file[REG_X_64] <= x_in; // Pre-load x into r0
                        state              <= ENG_FETCH_EXEC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE: ENG_FETCH_EXEC
                // -------------------------------------------------------------
                ENG_FETCH_EXEC: begin
                    if (current_instr.op == OP_END) begin
                        f_out     <= reg_file[REG_RESULT_64]; // Read result from r63
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
