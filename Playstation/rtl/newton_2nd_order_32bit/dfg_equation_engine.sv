// =============================================================================
// File Name   : dfg_equation_engine.sv
// Module Name : dfg_equation_engine
// Project     : Universal Newton 2nd-Order Optimization Accelerator
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This module is a miniature, programmable Data-Flow Graph (DFG) processor.
//   Think of it as a specialized microcode CPU whose only job is to evaluate
//   any custom mathematical equation f(x) for any given input value x.
//
// How It Solves the "Universal Equation" Problem:
//   Instead of hardcoding a specific equation (like x^2 - 4) into hardware gates,
//   this engine provides a 32-instruction Program Memory (prog_mem) and a 16-word
//   Register File (reg_file, r0..r15).
//
// Dual Operating Phases:
//   1. Programming Phase (Before optimization starts):
//      - The host/testbench sets 'prog_en = 1' and writes microcode instructions
//        into 'prog_mem' at 'prog_addr' (addresses 0 to 31).
//   2. Evaluation Phase (During each Newton iteration):
//      - The derivative engine asserts 'start_eval = 1' with 'x_in'.
//      - The engine automatically copies 'x_in' into register r0 (REG_X).
//      - It starts the Program Counter (PC = 0) and executes instructions sequentially.
//      - Single-cycle ALU instructions (ADD, SUB, MUL, LOADC) take 1 clock cycle each.
//      - Division instructions (OP_DIV) pause execution and wait 48 cycles for the divider.
//      - When it hits 'OP_END', it reads register r15 (REG_RESULT) and asserts 'eval_done'.
// =============================================================================

`timescale 1ns / 1ps

import newton_types_pkg::*;

module dfg_equation_engine (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,         // System Clock
    input  logic               rst_n,       // Active-Low Reset

    // -------------------------------------------------------------------------
    // Interface 1: Equation Programming Port (Host Writes Microcode Here)
    // -------------------------------------------------------------------------
    input  logic               prog_en,     // Program Write Enable (1'b1 to write)
    input  logic [4:0]         prog_addr,   // Program Memory Address (0 to 31)
    input  instr_t             prog_data,   // 32-bit Microcode Instruction Word

    // -------------------------------------------------------------------------
    // Interface 2: Function Evaluation Port (Called by Derivative Engine)
    // -------------------------------------------------------------------------
    input  logic               start_eval,  // 1-cycle strobe to begin evaluating f(x)
    input  q16_t               x_in,        // The input argument 'x' (Q16.16 format)
    output q16_t               f_out,       // The evaluated function result f(x)
    output logic               eval_done,   // 1-cycle completion pulse
    output logic               busy         // High while equation is being computed
);

    // -------------------------------------------------------------------------
    // INTERNAL REGISTERS & MEMORY ARRAYS
    // -------------------------------------------------------------------------
    // 1. Program Memory: Stores up to 32 micro-instructions (prog_mem[0..31])
    instr_t prog_mem [0:PROG_DEPTH-1];

    // 2. Register File: 16 General Purpose Registers (r0..r15)
    //    - r0  (REG_X)      : Pre-loaded with input 'x' on start_eval
    //    - r15 (REG_RESULT) : Sampled as the final output f(x) on OP_END
    q16_t   reg_file [0:NUM_REGS-1];

    // 3. Program Counter (PC): Points to the current instruction being executed
    logic [4:0] pc;

    // -------------------------------------------------------------------------
    // Engine State Machine Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        ENG_IDLE       = 2'd0, // Idle, waiting for start_eval pulse
        ENG_FETCH_EXEC = 2'd1, // Fetching instruction and executing on ALU
        ENG_DIV_WAIT   = 2'd2, // Pausing PC while iterative divider runs
        ENG_DONE       = 2'd3  // Evaluation complete
    } eng_state_t;

    eng_state_t state;

    // -------------------------------------------------------------------------
    // SUB-MODULE INTERCONNECT SIGNALS
    // -------------------------------------------------------------------------
    // Signals for Q16 Fixed-Point ALU
    q16_t               alu_src_a, alu_src_b;
    logic signed [15:0] alu_imm;
    opcode_t            alu_op;
    q16_t               alu_result;
    logic               alu_overflow;

    // Signals for Q16 Fixed-Point Divider
    logic               div_start;
    q16_t               div_dividend, div_divisor, div_quotient;
    logic               div_done, div_by_zero, div_busy;

    // -------------------------------------------------------------------------
    // SUB-MODULE INSTANTIATIONS
    // -------------------------------------------------------------------------
    // 1. Single-Cycle Combinational ALU
    q16_alu u_alu (
        .src_a   (alu_src_a),
        .src_b   (alu_src_b),
        .imm     (alu_imm),
        .op      (alu_op),
        .result  (alu_result),
        .overflow(alu_overflow)
    );

    // 2. Multi-Cycle Iterative Divider (for OP_DIV instructions)
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

    // -------------------------------------------------------------------------
    // SECTION 1: PROGRAM MEMORY WRITE LOGIC (Synchronous Programming)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (prog_en) begin
            prog_mem[prog_addr] <= prog_data; // Write microcode instruction
        end
    end

    // -------------------------------------------------------------------------
    // SECTION 2: COMBINATIONAL INSTRUCTION DECODING & ALU HOOKUP
    // -------------------------------------------------------------------------
    instr_t current_instr;
    assign current_instr = prog_mem[pc]; // Read instruction addressed by PC

    // Route register operands to ALU inputs
    assign alu_src_a     = reg_file[current_instr.src_a];
    assign alu_src_b     = reg_file[current_instr.src_b];
    assign alu_imm       = current_instr.imm;
    assign alu_op        = current_instr.op;

    // -------------------------------------------------------------------------
    // SECTION 3: SEQUENTIAL EXECUTION FSM & REGISTER FILE WRITEBACK
    // -------------------------------------------------------------------------
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
            // Clear all 16 registers
            for (int i = 0; i < NUM_REGS; i++) begin
                reg_file[i] <= Q16_ZERO;
            end
        end else begin
            div_start <= 1'b0; // Default pulse suppression

            case (state)
                // -------------------------------------------------------------
                // STATE: ENG_IDLE (Wait for start_eval)
                // -------------------------------------------------------------
                ENG_IDLE: begin
                    eval_done <= 1'b0;
                    if (start_eval) begin
                        busy            <= 1'b1;
                        pc              <= '0;   // Reset Program Counter to instruction 0
                        reg_file[REG_X] <= x_in; // Pre-load input 'x' into register r0
                        state           <= ENG_FETCH_EXEC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE: ENG_FETCH_EXEC (Execute Current Instruction)
                // -------------------------------------------------------------
                ENG_FETCH_EXEC: begin
                    // Case A: End of Program
                    if (current_instr.op == OP_END) begin
                        f_out     <= reg_file[REG_RESULT]; // Read result from r15
                        eval_done <= 1'b1;                 // Strobe completion
                        busy      <= 1'b0;
                        state     <= ENG_IDLE;             // Return to Idle
                    
                    // Case B: Division Instruction (Iterative, Multi-Cycle)
                    end else if (current_instr.op == OP_DIV) begin
                        div_dividend <= reg_file[current_instr.src_a];
                        div_divisor  <= reg_file[current_instr.src_b];
                        div_start    <= 1'b1;              // Launch divider
                        state        <= ENG_DIV_WAIT;      // Pause PC and wait
                    
                    // Case C: Standard ALU Instruction (Single-Cycle)
                    end else begin
                        // Write computed ALU result directly to destination register
                        reg_file[current_instr.dst] <= alu_result;
                        pc <= pc + 1'b1;                   // Advance to next instruction
                    end
                end

                // -------------------------------------------------------------
                // STATE: ENG_DIV_WAIT (Wait for 48-cycle Division to Complete)
                // -------------------------------------------------------------
                ENG_DIV_WAIT: begin
                    if (div_done) begin
                        // Write division quotient into destination register
                        reg_file[current_instr.dst] <= div_quotient;
                        pc    <= pc + 1'b1;                // Advance PC
                        state <= ENG_FETCH_EXEC;           // Resume normal execution
                    end
                end

                default: state <= ENG_IDLE;
            endcase
        end
    end

endmodule
