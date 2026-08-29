// =============================================================================
// File Name   : q32_alu.sv
// Module Name : q32_alu
// Project     : Universal Newton 2nd-Order Optimization Accelerator (64-Bit)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This module is a purely combinational 64-bit Arithmetic Logic Unit (ALU)
//   designed specifically for Q32.32 signed fixed-point math operations.
//
// Operations Supported:
//   - OP_NOP   : Outputs 64-bit zero
//   - OP_ADD   : 64-bit signed addition with overflow detection
//   - OP_SUB   : 64-bit signed subtraction with overflow detection
//   - OP_MUL   : Q32.32 multiplication using 128-bit product, sliced at [95:32]
//   - OP_NEG   : 64-bit two's complement negation (-src_a)
//   - OP_MOV   : Passes src_a directly to output
//   - OP_LOADC : Formats a 32-bit signed integer immediate into Q32.32 format
// =============================================================================

`timescale 1ns / 1ps

import newton_types_64bit_pkg::*;

module q32_alu (
    // -------------------------------------------------------------------------
    // Input Ports
    // -------------------------------------------------------------------------
    input  logic signed [63:0]  src_a,    // 1st 64-bit operand (Q32.32)
    input  logic signed [63:0]  src_b,    // 2nd 64-bit operand (Q32.32)
    input  logic signed [31:0]  imm,      // 32-bit immediate constant
    input  opcode_t             op,       // 4-bit operation selector

    // -------------------------------------------------------------------------
    // Output Ports
    // -------------------------------------------------------------------------
    output logic signed [63:0]  result,   // Computed 64-bit Q32.32 result
    output logic                overflow  // High if an arithmetic overflow occurs
);

    // -------------------------------------------------------------------------
    // INTERNAL SIGNALS: 128-Bit Multiplier Logic
    // -------------------------------------------------------------------------
    // Multiplying two 64-bit Q32.32 numbers produces a 128-bit product in Q64.64 format:
    //   Bits [127:96] : Upper integer overflow guard bits (must match sign bit)
    //   Bits [95:32]  : Normalized 64-bit Q32.32 result (1 sign, 31 int, 32 frac)
    //   Bits [31:0]   : Truncated lower 32 fractional precision bits
    // -------------------------------------------------------------------------
    logic signed [127:0] product_128;
    logic signed [63:0]  mul_result;
    logic                mul_overflow;

    // 1. Compute full 128-bit signed multiplication
    assign product_128   = 128'(src_a) * 128'(src_b);

    // 2. Extract normalized Q32.32 result from bits [95:32]
    assign mul_result    = product_128[95:32];

    // 3. Overflow Detection for Multiplication:
    // If the upper 33 bits [127:95] are NOT all zeros and NOT all ones,
    // then the product exceeded the representable 64-bit Q32.32 range.
    assign mul_overflow  = (product_128[127:95] != 33'sd0) && (product_128[127:95] != 33'sh1_FFFF_FFFF);

    // -------------------------------------------------------------------------
    // COMBINATIONAL LOGIC: ALU Operation Execution
    // -------------------------------------------------------------------------
    always_comb begin
        overflow = 1'b0;
        result   = 64'sd0;

        case (op)
            // -----------------------------------------------------------------
            // OP_NOP: No Operation
            // -----------------------------------------------------------------
            OP_NOP: begin
                result = 64'sd0;
            end

            // -----------------------------------------------------------------
            // OP_ADD: 64-Bit Signed Addition
            // -----------------------------------------------------------------
            OP_ADD: begin
                result = src_a + src_b;
                if ((src_a > 0 && src_b > 0 && result < 0) ||
                    (src_a < 0 && src_b < 0 && result > 0)) begin
                    overflow = 1'b1;
                end
            end

            // -----------------------------------------------------------------
            // OP_SUB: 64-Bit Signed Subtraction
            // -----------------------------------------------------------------
            OP_SUB: begin
                result = src_a - src_b;
                if ((src_a > 0 && src_b < 0 && result < 0) ||
                    (src_a < 0 && src_b > 0 && result > 0)) begin
                    overflow = 1'b1;
                end
            end

            // -----------------------------------------------------------------
            // OP_MUL: Signed Fixed-Point Multiplication (result = (src_a * src_b) >> 32)
            // -----------------------------------------------------------------
            OP_MUL: begin
                result   = mul_result;
                overflow = mul_overflow;
            end

            // -----------------------------------------------------------------
            // OP_NEG: Two's Complement Negation
            // -----------------------------------------------------------------
            OP_NEG: begin
                result = -src_a;
            end

            // -----------------------------------------------------------------
            // OP_MOV: Register-to-Register Pass-through
            // -----------------------------------------------------------------
            OP_MOV: begin
                result = src_a;
            end

            // -----------------------------------------------------------------
            // OP_LOADC: Load 32-Bit Immediate Constant into Q32.32 format
            // Places 32-bit integer 'imm' into integer slice [63:32], fractional bits zero
            // -----------------------------------------------------------------
            OP_LOADC: begin
                result = {imm, 32'h0000_0000};
            end

            default: begin
                result = 64'sd0;
            end
        endcase
    end

endmodule
