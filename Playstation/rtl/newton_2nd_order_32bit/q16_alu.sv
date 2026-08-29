// =============================================================================
// File Name   : q16_alu.sv
// Module Name : q16_alu
// Project     : Universal Newton 2nd-Order Optimization Accelerator
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This module is a purely combinational Arithmetic Logic Unit (ALU) designed
//   specifically for Q16.16 signed fixed-point mathematical operations.
//
//   It takes two 32-bit operands (src_a, src_b) or an immediate constant (imm),
//   executes the requested operation specified by 'op' in a single clock cycle,
//   and outputs the 32-bit Q16.16 result alongside an arithmetic overflow flag.
//
// Operations Supported:
//   - OP_NOP   : Outputs zero
//   - OP_ADD   : Signed addition with overflow check (src_a + src_b)
//   - OP_SUB   : Signed subtraction with overflow check (src_a - src_b)
//   - OP_MUL   : Q16.16 multiplication with 64-bit precision and bit-slicing
//   - OP_NEG   : Two's complement negation (-src_a)
//   - OP_MOV   : Passes src_a directly to output (register-to-register copy)
//   - OP_LOADC : Formats a 16-bit integer immediate constant into Q16.16 format
// =============================================================================

`timescale 1ns / 1ps

import newton_types_pkg::*;

module q16_alu (
    // -------------------------------------------------------------------------
    // Input Ports
    // -------------------------------------------------------------------------
    input  logic signed [31:0]  src_a,    // 1st operand (from register file)
    input  logic signed [31:0]  src_b,    // 2nd operand (from register file)
    input  logic signed [15:0]  imm,      // 16-bit immediate constant (from instruction)
    input  opcode_t             op,       // 4-bit operation selector (OP_ADD, OP_MUL, etc.)

    // -------------------------------------------------------------------------
    // Output Ports
    // -------------------------------------------------------------------------
    output logic signed [31:0]  result,   // Computed 32-bit Q16.16 result
    output logic                overflow  // High (1'b1) if an arithmetic overflow occurs
);

    // -------------------------------------------------------------------------
    // INTERNAL SIGNALS: Q16.16 Fixed-Point Multiplier Logic
    // -------------------------------------------------------------------------
    // Why do we need a 64-bit product for Q16.16 multiplication?
    //
    // Let src_a = A * 2^16 and src_b = B * 2^16 (both in Q16.16).
    // The standard integer product is: (A * 2^16) * (B * 2^16) = (A * B) * 2^32.
    // Notice that the product now has 32 fractional bits (Q32.32 format)!
    //
    // To convert back to standard Q16.16 format (16 fractional bits), we must
    // shift the 64-bit product right by 16 bits:
    //   Product_Q16 = product_64 >> 16, which corresponds to bits [47:16]!
    //
    // Diagram of product_64 [63:0]:
    //   Bits [63:48] : Upper integer overflow guard bits (must match sign bit)
    //   Bits [47:16] : The normalized 32-bit Q16.16 result (1 sign, 15 int, 16 frac)
    //   Bits [15:0]  : Truncated lower fractional precision bits
    // -------------------------------------------------------------------------
    logic signed [63:0] product_64;
    logic signed [31:0] mul_result;
    logic               mul_overflow;

    // 1. Compute full 64-bit signed multiplication
    assign product_64   = 64'(src_a) * 64'(src_b);

    // 2. Extract normalized Q16.16 result from bits [47:16]
    assign mul_result   = product_64[47:16];

    // 3. Overflow Detection for Multiplication:
    // If the upper 17 bits [63:47] are NOT all zeros (for positive product)
    // and NOT all ones (for negative product in two's complement), then the
    // result exceeded the representable 32-bit Q16.16 range!
    assign mul_overflow = (product_64[63:47] != 17'sd0) && (product_64[63:47] != 17'sh1FFFF);

    // -------------------------------------------------------------------------
    // COMBINATIONAL LOGIC: ALU Operation Execution
    // -------------------------------------------------------------------------
    always_comb begin
        // Default assignments to prevent unintended latches
        overflow = 1'b0;
        result   = 32'sd0;

        case (op)
            // -----------------------------------------------------------------
            // OP_NOP: No Operation
            // -----------------------------------------------------------------
            OP_NOP: begin
                result = 32'sd0;
            end

            // -----------------------------------------------------------------
            // OP_ADD: Signed Addition (result = src_a + src_b)
            // Overflow Rule: Adding two positives yielding negative, OR
            //                adding two negatives yielding positive.
            // -----------------------------------------------------------------
            OP_ADD: begin
                result = src_a + src_b;
                if ((src_a > 0 && src_b > 0 && result < 0) ||
                    (src_a < 0 && src_b < 0 && result > 0)) begin
                    overflow = 1'b1;
                end
            end

            // -----------------------------------------------------------------
            // OP_SUB: Signed Subtraction (result = src_a - src_b)
            // Overflow Rule: Positive minus negative yielding negative, OR
            //                negative minus positive yielding positive.
            // -----------------------------------------------------------------
            OP_SUB: begin
                result = src_a - src_b;
                if ((src_a > 0 && src_b < 0 && result < 0) ||
                    (src_a < 0 && src_b > 0 && result > 0)) begin
                    overflow = 1'b1;
                end
            end

            // -----------------------------------------------------------------
            // OP_MUL: Signed Fixed-Point Multiplication (result = (src_a * src_b) >> 16)
            // -----------------------------------------------------------------
            OP_MUL: begin
                result   = mul_result;
                overflow = mul_overflow;
            end

            // -----------------------------------------------------------------
            // OP_NEG: Two's Complement Negation (result = -src_a)
            // -----------------------------------------------------------------
            OP_NEG: begin
                result = -src_a;
            end

            // -----------------------------------------------------------------
            // OP_MOV: Register-to-Register Pass-through (result = src_a)
            // -----------------------------------------------------------------
            OP_MOV: begin
                result = src_a;
            end

            // -----------------------------------------------------------------
            // OP_LOADC: Load Immediate Constant into Q16.16 format
            // Places 16-bit integer 'imm' into the integer slice [31:16],
            // and sets fractional bits [15:0] to zero (e.g. imm=3 -> 0x00030000 = 3.0)
            // -----------------------------------------------------------------
            OP_LOADC: begin
                result = {imm, 16'h0000};
            end

            // -----------------------------------------------------------------
            // Default Case: Safe fallback
            // -----------------------------------------------------------------
            default: begin
                result = 32'sd0;
            end
        endcase
    end

endmodule
