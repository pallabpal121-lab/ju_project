// =============================================================================
// File Name   : q16_alu.sv
// Module Name : q16_alu
// Project     : Trust-Region Dogleg Non-Linear Optimizer Accelerator (Solver #13)
// -----------------------------------------------------------------------------
// Description: Combinational Q16.16 Fixed-Point Arithmetic Logic Unit.
// =============================================================================

`timescale 1ns / 1ps

import dogleg_types_pkg::*;

module q16_alu (
    input  logic signed [31:0]  src_a,
    input  logic signed [31:0]  src_b,
    input  logic signed [15:0]  imm,
    input  opcode_t             op,
    output logic signed [31:0]  result,
    output logic                overflow
);

    logic signed [63:0] product_64;
    logic signed [31:0] mul_result;
    logic               mul_overflow;

    assign product_64   = 64'(src_a) * 64'(src_b);
    assign mul_result   = product_64[47:16];
    assign mul_overflow = (product_64[63:47] != 17'sd0) && (product_64[63:47] != 17'sh1FFFF);

    always_comb begin
        overflow = 1'b0;
        result   = 32'sd0;

        case (op)
            OP_NOP: begin
                result = 32'sd0;
            end

            OP_ADD: begin
                result = src_a + src_b;
                if ((src_a > 0 && src_b > 0 && result < 0) ||
                    (src_a < 0 && src_b < 0 && result > 0)) begin
                    overflow = 1'b1;
                end
            end

            OP_SUB: begin
                result = src_a - src_b;
                if ((src_a > 0 && src_b < 0 && result < 0) ||
                    (src_a < 0 && src_b > 0 && result > 0)) begin
                    overflow = 1'b1;
                end
            end

            OP_MUL: begin
                result   = mul_result;
                overflow = mul_overflow;
            end

            OP_NEG: begin
                result = -src_a;
            end

            OP_MOV: begin
                result = src_a;
            end

            OP_LOADC: begin
                result = {imm, 16'h0000};
            end

            default: begin
                result = 32'sd0;
            end
        endcase
    end

endmodule
