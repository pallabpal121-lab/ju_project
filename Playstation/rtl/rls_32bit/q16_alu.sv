// =============================================================================
// File Name   : q16_alu.sv
// Module Name : q16_alu
// Project     : Recursive Least Squares (RLS) Accelerator (Solver #23)
// -----------------------------------------------------------------------------
// Description: Combinational Q16.16 Fixed-Point Arithmetic Logic Unit.
// =============================================================================

`timescale 1ns / 1ps

import rls_types_pkg::*;

module q16_alu (
    input  logic signed [31:0]  src_a,
    input  logic signed [31:0]  src_b,
    input  logic [1:0]          op, // 0: ADD, 1: SUB, 2: MUL, 3: NEG
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
            2'd0: begin // ADD
                result = src_a + src_b;
                if ((src_a > 0 && src_b > 0 && result < 0) ||
                    (src_a < 0 && src_b < 0 && result > 0)) begin
                    overflow = 1'b1;
                end
            end

            2'd1: begin // SUB
                result = src_a - src_b;
                if ((src_a > 0 && src_b < 0 && result < 0) ||
                    (src_a < 0 && src_b > 0 && result > 0)) begin
                    overflow = 1'b1;
                end
            end

            2'd2: begin // MUL
                result   = mul_result;
                overflow = mul_overflow;
            end

            2'd3: begin // NEG
                result = -src_a;
            end

            default: result = 32'sd0;
        endcase
    end

endmodule
