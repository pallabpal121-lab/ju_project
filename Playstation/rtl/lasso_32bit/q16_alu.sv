// =============================================================================
// File Name   : q16_alu.sv
// Module Name : q16_alu
// Project     : Coordinate Descent / LASSO L1 Sparsity Accelerator (Solver #10)
// -----------------------------------------------------------------------------
// Description: Combinational Q16.16 Fixed-Point Arithmetic Logic Unit.
// =============================================================================

`timescale 1ns / 1ps

import lasso_types_pkg::*;

module q16_alu (
    input  logic signed [31:0]  src_a,
    input  logic signed [31:0]  src_b,
    input  logic                is_sub,
    output logic signed [31:0]  result,
    output logic signed [63:0]  product_64,
    output logic signed [31:0]  mul_result
);

    assign result     = is_sub ? (src_a - src_b) : (src_a + src_b);
    assign product_64 = 64'(src_a) * 64'(src_b);
    assign mul_result = product_64[47:16];

endmodule
