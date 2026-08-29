// =============================================================================
// File Name   : q16_alu.sv
// Module Name : q16_alu
// Project     : Iteratively Reweighted Least Squares (IRLS) Accelerator
// -----------------------------------------------------------------------------
// Description: Combinational Q16.16 Fixed-Point Arithmetic Logic Unit.
// =============================================================================

`timescale 1ns / 1ps

import irls_types_pkg::*;

module q16_alu (
    input  logic signed [31:0]  src_a,
    input  logic signed [31:0]  src_b,
    output logic signed [31:0]  add_res,
    output logic signed [31:0]  sub_res,
    output logic signed [31:0]  mul_res
);

    logic signed [63:0] prod_64;

    assign add_res = src_a + src_b;
    assign sub_res = src_a - src_b;

    assign prod_64 = 64'(src_a) * 64'(src_b);
    assign mul_res = prod_64[47:16];

endmodule
