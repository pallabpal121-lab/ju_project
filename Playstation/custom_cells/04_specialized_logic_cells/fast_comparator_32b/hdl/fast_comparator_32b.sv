// =============================================================================
// File Name   : fast_comparator_32b.sv
// Module Name : fast_comparator_32b
// Project     : Custom Cell Optimization Library
// -----------------------------------------------------------------------------
// Description : High-Speed 32-bit Magnitude Comparator for Convergence Check
//
// Function:
//   Evaluates (a <= b) and (a == b) in a single parallel prefix tree cycle.
//   Optimized for the Newton Solver convergence test: |g(x)| <= tolerance.
// =============================================================================

`timescale 1ns / 1ps

module fast_comparator_32b (
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic        less_equal,
    output logic        equal
);

    // Bitwise comparison signals
    wire [31:0] eq_bit;
    wire [31:0] gt_bit;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gen_cmp_bits
            assign eq_bit[i] = ~(a[i] ^ b[i]);
            assign gt_bit[i] = a[i] & ~b[i];
        end
    endgenerate

    assign equal = &eq_bit;

    // Parallel prefix A > B detection
    wire [31:0] eq_prefix;
    assign eq_prefix[31] = 1'b1;

    generate
        for (i = 30; i >= 0; i = i - 1) begin : gen_eq_pfx
            assign eq_prefix[i] = eq_prefix[i+1] & eq_bit[i+1];
        end
    endgenerate

    wire [31:0] a_gt_b_chain;
    assign a_gt_b_chain[31] = gt_bit[31];

    generate
        for (i = 30; i >= 0; i = i - 1) begin : gen_gt_chain
            assign a_gt_b_chain[i] = a_gt_b_chain[i+1] | (gt_bit[i] & eq_prefix[i]);
        end
    endgenerate

    // a <= b is equivalent to ~(a > b)
    assign less_equal = ~a_gt_b_chain[0] | equal;

endmodule
