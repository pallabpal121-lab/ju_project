// =============================================================================
// File Name   : dynamic_overflow_detect.sv
// Module Name : dynamic_overflow_detect
// Project     : Custom Cell Optimization Library
// -----------------------------------------------------------------------------
// Description : Fast 17-bit Zero/One Equality Detector for Q16.16 Multiplier Overflow
//
// Function:
//   Evaluates whether bits [63:47] are all zeros (valid positive) OR all ones
//   (valid negative). If neither, it asserts 'overflow'.
// =============================================================================

`timescale 1ns / 1ps

module dynamic_overflow_detect (
    input  logic [16:0] upper_bits, // bits [63:47] of product
    output logic        overflow
);

    // All zeros check: NOR tree
    logic all_zeros;
    assign all_zeros = ~(|upper_bits);

    // All ones check: NAND tree
    logic all_ones;
    assign all_ones  = &upper_bits;

    // Overflow occurs when neither all zeros nor all ones
    assign overflow  = ~(all_zeros | all_ones);

endmodule
