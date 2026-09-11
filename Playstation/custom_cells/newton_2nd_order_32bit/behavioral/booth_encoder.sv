// =============================================================================
// File Name   : booth_encoder.sv
// Module Name : booth_encoder
// Project     : Custom Cell Optimization Library
// -----------------------------------------------------------------------------
// Description : Radix-4 Modified Booth Encoder (MBE) Standard Cell
//
// Function:
//   Inspects 3 overlapping multiplier bits {y[2i+1], y[2i], y[2i-1]} and
//   generates control signals for generating partial products:
//   - single : 1X multiplicand
//   - double : 2X multiplicand (shift left by 1)
//   - neg    : Invert and add 1 for two's complement (-1X, -2X)
//
// Encoding Table:
//   y[2i+1]  y[2i]  y[2i-1] | Operation | single | double | neg
//      0       0       0    |     0     |   0    |   0    |  0
//      0       0       1    |    +1*X   |   1    |   0    |  0
//      0       1       0    |    +1*X   |   1    |   0    |  0
//      0       1       1    |    +2*X   |   0    |   1    |  0
//      1       0       0    |    -2*X   |   0    |   1    |  1
//      1       0       1    |    -1*X   |   1    |   0    |  1
//      1       1       0    |    -1*X   |   1    |   0    |  1
//      1       1       1    |     0     |   0    |   0    |  0
// =============================================================================

`timescale 1ns / 1ps

module booth_encoder (
    input  logic y_plus,  // y[2i+1]
    input  logic y_curr,  // y[2i]
    input  logic y_prev,  // y[2i-1]
    output logic single,  // 1X multiplicand select
    output logic double,  // 2X multiplicand select
    output logic neg      // Negate / complement flag
);

    // Optimized boolean equations
    assign single = y_prev ^ y_curr;
    assign double = (y_plus & ~y_curr & ~y_prev) | (~y_plus & y_curr & y_prev);
    assign neg    = y_plus & (~y_curr | ~y_prev);

endmodule
