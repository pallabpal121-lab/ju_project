// =============================================================================
// File Name   : compressor_4to2.sv
// Module Name : compressor_4to2
// Project     : Custom Cell Optimization Library
// -----------------------------------------------------------------------------
// Description : High-Speed 4:2 Carry-Save Compressor Primitive
//
// Function:
//   Compresses 4 partial product inputs (x1, x2, x3, x4) plus a horizontal
//   carry-in (cin) from the adjacent bit position into 2 output vectors:
//   Sum and Carry, plus a horizontal carry-out (cout) to the next bit position.
//
// Arithmetic Relationship:
//   x1 + x2 + x3 + x4 + cin = sum + 2*(carry + cout)
//
// Architecture (Low-Power Transmission-Gate Optimized):
//   - Critical path: 3 XOR delays (independent of horizontal carry propagation)
//   - Uses MUX-based XOR logic for minimal transistor count and glitch reduction.
// =============================================================================

`timescale 1ns / 1ps

module compressor_4to2 (
    input  logic x1,
    input  logic x2,
    input  logic x3,
    input  logic x4,
    input  logic cin,
    output logic sum,
    output logic carry,
    output logic cout
);

    // Intermediate XOR nodes
    logic x12;
    logic x34;
    logic x1234;

    assign x12   = x1 ^ x2;
    assign x34   = x3 ^ x4;
    assign x1234 = x12 ^ x34;

    // Horizontal Carry-out (cout does NOT depend on cin, preventing carry ripple!)
    assign cout  = x12 ? x3 : x1;

    // Sum generation (3 XOR delays from inputs)
    assign sum   = x1234 ^ cin;

    // Vertical Carry generation
    assign carry = x1234 ? cin : x4;

endmodule
