// =============================================================================
// File Name   : cas_cell.sv
// Module Name : cas_cell
// Project     : Custom Cell Optimization Library
// -----------------------------------------------------------------------------
// Description : Controlled Add / Subtract (CAS) Bit-Slice Cell
//
// Function:
//   Performs conditional full addition (when ctrl = 0) or full subtraction
//   (when ctrl = 1) in a single compact standard cell layout.
//
// Logic Equations:
//   b_mod = b ^ ctrl
//   sum   = a ^ b_mod ^ cin
//   cout  = (a & b_mod) | (cin & (a ^ b_mod))
// =============================================================================

`timescale 1ns / 1ps

module cas_cell (
    input  logic a,
    input  logic b,
    input  logic cin,
    input  logic ctrl, // 0: Add (a + b), 1: Subtract (a - b)
    output logic sum,
    output logic cout
);

    logic b_mod;
    assign b_mod = b ^ ctrl;
    assign sum   = a ^ b_mod ^ cin;
    assign cout  = (a & b_mod) | (cin & (a ^ b_mod));

endmodule
