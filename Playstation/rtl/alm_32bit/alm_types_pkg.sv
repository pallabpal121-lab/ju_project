// =============================================================================
// File Name   : alm_types_pkg.sv
// Module Name : alm_types_pkg (SystemVerilog Package)
// Project     : Augmented Lagrangian Method (ALM) Accelerator (Solver #21)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), 4D coordinate vectors,
//   4x4 constraint and Hessian matrices, multiplier vectors, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package alm_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    localparam int Q_FRAC_BITS       = 16;
    localparam q16_t Q16_ONE         = 32'h0001_0000; // +1.0
    localparam q16_t Q16_TWO         = 32'h0002_0000; // +2.0
    localparam q16_t Q16_HALF        = 32'h0000_8000; // +0.5
    localparam q16_t Q16_ZERO        = 32'h0000_0000; //  0.0
    localparam q16_t Q16_MAX_POS     = 32'h7FFF_FFFF; // +32767.999
    localparam q16_t Q16_MIN_NEG     = 32'h8000_0000; // -32768.000

    localparam q16_t Q16_RHO_INIT    = 32'h0001_0000; // Initial penalty ρ = 1.0
    localparam q16_t Q16_RHO_MAX     = 32'h0020_0000; // Max penalty ρ = 32.0
    localparam q16_t Q16_FEAS_TOL    = 32'h0000_0040; // Feasibility tolerance ε = 0.000976
    localparam q16_t Q16_OPT_TOL     = 32'h0000_0040; // Optimality tolerance ε = 0.000976
    localparam q16_t Q16_DAMP_DEF    = 32'h0000_0010; // Numerical damping λ_eps = 0.000244

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & MATRIX DIMENSIONS (N <= 4, M_eq <= 4, M_ineq <= 4)
    // -------------------------------------------------------------------------
    localparam int MAX_DIM           = 4;

    // Packed 4-element coordinate / multiplier vector
    typedef logic signed [3:0][31:0] vec_t;

    // Packed 4x4 matrix (16 elements = 512 bits)
    typedef logic signed [15:0][31:0] mat_t;

    // -------------------------------------------------------------------------
    // SECTION 3: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // Feasibility and optimality satisfied
        STATUS_MAX_ITERS   = 3'd3, // Maximum outer iterations reached
        STATUS_ERROR       = 3'd4
    } status_t;

endpackage : alm_types_pkg
