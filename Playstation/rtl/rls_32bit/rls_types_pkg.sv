// =============================================================================
// File Name   : rls_types_pkg.sv
// Module Name : rls_types_pkg (SystemVerilog Package)
// Project     : Recursive Least Squares (RLS) Accelerator (Solver #23)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), filter weight vectors (vec_t),
//   4x4 inverse covariance matrices (mat_t), forgetting factors, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package rls_types_pkg;

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

    localparam q16_t Q16_LAMBDA_DEF  = 32'h0000_FE00; // Default forgetting factor λ = 0.9921875
    localparam q16_t Q16_DELTA_INIT  = 32'h0064_0000; // Default initial covariance P_0 = 100.0 * I

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & MATRIX DIMENSIONS (N <= 4)
    // -------------------------------------------------------------------------
    localparam int MAX_TAPS          = 4; // Up to 4 filter taps / weights

    // Packed 4-element coordinate vector w, x, k, v
    typedef logic signed [3:0][31:0] vec_t;

    // Packed 4x4 matrix P (inverse covariance)
    typedef logic signed [15:0][31:0] mat_t;

    // -------------------------------------------------------------------------
    // SECTION 3: STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_READY       = 3'd1, // Ready for next sample
        STATUS_BUSY        = 3'd2, // Calculating gain and updating P, w
        STATUS_UPDATED     = 3'd3, // Sample processed, weights updated
        STATUS_ERROR       = 3'd4
    } status_t;

endpackage : rls_types_pkg
