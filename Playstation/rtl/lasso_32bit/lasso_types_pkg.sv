// =============================================================================
// File Name   : lasso_types_pkg.sv
// Module Name : lasso_types_pkg (SystemVerilog Package)
// Project     : Coordinate Descent / LASSO L1 Sparsity Accelerator (Solver #10)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Defines data types, dataset matrix formats (M x N), observation vectors,
//   and status codes for the LASSO L1 Regularized Coordinate Descent Accelerator.
//
// Key Concepts:
//   - Dimension N       : Up to MAX_PARAMS = 4 features
//   - Observations M    : Up to MAX_OBS = 8 data samples
//   - Objective         : min_w 0.5 * ||y - X*w||^2 + lambda * ||w||_1
//   - Soft Thresholding : S_lambda(z) = sign(z) * max(|z| - lambda, 0)
//   - Exact Silicon Zero: Drives irrelevant weights to exact 0.0 in hardware!
// =============================================================================

`timescale 1ns / 1ps

package lasso_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    localparam int Q_FRAC_BITS   = 16;
    localparam q16_t Q16_ONE     = 32'h0001_0000; // +1.0
    localparam q16_t Q16_HALF    = 32'h0000_8000; // +0.5
    localparam q16_t Q16_ZERO    = 32'h0000_0000; //  0.0
    localparam q16_t Q16_NEG_ONE = 32'hFFFF_0000; // -1.0
    localparam q16_t Q16_EPS_DEF = 32'h0000_0020; // Default tolerance ε ≈ 0.00048

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & DATASET DEFINITIONS
    // -------------------------------------------------------------------------
    localparam int MAX_PARAMS = 4; // Maximum features (N <= 4)
    localparam int MAX_OBS    = 8; // Maximum observations (M <= 8)

    // Packed 4-element coordinate weight vector [w3, w2, w1, w0] (128 bits)
    typedef logic signed [127:0] vec_t;

    // Packed 8-element observation target/residual vector [y7..y0] (256 bits)
    typedef logic signed [255:0] obs_vec_t;

    // Packed 8x4 dataset feature matrix X [Row7..Row0] (1024 bits)
    typedef logic signed [1023:0] dataset_mat_t;

    // -------------------------------------------------------------------------
    // SECTION 3: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // max_j |delta w_j| <= tolerance
        STATUS_MAX_ITERS   = 3'd3, // Maximum cycle limit reached
        STATUS_DIV_BY_ZERO = 3'd4
    } status_t;

endpackage : lasso_types_pkg
