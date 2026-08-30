// =============================================================================
// File Name   : svm_types_pkg.sv
// Module Name : svm_types_pkg (SystemVerilog Package)
// Project     : Support Vector Machine Sequential Minimal Optimization (SVM-SMO)
//               Accelerator (Solver #27)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), sample arrays, alpha vectors,
//   kernel matrix arrays, label vectors, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package svm_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    localparam int Q_FRAC_BITS       = 16;
    localparam q16_t Q16_ONE         = 32'h0001_0000; // +1.0
    localparam q16_t Q16_TWO         = 32'h0002_0000; // +2.0
    localparam q16_t Q16_HALF        = 32'h0000_8000; // +0.5
    localparam q16_t Q16_ZERO        = 32'h0000_0000; //  0.0
    localparam q16_t Q16_NEG_ONE     = 32'hFFFF_0000; // -1.0
    localparam q16_t Q16_MAX_POS     = 32'h7FFF_FFFF; // +32767.999
    localparam q16_t Q16_MIN_NEG     = 32'h8000_0000; // -32768.000

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & MATRIX DIMENSIONS (M <= 8, D <= 4)
    // -------------------------------------------------------------------------
    localparam int MAX_SAMPLES       = 8; // Up to 8 training samples (M)
    localparam int MAX_FEATURES      = 4; // Up to 4 feature dimensions (D)

    // Packed 4-element single sample feature vector x
    typedef logic signed [3:0][31:0] sample_vec_t;

    // Linearized 8 samples x 4 features dataset array (idx = sample*4 + feature)
    typedef logic signed [31:0][31:0] dataset_arr_t;

    // Packed 8-element Lagrange multiplier alpha vector
    typedef logic signed [7:0][31:0] alpha_vec_t;

    // Packed 8-element label vector y_i in {-1.0, +1.0}
    typedef logic signed [7:0][31:0] label_vec_t;

    // Packed 8-element error cache vector E_i
    typedef logic signed [7:0][31:0] error_vec_t;

    // Linearized 8x8 kernel matrix K_ij = x_i^T x_j (idx = row*8 + col)
    typedef logic signed [63:0][31:0] kernel_mat_t;

    // -------------------------------------------------------------------------
    // SECTION 3: STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE          = 3'd0,
        STATUS_KERNEL_READY  = 3'd1, // Kernel matrix calculated
        STATUS_OPTIMIZING    = 3'd2, // SMO iterating
        STATUS_CONVERGED     = 3'd3, // KKT conditions satisfied
        STATUS_MAX_ITERS     = 3'd4, // Reached max passes
        STATUS_ERROR         = 3'd5
    } status_t;

endpackage : svm_types_pkg
