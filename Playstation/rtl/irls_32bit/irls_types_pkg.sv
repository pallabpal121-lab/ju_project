// =============================================================================
// File Name   : irls_types_pkg.sv
// Module Name : irls_types_pkg (SystemVerilog Package)
// Project     : Iteratively Reweighted Least Squares (IRLS) Accelerator (Solver #3)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This package defines all data types, vector formats, feature matrix representations,
//   and status codes used by the Iteratively Reweighted Least Squares (IRLS) Accelerator.
//
// Key Concepts:
//   - Weight Vector w      : [w0, w1, w2, w3] (up to MAX_FEATURES = 4)
//   - Feature Matrix X     : M samples x N features (up to 8 samples x 4 features)
//   - Label Vector y       : Binary target labels y_m in {0.0, 1.0}
//   - Probability Vector p : Predicted probabilities p_m = σ(x_mᵀ · w)
//   - Dynamic Weights W    : W_mm = p_m · (1 - p_m)
//   - Normal System        : (Xᵀ · W · X + λ · I) · Δw = Xᵀ · (y - p)
// =============================================================================

`timescale 1ns / 1ps

package irls_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    localparam int Q_FRAC_BITS       = 16;
    localparam q16_t Q16_ONE         = 32'h0001_0000; // +1.0
    localparam q16_t Q16_HALF        = 32'h0000_8000; // +0.5
    localparam q16_t Q16_ZERO        = 32'h0000_0000; //  0.0
    localparam q16_t Q16_NEG_ONE     = 32'hFFFF_0000; // -1.0
    localparam q16_t Q16_EPS_DEF     = 32'h0000_0080; // Default tolerance ε ≈ 0.00195
    localparam q16_t Q16_LAMBDA_DEF  = 32'h0000_0400; // Default regularization λ ≈ 0.0156

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR, LABEL & FEATURE MATRIX DEFINITIONS
    // -------------------------------------------------------------------------
    localparam int MAX_FEATURES = 4; // Maximum number of features / weights (N <= 4)
    localparam int MAX_SAMPLES  = 8; // Maximum number of training samples (M <= 8)

    // Packed 4-element weight/gradient vector [w3, w2, w1, w0] (128 bits)
    typedef logic signed [127:0] vec_t;

    // Packed 8-element label/probability vector [y7, ..., y0] (256 bits)
    typedef logic signed [255:0] label_vec_t;

    // Packed 4x4 matrix (512 bits) for Hessian XᵀWX and Cholesky factor L
    typedef logic signed [511:0] mat_t;

    // Packed 8 samples x 4 features matrix (8 x 4 x 32-bit = 1024 bits)
    typedef logic signed [1023:0] feat_mat_t;

    // -------------------------------------------------------------------------
    // SECTION 3: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // ||g||_inf <= tolerance or MSE loss <= tol
        STATUS_MAX_ITERS   = 3'd3, // Maximum iteration limit reached
        STATUS_SINGULAR    = 3'd4, // XᵀWX matrix non-invertible
        STATUS_DIV_BY_ZERO = 3'd5,
        STATUS_OVERFLOW    = 3'd6
    } status_t;

endpackage : irls_types_pkg
