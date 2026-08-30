// =============================================================================
// File Name   : pca_types_pkg.sv
// Module Name : pca_types_pkg (SystemVerilog Package)
// Project     : Principal Component Analysis (PCA) / Streaming SVD Accelerator
//               (Solver #28)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), feature vectors, covariance
//   matrices, eigenvector matrices, eigenvalue vectors, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package pca_types_pkg;

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
    // SECTION 2: VECTOR & MATRIX DIMENSIONS (M <= 8, D <= 4, K <= 4)
    // -------------------------------------------------------------------------
    localparam int MAX_SAMPLES       = 8; // Up to 8 dataset samples (M)
    localparam int MAX_FEATURES      = 4; // Up to 4 feature dimensions (D)
    localparam int MAX_COMPONENTS    = 4; // Up to 4 principal components (K)

    // Packed 4-element single sample feature vector x
    typedef logic signed [3:0][31:0] feature_vec_t;

    // Packed 4-element projected latent vector z
    typedef logic signed [3:0][31:0] latent_vec_t;

    // Linearized 8 samples x 4 features dataset array (idx = sample*4 + feature)
    typedef logic signed [31:0][31:0] dataset_arr_t;

    // Linearized 4x4 covariance matrix Σ (idx = row*4 + col)
    typedef logic signed [15:0][31:0] cov_mat_t;

    // Linearized 4x4 matrix of eigenvectors V = [v_1, v_2, v_3, v_4]
    // where col k contains eigenvector v_k (idx = row*4 + k)
    typedef logic signed [15:0][31:0] eigen_mat_t;

    // Packed 4-element eigenvalue vector λ
    typedef logic signed [3:0][31:0] lambda_vec_t;

    // -------------------------------------------------------------------------
    // SECTION 3: STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE          = 3'd0,
        STATUS_COV_READY     = 3'd1, // Covariance matrix Σ calculated
        STATUS_EXTRACTING    = 3'd2, // Power iteration active
        STATUS_CONVERGED     = 3'd3, // All K components extracted
        STATUS_PROJECTED     = 3'd4, // Latent projection complete
        STATUS_ERROR         = 3'd5
    } status_t;

endpackage : pca_types_pkg
