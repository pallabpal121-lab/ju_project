// =============================================================================
// File Name   : admm_types_pkg.sv
// Module Name : admm_types_pkg (SystemVerilog Package)
// Project     : Alternating Direction Method of Multipliers (ADMM) Accelerator (Solver #11)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Defines data types, vector formats, matrices, dataset buffers, and status
//   codes for the 3-Phase ADMM Hardware Accelerator.
//
// Key Concepts:
//   - Dimension N       : Up to MAX_PARAMS = 4 features/variables
//   - Observations M    : Up to MAX_OBS = 8 data samples
//   - Primal x-update   : x_{k+1} = (A^T*A + rho*I)^-1 * (A^T*b + rho*(z_k - u_k))
//   - Primal z-update   : z_{k+1} = S_{lambda/rho}(x_{k+1} + u_k) (Soft-thresholding)
//   - Dual u-update     : u_{k+1} = u_k + (x_{k+1} - z_{k+1})
// =============================================================================

`timescale 1ns / 1ps

package admm_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    localparam int Q_FRAC_BITS   = 16;
    localparam q16_t Q16_ONE     = 32'h0001_0000; // +1.0
    localparam q16_t Q16_HALF    = 32'h0000_8000; // +0.5
    localparam q16_t Q16_ZERO    = 32'h0000_0000; //  0.0
    localparam q16_t Q16_NEG_ONE = 32'hFFFF_0000; // -1.0
    localparam q16_t Q16_EPS_DEF = 32'h0000_0040; // Default tolerance ε ≈ 0.00097
    localparam q16_t Q16_RHO_DEF = 32'h0001_0000; // Default augmented Lagrangian penalty rho = 1.0

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & MATRIX DEFINITIONS
    // -------------------------------------------------------------------------
    localparam int MAX_PARAMS = 4; // Maximum features (N <= 4)
    localparam int MAX_OBS    = 8; // Maximum observations (M <= 8)

    // Packed 4-element coordinate vector [x3, x2, x1, x0] (128 bits)
    typedef logic signed [127:0] vec_t;

    // Packed 8-element observation target vector [b7..b0] (256 bits)
    typedef logic signed [255:0] obs_vec_t;

    // Packed 4x4 symmetric matrix [Row3..Row0] (512 bits)
    typedef logic signed [511:0] mat_t;

    // Packed 8x4 dataset feature matrix A [Row7..Row0] (1024 bits)
    typedef logic signed [1023:0] dataset_mat_t;

    // -------------------------------------------------------------------------
    // SECTION 3: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // ||r_pri||_inf <= tol AND ||s_dual||_inf <= tol
        STATUS_MAX_ITERS   = 3'd3, // Maximum iteration limit reached
        STATUS_DIV_BY_ZERO = 3'd4
    } status_t;

endpackage : admm_types_pkg
