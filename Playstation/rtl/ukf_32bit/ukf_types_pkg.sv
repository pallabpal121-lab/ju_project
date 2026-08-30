// =============================================================================
// File Name   : ukf_types_pkg.sv
// Module Name : ukf_types_pkg (SystemVerilog Package)
// Project     : Unscented Kalman Filter (UKF) Accelerator (Solver #25)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), state vectors, measurement
//   vectors, 2N+1 sigma point arrays, weights, covariance matrices, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package ukf_types_pkg;

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

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & MATRIX DIMENSIONS (N <= 4, M <= 2, Points <= 9)
    // -------------------------------------------------------------------------
    localparam int MAX_STATE_DIM     = 4; // Up to 4 state variables (x)
    localparam int MAX_MEAS_DIM      = 2; // Up to 2 measurement channels (z)
    localparam int MAX_SIGMA_POINTS  = 9; // 2*N + 1 = 9 sigma points

    // Packed 4-element state vector x, u
    typedef logic signed [3:0][31:0] state_vec_t;

    // Packed 2-element measurement vector z, y
    typedef logic signed [1:0][31:0] meas_vec_t;

    // Linearized 9 x 4 sigma point state array: 36 elements (idx = p*4 + d)
    typedef logic signed [35:0][31:0] sigma_state_arr_t;

    // Linearized 9 x 2 sigma point measurement array: 18 elements (idx = p*2 + d)
    typedef logic signed [17:0][31:0] sigma_meas_arr_t;

    // Packed 9-element sigma point weights array W_m, W_c
    typedef logic signed [8:0][31:0] weights_arr_t;

    // Packed 4x4 state covariance matrix P, Q, L
    typedef logic signed [15:0][31:0] state_mat_t;

    // Packed 4x2 cross-covariance & Kalman gain matrix P_xz, K (4 rows x 2 cols)
    typedef logic signed [7:0][31:0] cross_mat_t;

    // Packed 2x2 innovation covariance matrix P_zz, R, P_zz_inv
    typedef logic signed [3:0][31:0] innov_mat_t;

    // -------------------------------------------------------------------------
    // SECTION 3: STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_SIGMAS      = 3'd1, // Sigma points generated
        STATUS_PREDICTED   = 3'd2, // Time update complete
        STATUS_UPDATED     = 3'd3, // Measurement update complete
        STATUS_ERROR       = 3'd4
    } status_t;

endpackage : ukf_types_pkg
