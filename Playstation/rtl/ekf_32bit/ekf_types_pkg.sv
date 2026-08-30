// =============================================================================
// File Name   : ekf_types_pkg.sv
// Module Name : ekf_types_pkg (SystemVerilog Package)
// Project     : Extended Kalman Filter (EKF) Accelerator (Solver #24)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), state vectors, measurement
//   vectors, state transition matrices, measurement Jacobians, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package ekf_types_pkg;

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

    localparam q16_t Q16_DAMP_DEF    = 32'h0000_0004; // Minimum diagonal regularizer

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & MATRIX DIMENSIONS (N <= 4, M <= 2)
    // -------------------------------------------------------------------------
    localparam int MAX_STATE_DIM     = 4; // Up to 4 state variables (x)
    localparam int MAX_MEAS_DIM      = 2; // Up to 2 measurement channels (z)

    // Packed 4-element state vector x, f(x), u
    typedef logic signed [3:0][31:0] state_vec_t;

    // Packed 2-element measurement vector z, h(x), y (innovation)
    typedef logic signed [1:0][31:0] meas_vec_t;

    // Packed 4x4 state matrix P, F, Q (row-major: [row][col])
    typedef logic signed [15:0][31:0] state_mat_t;

    // Packed 2x4 measurement Jacobian H (row-major: 2 rows x 4 cols)
    typedef logic signed [7:0][31:0] meas_mat_t;

    // Packed 4x2 Kalman gain matrix K (row-major: 4 rows x 2 cols)
    typedef logic signed [7:0][31:0] gain_mat_t;

    // Packed 2x2 innovation covariance matrix S, R, S_inv
    typedef logic signed [3:0][31:0] innov_mat_t;

    // -------------------------------------------------------------------------
    // SECTION 3: STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_PREDICTED   = 3'd1, // Time update / state prediction complete
        STATUS_UPDATED     = 3'd2, // Measurement update complete
        STATUS_ERROR       = 3'd3
    } status_t;

endpackage : ekf_types_pkg
