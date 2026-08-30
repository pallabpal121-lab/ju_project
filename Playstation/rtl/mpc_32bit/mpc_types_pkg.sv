// =============================================================================
// File Name   : mpc_types_pkg.sv
// Module Name : mpc_types_pkg (SystemVerilog Package)
// Project     : Model Predictive Control (MPC) Accelerator (Solver #26)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), state vectors, control vectors,
//   stacked horizon vectors, Hessian matrices, gradient matrices, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package mpc_types_pkg;

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
    // SECTION 2: VECTOR & MATRIX DIMENSIONS (Nx <= 4, Nu <= 2, Stacked <= 4)
    // -------------------------------------------------------------------------
    localparam int MAX_STATE_DIM     = 4; // Up to 4 state variables (x)
    localparam int MAX_CTRL_DIM      = 2; // Up to 2 control inputs per step (u)
    localparam int MAX_STACKED_DIM   = 4; // Stacked control horizon (U = [u_0..u_{Np-1}])

    // Packed 4-element state vector x_curr, x_ref
    typedef logic signed [3:0][31:0] state_vec_t;

    // Packed 2-element immediate control vector u_0
    typedef logic signed [1:0][31:0] ctrl_vec_t;

    // Packed 4-element stacked control horizon vector U
    typedef logic signed [3:0][31:0] stacked_vec_t;

    // Packed 4x4 condensed Hessian matrix H_mpc
    typedef logic signed [15:0][31:0] hessian_mat_t;

    // Packed 4x4 condensed state mapping matrix M_x, M_ref
    typedef logic signed [15:0][31:0] grad_mat_t;

    // -------------------------------------------------------------------------
    // SECTION 3: STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_CONDENSED   = 3'd1, // Gradient computed
        STATUS_OPTIMIZED   = 3'd2, // Optimal control sequence found
        STATUS_SATURATED   = 3'd3, // Control saturated at physical limits
        STATUS_ERROR       = 3'd4
    } status_t;

endpackage : mpc_types_pkg
