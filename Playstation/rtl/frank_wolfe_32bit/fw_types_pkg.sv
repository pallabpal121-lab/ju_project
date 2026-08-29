// =============================================================================
// File Name   : fw_types_pkg.sv
// Module Name : fw_types_pkg (SystemVerilog Package)
// Project     : Frank-Wolfe / Conditional Gradient Accelerator (Solver #20)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), 4D coordinate vectors,
//   4x4 quadratic matrices, constraint geometry types, step modes, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package fw_types_pkg;

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

    localparam q16_t Q16_TOL_DEF     = 32'h0000_0040; // Default Duality Gap tolerance ε = 0.000976

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & MATRIX DIMENSIONS (N <= 4)
    // -------------------------------------------------------------------------
    localparam int MAX_DIM           = 4;

    // Packed 4-element coordinate vector
    typedef logic signed [3:0][31:0] vec_t;

    // Packed 4x4 quadratic matrix Q (16 elements = 512 bits)
    typedef logic signed [15:0][31:0] mat_t;

    // -------------------------------------------------------------------------
    // SECTION 3: CONSTRAINT GEOMETRIES & STEP MODES
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        GEOM_L1_BALL = 2'd0, // L1 Ball: ||x||_1 <= R
        GEOM_BOX     = 2'd1, // Hyperbox: l_i <= x_i <= u_i
        GEOM_SIMPLEX = 2'd2  // Probability Simplex: sum(x_i) = 1, x_i >= 0
    } fw_geom_t;

    typedef enum logic [1:0] {
        STEP_EXACT_LINE_SEARCH = 2'd0, // Exact line search: γ = -g^T d / (d^T Q d)
        STEP_DIMINISHING       = 2'd1  // Diminishing step: γ = 2 / (k + 2)
    } fw_step_mode_t;

    // -------------------------------------------------------------------------
    // SECTION 4: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // Frank-Wolfe duality gap <= tolerance
        STATUS_MAX_ITERS   = 3'd3, // Maximum iterations reached
        STATUS_ERROR       = 3'd4
    } status_t;

endpackage : fw_types_pkg
