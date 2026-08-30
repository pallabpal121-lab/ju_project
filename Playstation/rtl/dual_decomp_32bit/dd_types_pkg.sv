// =============================================================================
// File Name   : dd_types_pkg.sv
// Module Name : dd_types_pkg (SystemVerilog Package)
// Project     : Dual Decomposition Engine (Solver #22)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), agent local decision vectors,
//   global resource constraint vectors, coupling matrices, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package dd_types_pkg;

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

    localparam q16_t Q16_ALPHA_DEF   = 32'h0000_4000; // Default step size α = 0.25
    localparam q16_t Q16_FEAS_TOL    = 32'h0000_0040; // Feasibility tolerance ε = 0.000976
    localparam q16_t Q16_OPT_TOL     = 32'h0000_0040; // Dual price tolerance ε = 0.000976

    // -------------------------------------------------------------------------
    // SECTION 2: MULTI-AGENT DIMENSIONS (S <= 4, N_s <= 2, M <= 2)
    // -------------------------------------------------------------------------
    localparam int MAX_AGENTS        = 4; // Up to 4 parallel local agents
    localparam int MAX_LOCAL_DIM     = 2; // Up to 2 local variables per agent
    localparam int MAX_RESOURCES     = 2; // Up to 2 global coupled resources

    // Packed 2-element local decision vector x_s
    typedef logic signed [1:0][31:0] agent_vec_t;

    // Packed all-agents decision vector (4 x 2)
    typedef logic signed [MAX_AGENTS-1:0][MAX_LOCAL_DIM-1:0][31:0] all_agents_vec_t;

    // Packed 2x2 local matrix Q_inv_s
    typedef logic signed [3:0][31:0] agent_mat_t;

    // Packed 2x2 coupling matrix A_s (M x N_s)
    typedef logic signed [3:0][31:0] couple_mat_t;

    // Packed 2-element global resource / dual price vector c, λ
    typedef logic signed [1:0][31:0] res_vec_t;

    // -------------------------------------------------------------------------
    // SECTION 3: COUPLING MODES & STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic {
        COUPLE_EQ   = 1'b0, // Equality budget: sum(A_s * x_s) = c
        COUPLE_INEQ = 1'b1  // Inequality capacity: sum(A_s * x_s) <= c
    } dd_couple_mode_t;

    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // Resource balance and price consensus reached
        STATUS_MAX_ITERS   = 3'd3, // Maximum iterations reached
        STATUS_ERROR       = 3'd4
    } status_t;

endpackage : dd_types_pkg
