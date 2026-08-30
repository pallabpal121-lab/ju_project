// =============================================================================
// File Name   : cem_types_pkg.sv
// Module Name : cem_types_pkg (SystemVerilog Package)
// Project     : Cross-Entropy Method (CEM) Accelerator (Solver #31)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), trajectory sample arrays,
//   distribution parameter vectors (mean μ, std dev σ), fitness arrays, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package cem_types_pkg;

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
    // SECTION 2: SAMPLING & DISTRIBUTION DIMENSIONS (S <= 8, D <= 4, K <= 4)
    // -------------------------------------------------------------------------
    localparam int MAX_SAMPLES       = 8; // Up to 8 candidate samples (S)
    localparam int MAX_DIM           = 4; // Up to 4 continuous parameters (D)
    localparam int MAX_ELITES        = 4; // Up to 4 elite individuals (K)

    // Packed 4-element single sample / parameter vector x
    typedef logic signed [3:0][31:0] param_vec_t;

    // Linearized 8 samples x 4 params sample array (idx = sample*4 + dim)
    typedef logic signed [31:0][31:0] sample_arr_t;

    // Packed 8-element sample fitness array
    typedef logic signed [7:0][31:0] fitness_arr_t;

    // -------------------------------------------------------------------------
    // SECTION 3: FITNESS FUNCTION TYPES
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        FN_QUADRATIC   = 2'd0, // f(x) = (x0 - 3)^2 + 2(x1 - 4)^2
        FN_ROSENBROCK  = 2'd1, // f(x) = 10(x1 - x0^2)^2 + (1 - x0)^2
        FN_SPHERE      = 2'd2, // f(x) = ∑ (x_d)^2
        FN_RASTRIGIN   = 2'd3  // f(x) = ∑ (x_d^2 + x_d^4)
    } fitness_fn_t;

    // -------------------------------------------------------------------------
    // SECTION 4: STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE         = 3'd0,
        STATUS_SAMPLING     = 3'd1, // Generating candidate samples from N(μ, σ^2)
        STATUS_EVALUATING   = 3'd2, // Evaluating sample fitness
        STATUS_UPDATING     = 3'd3, // Ranking elites and updating (μ, σ)
        STATUS_CONVERGED    = 3'd4, // Variance below target tolerance
        STATUS_MAX_GENS     = 3'd5, // Maximum iterations reached
        STATUS_ERROR        = 3'd6
    } status_t;

endpackage : cem_types_pkg
