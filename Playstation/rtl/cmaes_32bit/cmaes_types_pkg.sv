// =============================================================================
// File Name   : cmaes_types_pkg.sv
// Module Name : cmaes_types_pkg (SystemVerilog Package)
// Project     : Covariance Matrix Adaptation Evolution Strategy (CMA-ES) (Solver #34)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), parameter vectors (x, m, y, z),
//   covariance matrices (C, A), candidate arrays, fitness arrays, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package cmaes_types_pkg;

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
    // SECTION 2: DIMENSIONS (D <= 4, Lambda <= 4, Mu = 2)
    // -------------------------------------------------------------------------
    localparam int MAX_DIM           = 4; // Dimension D (1..4)
    localparam int MAX_POP           = 4; // Population size λ (2..4)
    localparam int MAX_ELITES        = 2; // Elite count μ (2)

    // Packed 4-element parameter vector x, m, z, y, p_sigma, p_c (4x1)
    typedef logic signed [3:0][31:0] param_vec_t;

    // Packed 4x4 matrix C, A (16 elements, idx = r*4 + c)
    typedef logic signed [15:0][31:0] cov_mat_t;

    // Packed candidate array (4 candidates x 4 dims = 16 elements, idx = k*4 + d)
    typedef logic signed [15:0][31:0] cand_arr_t;

    // Packed 4-element fitness array (4 candidates)
    typedef logic signed [3:0][31:0] fitness_arr_t;

    // Objective function selection
    typedef enum logic [1:0] {
        FN_QUADRATIC  = 2'd0, // Decoupled quadratic paraboloid
        FN_ROSENBROCK = 2'd1, // Non-convex curved Rosenbrock valley
        FN_RASTRIGIN  = 2'd2  // Multi-modal landscape
    } cmaes_fn_t;

    // -------------------------------------------------------------------------
    // SECTION 3: STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE         = 3'd0,
        STATUS_SAMPLING     = 3'd1, // Generating anisotropic candidates
        STATUS_EVALUATING   = 3'd2, // Evaluating objective fitnesses
        STATUS_UPDATING     = 3'd3, // Recombination, step-size & covariance update
        STATUS_CONVERGED    = 3'd4, // Stopping tolerance reached
        STATUS_MAX_ITERS    = 3'd5, // Max iterations reached
        STATUS_ERROR        = 3'd6
    } status_t;

endpackage : cmaes_types_pkg
