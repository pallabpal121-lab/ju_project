// =============================================================================
// File Name   : nes_types_pkg.sv
// Module Name : nes_types_pkg (SystemVerilog Package)
// Project     : Natural Evolution Strategies (NES) Accelerator (Solver #32)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), policy parameter vectors (θ),
//   antithetic perturbation noise arrays (ε), reward vectors (R+, R-), gradient
//   vectors (g), and status codes.
// =============================================================================

`timescale 1ns / 1ps

package nes_types_pkg;

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
    // SECTION 2: DIMENSIONS & POPULATION PAIRS (P <= 4 pairs, D <= 4)
    // -------------------------------------------------------------------------
    localparam int MAX_PAIRS         = 4; // Up to 4 antithetic pairs (2P = 8 rollouts)
    localparam int MAX_DIM           = 4; // Up to 4 policy parameters (D)

    // Packed 4-element policy / gradient vector
    typedef logic signed [3:0][31:0] policy_vec_t;

    // Linearized 4 pairs x 4 dimensions noise perturbation array ε (idx = p*4 + dim)
    typedef logic signed [15:0][31:0] noise_arr_t;

    // Packed 4-element reward array for positive/negative mirrored evaluations
    typedef logic signed [3:0][31:0] reward_arr_t;

    // -------------------------------------------------------------------------
    // SECTION 3: REWARD FUNCTION TYPES
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        FN_QUADRATIC   = 2'd0, // R(θ) = -(θ0 - 3)^2 - 2(θ1 - 4)^2
        FN_ROSENBROCK  = 2'd1, // R(θ) = -10(θ1 - θ0^2)^2 - (1 - θ0)^2
        FN_SPHERE      = 2'd2, // R(θ) = -∑ (θ_d)^2
        FN_RASTRIGIN   = 2'd3  // R(θ) = -∑ (θ_d^2 + θ_d^4)
    } reward_fn_t;

    // -------------------------------------------------------------------------
    // SECTION 4: STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE         = 3'd0,
        STATUS_SAMPLING     = 3'd1, // Generating antithetic perturbations ±ε
        STATUS_EVALUATING   = 3'd2, // Evaluating mirrored policy rollouts R+, R-
        STATUS_GRAD_EST     = 3'd3, // Estimating stochastic policy gradient g
        STATUS_UPDATING     = 3'd4, // Updating policy parameters θ
        STATUS_CONVERGED    = 3'd5, // Gradient norm below target tolerance
        STATUS_MAX_ITERS    = 3'd6, // Maximum iterations reached
        STATUS_ERROR        = 3'd7
    } status_t;

endpackage : nes_types_pkg
