// =============================================================================
// File Name   : de_types_pkg.sv
// Module Name : de_types_pkg (SystemVerilog Package)
// Project     : Differential Evolution (DE) Accelerator (Solver #30)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), population structures,
//   individual vectors, fitness arrays, differential strategy types, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package de_types_pkg;

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
    // SECTION 2: POPULATION & GENE DIMENSIONS (Np <= 8, D <= 4)
    // -------------------------------------------------------------------------
    localparam int MAX_POP           = 8; // Up to 8 individuals
    localparam int MAX_DIM           = 4; // Up to 4 continuous parameters

    // Packed 4-element single individual gene/parameter vector x
    typedef logic signed [3:0][31:0] gene_vec_t;

    // Linearized 8 individuals x 4 genes population array (idx = ind*4 + dim)
    typedef logic signed [31:0][31:0] pop_arr_t;

    // Packed 8-element population fitness array
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
        STATUS_EVAL_INIT    = 3'd1, // Initial fitness evaluation
        STATUS_EVOLVING     = 3'd2, // Generation mutation, crossover & selection
        STATUS_CONVERGED    = 3'd3, // Fitness below target tolerance
        STATUS_MAX_GENS     = 3'd4, // Maximum generations reached
        STATUS_ERROR        = 3'd5
    } status_t;

endpackage : de_types_pkg
