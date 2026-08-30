// =============================================================================
// File Name   : mppi_types_pkg.sv
// Module Name : mppi_types_pkg (SystemVerilog Package)
// Project     : Model Predictive Path Integral (MPPI) Accelerator (Solver #33)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), state vectors (x), control
//   vectors (u), horizon sequences (U), rollout cost arrays (S), importance
//   weights (w), and status codes using packed vectors.
// =============================================================================

`timescale 1ns / 1ps

package mppi_types_pkg;

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
    // SECTION 2: DIMENSIONS (T <= 4 horizon, Ns <= 2 states, M <= 2 controls, K <= 4 rollouts)
    // -------------------------------------------------------------------------
    localparam int MAX_HORIZON       = 4; // Prediction horizon T (1..4)
    localparam int MAX_STATE         = 2; // State dimension Ns (1..2)
    localparam int MAX_CTRL          = 2; // Control dimension M (1..2)
    localparam int MAX_ROLLOUTS      = 4; // Stochastic rollouts K (1..4)

    // Packed 2-element state vector x (2x1)
    typedef logic signed [1:0][31:0] state_vec_t;

    // Packed 2-element control vector u (2x1)
    typedef logic signed [1:0][31:0] ctrl_vec_t;

    // Packed 2x2 matrix (idx = r*2 + c)
    typedef logic signed [3:0][31:0] mat22_t;

    // Packed horizon control sequence U (4 steps x 2 controls = 8 elements, idx = t*2 + m)
    typedef logic signed [7:0][31:0] ctrl_seq_t;

    // Packed noise perturbations array (4 rollouts x 4 steps x 2 controls = 32 elements, idx = (k*4 + t)*2 + m)
    typedef logic signed [31:0][31:0] noise_arr_t;

    // Packed 4-element rollout cost array (4 rollouts)
    typedef logic signed [3:0][31:0] cost_arr_t;

    // Packed 4-element importance weights array (4 rollouts)
    typedef logic signed [3:0][31:0] weight_arr_t;

    // -------------------------------------------------------------------------
    // SECTION 3: STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE         = 3'd0,
        STATUS_ROLLOUT      = 3'd1, // Generating stochastic rollouts and forward dynamics
        STATUS_WEIGHTS      = 3'd2, // Computing softmax importance sampling weights
        STATUS_UPDATE       = 3'd3, // Updating control sequence and streaming u0*
        STATUS_DONE         = 3'd4, // MPPI step complete
        STATUS_ERROR        = 3'd5
    } status_t;

endpackage : mppi_types_pkg
