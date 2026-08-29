// =============================================================================
// File Name   : qubo_types_pkg.sv
// Module Name : qubo_types_pkg (SystemVerilog Package)
// Project     : QUBO / Simulated Annealing Ising Accelerator (Solver #18)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), 8-spin binary vectors,
//   8x8 QUBO coefficient matrices, temperature annealing schedules,
//   and status codes.
// =============================================================================

`timescale 1ns / 1ps

package qubo_types_pkg;

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

    // Simulated Annealing Temperature Defaults
    localparam q16_t Q16_T_START_DEF = 32'h000A_0000; // T_start = 10.0
    localparam q16_t Q16_T_END_DEF   = 32'h0000_028F; // T_end   = 0.01 (655 LSBs)
    localparam q16_t Q16_GAMMA_DEF   = 32'h0000_F333; // Cooling rate γ = 0.95 (62259/65536)
    localparam q16_t Q16_LOG2_E      = 32'h0001_7155; // log2(e) ≈ 1.442695 (94549/65536)
    localparam q16_t Q16_LN_2        = 32'h0000_B172; // ln(2)   ≈ 0.693147 (45426/65536)

    // -------------------------------------------------------------------------
    // SECTION 2: BINARY SPINS & QUBO MATRIX DIMENSIONS (N <= 8)
    // -------------------------------------------------------------------------
    localparam int MAX_SPINS         = 8;

    // Packed 8-spin binary decision vector q in {0, 1}^8 (8 bits)
    typedef logic [7:0] spin_vec_t;

    // Packed 8x8 QUBO Matrix (64 elements of 32 bits = 2048 bits)
    typedef logic signed [63:0][31:0] qubo_mat_t;

    // -------------------------------------------------------------------------
    // SECTION 3: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // Annealing complete down to T_end
        STATUS_MAX_ITERS   = 3'd3, // Maximum cooling cycles reached
        STATUS_ERROR       = 3'd4
    } status_t;

endpackage : qubo_types_pkg
