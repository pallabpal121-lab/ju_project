// =============================================================================
// File Name   : pgd_types_pkg.sv
// Module Name : pgd_types_pkg (SystemVerilog Package)
// Project     : Projected Gradient Descent (PGD) Accelerator (Solver #12)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Defines data types, projection geometry modes, microcode DFG formats,
//   and status codes for the Projected Gradient Descent Hardware Accelerator.
// =============================================================================

`timescale 1ns / 1ps

package pgd_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    localparam int Q_FRAC_BITS   = 16;
    localparam q16_t Q16_ONE     = 32'h0001_0000; // +1.0
    localparam q16_t Q16_HALF    = 32'h0000_8000; // +0.5
    localparam q16_t Q16_ZERO    = 32'h0000_0000; //  0.0
    localparam q16_t Q16_NEG_ONE = 32'hFFFF_0000; // -1.0
    localparam q16_t Q16_EPS_DEF = 32'h0000_0040; // Default tolerance ε ≈ 0.00097
    localparam q16_t Q16_ALPHA_DEF = 32'h0000_2000; // Default step size alpha = 0.125

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR DEFINITIONS
    // -------------------------------------------------------------------------
    localparam int MAX_PARAMS = 4; // Maximum coordinates (N <= 4)
    localparam int PROG_DEPTH = 32;
    localparam int NUM_REGS   = 16;

    // Packed 4-element coordinate vector [x3, x2, x1, x0] (128 bits)
    typedef logic signed [127:0] vec_t;

    // -------------------------------------------------------------------------
    // SECTION 3: PROJECTION GEOMETRY MODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        PROJ_NONE         = 3'd0, // Unconstrained Gradient Descent
        PROJ_NON_NEGATIVE = 3'd1, // x_i >= 0  (max(x, 0))
        PROJ_BOX          = 3'd2, // l_i <= x_i <= u_i (clamp(x, l, u))
        PROJ_L2_BALL      = 3'd3, // ||x||_2 <= R (x * min(1, R / ||x||_2))
        PROJ_SIMPLEX      = 3'd4  // sum x_i = 1, x_i >= 0 (Probability Simplex)
    } proj_mode_t;

    // -------------------------------------------------------------------------
    // SECTION 4: PROGRAMMABLE DFG MICROCODE INSTRUCTION FORMAT
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        OP_NOP   = 4'd0,
        OP_ADD   = 4'd1,
        OP_SUB   = 4'd2,
        OP_MUL   = 4'd3,
        OP_LOAD  = 4'd4,
        OP_LOADI = 4'd5,
        OP_NEG   = 4'd6,
        OP_ABS   = 4'd7
    } opcode_t;

    typedef struct packed {
        opcode_t            op;
        logic [3:0]         dst;
        logic [3:0]         src_a;
        logic [3:0]         src_b;
        logic signed [15:0] imm;
    } instr_t;

    // -------------------------------------------------------------------------
    // SECTION 5: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // ||delta x||_inf <= tolerance
        STATUS_MAX_ITERS   = 3'd3, // Maximum iteration limit reached
        STATUS_DIV_BY_ZERO = 3'd4
    } status_t;

endpackage : pgd_types_pkg
