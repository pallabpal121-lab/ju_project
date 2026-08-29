// =============================================================================
// File Name   : nm_types_pkg.sv
// Module Name : nm_types_pkg (SystemVerilog Package)
// Project     : Nelder-Mead Simplex Direct Search Accelerator (Solver #8)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Defines all data types, vector formats, simplex structures (N+1 vertices),
//   and microcode opcodes for the Nelder-Mead Derivative-Free Accelerator.
//
// Key Concepts:
//   - Dimension N       : Up to MAX_PARAMS = 4
//   - Simplex Size M    : N + 1 vertices (up to MAX_VERTICES = 5)
//   - Geometric Updates : Reflection (α=1), Expansion (γ=2), Contraction (ρ=0.5), Shrink (σ=0.5)
//   - Non-Smooth OP_ABS : Allows optimizing non-differentiable / noisy functions!
// =============================================================================

`timescale 1ns / 1ps

package nm_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    localparam int Q_FRAC_BITS       = 16;
    localparam q16_t Q16_ONE         = 32'h0001_0000; // +1.0
    localparam q16_t Q16_HALF        = 32'h0000_8000; // +0.5
    localparam q16_t Q16_TWO         = 32'h0002_0000; // +2.0
    localparam q16_t Q16_ZERO        = 32'h0000_0000; //  0.0
    localparam q16_t Q16_NEG_ONE     = 32'hFFFF_0000; // -1.0
    localparam q16_t Q16_EPS_DEF     = 32'h0000_0080; // Default tolerance ε ≈ 0.00195
    localparam q16_t Q16_INIT_STEP   = 32'h0001_0000; // Default initial simplex step size = 1.0

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & SIMPLEX DEFINITIONS
    // -------------------------------------------------------------------------
    localparam int MAX_PARAMS   = 4; // Maximum parameter dimensions (N <= 4)
    localparam int MAX_VERTICES = 5; // Maximum simplex vertices (N + 1 <= 5)

    // Packed 4-element coordinate vector [x3, x2, x1, x0] (128 bits)
    typedef logic signed [127:0] vec_t;

    // Packed 5-vertex simplex coordinate table [V4, V3, V2, V1, V0] (640 bits)
    typedef logic signed [639:0] simplex_vec_t;

    // Packed 5-element fitness array [f4, f3, f2, f1, f0] (160 bits)
    typedef logic signed [159:0] simplex_f_t;

    // -------------------------------------------------------------------------
    // SECTION 3: MICROCODE INSTRUCTION SET FOR DFG OBJECTIVE EVALUATION
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        OP_NOP    = 4'd0, // No Operation
        OP_ADD    = 4'd1, // r[dst] = r[src_a] + r[src_b]
        OP_SUB    = 4'd2, // r[dst] = r[src_a] - r[src_b]
        OP_MUL    = 4'd3, // r[dst] = (r[src_a] * r[src_b]) >> 16
        OP_DIV    = 4'd4, // r[dst] = (r[src_a] << 16) / r[src_b]
        OP_NEG    = 4'd5, // r[dst] = -r[src_a]
        OP_ABS    = 4'd6, // r[dst] = |r[src_a]| (for non-smooth optimization!)
        OP_MOV    = 4'd7, // r[dst] = r[src_a]
        OP_LOADC  = 4'd8, // r[dst] = {imm[15:0], 16'h0000}
        OP_END    = 4'd9  // End of program / Output scalar f(x) in r[15]
    } opcode_t;

    typedef struct packed {
        opcode_t            op;    // 4-bit Opcode
        logic [3:0]         dst;   // Destination register index (r0..r15)
        logic [3:0]         src_a; // Source A register index (r0..r15)
        logic [3:0]         src_b; // Source B register index (r0..r15)
        logic signed [15:0] imm;   // 16-bit signed immediate constant
    } instr_t;

    localparam int NUM_REGS   = 16;
    localparam int PROG_DEPTH = 32;

    localparam int REG_X0     = 0;  // Register r0 holds parameter x0
    localparam int REG_X1     = 1;  // Register r1 holds parameter x1
    localparam int REG_X2     = 2;  // Register r2 holds parameter x2
    localparam int REG_X3     = 3;  // Register r3 holds parameter x3
    localparam int REG_RESULT = 15; // Register r15 holds output objective f(x)

    // -------------------------------------------------------------------------
    // SECTION 4: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // Simplex diameter or fitness spread <= tolerance
        STATUS_MAX_ITERS   = 3'd3, // Maximum iteration limit reached
        STATUS_DIV_BY_ZERO = 3'd4
    } status_t;

endpackage : nm_types_pkg
