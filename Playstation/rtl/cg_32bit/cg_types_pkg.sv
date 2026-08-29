// =============================================================================
// File Name   : cg_types_pkg.sv
// Module Name : cg_types_pkg (SystemVerilog Package)
// Project     : Non-Linear Conjugate Gradient (CG) Accelerator (Solver #5)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This package defines all data types, vector formats, opcodes, and status
//   codes for the matrix-free Non-Linear Conjugate Gradient (CG) Accelerator.
//
// Key Concepts:
//   - Position Vector x        : [x0, x1, x2, x3] (up to MAX_VARS = 4)
//   - Gradient Vector g        : [g0, g1, g2, g3] = ∇f(x)
//   - Conjugate Direction d    : d_0 = -g_0, d_(k+1) = -g_(k+1) + β · d_k
//   - Directional Curvature κ  : κ = dᵀ H d = (f(x+hd) - 2f(x) + f(x-hd)) << 8
//   - Optimal Step Size α      : α = -(gᵀ d) / κ
//   - Polak-Ribière Parameter β: β = max(0, g_(k+1)ᵀ(g_(k+1) - g_k) / (g_kᵀ g_k))
// =============================================================================

`timescale 1ns / 1ps

package cg_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    localparam int Q_FRAC_BITS       = 16;
    localparam q16_t Q16_ONE         = 32'h0001_0000; // +1.0
    localparam q16_t Q16_HALF        = 32'h0000_8000; // +0.5
    localparam q16_t Q16_ZERO        = 32'h0000_0000; //  0.0
    localparam q16_t Q16_NEG_ONE     = 32'hFFFF_0000; // -1.0
    localparam q16_t Q16_EPS_DEF     = 32'h0000_0080; // Default tolerance ε ≈ 0.00195
    localparam q16_t Q16_H_STEP      = 32'h0000_1000; // Finite difference step h = 0.0625 = 2^-4

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR DEFINITIONS (Matrix-Free O(N) Memory!)
    // -------------------------------------------------------------------------
    localparam int MAX_VARS = 4; // Maximum supported variable dimensions (N <= 4)

    // Packed 4-element vector [x3, x2, x1, x0] (128 bits)
    typedef logic signed [127:0] vec_t;

    // -------------------------------------------------------------------------
    // SECTION 3: MICROCODE INSTRUCTION SET FOR DFG MODEL EVALUATION
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        OP_NOP    = 4'd0, // No Operation
        OP_ADD    = 4'd1, // r[dst] = r[src_a] + r[src_b]
        OP_SUB    = 4'd2, // r[dst] = r[src_a] - r[src_b]
        OP_MUL    = 4'd3, // r[dst] = (r[src_a] * r[src_b]) >> 16
        OP_DIV    = 4'd4, // r[dst] = (r[src_a] << 16) / r[src_b]
        OP_NEG    = 4'd5, // r[dst] = -r[src_a]
        OP_MOV    = 4'd6, // r[dst] = r[src_a]
        OP_LOADC  = 4'd7, // r[dst] = {imm[15:0], 16'h0000}
        OP_END    = 4'd8  // End of program / Output scalar f(x) in r[15]
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

    localparam int REG_RESULT = 15;

    // -------------------------------------------------------------------------
    // SECTION 4: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // ||g||_inf <= tolerance
        STATUS_MAX_ITERS   = 3'd3, // Maximum iteration limit reached
        STATUS_SINGULAR    = 3'd4, // Negative curvature
        STATUS_DIV_BY_ZERO = 3'd5,
        STATUS_OVERFLOW    = 3'd6
    } status_t;

endpackage : cg_types_pkg
