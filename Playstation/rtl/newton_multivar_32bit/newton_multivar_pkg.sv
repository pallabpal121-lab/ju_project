// =============================================================================
// File Name   : newton_multivar_pkg.sv
// Module Name : newton_multivar_pkg (SystemVerilog Package)
// Project     : Universal Multivariable Newton 2nd-Order Accelerator
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This package defines all vector, matrix, and microcode data types used by
//   the Multivariable N-Dimensional Newton Optimization Accelerator.
// =============================================================================

`timescale 1ns / 1ps

package newton_multivar_pkg;

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
    localparam q16_t Q16_LAMBDA_DEF  = 32'h0000_0400; // Default damping λ ≈ 0.0156
    localparam q16_t Q16_H_STEP      = 32'h0000_1000; // h = 0.0625 = 2^-4

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & MATRIX DEFINITIONS (Robust Packed Bit-Vectors)
    // -------------------------------------------------------------------------
    localparam int MAX_VARS = 4; // Maximum supported variable dimensions (N <= 4)

    // Packed Vector: 4 x 32-bit = 128-bit vector [x3, x2, x1, x0]
    typedef logic signed [127:0] vec_t;

    // Packed Matrix: 4 x 4 x 32-bit = 512-bit matrix
    typedef logic signed [511:0] mat_t;

    // -------------------------------------------------------------------------
    // SECTION 3: MICRO-OPCODES FOR MULTIVARIABLE DFG ENGINE
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

    // -------------------------------------------------------------------------
    // SECTION 4: 32-BIT MICRO-INSTRUCTION WORD FORMAT
    // -------------------------------------------------------------------------
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
    // SECTION 5: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // ||g|| <= tolerance
        STATUS_MAX_ITERS   = 3'd3, // Max iteration limit reached
        STATUS_SINGULAR    = 3'd4, // Hessian matrix non-invertible / Cholesky failed
        STATUS_DIV_BY_ZERO = 3'd5, // Arithmetic error
        STATUS_OVERFLOW    = 3'd6
    } status_t;

endpackage : newton_multivar_pkg
