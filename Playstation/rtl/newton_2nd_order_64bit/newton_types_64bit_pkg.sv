// =============================================================================
// File Name   : newton_types_64bit_pkg.sv
// Module Name : newton_types_64bit_pkg (SystemVerilog Package)
// Project     : Universal Newton 2nd-Order Optimization Accelerator (64-Bit)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This package defines all custom types, Q32.32 fixed-point constants,
//   64-bit microcode instruction formats, and status flags for the 64-bit
//   high-precision Newton optimization accelerator.
//
// Key Concepts:
//   1. Q32.32 Signed Fixed-Point Format:
//      - 64-bit total width: 1 sign bit, 31 integer bits, 32 fractional bits.
//      - Ultra-high resolution: 2^-32 ≈ 2.328 × 10^-10.
//      - Range: [-2147483648.0, +2147483647.9999999997].
//   2. 64-Bit Microcode Instruction Word (instr64_t):
//      - Supports 64 registers (r0..r63), 64 program depth, and 32-bit immediates.
//   3. Finite-Difference Step (h = 2^-8 = 0.00390625):
//      - Enables zero-cost bit-shift calculus in 64-bit hardware.
// =============================================================================

`timescale 1ns / 1ps

package newton_types_64bit_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: 64-BIT FIXED-POINT NUMBER FORMAT (Q32.32 Signed)
    // -------------------------------------------------------------------------
    // Bit layout of a 64-bit signed Q32.32 fixed-point number:
    //
    //   Bit 63       Bits [62:32] (31 bits)        Bits [31:0] (32 bits)
    //  +-----------+-----------------------------+-----------------------------+
    //  | Sign Bit  |     Integer Magnitude       |     Fractional Value        |
    //  +-----------+-----------------------------+-----------------------------+
    //
    // Conversion Formula:
    //   Real Float Value = (Signed 64-bit Integer Value) / 4294967296.0 (i.e. 2^32)
    //   Integer in Hex   = Round(Real Float Value * 4294967296.0)
    // -------------------------------------------------------------------------
    typedef logic signed [63:0] q32_t;

    // Number of fractional bits in the Q32.32 format
    localparam int Q_FRAC_BITS_64   = 32;

    // Common Fixed-Point Constants in Q32.32 Format:
    localparam q32_t Q32_ONE        = 64'h0000_0001_0000_0000; //  1.0  = 1 * 2^32
    localparam q32_t Q32_HALF       = 64'h0000_0000_8000_0000; //  0.5  = 0.5 * 2^32
    localparam q32_t Q32_ZERO       = 64'h0000_0000_0000_0000; //  0.0  = 0
    localparam q32_t Q32_NEG_ONE    = 64'hFFFF_FFFF_0000_0000; // -1.0  = Two's complement of 1.0

    // Default Algorithm Parameters:
    localparam q32_t Q32_EPS_DEF    = 64'h0000_0000_0010_0000; // Default tolerance ε ≈ 0.000244 (|g| <= eps)
    localparam q32_t Q32_LAMBDA_DEF = 64'h0000_0000_0000_0100; // Default damping λ ≈ 0.00000006 (scaled with h^2)

    // -------------------------------------------------------------------------
    // SECTION 2: FINITE DIFFERENCE STEP PARAMETERS (Zero-Cost Shifts)
    // -------------------------------------------------------------------------
    // Perturbation step h = 2^-8 = 0.00390625.
    // In Q32.32 format: 0.00390625 * 2^32 = 2^24 = 64'h0000_0000_0100_0000.
    //
    // Why h = 2^-8 is optimal for 64-bit hardware:
    //   1. Multiplying by h is arithmetic right shift by 8 (>>> 8).
    //   2. Dividing by (2h = 2^-7) is arithmetic left shift by 7 (<<< 7).
    //   3. Multiplying by 2 is arithmetic left shift by 1 (<<< 1).
    // -------------------------------------------------------------------------
    localparam q32_t Q32_H_STEP     = 64'h0000_0000_0100_0000; // h = 2^-8 = 0.00390625

    // -------------------------------------------------------------------------
    // SECTION 3: MICRO-OPCODES FOR 64-BIT DFG ENGINE
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        OP_NOP    = 4'd0, // No Operation
        OP_ADD    = 4'd1, // r[dst] = r[src_a] + r[src_b]
        OP_SUB    = 4'd2, // r[dst] = r[src_a] - r[src_b]
        OP_MUL    = 4'd3, // r[dst] = (r[src_a] * r[src_b]) >> 32 (128-bit product sliced)
        OP_DIV    = 4'd4, // r[dst] = (r[src_a] << 32) / r[src_b] (96 cycles)
        OP_NEG    = 4'd5, // r[dst] = -r[src_a]
        OP_MOV    = 4'd6, // r[dst] = r[src_a]
        OP_LOADC  = 4'd7, // r[dst] = {imm[31:0], 32'h0000_0000} (converts 32-bit int to Q32.32)
        OP_END    = 4'd8  // End of program / Return result in r[63]
    } opcode_t;

    // -------------------------------------------------------------------------
    // SECTION 4: 64-BIT MICRO-INSTRUCTION FORMAT (64-bit packed struct)
    // -------------------------------------------------------------------------
    // Bit Fields:
    //   [63:60] (4 bits) : Opcode
    //   [59:54] (6 bits) : Destination Register (r0 to r63)
    //   [53:48] (6 bits) : Source A Register (r0 to r63)
    //   [47:42] (6 bits) : Source B Register (r0 to r63)
    //   [41:32] (10 bits): Reserved for future extensions
    //   [31:0]  (32 bits): Signed Immediate Constant
    // -------------------------------------------------------------------------
    typedef struct packed {
        opcode_t            op;       // 4-bit Opcode identifier
        logic [5:0]         dst;      // Destination register index (0..63)
        logic [5:0]         src_a;    // 1st operand register index (0..63)
        logic [5:0]         src_b;    // 2nd operand register index (0..63)
        logic [9:0]         reserved; // Reserved field
        logic signed [31:0] imm;      // 32-bit signed immediate constant
    } instr64_t;

    // Architecture Capacity Constraints:
    localparam int NUM_REGS_64   = 64; // 64 General Purpose Registers (r0..r63)
    localparam int PROG_DEPTH_64 = 64; // Microcode memory holds up to 64 instructions

    // Special Register Aliases:
    localparam int REG_X_64      = 0;  // r0 is ALWAYS pre-loaded with input variable 'x'
    localparam int REG_RESULT_64 = 63; // r63 is ALWAYS sampled as final output f(x) on OP_END

    // -------------------------------------------------------------------------
    // SECTION 5: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // Success: Gradient norm ||g|| <= tolerance
        STATUS_MAX_ITERS   = 3'd3, // Stopped: Reached maximum iteration limit
        STATUS_SINGULAR    = 3'd4, // Error: Hessian curvature denominator is singular/zero
        STATUS_DIV_BY_ZERO = 3'd5, // Error: Division by zero inside equation evaluation
        STATUS_OVERFLOW    = 3'd6  // Error: Arithmetic overflow occurred
    } status_t;

endpackage : newton_types_64bit_pkg
