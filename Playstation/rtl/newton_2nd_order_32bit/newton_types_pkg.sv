// =============================================================================
// File Name   : newton_types_pkg.sv
// Module Name : newton_types_pkg (SystemVerilog Package)
// Project     : Universal Newton 2nd-Order Optimization Accelerator
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This package serves as the "single source of truth" for all custom data types,
//   fixed-point mathematical constants, micro-opcodes, instruction layouts, and
//   status flags used throughout the entire Newton accelerator hardware design.
//
// Key Concepts Covered in this Package:
//   1. Q16.16 Signed Fixed-Point Arithmetic:
//      - How floating-point numbers (e.g. 1.0, 0.5, 0.0625) are represented as
//        standard 32-bit integers in binary hardware.
//   2. Microcode Instruction Format:
//      - How equations like f(x) = x^2 - 6x + 9 are encoded into 32-bit binary
//        instructions for the programmable DFG equation engine.
//   3. Solver Status Codes:
//      - How the hardware reports completion, convergence, or errors.
// =============================================================================

`timescale 1ns / 1ps

package newton_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    // Bit layout of a 32-bit signed Q16.16 fixed-point number:
    //
    //   Bit 31       Bits [30:16] (15 bits)        Bits [15:0] (16 bits)
    //  +-----------+-----------------------------+-----------------------------+
    //  | Sign Bit  |     Integer Magnitude       |     Fractional Value        |
    //  +-----------+-----------------------------+-----------------------------+
    //
    // Conversion Formula:
    //   Real Float Value = (Signed 32-bit Integer Value) / 65536.0 (i.e. 2^16)
    //   Integer in Hex   = Round(Real Float Value * 65536.0)
    //
    // Characteristics:
    //   - Maximum Positive Value : +32767.9999847 (32'h7FFF_FFFF)
    //   - Minimum Negative Value : -32768.0000000 (32'h8000_0000)
    //   - Smallest Step / LSB    : 2^-16 = 0.0000152587890625
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    // Number of fractional bits in the Q16.16 format
    localparam int Q_FRAC_BITS     = 16;

    // Common Fixed-Point Constants in Q16.16 Format:
    localparam q16_t Q16_ONE       = 32'h0001_0000; //  1.0  = 1 * 65536 = 0x00010000
    localparam q16_t Q16_HALF      = 32'h0000_8000; //  0.5  = 0.5 * 65536 = 0x00008000
    localparam q16_t Q16_ZERO      = 32'h0000_0000; //  0.0  = 0
    localparam q16_t Q16_NEG_ONE   = 32'hFFFF_0000; // -1.0  = Two's complement of 0x00010000

    // Default Algorithm Parameters:
    localparam q16_t Q16_EPS_DEF   = 32'h0000_0040; // Default convergence tolerance ε ≈ 0.000976 (|g| <= eps)
    localparam q16_t Q16_LAMBDA_DEF= 32'h0000_0100; // Default Levenberg-Marquardt damping factor λ ≈ 0.003906

    // -------------------------------------------------------------------------
    // SECTION 2: FINITE DIFFERENCE STEP PARAMETERS (Zero-Cost Shifts)
    // -------------------------------------------------------------------------
    // To calculate derivatives f'(x) and f''(x) without symbolic math, the hardware
    // samples f(x) at points x, (x + h), and (x - h).
    //
    // We choose perturbation step h = 2^-4 = 0.0625 (Hex: 32'h0000_1000).
    // Why choose a power of 2 for h?
    //   1. Multiplying by h is a simple arithmetic right-shift by 4 bits (>>> 4).
    //   2. Dividing by (2h = 0.125 = 1/8) is an arithmetic left-shift by 3 bits (<<< 3).
    //   3. Dividing by (h^2 = 1/256) is an arithmetic left-shift by 8 bits (<<< 8).
    // This eliminates multiple expensive multipliers/dividers from the derivative engine!
    // -------------------------------------------------------------------------
    localparam q16_t Q16_H_STEP    = 32'h0000_1000; // h = 0.0625 (Q16.16 = 32'h0000_1000)

    // -------------------------------------------------------------------------
    // SECTION 3: MICRO-OPCODES FOR PROGRAMMABLE EQUATION DFG ENGINE
    // -------------------------------------------------------------------------
    // These 4-bit opcodes define the instruction set supported by the DFG processor.
    // Each opcode directs the ALU or Divider to execute a specific mathematical step.
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        OP_NOP    = 4'd0, // No Operation: Does nothing (r[dst] remains unchanged)
        OP_ADD    = 4'd1, // Addition:     r[dst] = r[src_a] + r[src_b]
        OP_SUB    = 4'd2, // Subtraction:  r[dst] = r[src_a] - r[src_b]
        OP_MUL    = 4'd3, // Multiply:     r[dst] = (r[src_a] * r[src_b]) >> 16
        OP_DIV    = 4'd4, // Division:     r[dst] = (r[src_a] << 16) / r[src_b] (48 cycles)
        OP_NEG    = 4'd5, // Negate:       r[dst] = -r[src_a]
        OP_MOV    = 4'd6, // Move/Copy:    r[dst] = r[src_a]
        OP_LOADC  = 4'd7, // Load Const:   r[dst] = {imm[15:0], 16'h0000} (Converts integer imm to Q16.16)
        OP_END    = 4'd8  // End Program:  Signals evaluation complete, outputs r[15] as f(x)
    } opcode_t;

    // -------------------------------------------------------------------------
    // SECTION 4: MICRO-INSTRUCTION WORD FORMAT (32-bit packed struct)
    // -------------------------------------------------------------------------
    // Bit Fields:
    //   [31:28] (4 bits) : Opcode (e.g. OP_ADD, OP_MUL)
    //   [27:24] (4 bits) : Destination Register Index (r0 to r15)
    //   [23:20] (4 bits) : Source A Register Index (r0 to r15)
    //   [19:16] (4 bits) : Source B Register Index (r0 to r15)
    //   [15:0]  (16 bits): Signed Immediate Constant (used by OP_LOADC)
    // -------------------------------------------------------------------------
    typedef struct packed {
        opcode_t            op;    // 4-bit Opcode identifier
        logic [3:0]         dst;   // Destination register address (0..15)
        logic [3:0]         src_a; // 1st operand register address (0..15)
        logic [3:0]         src_b; // 2nd operand register address (0..15)
        logic signed [15:0] imm;   // 16-bit immediate constant
    } instr_t;

    // Architecture Capacity Constraints:
    localparam int NUM_REGS   = 16; // 16 General Purpose Registers (r0 to r15)
    localparam int PROG_DEPTH = 32; // Microcode memory holds up to 32 instructions

    // Special Register Aliases:
    localparam int REG_X      = 0;  // r0 is ALWAYS pre-loaded with input variable 'x'
    localparam int REG_RESULT = 15; // r15 is ALWAYS sampled as final output f(x) on OP_END

    // -------------------------------------------------------------------------
    // SECTION 5: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    // The top-level solver outputs a 3-bit status code upon asserting 'done'.
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0, // System is idle, waiting for 'start' pulse
        STATUS_RUNNING     = 3'd1, // Iterative Newton loop is currently running
        STATUS_CONVERGED   = 3'd2, // Success: Gradient norm ||g|| <= tolerance
        STATUS_MAX_ITERS   = 3'd3, // Stopped: Reached user-defined max iteration limit
        STATUS_SINGULAR    = 3'd4, // Error: Hessian curvature denominator is zero / singular
        STATUS_DIV_BY_ZERO = 3'd5, // Error: Division by zero encountered inside equation evaluation
        STATUS_OVERFLOW    = 3'd6  // Error: Arithmetic overflow occurred
    } status_t;

endpackage : newton_types_pkg
