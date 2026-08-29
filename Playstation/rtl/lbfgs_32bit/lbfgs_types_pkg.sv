// =============================================================================
// File Name   : lbfgs_types_pkg.sv
// Module Name : lbfgs_types_pkg (SystemVerilog Package)
// Project     : Limited-Memory BFGS (L-BFGS) Hardware Accelerator (Solver #9)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Defines data types, vector formats, circular history buffers (O(mN) memory),
//   and microcode opcodes for the L-BFGS Two-Loop Recursion Accelerator.
//
// Key Concepts:
//   - Dimension N       : Up to MAX_PARAMS = 4
//   - Memory Depth M    : Up to MEM_DEPTH = 4 vector displacement pairs (s_i, y_i)
//   - Memory Footprint  : Only O(mN) = 4*4 = 16 words instead of O(N^2) full matrix!
//   - Two-Loop Recursion: Computes p = -H*g in silicon without storing H!
// =============================================================================

`timescale 1ns / 1ps

package lbfgs_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    localparam int Q_FRAC_BITS     = 16;
    localparam q16_t Q16_ONE       = 32'h0001_0000; // +1.0
    localparam q16_t Q16_HALF      = 32'h0000_8000; // +0.5
    localparam q16_t Q16_ZERO      = 32'h0000_0000; //  0.0
    localparam q16_t Q16_NEG_ONE   = 32'hFFFF_0000; // -1.0
    localparam q16_t Q16_EPS_DEF   = 32'h0000_0040; // Default tolerance ε ≈ 0.00097
    localparam q16_t Q16_STEP_H    = 32'h0000_1000; // Numerical step size h = 2^-4 = 0.0625

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & CIRCULAR HISTORY BUFFER DEFINITIONS
    // -------------------------------------------------------------------------
    localparam int MAX_PARAMS = 4; // Maximum parameter dimensions (N <= 4)
    localparam int MEM_DEPTH  = 4; // L-BFGS history depth (M <= 4)

    // Packed 4-element coordinate vector [x3, x2, x1, x0] (128 bits)
    typedef logic signed [127:0] vec_t;

    // Packed 4-vector circular history buffer [V3, V2, V1, V0] (512 bits)
    typedef logic signed [511:0] history_vec_t;

    // Packed 4-scalar history buffer [s3, s2, s1, s0] (128 bits)
    typedef logic signed [127:0] history_scalar_t;

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
        STATUS_CONVERGED   = 3'd2, // ||g||_inf <= tolerance
        STATUS_MAX_ITERS   = 3'd3, // Maximum iteration limit reached
        STATUS_DIV_BY_ZERO = 3'd4
    } status_t;

endpackage : lbfgs_types_pkg
