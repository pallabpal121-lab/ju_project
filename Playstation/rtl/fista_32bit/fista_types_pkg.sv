// =============================================================================
// File Name   : fista_types_pkg.sv
// Module Name : fista_types_pkg (SystemVerilog Package)
// Project     : Fast Iterative Shrinkage-Thresholding Algorithm (FISTA) Accelerator (Solver #15)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), vector structures,
//   DFG microcode instructions, default step sizes, regularization parameters,
//   Nesterov momentum scalar constants, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package fista_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    localparam int Q_FRAC_BITS     = 16;
    localparam q16_t Q16_ONE       = 32'h0001_0000; // +1.0
    localparam q16_t Q16_TWO       = 32'h0002_0000; // +2.0
    localparam q16_t Q16_FOUR      = 32'h0004_0000; // +4.0
    localparam q16_t Q16_HALF      = 32'h0000_8000; // +0.5
    localparam q16_t Q16_ZERO      = 32'h0000_0000; //  0.0
    localparam q16_t Q16_NEG_ONE   = 32'hFFFF_0000; // -1.0
    localparam q16_t Q16_EPS_DEF   = 32'h0000_0080; // Default convergence tolerance ε ≈ 0.00195

    // Default Algorithm Hyperparameters
    localparam q16_t Q16_GAMMA_DEF = 32'h0000_199A; // Step size γ = 0.1
    localparam q16_t Q16_LAMBDA_DEF= 32'h0001_0000; // L1 Regularization λ = 1.0

    // Numerical Gradient Perturbation Step Size h = 2^-4 = 0.0625
    localparam int H_SHIFT         = 4;
    localparam q16_t Q16_H_STEP    = 32'h0000_1000; // 0.0625

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & MATRIX DIMENSIONS (N <= 4)
    // -------------------------------------------------------------------------
    localparam int MAX_PARAMS = 4;

    // Packed 4-element coordinate vector [x3, x2, x1, x0] (128 bits)
    typedef logic signed [127:0] vec_t;

    // -------------------------------------------------------------------------
    // SECTION 3: MICROCODE INSTRUCTION SET FOR DFG EVALUATION
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
        OP_END    = 4'd8  // End of program / Output in r[15]
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

    localparam int REG_X0     = 0;
    localparam int REG_X1     = 1;
    localparam int REG_X2     = 2;
    localparam int REG_X3     = 3;
    localparam int REG_RESULT = 15;

    // -------------------------------------------------------------------------
    // SECTION 4: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // ||x_k - x_{k-1}|| <= tol
        STATUS_MAX_ITERS   = 3'd3, // Maximum iterations reached
        STATUS_ERROR       = 3'd4
    } status_t;

endpackage : fista_types_pkg
