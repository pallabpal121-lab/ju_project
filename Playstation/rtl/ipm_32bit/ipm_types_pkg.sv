// =============================================================================
// File Name   : ipm_types_pkg.sv
// Module Name : ipm_types_pkg (SystemVerilog Package)
// Project     : Primal-Dual Interior Point Method (IPM) Accelerator (Solver #16)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), packed vector/matrix
//   structures, microcode instructions, centering parameter σ, boundary
//   fraction parameter τ, regularizing epsilons, and status codes.
// =============================================================================

`timescale 1ns / 1ps

package ipm_types_pkg;

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
    localparam q16_t Q16_EPS_DEF     = 32'h0000_0080; // Convergence tolerance ε ≈ 0.00195
    localparam q16_t Q16_SIGMA_DEF   = 32'h0000_199A; // Centering parameter σ = 0.1
    localparam q16_t Q16_TAU_DEF     = 32'h0000_F333; // Fraction-to-boundary τ = 0.95
    localparam q16_t Q16_REG_EPS     = 32'h0000_0010; // Regularizing epsilon for Cholesky

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & MATRIX DIMENSIONS (N <= 4, M <= 4)
    // -------------------------------------------------------------------------
    localparam int MAX_VARS        = 4; // Max primal variables N <= 4
    localparam int MAX_CONSTRAINTS = 4; // Max inequality constraints M <= 4

    // Packed 4-element vector (128 bits)
    typedef logic signed [127:0] vec_t;

    // Packed 4x4 matrix (512 bits, column-major / row-major packed)
    typedef logic signed [511:0] mat_t;

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
        STATUS_CONVERGED   = 3'd2, // Duality gap and residuals <= tolerance
        STATUS_MAX_ITERS   = 3'd3, // Maximum iteration limit reached
        STATUS_NUM_ERROR   = 3'd4  // Cholesky failure or zero pivot
    } status_t;

endpackage : ipm_types_pkg
