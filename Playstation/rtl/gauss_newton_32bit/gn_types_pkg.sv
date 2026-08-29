// =============================================================================
// File Name   : gn_types_pkg.sv
// Module Name : gn_types_pkg (SystemVerilog Package)
// Project     : Gauss-Newton Non-Linear Least Squares Accelerator (Solver #6)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This package defines all data types, vector formats, matrix structures,
//   observation dataset definitions, and microcode opcodes for the Gauss-Newton
//   Hardware Accelerator.
//
// Key Concepts:
//   - Parameter Vector x : [x0, x1, x2, x3] (up to MAX_PARAMS = 4)
//   - Observation Table  : (t_m, y_m) for m = 0..MAX_OBS-1 (up to 8 points)
//   - Residual Vector r  : r_m = f(t_m, x) - y_m
//   - Jacobian Matrix J  : J_mj = ∂r_m / ∂x_j = (f_+ - f_-) <<< 3
//   - Normal System      : (Jᵀ · J + λ_eps · I) · p = -Jᵀ · r
// =============================================================================

`timescale 1ns / 1ps

package gn_types_pkg;

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
    localparam q16_t Q16_LAMBDA_EPS  = 32'h0000_0040; // Tiny ridge regularization λ ≈ 0.00097
    localparam q16_t Q16_H_STEP      = 32'h0000_1000; // Finite difference step h = 0.0625 = 2^-4

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR, RESIDUAL & MATRIX DEFINITIONS
    // -------------------------------------------------------------------------
    localparam int MAX_PARAMS = 4; // Maximum parameter dimensions (N <= 4)
    localparam int MAX_OBS    = 8; // Maximum observation points (M <= 8)

    // Packed 4-element parameter/gradient vector [x3, x2, x1, x0] (128 bits)
    typedef logic signed [127:0] vec_t;

    // Packed 8-element residual/target vector [r7, ..., r0] (256 bits)
    typedef logic signed [255:0] res_vec_t;

    // Packed 4x4 matrix (512 bits) for Gram matrix JᵀJ and Cholesky factor L
    typedef logic signed [511:0] mat_t;

    // Observation Data Point Structure (t_m, y_m)
    typedef struct packed {
        q16_t t_val; // Independent variable (e.g. time, frequency, sensor ID)
        q16_t y_val; // Measured target value
    } obs_t;

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
        OP_END    = 4'd8  // End of program / Output scalar f(t_m, x) in r[15]
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

    localparam int REG_T_VAL  = 0;  // Register r0 holds observation t_m
    localparam int REG_X0     = 1;  // Register r1 holds parameter x0
    localparam int REG_X1     = 2;  // Register r2 holds parameter x1
    localparam int REG_X2     = 3;  // Register r3 holds parameter x2
    localparam int REG_X3     = 4;  // Register r4 holds parameter x3
    localparam int REG_RESULT = 15; // Register r15 holds output model prediction f(t_m, x)

    // -------------------------------------------------------------------------
    // SECTION 4: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // ||g||_inf <= tolerance or cost <= tol
        STATUS_MAX_ITERS   = 3'd3, // Maximum iteration limit reached
        STATUS_SINGULAR    = 3'd4, // JᵀJ matrix non-invertible
        STATUS_DIV_BY_ZERO = 3'd5,
        STATUS_OVERFLOW    = 3'd6
    } status_t;

endpackage : gn_types_pkg
