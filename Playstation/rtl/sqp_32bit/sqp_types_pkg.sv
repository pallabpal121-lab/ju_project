// =============================================================================
// File Name   : sqp_types_pkg.sv
// Module Name : sqp_types_pkg (SystemVerilog Package)
// Project     : Sequential Quadratic Programming (SQP) Accelerator (Solver #7)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Defines data types, vectors, matrices, constraint bounds, and microcode
//   opcodes for the SQP Constrained Hardware Optimization Accelerator.
//
// Key Concepts:
//   - Parameter Vector x : [x0, x1, x2, x3] (up to MAX_PARAMS = 4)
//   - Box Bounds (l, u)  : Lower and Upper physical bound vectors
//   - Active-Set Mask    : Identifies clamped boundary variables (p_i = 0)
//   - KKT Shadow Prices  : Lagrange multipliers for active constraints
// =============================================================================

`timescale 1ns / 1ps

package sqp_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    localparam int Q_FRAC_BITS       = 16;
    localparam q16_t Q16_ONE         = 32'h0001_0000; // +1.0
    localparam q16_t Q16_HALF        = 32'h0000_8000; // +0.5
    localparam q16_t Q16_ZERO        = 32'h0000_0000; //  0.0
    localparam q16_t Q16_NEG_ONE     = 32'hFFFF_0000; // -1.0
    localparam q16_t Q16_INF_POS     = 32'h7FFF_0000; // Large positive (+32767.0)
    localparam q16_t Q16_INF_NEG     = 32'h8000_0000; // Large negative (-32768.0)
    localparam q16_t Q16_EPS_DEF     = 32'h0000_0080; // Default tolerance ε ≈ 0.00195
    localparam q16_t Q16_RIDGE_LAMBDA= 32'h0000_0040; // Tiny regularization λ ≈ 0.00097
    localparam q16_t Q16_H_STEP      = 32'h0000_1000; // Step size h = 0.0625 = 2^-4
    localparam q16_t Q16_BOUND_TOL   = 32'h0000_0200; // Boundary detection threshold ≈ 0.0078

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR, MATRIX & BOUNDS DEFINITIONS
    // -------------------------------------------------------------------------
    localparam int MAX_PARAMS = 4; // Maximum parameter dimensions (N <= 4)

    // Packed 4-element parameter/gradient/multiplier vector [x3, x2, x1, x0] (128 bits)
    typedef logic signed [127:0] vec_t;

    // Packed 4x4 Hessian / KKT matrix (512 bits)
    typedef logic signed [511:0] mat_t;

    // Physical Box Constraint Bounds Structure
    typedef struct packed {
        vec_t lower_bound; // l_i
        vec_t upper_bound; // u_i
    } box_bounds_t;

    // Active-set bitmask (1 = variable clamped to constraint boundary)
    typedef logic [3:0] active_mask_t;

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
        STATUS_CONVERGED   = 3'd2, // KKT optimality conditions satisfied
        STATUS_MAX_ITERS   = 3'd3, // Maximum iteration limit reached
        STATUS_INFEASIBLE  = 3'd4, // Bounds inconsistent (l_i > u_i)
        STATUS_SINGULAR    = 3'd5, // Reduced Hessian singular
        STATUS_DIV_BY_ZERO = 3'd6
    } status_t;

endpackage : sqp_types_pkg
