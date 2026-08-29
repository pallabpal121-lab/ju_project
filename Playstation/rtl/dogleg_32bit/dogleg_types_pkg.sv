// =============================================================================
// File Name   : dogleg_types_pkg.sv
// Module Name : dogleg_types_pkg (SystemVerilog Package)
// Project     : Trust-Region Dogleg Non-Linear Optimizer Accelerator (Solver #13)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), vector/matrix structures,
//   DFG microcode instructions, trust-region parameter defaults, step type
//   classifications, and status codes for the Trust-Region Dogleg Accelerator.
// =============================================================================

`timescale 1ns / 1ps

package dogleg_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    localparam int Q_FRAC_BITS       = 16;
    localparam q16_t Q16_ONE         = 32'h0001_0000; // +1.0
    localparam q16_t Q16_TWO         = 32'h0002_0000; // +2.0
    localparam q16_t Q16_FOUR        = 32'h0004_0000; // +4.0
    localparam q16_t Q16_HALF        = 32'h0000_8000; // +0.5
    localparam q16_t Q16_QUARTER     = 32'h0000_4000; // +0.25
    localparam q16_t Q16_THREE_QUART = 32'h0000_C000; // +0.75
    localparam q16_t Q16_ZERO        = 32'h0000_0000; //  0.0
    localparam q16_t Q16_NEG_ONE     = 32'hFFFF_0000; // -1.0
    localparam q16_t Q16_EPS_DEF     = 32'h0000_0080; // Default tolerance ε ≈ 0.00195
    localparam q16_t Q16_LAMBDA_EPS  = 32'h0000_0040; // Diagonal regularizer for Hessian
    localparam q16_t Q16_H_STEP      = 32'h0000_1000; // Finite diff step h = 0.0625 = 2^-4
    localparam q16_t Q16_DELTA_INIT  = 32'h0001_0000; // Default initial trust radius Δ_0 = 1.0
    localparam q16_t Q16_DELTA_MIN   = 32'h0000_0010; // Min trust radius Δ_min ≈ 0.00024
    localparam q16_t Q16_DELTA_MAX   = 32'h0020_0000; // Max trust radius Δ_max = 32.0
    localparam q16_t Q16_ETA_DEF     = 32'h0000_0000; // Acceptance threshold η = 0.0

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & MATRIX DEFINITIONS
    // -------------------------------------------------------------------------
    localparam int MAX_PARAMS = 4; // Maximum parameter dimensions N <= 4
    localparam int MAX_OBS    = 8; // Maximum observation data points M <= 8

    // Packed 4-element parameter vector [x3, x2, x1, x0] (128 bits)
    typedef logic signed [127:0] vec_t;

    // Packed 8-element residual vector [r7, ..., r0] (256 bits)
    typedef logic signed [255:0] res_vec_t;

    // Packed 4x4 matrix (512 bits) for JᵀJ and Cholesky factor L
    typedef logic signed [511:0] mat_t;

    // Observation Data Point: pair of (t_m, y_m)
    typedef struct packed {
        q16_t t_val; // Independent input value t_m
        q16_t y_val; // Measured target value y_m
    } obs_t;

    // -------------------------------------------------------------------------
    // SECTION 3: STEP TYPE CLASSIFICATION
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        STEP_NONE          = 2'd0,
        STEP_GAUSS_NEWTON  = 2'd1, // Full GN step within trust region (||p_gn|| <= Δ)
        STEP_CAUCHY_TRUNC  = 2'd2, // Truncated Cauchy step (Δ <= ||p_c||)
        STEP_DOGLEG_INTERP = 2'd3  // Dogleg interpolation (||p_c|| < Δ < ||p_gn||)
    } step_type_t;

    // -------------------------------------------------------------------------
    // SECTION 4: MICROCODE INSTRUCTION SET FOR DFG MODEL EVALUATION
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
        OP_END    = 4'd8  // End of program / Output model value in r[15]
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
    // SECTION 5: SOLVER STATUS CODES
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        STATUS_IDLE        = 3'd0,
        STATUS_RUNNING     = 3'd1,
        STATUS_CONVERGED   = 3'd2, // ||g|| <= tolerance or radius collapsed / min cost change
        STATUS_MAX_ITERS   = 3'd3, // Max iteration limit reached
        STATUS_SINGULAR    = 3'd4, // Hessian matrix non-invertible
        STATUS_RADIUS_MIN  = 3'd5, // Trust region radius reached minimum threshold
        STATUS_ERROR       = 3'd6
    } status_t;

endpackage : dogleg_types_pkg
