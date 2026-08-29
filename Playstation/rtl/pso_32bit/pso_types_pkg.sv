// =============================================================================
// File Name   : pso_types_pkg.sv
// Module Name : pso_types_pkg (SystemVerilog Package)
// Project     : Particle Swarm Optimization (PSO) Accelerator (Solver #14)
// -----------------------------------------------------------------------------
// Description:
//   Defines fixed-point arithmetic types (Q16.16), vector structures,
//   DFG microcode instructions, PSO swarm defaults (inertia weight, acceleration
//   coefficients, velocity clamping, bounds), and status codes.
// =============================================================================

`timescale 1ns / 1ps

package pso_types_pkg;

    // -------------------------------------------------------------------------
    // SECTION 1: FIXED-POINT NUMBER FORMAT (Q16.16 Signed)
    // -------------------------------------------------------------------------
    typedef logic signed [31:0] q16_t;

    localparam int Q_FRAC_BITS      = 16;
    localparam q16_t Q16_ONE        = 32'h0001_0000; // +1.0
    localparam q16_t Q16_TWO        = 32'h0002_0000; // +2.0
    localparam q16_t Q16_HALF       = 32'h0000_8000; // +0.5
    localparam q16_t Q16_QUARTER    = 32'h0000_4000; // +0.25
    localparam q16_t Q16_ZERO       = 32'h0000_0000; //  0.0
    localparam q16_t Q16_NEG_ONE    = 32'hFFFF_0000; // -1.0
    localparam q16_t Q16_INF_POS    = 32'h7FFF_0000; // +32767.0 (Large positive initial cost)
    localparam q16_t Q16_EPS_DEF    = 32'h0000_0080; // Default tolerance ε ≈ 0.00195

    // Default PSO Hyperparameters
    localparam q16_t Q16_INERTIA_DEF = 32'h0000_C000; // Inertia weight ω = 0.75
    localparam q16_t Q16_C1_DEF      = 32'h0001_8000; // Cognitive acceleration c1 = 1.5
    localparam q16_t Q16_C2_DEF      = 32'h0001_8000; // Social acceleration c2 = 1.5
    localparam q16_t Q16_VMAX_DEF    = 32'h0002_0000; // Default velocity clamp v_max = 2.0

    // -------------------------------------------------------------------------
    // SECTION 2: VECTOR & SWARM DIMENSIONS
    // -------------------------------------------------------------------------
    localparam int MAX_DIMS      = 4; // Maximum spatial dimensions N <= 4
    localparam int MAX_PARTICLES = 8; // Maximum swarm particles P <= 8

    // Packed 4-element coordinate/velocity vector [x3, x2, x1, x0] (128 bits)
    typedef logic signed [127:0] vec_t;

    // -------------------------------------------------------------------------
    // SECTION 3: MICROCODE INSTRUCTION SET FOR DFG FITNESS EVALUATION
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
        OP_END    = 4'd8  // End of program / Output fitness in r[15]
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

    // Register Conventions:
    //   - r0..r3 : Parameter vector x0..x3
    //   - r15    : Evaluated fitness cost f(x)
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
        STATUS_CONVERGED   = 3'd2, // Fitness <= target_fitness or cost difference <= tol
        STATUS_MAX_ITERS   = 3'd3, // Maximum iteration limit reached
        STATUS_ERROR       = 3'd4
    } status_t;

endpackage : pso_types_pkg
