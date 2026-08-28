// ============================================================================
// File: q16_types.sv
// Description: Fixed-point Q16.16 definitions and DFG instruction formats
// Compatible with Icarus Verilog and standard SystemVerilog
// ============================================================================

package q16_types;

    parameter int DATA_WIDTH = 32;
    parameter int FRAC_BITS  = 16;
    parameter int REG_DEPTH  = 32;
    parameter int PROG_DEPTH = 64;

    typedef logic signed [31:0] q16_t;

    typedef enum logic [3:0] {
        OP_NOP        = 4'd0,
        OP_LOAD_CONST = 4'd1,
        OP_LOAD_X     = 4'd2,
        OP_ADD        = 4'd3,
        OP_SUB        = 4'd4,
        OP_MUL        = 4'd5,
        OP_END        = 4'd15
    } opcode_e;

    // 64-bit packed instruction format:
    // [63:60] opcode (4 bits)
    // [59:55] dst    (5 bits)
    // [54:50] srcA   (5 bits)
    // [49:45] srcB   (5 bits)
    // [44:32] reserved
    // [31:0]  imm    (32 bits)
    typedef logic [63:0] instr_word_t;

    typedef enum logic [3:0] {
        STATUS_IDLE            = 4'd0,
        STATUS_BUSY            = 4'd1,
        STATUS_SUCCESS         = 4'd2,
        STATUS_MAX_ITER        = 4'd3,
        STATUS_DIV_BY_ZERO     = 4'd4,
        STATUS_INVALID_HESSIAN = 4'd5,
        STATUS_OVERFLOW        = 4'd6
    } status_e;

endpackage: q16_types
