// =============================================================================
// Company / Institution : Jadavpur University (Dept. of ETCE)
// Project               : Dedicated AI Hardware Accelerator
// File Name             : newton_bcd_axi_regs.svh
// Description           : Industry-Grade Memory-Mapped Register Map Header for
//                         the AXI4-Lite Newton Block Coordinate Descent (BCD)
//                         Coprocessor.
//                         Provides register address offsets, bit masks, and
//                         field definitions for SoC integration and firmware.
// =============================================================================

`ifndef NEWTON_BCD_AXI_REGS_SVH
`define NEWTON_BCD_AXI_REGS_SVH

// -----------------------------------------------------------------------------
// SECTION 1: AXI4-LITE MEMORY-MAPPED REGISTER OFFSETS (12-bit Address Space)
// -----------------------------------------------------------------------------
// Base Address typically assigned by SoC Interconnect (e.g., 0x4000_0000)

// Control & Execution Command Register (R/W)
// [0] : Start pulse (Write 1 to trigger optimization sweep)
// [31:1] : Reserved
localparam logic [11:0] AXI_REG_CTRL         = 12'h000;

// Status & Progress Register (RO)
// [0]   : Sticky Done flag (1 = Optimization complete, cleared on Start)
// [1]   : Busy flag (1 = Solver currently calculating)
// [4:2] : Convergence Status code (see status_t in newton_multivar_pkg.sv)
// [31:5]: Reserved
localparam logic [11:0] AXI_REG_STATUS       = 12'h004;

// Number of Active Variables (R/W)
// [4:0] : Total dimension N (Range: 2 to MAX_VARS, default = 16)
// [31:5]: Reserved
localparam logic [11:0] AXI_REG_NUM_VARS     = 12'h008;

// Convergence Step Tolerance Threshold (R/W)
// [31:0]: Q16.16 signed threshold (e.g., 0x0000_0080 = 0.00195)
localparam logic [11:0] AXI_REG_TOLERANCE    = 12'h00C;

// Step Size / Learning Rate Alpha (R/W)
// [31:0]: Q16.16 signed scale factor (e.g., 0x0000_8000 = 0.5)
localparam logic [11:0] AXI_REG_ALPHA        = 12'h010;

// Levenberg-Marquardt Damping Factor Lambda (R/W)
// [31:0]: Q16.16 signed regularization factor (e.g., 0x0000_0400 = 0.0156)
localparam logic [11:0] AXI_REG_LAMBDA       = 12'h014;

// Maximum Sweeps Limit (R/W)
// [7:0] : Maximum coordinate descent sweeps (default = 50)
// [31:8]: Reserved
localparam logic [11:0] AXI_REG_MAX_SWEEPS   = 12'h018;

// Executed Sweeps Count (RO)
// [7:0] : Number of sweeps actually executed before convergence
// [31:8]: Reserved
localparam logic [11:0] AXI_REG_SWEEP_COUNT  = 12'h01C;

// Final Optimal Cost Function Value f(x*) (RO)
// [31:0]: Q16.16 signed value of cost function at converged minimum
localparam logic [11:0] AXI_REG_F_OPTIMAL    = 12'h020;

// Maximum Parameter Update Delta in Last Sweep (RO)
// [31:0]: Q16.16 signed magnitude of largest delta across variables
localparam logic [11:0] AXI_REG_MAX_DELTA    = 12'h024;

// Microcode Instruction Memory Write Address (R/W)
// [5:0] : Target instruction address (0 to PROG_DEPTH - 1)
// [31:6]: Reserved
localparam logic [11:0] AXI_REG_PROG_ADDR    = 12'h040;

// Microcode Instruction Memory Write Data (WO)
// [31:0]: 32-bit packed micro-instruction word (opcode, dst, src_a, src_b, imm)
localparam logic [11:0] AXI_REG_PROG_DATA    = 12'h044;

// State Vector Window Base Address (R/W)
// 0x100 to 0x13C: x[0] through x[15] (each word is 32-bit Q16.16)
// Write: Initial guess vector x_init
// Read : Converged optimal vector x*
localparam logic [11:0] AXI_STATE_VEC_BASE   = 12'h100;
localparam logic [11:0] AXI_STATE_VEC_LIMIT  = 12'h13C;

// -----------------------------------------------------------------------------
// SECTION 2: CONTROL & STATUS BIT MASKS
// -----------------------------------------------------------------------------
localparam logic [31:0] CTRL_START_MASK      = 32'h0000_0001;
localparam logic [31:0] STATUS_DONE_MASK     = 32'h0000_0001;
localparam logic [31:0] STATUS_BUSY_MASK     = 32'h0000_0002;
localparam logic [31:0] STATUS_CODE_MASK     = 32'h0000_001C;
localparam int          STATUS_CODE_SHIFT    = 2;

`endif // NEWTON_BCD_AXI_REGS_SVH
