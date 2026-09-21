// =============================================================================
// File Name   : newton_axi_regs_pkg.sv
// Package Name: newton_axi_regs_pkg
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Package defining all AXI4-Lite memory-mapped register offsets,
//               masks, and bit-field shifts for verification components.
// =============================================================================

`timescale 1ns / 1ps

package newton_axi_regs_pkg;

    // Control & Execution Command Register (R/W)
    localparam logic [11:0] AXI_REG_CTRL         = 12'h000;

    // Status & Progress Register (RO)
    localparam logic [11:0] AXI_REG_STATUS       = 12'h004;

    // Number of Active Variables (R/W)
    localparam logic [11:0] AXI_REG_NUM_VARS     = 12'h008;

    // Convergence Step Tolerance Threshold (R/W)
    localparam logic [11:0] AXI_REG_TOLERANCE    = 12'h00C;

    // Step Size / Learning Rate Alpha (R/W)
    localparam logic [11:0] AXI_REG_ALPHA        = 12'h010;

    // Levenberg-Marquardt Damping Factor Lambda (R/W)
    localparam logic [11:0] AXI_REG_LAMBDA       = 12'h014;

    // Maximum Sweeps Limit (R/W)
    localparam logic [11:0] AXI_REG_MAX_SWEEPS   = 12'h018;

    // Executed Sweeps Count (RO)
    localparam logic [11:0] AXI_REG_SWEEP_COUNT  = 12'h01C;

    // Final Optimal Cost Function Value f(x*) (RO)
    localparam logic [11:0] AXI_REG_F_OPTIMAL    = 12'h020;

    // Maximum Parameter Update Delta in Last Sweep (RO)
    localparam logic [11:0] AXI_REG_MAX_DELTA    = 12'h024;

    // Microcode Instruction Memory Write Address (R/W)
    localparam logic [11:0] AXI_REG_PROG_ADDR    = 12'h040;

    // Microcode Instruction Memory Write Data (WO)
    localparam logic [11:0] AXI_REG_PROG_DATA    = 12'h044;

    // State Vector Window Base & Limit Addresses (R/W)
    localparam logic [11:0] AXI_STATE_VEC_BASE   = 12'h100;
    localparam logic [11:0] AXI_STATE_VEC_LIMIT  = 12'h13C;

    // Control & Status Bit Masks
    localparam logic [31:0] CTRL_START_MASK      = 32'h0000_0001;
    localparam logic [31:0] STATUS_DONE_MASK     = 32'h0000_0001;
    localparam logic [31:0] STATUS_BUSY_MASK     = 32'h0000_0002;
    localparam logic [31:0] STATUS_CODE_MASK     = 32'h0000_001C;
    localparam int          STATUS_CODE_SHIFT    = 2;

endpackage : newton_axi_regs_pkg
