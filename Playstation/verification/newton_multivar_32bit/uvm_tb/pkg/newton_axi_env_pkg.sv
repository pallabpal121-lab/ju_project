// =============================================================================
// File Name   : newton_axi_env_pkg.sv
// Package Name: newton_axi_env_pkg (Verification Environment Package)
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

package newton_axi_env_pkg;

    import uvm_pkg::*;
    `include "uvm_macros.svh"
    import newton_multivar_pkg::*;
    import newton_axi_regs_pkg::*;
    import newton_axi_agent_pkg::*;

    // Scoreboard, Reference Model, Coverage, and Environment
    `include "newton_multivar_ref_model.sv"
    `include "newton_scoreboard.sv"
    `include "newton_coverage.sv"
    `include "newton_env.sv"

endpackage : newton_axi_env_pkg
