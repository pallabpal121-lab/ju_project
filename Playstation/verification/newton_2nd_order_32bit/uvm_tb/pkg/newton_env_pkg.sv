// =============================================================================
// File Name   : newton_env_pkg.sv
// Package Name: newton_env_pkg (Verification Environment & Checking Package)
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

package newton_env_pkg;

    import uvm_pkg::*;
    `include "uvm_macros.svh"
    import newton_types_pkg::*;
    import newton_agent_pkg::*;

    // Scoreboard, Reference Model, Coverage, and Environment
    `include "newton_ref_model.sv"
    `include "newton_scoreboard.sv"
    `include "newton_coverage.sv"
    `include "newton_env.sv"

endpackage : newton_env_pkg
