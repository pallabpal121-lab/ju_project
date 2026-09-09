// =============================================================================
// File Name   : newton_tb_pkg.sv
// Package Name: newton_tb_pkg (Unified Umbrella Package)
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

package newton_tb_pkg;

    import uvm_pkg::*;
    `include "uvm_macros.svh"
    import newton_types_pkg::*;

    // Import modular sub-packages
    import newton_agent_pkg::*;
    import newton_seq_pkg::*;
    import newton_env_pkg::*;
    import newton_test_pkg::*;

    // Export all sub-packages for unified/backward-compatible scope
    export newton_agent_pkg::*;
    export newton_seq_pkg::*;
    export newton_env_pkg::*;
    export newton_test_pkg::*;

endpackage : newton_tb_pkg
