// =============================================================================
// File Name   : newton_axi_tb_pkg.sv
// Package Name: newton_axi_tb_pkg (Unified Umbrella Package)
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

package newton_axi_tb_pkg;

    import uvm_pkg::*;
    `include "uvm_macros.svh"
    import newton_multivar_pkg::*;
    import newton_axi_regs_pkg::*;

    // Import modular sub-packages
    import newton_axi_agent_pkg::*;
    import newton_axi_seq_pkg::*;
    import newton_axi_env_pkg::*;
    import newton_axi_test_pkg::*;

    // Export all sub-packages for unified scope
    export newton_axi_regs_pkg::*;
    export newton_axi_agent_pkg::*;
    export newton_axi_seq_pkg::*;
    export newton_axi_env_pkg::*;
    export newton_axi_test_pkg::*;

endpackage : newton_axi_tb_pkg
