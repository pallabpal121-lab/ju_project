// =============================================================================
// File Name   : newton_axi_agent_pkg.sv
// Package Name: newton_axi_agent_pkg (AXI4-Lite VIP / Agent Package)
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

package newton_axi_agent_pkg;

    import uvm_pkg::*;
    `include "uvm_macros.svh"
    import newton_multivar_pkg::*;
    import newton_axi_regs_pkg::*;

    // VIP Configuration & Transactions
    `include "newton_agent_config.sv"
    `include "newton_axi_seq_item.sv"
    `include "newton_sequencer.sv"
    `include "newton_driver.sv"
    `include "newton_monitor.sv"
    `include "newton_agent.sv"

endpackage : newton_axi_agent_pkg
