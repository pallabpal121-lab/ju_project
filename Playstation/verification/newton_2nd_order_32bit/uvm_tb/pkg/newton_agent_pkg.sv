// =============================================================================
// File Name   : newton_agent_pkg.sv
// Package Name: newton_agent_pkg (Verification IP / Agent Package)
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

package newton_agent_pkg;

    import uvm_pkg::*;
    `include "uvm_macros.svh"
    import newton_types_pkg::*;

    // VIP Configuration & Transaction item
    `include "newton_agent_config.sv"
    `include "newton_seq_item.sv"
    `include "newton_sequencer.sv"
    `include "newton_driver.sv"
    `include "newton_monitor.sv"
    `include "newton_agent.sv"

endpackage : newton_agent_pkg
