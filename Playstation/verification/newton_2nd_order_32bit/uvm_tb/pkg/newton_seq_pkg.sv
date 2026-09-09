// =============================================================================
// File Name   : newton_seq_pkg.sv
// Package Name: newton_seq_pkg (Stimulus / Sequence Library Package)
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

package newton_seq_pkg;

    import uvm_pkg::*;
    `include "uvm_macros.svh"
    import newton_types_pkg::*;
    import newton_agent_pkg::*;

    // Sequences
    `include "newton_base_seq.sv"
    `include "newton_quadratic_seq.sv"
    `include "newton_cubic_seq.sv"
    `include "newton_rational_seq.sv"
    `include "newton_corner_seq.sv"
    `include "newton_random_seq.sv"

endpackage : newton_seq_pkg
