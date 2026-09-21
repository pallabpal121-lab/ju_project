// =============================================================================
// File Name   : newton_axi_seq_pkg.sv
// Package Name: newton_axi_seq_pkg (Stimulus / Sequence Library Package)
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

package newton_axi_seq_pkg;

    import uvm_pkg::*;
    `include "uvm_macros.svh"
    import newton_multivar_pkg::*;
    import newton_axi_regs_pkg::*;
    import newton_axi_agent_pkg::*;

    // Sequences
    `include "newton_base_seq.sv"
    `include "newton_multivar_quadratic_seq.sv"
    `include "newton_multivar_rosenbrock_seq.sv"
    `include "newton_multivar_corner_seq.sv"
    `include "newton_multivar_random_seq.sv"
    `include "newton_axi_stress_seq.sv"
    `include "newton_multivar_tapeout_seq.sv"
    `include "newton_multivar_reset_seq.sv"

endpackage : newton_axi_seq_pkg
