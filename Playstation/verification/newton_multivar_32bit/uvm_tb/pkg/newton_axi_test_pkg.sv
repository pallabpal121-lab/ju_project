// =============================================================================
// File Name   : newton_axi_test_pkg.sv
// Package Name: newton_axi_test_pkg (UVM Test Suite Package)
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

package newton_axi_test_pkg;

    import uvm_pkg::*;
    `include "uvm_macros.svh"
    import newton_multivar_pkg::*;
    import newton_axi_regs_pkg::*;
    import newton_axi_agent_pkg::*;
    import newton_axi_seq_pkg::*;
    import newton_axi_env_pkg::*;

    // Tests
    `include "newton_base_test.sv"
    `include "newton_multivar_quadratic_test.sv"
    `include "newton_multivar_rosenbrock_test.sv"
    `include "newton_multivar_corner_test.sv"
    `include "newton_multivar_random_test.sv"
    `include "newton_axi_stress_test.sv"
    `include "newton_multivar_tapeout_test.sv"
    `include "newton_multivar_reset_test.sv"

endpackage : newton_axi_test_pkg
