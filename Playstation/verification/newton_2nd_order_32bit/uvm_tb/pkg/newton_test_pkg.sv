// =============================================================================
// File Name   : newton_test_pkg.sv
// Package Name: newton_test_pkg (UVM Test Suite Package)
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

package newton_test_pkg;

    import uvm_pkg::*;
    `include "uvm_macros.svh"
    import newton_types_pkg::*;
    import newton_agent_pkg::*;
    import newton_env_pkg::*;
    import newton_seq_pkg::*;

    // Tests
    `include "newton_base_test.sv"
    `include "newton_quadratic_test.sv"
    `include "newton_cubic_test.sv"
    `include "newton_rational_test.sv"
    `include "newton_corner_test.sv"
    `include "newton_random_test.sv"

endpackage : newton_test_pkg
