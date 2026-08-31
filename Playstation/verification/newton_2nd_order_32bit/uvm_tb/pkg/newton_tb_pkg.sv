// =============================================================================
// File Name   : newton_tb_pkg.sv
// Module Name : newton_tb_pkg (SystemVerilog UVM Package)
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

package newton_tb_pkg;

    import uvm_pkg::*;
    `include "uvm_macros.svh"
    import newton_types_pkg::*;

    `include "newton_seq_item.sv"
    `include "newton_sequencer.sv"
    `include "newton_driver.sv"
    `include "newton_monitor.sv"
    `include "newton_ref_model.sv"
    `include "newton_scoreboard.sv"
    `include "newton_coverage.sv"
    `include "newton_agent.sv"
    `include "newton_env.sv"

    // Sequences
    `include "newton_base_seq.sv"
    `include "newton_quadratic_seq.sv"
    `include "newton_cubic_seq.sv"
    `include "newton_rational_seq.sv"
    `include "newton_corner_seq.sv"
    `include "newton_random_seq.sv"

    // Tests
    `include "newton_base_test.sv"
    `include "newton_quadratic_test.sv"
    `include "newton_cubic_test.sv"
    `include "newton_rational_test.sv"
    `include "newton_corner_test.sv"
    `include "newton_random_test.sv"

endpackage : newton_tb_pkg
