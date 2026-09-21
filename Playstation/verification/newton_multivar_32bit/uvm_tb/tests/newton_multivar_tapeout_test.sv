// =============================================================================
// File Name   : newton_multivar_tapeout_test.sv
// Class Name  : newton_multivar_tapeout_test
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Tapeout-grade UVM Test executing comprehensive optimization
//               sequence targeting 100% functional and code coverage closure.
// =============================================================================

`ifndef NEWTON_MULTIVAR_TAPEOUT_TEST_SV
`define NEWTON_MULTIVAR_TAPEOUT_TEST_SV

class newton_multivar_tapeout_test extends newton_base_test;
    `uvm_component_utils(newton_multivar_tapeout_test)

    function new(string name = "newton_multivar_tapeout_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual task run_phase(uvm_phase phase);
        newton_multivar_tapeout_seq seq;

        phase.raise_objection(this, "Starting newton_multivar_tapeout_test");
        `uvm_info(get_type_name(), "Starting Tapeout-Grade Comprehensive Test...", UVM_LOW)

        seq = newton_multivar_tapeout_seq::type_id::create("tapeout_seq");
        seq.start(env.agent.sqr);

        #100ns;
        `uvm_info(get_type_name(), "Tapeout-Grade Comprehensive Test Finished.", UVM_LOW)
        phase.drop_objection(this, "Completed newton_multivar_tapeout_test");
    endtask

endclass : newton_multivar_tapeout_test

`endif // NEWTON_MULTIVAR_TAPEOUT_TEST_SV
