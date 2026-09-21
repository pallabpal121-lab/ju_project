// =============================================================================
// File Name   : newton_multivar_random_test.sv
// Class Name  : newton_multivar_random_test
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : UVM Test executing constrained-random optimization regression.
// =============================================================================

`ifndef NEWTON_MULTIVAR_RANDOM_TEST_SV
`define NEWTON_MULTIVAR_RANDOM_TEST_SV

class newton_multivar_random_test extends newton_base_test;
    `uvm_component_utils(newton_multivar_random_test)

    function new(string name = "newton_multivar_random_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual task run_phase(uvm_phase phase);
        newton_multivar_random_seq seq;

        phase.raise_objection(this, "Starting newton_multivar_random_test");
        `uvm_info(get_type_name(), "Starting Constrained-Random Optimization Test...", UVM_LOW)

        seq = newton_multivar_random_seq::type_id::create("seq");
        if (!seq.randomize()) `uvm_fatal("RAND_FAIL", "Failed to randomize random sequence!")
        seq.start(env.agent.sqr);

        #100ns;
        `uvm_info(get_type_name(), "Constrained-Random Optimization Test Finished.", UVM_LOW)
        phase.drop_objection(this, "Completed newton_multivar_random_test");
    endtask

endclass : newton_multivar_random_test

`endif // NEWTON_MULTIVAR_RANDOM_TEST_SV
