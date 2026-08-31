// =============================================================================
// File Name   : newton_random_test.sv
// Class Name  : newton_random_test
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_RANDOM_TEST_SV
`define NEWTON_RANDOM_TEST_SV

class newton_random_test extends newton_base_test;
    `uvm_component_utils(newton_random_test)

    function new(string name = "newton_random_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual task run_phase(uvm_phase phase);
        newton_random_seq seq;
        phase.raise_objection(this);

        `uvm_info(get_type_name(), "Starting Newton Randomized Coverage Test...", UVM_LOW)
        seq = newton_random_seq::type_id::create("rnd_seq");
        seq.start(env.agent.sqr);

        #100ns;
        `uvm_info(get_type_name(), "Newton Randomized Coverage Test Completed.", UVM_LOW)
        phase.drop_objection(this);
    endtask

endclass : newton_random_test

`endif // NEWTON_RANDOM_TEST_SV
