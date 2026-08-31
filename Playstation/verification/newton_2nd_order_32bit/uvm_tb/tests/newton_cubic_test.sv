// =============================================================================
// File Name   : newton_cubic_test.sv
// Class Name  : newton_cubic_test
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_CUBIC_TEST_SV
`define NEWTON_CUBIC_TEST_SV

class newton_cubic_test extends newton_base_test;
    `uvm_component_utils(newton_cubic_test)

    function new(string name = "newton_cubic_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual task run_phase(uvm_phase phase);
        newton_cubic_seq seq;
        phase.raise_objection(this);

        `uvm_info(get_type_name(), "Starting Newton Cubic Optimization Test...", UVM_LOW)
        seq = newton_cubic_seq::type_id::create("cubic_seq");
        seq.start(env.agent.sqr);

        #100ns;
        `uvm_info(get_type_name(), "Newton Cubic Optimization Test Completed.", UVM_LOW)
        phase.drop_objection(this);
    endtask

endclass : newton_cubic_test

`endif // NEWTON_CUBIC_TEST_SV
