// =============================================================================
// File Name   : newton_corner_test.sv
// Class Name  : newton_corner_test
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_CORNER_TEST_SV
`define NEWTON_CORNER_TEST_SV

class newton_corner_test extends newton_base_test;
    `uvm_component_utils(newton_corner_test)

    function new(string name = "newton_corner_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual task run_phase(uvm_phase phase);
        newton_corner_seq seq;
        phase.raise_objection(this);

        `uvm_info(get_type_name(), "Starting Newton Corner Case Test...", UVM_LOW)
        seq = newton_corner_seq::type_id::create("corner_seq");
        seq.start(env.agent.sqr);

        #100ns;
        `uvm_info(get_type_name(), "Newton Corner Case Test Completed.", UVM_LOW)
        phase.drop_objection(this);
    endtask

endclass : newton_corner_test

`endif // NEWTON_CORNER_TEST_SV
