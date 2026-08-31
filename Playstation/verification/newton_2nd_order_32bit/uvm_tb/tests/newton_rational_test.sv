// =============================================================================
// File Name   : newton_rational_test.sv
// Class Name  : newton_rational_test
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_RATIONAL_TEST_SV
`define NEWTON_RATIONAL_TEST_SV

class newton_rational_test extends newton_base_test;
    `uvm_component_utils(newton_rational_test)

    function new(string name = "newton_rational_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual task run_phase(uvm_phase phase);
        newton_rational_seq seq;
        phase.raise_objection(this);

        `uvm_info(get_type_name(), "Starting Newton Rational (Division) Test...", UVM_LOW)
        seq = newton_rational_seq::type_id::create("rational_seq");
        seq.start(env.agent.sqr);

        #100ns;
        `uvm_info(get_type_name(), "Newton Rational Test Completed.", UVM_LOW)
        phase.drop_objection(this);
    endtask

endclass : newton_rational_test

`endif // NEWTON_RATIONAL_TEST_SV
