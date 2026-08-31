// =============================================================================
// File Name   : newton_quadratic_test.sv
// Class Name  : newton_quadratic_test
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_QUADRATIC_TEST_SV
`define NEWTON_QUADRATIC_TEST_SV

class newton_quadratic_test extends newton_base_test;
    `uvm_component_utils(newton_quadratic_test)

    function new(string name = "newton_quadratic_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual task run_phase(uvm_phase phase);
        newton_quadratic_seq seq;
        phase.raise_objection(this);

        `uvm_info(get_type_name(), "Starting Newton Quadratic Optimization Test...", UVM_LOW)
        seq = newton_quadratic_seq::type_id::create("quad_seq");
        seq.start(env.agent.sqr);

        #100ns;
        `uvm_info(get_type_name(), "Newton Quadratic Optimization Test Completed.", UVM_LOW)
        phase.drop_objection(this);
    endtask

endclass : newton_quadratic_test

`endif // NEWTON_QUADRATIC_TEST_SV
