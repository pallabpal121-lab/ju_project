// =============================================================================
// File Name   : newton_multivar_quadratic_test.sv
// Class Name  : newton_multivar_quadratic_test
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : UVM Test executing quadratic bowl optimization sequence.
// =============================================================================

`ifndef NEWTON_MULTIVAR_QUADRATIC_TEST_SV
`define NEWTON_MULTIVAR_QUADRATIC_TEST_SV

class newton_multivar_quadratic_test extends newton_base_test;
    `uvm_component_utils(newton_multivar_quadratic_test)

    function new(string name = "newton_multivar_quadratic_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual task run_phase(uvm_phase phase);
        newton_multivar_quadratic_seq seq;

        phase.raise_objection(this, "Starting newton_multivar_quadratic_test");
        `uvm_info(get_type_name(), "Starting Multivariable Quadratic Test...", UVM_LOW)

        seq = newton_multivar_quadratic_seq::type_id::create("seq");
        seq.start(env.agent.sqr);

        #100ns;
        `uvm_info(get_type_name(), "Multivariable Quadratic Test Finished.", UVM_LOW)
        phase.drop_objection(this, "Completed newton_multivar_quadratic_test");
    endtask

endclass : newton_multivar_quadratic_test

`endif // NEWTON_MULTIVAR_QUADRATIC_TEST_SV
