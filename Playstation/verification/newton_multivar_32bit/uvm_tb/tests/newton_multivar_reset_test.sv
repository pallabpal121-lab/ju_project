// =============================================================================
// File Name   : newton_multivar_reset_test.sv
// Class Name  : newton_multivar_reset_test
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Silicon sign-off grade synchronous reset verification test.
//               Executes newton_multivar_reset_seq to verify FSM transitions
//               to IDLE under mid-flight reset and complete post-reset recovery.
// =============================================================================

`ifndef NEWTON_MULTIVAR_RESET_TEST_SV
`define NEWTON_MULTIVAR_RESET_TEST_SV

class newton_multivar_reset_test extends newton_base_test;
    `uvm_component_utils(newton_multivar_reset_test)

    function new(string name = "newton_multivar_reset_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual task run_phase(uvm_phase phase);
        newton_multivar_reset_seq seq;

        phase.raise_objection(this, "Starting newton_multivar_reset_test");
        `uvm_info(get_type_name(), "Starting Silicon Sign-Off Synchronous Reset Test...", UVM_LOW)

        seq = newton_multivar_reset_seq::type_id::create("reset_seq");
        seq.start(env.agent.sqr);

        #100ns;
        `uvm_info(get_type_name(), "Silicon Sign-Off Synchronous Reset Test Finished.", UVM_LOW)
        phase.drop_objection(this, "Completed newton_multivar_reset_test");
    endtask

endclass : newton_multivar_reset_test

`endif // NEWTON_MULTIVAR_RESET_TEST_SV
