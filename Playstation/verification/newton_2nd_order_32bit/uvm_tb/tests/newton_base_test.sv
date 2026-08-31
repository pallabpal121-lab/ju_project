// =============================================================================
// File Name   : newton_base_test.sv
// Class Name  : newton_base_test
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_BASE_TEST_SV
`define NEWTON_BASE_TEST_SV

class newton_base_test extends uvm_test;
    `uvm_component_utils(newton_base_test)

    newton_env env;

    function new(string name = "newton_base_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        env = newton_env::type_id::create("env", this);
    endfunction

    virtual function void end_of_elaboration_phase(uvm_phase phase);
        super.end_of_elaboration_phase(phase);
        uvm_top.print_topology();
    endfunction

endclass : newton_base_test

`endif // NEWTON_BASE_TEST_SV
