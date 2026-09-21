// =============================================================================
// File Name   : newton_axi_stress_test.sv
// Class Name  : newton_axi_stress_test
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : UVM Test executing AXI4-Lite bus stress and register R/W sequence.
// =============================================================================

`ifndef NEWTON_AXI_STRESS_TEST_SV
`define NEWTON_AXI_STRESS_TEST_SV

class newton_axi_stress_test extends newton_base_test;
    `uvm_component_utils(newton_axi_stress_test)

    function new(string name = "newton_axi_stress_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual task run_phase(uvm_phase phase);
        newton_axi_stress_seq seq;

        phase.raise_objection(this, "Starting newton_axi_stress_test");
        `uvm_info(get_type_name(), "Starting AXI4-Lite Stress Test...", UVM_LOW)

        seq = newton_axi_stress_seq::type_id::create("seq");
        seq.start(env.agent.sqr);

        #100ns;
        `uvm_info(get_type_name(), "AXI4-Lite Stress Test Finished.", UVM_LOW)
        phase.drop_objection(this, "Completed newton_axi_stress_test");
    endtask

endclass : newton_axi_stress_test

`endif // NEWTON_AXI_STRESS_TEST_SV
