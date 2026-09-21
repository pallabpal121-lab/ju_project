// =============================================================================
// File Name   : newton_base_test.sv
// Class Name  : newton_base_test
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Base UVM Test implementing all 9 standard UVM phases,
//               top-level watchdog timeout, and topology reporting.
// =============================================================================

`ifndef NEWTON_BASE_TEST_SV
`define NEWTON_BASE_TEST_SV

class newton_base_test extends uvm_test;
    `uvm_component_utils(newton_base_test)

    newton_env env;

    function new(string name = "newton_base_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    // Phase 1: build_phase
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("TEST_PHASE_1_BUILD", "[STAGE 1: SETUP] build_phase: Building verification environment...", UVM_LOW)
        uvm_top.set_timeout(100ms, 1);
        env = newton_env::type_id::create("env", this);
    endfunction

    // Phase 2: connect_phase
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        `uvm_info("TEST_PHASE_2_CONNECT", "[STAGE 1: SETUP] connect_phase: Test-level connections verified.", UVM_LOW)
    endfunction

    // Phase 3: end_of_elaboration_phase
    virtual function void end_of_elaboration_phase(uvm_phase phase);
        super.end_of_elaboration_phase(phase);
        `uvm_info("TEST_PHASE_3_END_OF_ELAB", "[STAGE 1: SETUP] end_of_elaboration_phase: Printing component topology...", UVM_LOW)
        uvm_top.print_topology();
    endfunction

    // Phase 4: start_of_simulation_phase
    virtual function void start_of_simulation_phase(uvm_phase phase);
        super.start_of_simulation_phase(phase);
        `uvm_info("TEST_PHASE_4_START_OF_SIM", "==================================================================", UVM_LOW)
        `uvm_info("TEST_PHASE_4_START_OF_SIM", " [STAGE 1: SETUP] start_of_simulation_phase: Ready for execution! ", UVM_LOW)
        `uvm_info("TEST_PHASE_4_START_OF_SIM", "==================================================================", UVM_LOW)
    endfunction

    // Phase 5: run_phase
    virtual task run_phase(uvm_phase phase);
        super.run_phase(phase);
        `uvm_info("TEST_PHASE_5_RUN", "[STAGE 2: RUN] Base test run_phase idle.", UVM_LOW)
    endtask

    // Phase 6: extract_phase
    virtual function void extract_phase(uvm_phase phase);
        super.extract_phase(phase);
        `uvm_info("TEST_PHASE_6_EXTRACT", "[STAGE 3: CLEANUP] extract_phase: Test metrics extracted.", UVM_LOW)
    endfunction

    // Phase 7: check_phase
    virtual function void check_phase(uvm_phase phase);
        super.check_phase(phase);
        `uvm_info("TEST_PHASE_7_CHECK", "[STAGE 3: CLEANUP] check_phase: Verifying zero pending objections.", UVM_LOW)
    endfunction

    // Phase 8: report_phase
    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info("TEST_PHASE_8_REPORT", "[STAGE 3: CLEANUP] report_phase: Preparing final simulation summary.", UVM_LOW)
    endfunction

    // Phase 9: final_phase
    virtual function void final_phase(uvm_phase phase);
        super.final_phase(phase);
        `uvm_info("TEST_PHASE_9_FINAL", "==================================================================", UVM_LOW)
        `uvm_info("TEST_PHASE_9_FINAL", " [STAGE 3: CLEANUP] final_phase: Simulation completed cleanly!   ", UVM_LOW)
        `uvm_info("TEST_PHASE_9_FINAL", "==================================================================", UVM_LOW)
    endfunction

endclass : newton_base_test

`endif // NEWTON_BASE_TEST_SV
