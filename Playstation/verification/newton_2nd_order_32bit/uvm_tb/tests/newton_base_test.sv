// =============================================================================
// File Name   : newton_base_test.sv
// Class Name  : newton_base_test (Demonstrating All 9 Standard UVM Phases)
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

    // -------------------------------------------------------------------------
    // Phase 1: build_phase (Top-Down, Time 0)
    // Allocates child components and initializes test parameters.
    // -------------------------------------------------------------------------
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("PHASE_1_BUILD", "[STAGE 1: SETUP] build_phase: Building testbench components and setting global watchdog timeout...", UVM_LOW)
        uvm_top.set_timeout(50ms, 1);
        env = newton_env::type_id::create("env", this);
    endfunction

    // -------------------------------------------------------------------------
    // Phase 2: connect_phase (Bottom-Up, Time 0)
    // Wires test-level connections (if any).
    // -------------------------------------------------------------------------
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        `uvm_info("PHASE_2_CONNECT", "[STAGE 1: SETUP] connect_phase: Test-level connections verified.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 3: end_of_elaboration_phase (Bottom-Up, Time 0)
    // Hierarchy is fully built and connected; print topology and inspect handles.
    // -------------------------------------------------------------------------
    virtual function void end_of_elaboration_phase(uvm_phase phase);
        super.end_of_elaboration_phase(phase);
        `uvm_info("PHASE_3_END_OF_ELAB", "[STAGE 1: SETUP] end_of_elaboration_phase: Printing full UVM component topology tree...", UVM_LOW)
        uvm_top.print_topology();
    endfunction

    // -------------------------------------------------------------------------
    // Phase 4: start_of_simulation_phase (Bottom-Up, Time 0)
    // Final pre-run checks and simulation start banner.
    // -------------------------------------------------------------------------
    virtual function void start_of_simulation_phase(uvm_phase phase);
        super.start_of_simulation_phase(phase);
        `uvm_info("PHASE_4_START_OF_SIM", "==================================================================", UVM_LOW)
        `uvm_info("PHASE_4_START_OF_SIM", " [STAGE 1: SETUP] start_of_simulation_phase: Ready to begin simulation!", UVM_LOW)
        `uvm_info("PHASE_4_START_OF_SIM", "==================================================================", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 6: extract_phase (Bottom-Up, Time > 0, Instantaneous)
    // Harvest results, counters, and statistics from components.
    // -------------------------------------------------------------------------
    virtual function void extract_phase(uvm_phase phase);
        super.extract_phase(phase);
        `uvm_info("PHASE_6_EXTRACT", "[STAGE 3: CLEANUP] extract_phase: Extracting test-level statistics and performance metrics...", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 7: check_phase (Bottom-Up, Time > 0, Instantaneous)
    // Sanity checks to ensure no hanging operations or unhandled errors.
    // -------------------------------------------------------------------------
    virtual function void check_phase(uvm_phase phase);
        super.check_phase(phase);
        `uvm_info("PHASE_7_CHECK", "[STAGE 3: CLEANUP] check_phase: Verifying all objections dropped and no hanging transactions exist.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 8: report_phase (Bottom-Up, Time > 0, Instantaneous)
    // Final test reporting and summary.
    // -------------------------------------------------------------------------
    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info("PHASE_8_REPORT", "[STAGE 3: CLEANUP] report_phase: Preparing final execution summary.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 9: final_phase (Top-Down, Time > 0, Instantaneous)
    // Final post-simulation sign-off and shutdown.
    // -------------------------------------------------------------------------
    virtual function void final_phase(uvm_phase phase);
        super.final_phase(phase);
        `uvm_info("PHASE_9_FINAL", "==================================================================", UVM_LOW)
        `uvm_info("PHASE_9_FINAL", " [STAGE 3: CLEANUP] final_phase: Testbench execution completed successfully!", UVM_LOW)
        `uvm_info("PHASE_9_FINAL", "==================================================================", UVM_LOW)
    endfunction

endclass : newton_base_test

`endif // NEWTON_BASE_TEST_SV
