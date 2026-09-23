// =============================================================================
// File Name   : newton_env.sv
// Class Name  : newton_env (Demonstrating All 9 Standard UVM Phases)
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_ENV_SV
`define NEWTON_ENV_SV

class newton_env extends uvm_env;
    `uvm_component_utils(newton_env)

    newton_agent      agent;
    newton_scoreboard scb;
    newton_coverage   cov;

    function new(string name = "newton_env", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    // -------------------------------------------------------------------------
    // Phase 1: build_phase (Top-Down, Time 0)
    // Instantiates the agent, scoreboard, and coverage components.
    // -------------------------------------------------------------------------
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("ENV_PHASE_1_BUILD", "[STAGE 1: SETUP] build_phase: Instantiating agent, scoreboard, and coverage...", UVM_LOW)
        agent = newton_agent::type_id::create("agent", this);
        scb   = newton_scoreboard::type_id::create("scb", this);
        cov   = newton_coverage::type_id::create("cov", this);
    endfunction

    // -------------------------------------------------------------------------
    // Phase 2: connect_phase (Bottom-Up, Time 0)
    // Wires analysis ports from the agent to the scoreboard and coverage.
    // -------------------------------------------------------------------------
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        `uvm_info("ENV_PHASE_2_CONNECT", "[STAGE 1: SETUP] connect_phase: Connecting agent analysis ports to scoreboard and coverage...", UVM_LOW)
        agent.agt_ap.connect(scb.item_export);
        agent.agt_ap.connect(cov.analysis_export);
    endfunction

    // -------------------------------------------------------------------------
    // Phase 3: end_of_elaboration_phase (Bottom-Up, Time 0)
    // Validates that all environmental handles and subcomponents are non-null.
    // -------------------------------------------------------------------------
    virtual function void end_of_elaboration_phase(uvm_phase phase);
        super.end_of_elaboration_phase(phase);
        `uvm_info("ENV_PHASE_3_END_OF_ELAB", "[STAGE 1: SETUP] end_of_elaboration_phase: Verifying environment component handles...", UVM_LOW)
        if (agent == null || scb == null || cov == null) begin
            `uvm_fatal("ENV_NULL_COMP", "One or more environment subcomponents failed instantiation!")
        end
    endfunction

    // -------------------------------------------------------------------------
    // Phase 4: start_of_simulation_phase (Bottom-Up, Time 0)
    // Announces that the verification environment is armed and ready.
    // -------------------------------------------------------------------------
    virtual function void start_of_simulation_phase(uvm_phase phase);
        super.start_of_simulation_phase(phase);
        `uvm_info("ENV_PHASE_4_START_OF_SIM", "[STAGE 1: SETUP] start_of_simulation_phase: Environment is armed and ready for simulation run.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 6: extract_phase (Bottom-Up, Time > 0, Instantaneous)
    // Extracts environment-level transaction metrics.
    // -------------------------------------------------------------------------
    virtual function void extract_phase(uvm_phase phase);
        super.extract_phase(phase);
        `uvm_info("ENV_PHASE_6_EXTRACT", "[STAGE 3: CLEANUP] extract_phase: Environment harvesting final verification states.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 7: check_phase (Bottom-Up, Time > 0, Instantaneous)
    // Confirms that all environment components finished cleanly.
    // -------------------------------------------------------------------------
    virtual function void check_phase(uvm_phase phase);
        super.check_phase(phase);
        `uvm_info("ENV_PHASE_7_CHECK", "[STAGE 3: CLEANUP] check_phase: Performing environment sanity and integrity checks.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 8: report_phase (Bottom-Up, Time > 0, Instantaneous)
    // Environment reporting notification.
    // -------------------------------------------------------------------------
    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info("ENV_PHASE_8_REPORT", "[STAGE 3: CLEANUP] report_phase: Environment delegating report generation to scoreboard and coverage.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 9: final_phase (Top-Down, Time > 0, Instantaneous)
    // Environment final sign-off.
    // -------------------------------------------------------------------------
    virtual function void final_phase(uvm_phase phase);
        super.final_phase(phase);
        `uvm_info("ENV_PHASE_9_FINAL", "[STAGE 3: CLEANUP] final_phase: Environment tear-down complete.", UVM_LOW)
    endfunction

endclass : newton_env

`endif // NEWTON_ENV_SV
