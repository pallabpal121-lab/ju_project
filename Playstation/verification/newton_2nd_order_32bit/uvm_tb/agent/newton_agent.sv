// =============================================================================
// File Name   : newton_agent.sv
// Class Name  : newton_agent (Demonstrating All 9 Standard UVM Phases)
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_AGENT_SV
`define NEWTON_AGENT_SV

class newton_agent extends uvm_agent;
    `uvm_component_utils(newton_agent)

    newton_agent_config                  cfg;
    newton_driver                        drv;
    newton_sequencer                     sqr;
    newton_monitor                       mon;
    uvm_analysis_port #(newton_seq_item) agt_ap;

    function new(string name = "newton_agent", uvm_component parent = null);
        super.new(name, parent);
        agt_ap = new("agt_ap", this);
    endfunction

    // -------------------------------------------------------------------------
    // Phase 1: build_phase (Top-Down, Time 0)
    // Retrieves agent configuration and instantiates monitor, driver, sequencer.
    // -------------------------------------------------------------------------
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("AGT_PHASE_1_BUILD", "[STAGE 1: SETUP] build_phase: Agent building components...", UVM_LOW)
        if (uvm_config_db#(newton_agent_config)::get(this, "", "cfg", cfg)) begin
            is_active = cfg.is_active;
        end
        mon = newton_monitor::type_id::create("mon", this);
        if (get_is_active() == UVM_ACTIVE) begin
            drv = newton_driver::type_id::create("drv", this);
            sqr = newton_sequencer::type_id::create("sqr", this);
        end
    endfunction

    // -------------------------------------------------------------------------
    // Phase 2: connect_phase (Bottom-Up, Time 0)
    // Connects sequencer-to-driver TLM ports and monitor-to-agent analysis port.
    // -------------------------------------------------------------------------
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        `uvm_info("AGT_PHASE_2_CONNECT", "[STAGE 1: SETUP] connect_phase: Wiring driver-sequencer and monitor analysis port...", UVM_LOW)
        if (get_is_active() == UVM_ACTIVE) begin
            drv.seq_item_port.connect(sqr.seq_item_export);
        end
        mon.mon_ap.connect(agt_ap);
    endfunction

    // -------------------------------------------------------------------------
    // Phase 3: end_of_elaboration_phase (Bottom-Up, Time 0)
    // Confirms agent configuration and subcomponent binding.
    // -------------------------------------------------------------------------
    virtual function void end_of_elaboration_phase(uvm_phase phase);
        super.end_of_elaboration_phase(phase);
        `uvm_info("AGT_PHASE_3_END_OF_ELAB", $sformatf("[STAGE 1: SETUP] end_of_elaboration_phase: Agent configured as %s.", (get_is_active() == UVM_ACTIVE) ? "ACTIVE" : "PASSIVE"), UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 4: start_of_simulation_phase (Bottom-Up, Time 0)
    // -------------------------------------------------------------------------
    virtual function void start_of_simulation_phase(uvm_phase phase);
        super.start_of_simulation_phase(phase);
        `uvm_info("AGT_PHASE_4_START_OF_SIM", "[STAGE 1: SETUP] start_of_simulation_phase: Agent armed and ready.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 6: extract_phase (Bottom-Up, Time > 0)
    // -------------------------------------------------------------------------
    virtual function void extract_phase(uvm_phase phase);
        super.extract_phase(phase);
        `uvm_info("AGT_PHASE_6_EXTRACT", "[STAGE 3: CLEANUP] extract_phase: Agent harvesting status.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 7: check_phase (Bottom-Up, Time > 0)
    // -------------------------------------------------------------------------
    virtual function void check_phase(uvm_phase phase);
        super.check_phase(phase);
        `uvm_info("AGT_PHASE_7_CHECK", "[STAGE 3: CLEANUP] check_phase: Agent confirming clean termination.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 8: report_phase (Bottom-Up, Time > 0)
    // -------------------------------------------------------------------------
    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info("AGT_PHASE_8_REPORT", "[STAGE 3: CLEANUP] report_phase: Agent reporting completed.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 9: final_phase (Top-Down, Time > 0)
    // -------------------------------------------------------------------------
    virtual function void final_phase(uvm_phase phase);
        super.final_phase(phase);
        `uvm_info("AGT_PHASE_9_FINAL", "[STAGE 3: CLEANUP] final_phase: Agent shutdown complete.", UVM_LOW)
    endfunction

endclass : newton_agent

`endif // NEWTON_AGENT_SV
