// =============================================================================
// File Name   : newton_agent.sv
// Class Name  : newton_agent
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : UVM Agent encapsulating sequencer, driver, and monitor,
//               implementing all 9 standard UVM phases.
// =============================================================================

`ifndef NEWTON_AGENT_SV
`define NEWTON_AGENT_SV

class newton_agent extends uvm_agent;
    `uvm_component_utils(newton_agent)

    newton_agent_config                      cfg;
    newton_sequencer                         sqr;
    newton_driver                            drv;
    newton_monitor                           mon;
    uvm_analysis_port #(newton_axi_seq_item) agt_ap;

    function new(string name = "newton_agent", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    // Phase 1: build_phase
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("AGT_PHASE_1_BUILD", "[STAGE 1: SETUP] build_phase: Building Newton Agent hierarchy...", UVM_LOW)

        if (!uvm_config_db#(newton_agent_config)::get(this, "", "cfg", cfg)) begin
            `uvm_info("AGT_NO_CFG", "Agent configuration not found; creating default active config.", UVM_MEDIUM)
            cfg = newton_agent_config::type_id::create("cfg");
        end

        // Industry Best Practice: If vif was not directly set in cfg, fetch from uvm_config_db into cfg
        if (cfg.vif == null) begin
            if (!uvm_config_db#(newton_vif_t)::get(this, "", "vif", cfg.vif)) begin
                `uvm_info("AGT_VIF_LOOKUP", "vif not found in agent scope; subcomponents will check parent/config_db.", UVM_HIGH)
            end
        end

        // Propagate config down to driver and monitor
        uvm_config_db#(newton_agent_config)::set(this, "*", "cfg", cfg);
        if (cfg.vif != null) begin
            uvm_config_db#(newton_vif_t)::set(this, "*", "vif", cfg.vif);
        end

        mon    = newton_monitor::type_id::create("mon", this);
        agt_ap = new("agt_ap", this);

        if (cfg.is_active == UVM_ACTIVE) begin
            sqr = newton_sequencer::type_id::create("sqr", this);
            drv = newton_driver::type_id::create("drv", this);
        end
    endfunction

    // Phase 2: connect_phase
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        `uvm_info("AGT_PHASE_2_CONNECT", "[STAGE 1: SETUP] connect_phase: Connecting agent ports...", UVM_LOW)

        mon.mon_ap.connect(agt_ap);

        if (cfg.is_active == UVM_ACTIVE) begin
            drv.seq_item_port.connect(sqr.seq_item_export);
        end
    endfunction

    // Phase 3: end_of_elaboration_phase
    virtual function void end_of_elaboration_phase(uvm_phase phase);
        super.end_of_elaboration_phase(phase);
        `uvm_info("AGT_PHASE_3_END_OF_ELAB", "[STAGE 1: SETUP] end_of_elaboration_phase: Agent elaboration complete.", UVM_LOW)
    endfunction

    // Phase 4: start_of_simulation_phase
    virtual function void start_of_simulation_phase(uvm_phase phase);
        super.start_of_simulation_phase(phase);
        `uvm_info("AGT_PHASE_4_START_OF_SIM", "[STAGE 1: SETUP] start_of_simulation_phase: Agent armed and ready.", UVM_LOW)
    endfunction

    // Phase 5: run_phase
    virtual task run_phase(uvm_phase phase);
        super.run_phase(phase);
        `uvm_info("AGT_PHASE_5_RUN", "[STAGE 2: RUN] run_phase: Agent operational.", UVM_HIGH)
    endtask

    // Phase 6: extract_phase
    virtual function void extract_phase(uvm_phase phase);
        super.extract_phase(phase);
        `uvm_info("AGT_PHASE_6_EXTRACT", "[STAGE 3: CLEANUP] extract_phase: Agent metrics extracted.", UVM_LOW)
    endfunction

    // Phase 7: check_phase
    virtual function void check_phase(uvm_phase phase);
        super.check_phase(phase);
        `uvm_info("AGT_PHASE_7_CHECK", "[STAGE 3: CLEANUP] check_phase: Agent sanity checks complete.", UVM_LOW)
    endfunction

    // Phase 8: report_phase
    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info("AGT_PHASE_8_REPORT", "[STAGE 3: CLEANUP] report_phase: Agent report complete.", UVM_LOW)
    endfunction

    // Phase 9: final_phase
    virtual function void final_phase(uvm_phase phase);
        super.final_phase(phase);
        `uvm_info("AGT_PHASE_9_FINAL", "[STAGE 3: CLEANUP] final_phase: Agent shutdown complete.", UVM_LOW)
    endfunction

endclass : newton_agent

`endif // NEWTON_AGENT_SV
