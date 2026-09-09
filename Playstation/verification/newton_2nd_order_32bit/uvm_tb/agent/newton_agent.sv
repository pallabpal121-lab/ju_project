// =============================================================================
// File Name   : newton_agent.sv
// Class Name  : newton_agent
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

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (uvm_config_db#(newton_agent_config)::get(this, "", "cfg", cfg)) begin
            set_is_active(cfg.is_active);
        end
        mon = newton_monitor::type_id::create("mon", this);
        if (get_is_active() == UVM_ACTIVE) begin
            drv = newton_driver::type_id::create("drv", this);
            sqr = newton_sequencer::type_id::create("sqr", this);
        end
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        if (get_is_active() == UVM_ACTIVE) begin
            drv.seq_item_port.connect(sqr.seq_item_export);
        end
        mon.mon_ap.connect(agt_ap);
    endfunction

endclass : newton_agent

`endif // NEWTON_AGENT_SV
