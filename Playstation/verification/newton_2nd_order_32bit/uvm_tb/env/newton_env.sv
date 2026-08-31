// =============================================================================
// File Name   : newton_env.sv
// Class Name  : newton_env
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

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        agent = newton_agent::type_id::create("agent", this);
        scb   = newton_scoreboard::type_id::create("scb", this);
        cov   = newton_coverage::type_id::create("cov", this);
    endfunction

    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        agent.agt_ap.connect(scb.item_export);
        agent.agt_ap.connect(cov.analysis_export);
    endfunction

endclass : newton_env

`endif // NEWTON_ENV_SV
