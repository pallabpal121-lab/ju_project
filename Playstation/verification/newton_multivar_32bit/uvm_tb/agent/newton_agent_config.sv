// =============================================================================
// File Name   : newton_agent_config.sv
// Class Name  : newton_agent_config
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_AGENT_CONFIG_SV
`define NEWTON_AGENT_CONFIG_SV

class newton_agent_config extends uvm_object;
    `uvm_object_utils(newton_agent_config)

    // Industry Best Practice: Encapsulated Virtual Interface Handle
    newton_vif_t            vif;

    uvm_active_passive_enum is_active    = UVM_ACTIVE;
    bit                     has_coverage = 1'b1;
    bit                     has_checks   = 1'b1;

    function new(string name = "newton_agent_config");
        super.new(name);
    endfunction

endclass : newton_agent_config

`endif // NEWTON_AGENT_CONFIG_SV
