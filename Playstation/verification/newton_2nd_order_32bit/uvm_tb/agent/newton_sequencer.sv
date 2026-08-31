// =============================================================================
// File Name   : newton_sequencer.sv
// Class Name  : newton_sequencer
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_SEQUENCER_SV
`define NEWTON_SEQUENCER_SV

class newton_sequencer extends uvm_sequencer #(newton_seq_item);
    `uvm_component_utils(newton_sequencer)

    function new(string name = "newton_sequencer", uvm_component parent = null);
        super.new(name, parent);
    endfunction
endclass : newton_sequencer

`endif // NEWTON_SEQUENCER_SV
