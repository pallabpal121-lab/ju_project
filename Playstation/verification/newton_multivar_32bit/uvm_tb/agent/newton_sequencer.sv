// =============================================================================
// File Name   : newton_sequencer.sv
// Class Name  : newton_sequencer
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : UVM Sequencer for Newton BCD AXI-Lite VIP implementing
//               all 9 standard UVM phases.
// =============================================================================

`ifndef NEWTON_SEQUENCER_SV
`define NEWTON_SEQUENCER_SV

class newton_sequencer extends uvm_sequencer #(newton_axi_seq_item);
    `uvm_component_utils(newton_sequencer)

    function new(string name = "newton_sequencer", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    // Phase 1: build_phase
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("SQR_PHASE_1_BUILD", "[STAGE 1: SETUP] build_phase: Initializing sequencer resources.", UVM_LOW)
    endfunction

    // Phase 2: connect_phase
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        `uvm_info("SQR_PHASE_2_CONNECT", "[STAGE 1: SETUP] connect_phase: Sequencer connection verified.", UVM_LOW)
    endfunction

    // Phase 3: end_of_elaboration_phase
    virtual function void end_of_elaboration_phase(uvm_phase phase);
        super.end_of_elaboration_phase(phase);
        `uvm_info("SQR_PHASE_3_END_OF_ELAB", "[STAGE 1: SETUP] end_of_elaboration_phase: Sequencer elaboration complete.", UVM_LOW)
    endfunction

    // Phase 4: start_of_simulation_phase
    virtual function void start_of_simulation_phase(uvm_phase phase);
        super.start_of_simulation_phase(phase);
        `uvm_info("SQR_PHASE_4_START_OF_SIM", "[STAGE 1: SETUP] start_of_simulation_phase: Sequencer ready for sequence execution.", UVM_LOW)
    endfunction

    // Phase 5: run_phase
    virtual task run_phase(uvm_phase phase);
        super.run_phase(phase);
        `uvm_info("SQR_PHASE_5_RUN", "[STAGE 2: RUN] run_phase: Sequencer arbitration active.", UVM_HIGH)
    endtask

    // Phase 6: extract_phase
    virtual function void extract_phase(uvm_phase phase);
        super.extract_phase(phase);
        `uvm_info("SQR_PHASE_6_EXTRACT", "[STAGE 3: CLEANUP] extract_phase: Sequencer extraction completed.", UVM_LOW)
    endfunction

    // Phase 7: check_phase
    virtual function void check_phase(uvm_phase phase);
        super.check_phase(phase);
        `uvm_info("SQR_PHASE_7_CHECK", "[STAGE 3: CLEANUP] check_phase: Verifying sequencer FIFO queues are clear.", UVM_LOW)
    endfunction

    // Phase 8: report_phase
    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info("SQR_PHASE_8_REPORT", "[STAGE 3: CLEANUP] report_phase: Sequencer report complete.", UVM_LOW)
    endfunction

    // Phase 9: final_phase
    virtual function void final_phase(uvm_phase phase);
        super.final_phase(phase);
        `uvm_info("SQR_PHASE_9_FINAL", "[STAGE 3: CLEANUP] final_phase: Sequencer shutdown cleanly.", UVM_LOW)
    endfunction

endclass : newton_sequencer

`endif // NEWTON_SEQUENCER_SV
