// =============================================================================
// File Name   : newton_scoreboard.sv
// Class Name  : newton_scoreboard
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Production UVM Scoreboard comparing DUT outputs against the
//               bit-accurate Golden Reference Model across all 9 UVM phases.
// =============================================================================

`ifndef NEWTON_SCOREBOARD_SV
`define NEWTON_SCOREBOARD_SV

class newton_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(newton_scoreboard)

    uvm_analysis_imp #(newton_axi_seq_item, newton_scoreboard) item_export;

    int match_count;
    int mismatch_count;
    int raw_axi_count;

    function new(string name = "newton_scoreboard", uvm_component parent = null);
        super.new(name, parent);
        item_export    = new("item_export", this);
        match_count    = 0;
        mismatch_count = 0;
        raw_axi_count  = 0;
    endfunction

    // Phase 1: build_phase
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("SCB_PHASE_1_BUILD", "[STAGE 1: SETUP] build_phase: Scoreboard initialized.", UVM_LOW)
    endfunction

    // Phase 2: connect_phase
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        `uvm_info("SCB_PHASE_2_CONNECT", "[STAGE 1: SETUP] connect_phase: Scoreboard export connected.", UVM_LOW)
    endfunction

    // Phase 3: end_of_elaboration_phase
    virtual function void end_of_elaboration_phase(uvm_phase phase);
        super.end_of_elaboration_phase(phase);
        `uvm_info("SCB_PHASE_3_END_OF_ELAB", "[STAGE 1: SETUP] end_of_elaboration_phase: Scoreboard comparator verified.", UVM_LOW)
    endfunction

    // Phase 4: start_of_simulation_phase
    virtual function void start_of_simulation_phase(uvm_phase phase);
        super.start_of_simulation_phase(phase);
        `uvm_info("SCB_PHASE_4_START_OF_SIM", "[STAGE 1: SETUP] start_of_simulation_phase: Scoreboard armed with BCD reference model.", UVM_LOW)
    endfunction

    // Phase 5: run_phase
    virtual task run_phase(uvm_phase phase);
        super.run_phase(phase);
        `uvm_info("SCB_PHASE_5_RUN", "[STAGE 2: RUN] run_phase: Scoreboard active and listening.", UVM_HIGH)
    endtask

    // -------------------------------------------------------------------------
    // Scoreboard Write Implementation
    // -------------------------------------------------------------------------
    virtual function void write(newton_axi_seq_item item);
        if (item.is_raw_axi) begin
            raw_axi_count++;
            if (item.axi_resp !== 2'b00) begin
                `uvm_error("SCB_AXI_RESP_ERR", $sformatf("AXI transfer error response: 2'b%02b on addr=0x%03h", item.axi_resp, item.axi_addr))
                mismatch_count++;
            end else begin
                match_count++;
            end
            return;
        end

        begin
            q16_t       exp_x_opt [MAX_VARS];
            q16_t       exp_f_opt;
            q16_t       exp_max_delta;
            logic [7:0] exp_sweep_cnt;
            status_t    exp_status;
            bit         mismatch;

            mismatch = 1'b0;

            newton_multivar_ref_model::solve_bcd_golden(
                item.program_mem,
                item.num_vars,
                item.x_init,
                item.tolerance,
                item.step_alpha,
                item.lambda_reg,
                item.max_sweeps,
                exp_x_opt,
                exp_f_opt,
                exp_max_delta,
                exp_sweep_cnt,
                exp_status
            );

            // 1. Status Check
            if (item.status !== exp_status) begin
                `uvm_error("SCB_STATUS_MISMATCH", $sformatf("Status mismatch! DUT=%s, EXP=%s", item.status.name(), exp_status.name()))
                mismatch = 1'b1;
            end

            // 2. Sweep Count Check
            if (item.sweep_count !== exp_sweep_cnt) begin
                `uvm_error("SCB_SWEEPS_MISMATCH", $sformatf("Sweep count mismatch! DUT=%0d, EXP=%0d", item.sweep_count, exp_sweep_cnt))
                mismatch = 1'b1;
            end

            // 3. Optimal State Vector Check (tolerance: 64 LSBs ≈ 0.00097)
            for (int i = 0; i < item.num_vars; i++) begin
                int diff = (item.x_optimal[i] > exp_x_opt[i]) ? (item.x_optimal[i] - exp_x_opt[i]) : (exp_x_opt[i] - item.x_optimal[i]);
                if (diff > 64) begin
                    `uvm_error("SCB_X_MISMATCH", $sformatf("x[%0d] mismatch! DUT=0x%08h (%0.4f), EXP=0x%08h (%0.4f), diff=%0d LSBs",
                               i, item.x_optimal[i], real'(item.x_optimal[i])/65536.0,
                               exp_x_opt[i], real'(exp_x_opt[i])/65536.0, diff))
                    mismatch = 1'b1;
                end
            end

            // 4. Optimal Function Value Check (tolerance: 4096 LSBs ≈ 0.0625)
            begin
                int diff_f = (item.f_optimal > exp_f_opt) ? (item.f_optimal - exp_f_opt) : (exp_f_opt - item.f_optimal);
                if (diff_f > 4096) begin
                    `uvm_error("SCB_F_MISMATCH", $sformatf("f(x*) mismatch! DUT=0x%08h (%0.4f), EXP=0x%08h (%0.4f), diff=%0d LSBs",
                               item.f_optimal, real'(item.f_optimal)/65536.0,
                               exp_f_opt, real'(exp_f_opt)/65536.0, diff_f))
                    mismatch = 1'b1;
                end
            end

            if (mismatch) begin
                mismatch_count++;
                `uvm_error(get_type_name(), $sformatf("FAILED Transaction:\n%s", item.convert2string()))
            end else begin
                match_count++;
                `uvm_info(get_type_name(), $sformatf("PASSED [Match #%0d]: status=%s, sweeps=%0d, f*=%0.4f",
                          match_count, item.status.name(), item.sweep_count, real'(item.f_optimal)/65536.0), UVM_LOW)
            end
        end
    endfunction

    // Phase 6: extract_phase
    virtual function void extract_phase(uvm_phase phase);
        super.extract_phase(phase);
        `uvm_info("SCB_PHASE_6_EXTRACT", $sformatf("[STAGE 3: CLEANUP] extract_phase: Matches=%0d, Mismatches=%0d, Raw AXI Transfers=%0d",
                  match_count, mismatch_count, raw_axi_count), UVM_LOW)
    endfunction

    // Phase 7: check_phase
    virtual function void check_phase(uvm_phase phase);
        super.check_phase(phase);
        `uvm_info("SCB_PHASE_7_CHECK", "[STAGE 3: CLEANUP] check_phase: Verifying scoreboard results...", UVM_LOW)
        if (mismatch_count > 0) begin
            `uvm_error("SCB_CHECK_FAIL", $sformatf("Scoreboard check failed with %0d mismatches!", mismatch_count))
        end else if (match_count == 0) begin
            `uvm_error("SCB_NO_TX", "Scoreboard check failed: Zero transactions evaluated!")
        end else begin
            `uvm_info("SCB_CHECK_PASS", "Scoreboard integrity check PASSED with 0 mismatches.", UVM_LOW)
        end
    endfunction

    // Phase 8: report_phase
    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info(get_type_name(), "==================================================================", UVM_NONE)
        `uvm_info(get_type_name(), "      [STAGE 3: CLEANUP] SCOREBOARD VERIFICATION FINAL REPORT     ", UVM_NONE)
        `uvm_info(get_type_name(), "==================================================================", UVM_NONE)
        `uvm_info(get_type_name(), $sformatf("  Total Successful Matches : %0d", match_count), UVM_NONE)
        `uvm_info(get_type_name(), $sformatf("  Total Scoreboard Errors  : %0d", mismatch_count), UVM_NONE)
        `uvm_info(get_type_name(), $sformatf("  Raw AXI Bus Transfers    : %0d", raw_axi_count), UVM_NONE)
        if (mismatch_count == 0 && match_count > 0) begin
            `uvm_info(get_type_name(), "  >>> TEST STATUS: ALL OPTIMIZATION & BUS CHECKS PASSED! <<<", UVM_NONE)
        end else begin
            `uvm_error(get_type_name(), "  >>> TEST STATUS: SIMULATION FAILED CHECKS! <<<")
        end
        `uvm_info(get_type_name(), "==================================================================", UVM_NONE)
    endfunction

    // Phase 9: final_phase
    virtual function void final_phase(uvm_phase phase);
        super.final_phase(phase);
        `uvm_info("SCB_PHASE_9_FINAL", "[STAGE 3: CLEANUP] final_phase: Scoreboard closed successfully.", UVM_LOW)
    endfunction

endclass : newton_scoreboard

`endif // NEWTON_SCOREBOARD_SV
