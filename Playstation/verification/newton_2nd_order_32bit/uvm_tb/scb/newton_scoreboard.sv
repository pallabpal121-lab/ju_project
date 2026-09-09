// =============================================================================
// File Name   : newton_scoreboard.sv
// Class Name  : newton_scoreboard
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_SCOREBOARD_SV
`define NEWTON_SCOREBOARD_SV

class newton_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(newton_scoreboard)

    uvm_analysis_imp #(newton_seq_item, newton_scoreboard) item_export;

    int match_count;
    int mismatch_count;

    function new(string name = "newton_scoreboard", uvm_component parent = null);
        super.new(name, parent);
        item_export    = new("item_export", this);
        match_count    = 0;
        mismatch_count = 0;
    endfunction

    virtual function void write(newton_seq_item item);
        q16_t       exp_x_opt;
        q16_t       exp_f_opt;
        q16_t       exp_g_final;
        logic [7:0] exp_iter_cnt;
        status_t    exp_status;
        bit         mismatch;

        newton_ref_model::solve_golden(
            item.program_mem,
            item.x_init,
            item.tolerance,
            item.step_alpha,
            item.lambda_reg,
            item.max_iters,
            exp_x_opt,
            exp_f_opt,
            exp_g_final,
            exp_iter_cnt,
            exp_status
        );

        mismatch = 1'b0;

        // 1. Status Check
        if (item.status !== exp_status) begin
            `uvm_error("SCB_STATUS_MISMATCH", $sformatf("Status mismatch! DUT=%s, EXP=%s", item.status.name(), exp_status.name()))
            mismatch = 1'b1;
        end

        // 2. Iteration Count Check
        if (item.iter_count !== exp_iter_cnt) begin
            `uvm_error("SCB_ITER_MISMATCH", $sformatf("Iteration mismatch! DUT=%0d, EXP=%0d", item.iter_count, exp_iter_cnt))
            mismatch = 1'b1;
        end

        // 3. Optimal X Check
        if (item.x_optimal !== exp_x_opt) begin
            int diff = (item.x_optimal > exp_x_opt) ? (item.x_optimal - exp_x_opt) : (exp_x_opt - item.x_optimal);
            if (diff > 4) begin
                `uvm_error("SCB_X_MISMATCH", $sformatf("X_opt mismatch! DUT=0x%08h (%0.4f), EXP=0x%08h (%0.4f), diff=%0d",
                           item.x_optimal, real'(item.x_optimal)/65536.0, exp_x_opt, real'(exp_x_opt)/65536.0, diff))
                mismatch = 1'b1;
            end
        end

        // 4. Function Value Check
        if (item.f_optimal !== exp_f_opt) begin
            int diff_f = (item.f_optimal > exp_f_opt) ? (item.f_optimal - exp_f_opt) : (exp_f_opt - item.f_optimal);
            if (diff_f > 4) begin
                `uvm_error("SCB_F_MISMATCH", $sformatf("F_opt mismatch! DUT=0x%08h (%0.4f), EXP=0x%08h (%0.4f)",
                           item.f_optimal, real'(item.f_optimal)/65536.0, exp_f_opt, real'(exp_f_opt)/65536.0))
                mismatch = 1'b1;
            end
        end

        if (mismatch) begin
            mismatch_count++;
            `uvm_error(get_type_name(), $sformatf("FAILED Transaction: %s", item.convert2string()))
        end else begin
            match_count++;
            `uvm_info(get_type_name(), $sformatf("PASSED [Match #%0d]: %s", match_count, item.convert2string()), UVM_LOW)
        end
    endfunction

    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info(get_type_name(), "==================================================", UVM_NONE)
        `uvm_info(get_type_name(), "  SCOREBOARD FINAL REPORT:", UVM_NONE)
        `uvm_info(get_type_name(), $sformatf("    Matches   : %0d", match_count), UVM_NONE)
        `uvm_info(get_type_name(), $sformatf("    Mismatches: %0d", mismatch_count), UVM_NONE)
        if (mismatch_count == 0 && match_count > 0) begin
            `uvm_info(get_type_name(), "  >>> TEST STATUS: ALL CHECKS PASSED PERFECTLY! <<<", UVM_NONE)
        end else if (match_count == 0) begin
            `uvm_error("SCB_NO_TX", "  >>> TEST STATUS: SIMULATION FAILED - NO TRANSACTIONS EVALUATED! <<<")
        end else begin
            `uvm_error(get_type_name(), "  >>> TEST STATUS: SIMULATION FAILED WITH MISMATCHES! <<<")
        end
        `uvm_info(get_type_name(), "==================================================", UVM_NONE)
    endfunction

endclass : newton_scoreboard

`endif // NEWTON_SCOREBOARD_SV
