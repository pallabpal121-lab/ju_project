// =============================================================================
// File Name   : newton_monitor.sv
// Class Name  : newton_monitor
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_MONITOR_SV
`define NEWTON_MONITOR_SV

class newton_monitor extends uvm_monitor;
    `uvm_component_utils(newton_monitor)

    virtual newton_if vif;
    uvm_analysis_port #(newton_seq_item) mon_ap;

    instr_t mirr_prog_mem [PROG_DEPTH];
    int     mirr_prog_len;

    function new(string name = "newton_monitor", uvm_component parent = null);
        super.new(name, parent);
        mon_ap = new("mon_ap", this);
        mirr_prog_len = 0;
        for (int i = 0; i < PROG_DEPTH; i++) begin
            mirr_prog_mem[i] = '0;
        end
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual newton_if)::get(this, "", "vif", vif)) begin
            `uvm_fatal("MON_NO_VIF", "Virtual interface 'vif' not found in uvm_config_db!")
        end
    endfunction

    virtual task run_phase(uvm_phase phase);
        wait (vif.rst_n === 1'b1);
        @(vif.mon_cb);

        fork
            track_programming();
            monitor_execution();
        join
    endtask

    virtual task track_programming();
        forever begin
            @(vif.mon_cb);
            if (vif.mon_cb.prog_en) begin
                mirr_prog_mem[vif.mon_cb.prog_addr] = vif.mon_cb.prog_data;
                if (int'(vif.mon_cb.prog_addr) + 1 > mirr_prog_len) begin
                    mirr_prog_len = int'(vif.mon_cb.prog_addr) + 1;
                end
            end
        end
    endtask

    virtual task monitor_execution();
        newton_seq_item item;

        forever begin
            @(vif.mon_cb);
            if (vif.mon_cb.start) begin
                item = newton_seq_item::type_id::create("mon_item");
                
                item.x_init      = vif.mon_cb.x_init;
                item.tolerance   = vif.mon_cb.tolerance;
                item.step_alpha  = vif.mon_cb.step_alpha;
                item.lambda_reg  = vif.mon_cb.lambda_reg;
                item.max_iters   = vif.mon_cb.max_iters;
                item.prog_length = mirr_prog_len;
                for (int i = 0; i < PROG_DEPTH; i++) begin
                    item.program_mem[i] = mirr_prog_mem[i];
                end

                while (!vif.mon_cb.done) begin
                    @(vif.mon_cb);
                end

                item.x_optimal  = vif.mon_cb.x_optimal;
                item.f_optimal  = vif.mon_cb.f_optimal;
                item.g_final    = vif.mon_cb.g_final;
                item.iter_count = vif.mon_cb.iter_count;
                item.status     = status_t'(vif.mon_cb.status);

                `uvm_info(get_type_name(), $sformatf("Monitored Transaction: %s", item.convert2string()), UVM_HIGH)
                mon_ap.write(item);
            end
        end
    endtask

endclass : newton_monitor

`endif // NEWTON_MONITOR_SV
