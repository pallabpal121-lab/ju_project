// =============================================================================
// File Name   : newton_driver.sv
// Class Name  : newton_driver
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_DRIVER_SV
`define NEWTON_DRIVER_SV

class newton_driver extends uvm_driver #(newton_seq_item);
    `uvm_component_utils(newton_driver)

    virtual newton_if vif;

    function new(string name = "newton_driver", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual newton_if)::get(this, "", "vif", vif)) begin
            `uvm_fatal("DRV_NO_VIF", "Virtual interface 'vif' not found in uvm_config_db!")
        end
    endfunction

    virtual task run_phase(uvm_phase phase);
        reset_signals();

        forever begin
            seq_item_port.get_next_item(req);
            drive_item(req);
            seq_item_port.item_done();
        end
    endtask

    virtual task reset_signals();
        vif.drv_cb.prog_en    <= 1'b0;
        vif.drv_cb.prog_addr  <= '0;
        vif.drv_cb.prog_data  <= '0;
        vif.drv_cb.start      <= 1'b0;
        vif.drv_cb.x_init     <= '0;
        vif.drv_cb.tolerance  <= '0;
        vif.drv_cb.step_alpha <= '0;
        vif.drv_cb.lambda_reg <= '0;
        vif.drv_cb.max_iters  <= '0;

        wait (vif.rst_n === 1'b1);
        @(vif.drv_cb);
    endtask

    virtual task drive_item(newton_seq_item item);
        int timeout_cnt = 0;
        const int MAX_TIMEOUT_CYCLES = 50000;

        // Step 1: Program microcode
        if (item.reprogram || item.prog_length > 0) begin
            `uvm_info(get_type_name(), $sformatf("Programming %0d micro-instructions into DFG memory...", item.prog_length), UVM_HIGH)
            for (int i = 0; i < item.prog_length; i++) begin
                vif.drv_cb.prog_en   <= 1'b1;
                vif.drv_cb.prog_addr <= i[4:0];
                vif.drv_cb.prog_data <= item.program_mem[i];
                @(vif.drv_cb);
            end
            vif.drv_cb.prog_en   <= 1'b0;
            vif.drv_cb.prog_addr <= '0;
            vif.drv_cb.prog_data <= '0;
            @(vif.drv_cb);
        end

        // Step 2: Launch Solver
        `uvm_info(get_type_name(), $sformatf("Launching Solver: x_init=0x%08h (%0.4f)", item.x_init, real'(item.x_init)/65536.0), UVM_MEDIUM)
        vif.drv_cb.start      <= 1'b1;
        vif.drv_cb.x_init     <= item.x_init;
        vif.drv_cb.tolerance  <= item.tolerance;
        vif.drv_cb.step_alpha <= item.step_alpha;
        vif.drv_cb.lambda_reg <= item.lambda_reg;
        vif.drv_cb.max_iters  <= item.max_iters;

        @(vif.drv_cb);
        vif.drv_cb.start      <= 1'b0;

        // Step 3: Wait for done
        while (!vif.drv_cb.done) begin
            @(vif.drv_cb);
            timeout_cnt++;
            if (timeout_cnt > MAX_TIMEOUT_CYCLES) begin
                `uvm_error("DRV_TIMEOUT", $sformatf("DUT timed out after %0d clock cycles!", MAX_TIMEOUT_CYCLES))
                break;
            end
        end

        // Sample outputs
        item.x_optimal  = vif.drv_cb.x_optimal;
        item.f_optimal  = vif.drv_cb.f_optimal;
        item.g_final    = vif.drv_cb.g_final;
        item.iter_count = vif.drv_cb.iter_count;
        item.status     = status_t'(vif.drv_cb.status);

        `uvm_info(get_type_name(), $sformatf("Done received. Result: %s", item.convert2string()), UVM_HIGH)

        repeat(2) @(vif.drv_cb);
    endtask

endclass : newton_driver

`endif // NEWTON_DRIVER_SV
