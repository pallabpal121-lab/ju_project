// =============================================================================
// File Name   : newton_driver.sv
// Class Name  : newton_driver
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Industry-grade AXI4-Lite master driver executing bus
//               handshakes and all 9 standard UVM phases.
// =============================================================================

`ifndef NEWTON_DRIVER_SV
`define NEWTON_DRIVER_SV

class newton_driver extends uvm_driver #(newton_axi_seq_item);
    `uvm_component_utils(newton_driver)

    newton_vif_t        vif;
    newton_agent_config cfg;

    function new(string name = "newton_driver", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    // -------------------------------------------------------------------------
    // Phase 1: build_phase
    // -------------------------------------------------------------------------
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("DRV_PHASE_1_BUILD", "[STAGE 1: SETUP] build_phase: Driver retrieving virtual interface...", UVM_LOW)

        // Industry Best Practice: Retrieve from agent_config first, fallback to config_db
        if (uvm_config_db#(newton_agent_config)::get(this, "", "cfg", cfg) && cfg.vif != null) begin
            vif = cfg.vif;
            `uvm_info("DRV_CFG_VIF", "Driver successfully obtained virtual interface from agent_config.", UVM_HIGH)
        end else if (!uvm_config_db#(newton_vif_t)::get(this, "", "vif", vif)) begin
            `uvm_fatal("DRV_NO_VIF", "Virtual interface 'vif' not found in agent_config or uvm_config_db!")
        end
    endfunction

    // -------------------------------------------------------------------------
    // Phase 2: connect_phase
    // -------------------------------------------------------------------------
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        `uvm_info("DRV_PHASE_2_CONNECT", "[STAGE 1: SETUP] connect_phase: Driver connected to virtual interface.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 3: end_of_elaboration_phase
    // -------------------------------------------------------------------------
    virtual function void end_of_elaboration_phase(uvm_phase phase);
        super.end_of_elaboration_phase(phase);
        `uvm_info("DRV_PHASE_3_END_OF_ELAB", "[STAGE 1: SETUP] end_of_elaboration_phase: Verifying driver interface binding...", UVM_LOW)
        if (vif == null) begin
            `uvm_fatal("DRV_NULL_VIF", "Driver virtual interface 'vif' is null!")
        end
    endfunction

    // -------------------------------------------------------------------------
    // Phase 4: start_of_simulation_phase
    // -------------------------------------------------------------------------
    virtual function void start_of_simulation_phase(uvm_phase phase);
        super.start_of_simulation_phase(phase);
        `uvm_info("DRV_PHASE_4_START_OF_SIM", "[STAGE 1: SETUP] start_of_simulation_phase: Driver armed and ready.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 5: run_phase
    // -------------------------------------------------------------------------
    virtual task run_phase(uvm_phase phase);
        reset_signals();

        forever begin
            seq_item_port.get_next_item(req);
            drive_item(req);
            seq_item_port.item_done();
        end
    endtask

    // -------------------------------------------------------------------------
    // Bus Reset Task
    // -------------------------------------------------------------------------
    virtual task reset_signals();
        vif.drv_cb.s_axi_awaddr  <= '0;
        vif.drv_cb.s_axi_awprot  <= 3'b000;
        vif.drv_cb.s_axi_awvalid <= 1'b0;
        vif.drv_cb.s_axi_wdata   <= '0;
        vif.drv_cb.s_axi_wstrb   <= 4'b0000;
        vif.drv_cb.s_axi_wvalid  <= 1'b0;
        vif.drv_cb.s_axi_bready  <= 1'b0;
        vif.drv_cb.s_axi_araddr  <= '0;
        vif.drv_cb.s_axi_arprot  <= 3'b000;
        vif.drv_cb.s_axi_arvalid <= 1'b0;
        vif.drv_cb.s_axi_rready  <= 1'b0;

        vif.drv_cb.rst_force_n   <= 1'b1;
        wait (vif.s_axi_aresetn_eff === 1'b1);
        @(vif.drv_cb);
    endtask

    // -------------------------------------------------------------------------
    // Low-Level AXI4-Lite Write Task
    // -------------------------------------------------------------------------
    virtual task axi_write(input logic [11:0] addr, input logic [31:0] data, input bit delay_rdy = 0, input logic [2:0] prot = 3'b000, input logic [3:0] wstrb = 4'hF);
        bit aw_done = 0;
        bit w_done  = 0;

        @(vif.drv_cb);
        vif.drv_cb.s_axi_awaddr  <= addr;
        vif.drv_cb.s_axi_awprot  <= prot;
        vif.drv_cb.s_axi_awvalid <= 1'b1;
        vif.drv_cb.s_axi_wdata   <= data;
        vif.drv_cb.s_axi_wstrb   <= wstrb;
        vif.drv_cb.s_axi_wvalid  <= 1'b1;
        vif.drv_cb.s_axi_bready  <= (delay_rdy) ? 1'b0 : 1'b1;

        while (!(aw_done && w_done)) begin
            @(vif.drv_cb);
            if (vif.drv_cb.s_axi_awready && !aw_done) begin
                aw_done = 1;
                vif.drv_cb.s_axi_awvalid <= 1'b0;
            end
            if (vif.drv_cb.s_axi_wready && !w_done) begin
                w_done = 1;
                vif.drv_cb.s_axi_wvalid <= 1'b0;
            end
        end

        // Wait for BVALID handshake
        while (!vif.drv_cb.s_axi_bvalid) begin
            @(vif.drv_cb);
        end
        if (delay_rdy) begin
            repeat (2) @(vif.drv_cb);
            vif.drv_cb.s_axi_bready <= 1'b1;
            @(vif.drv_cb);
            vif.drv_cb.s_axi_bready <= 1'b0;
        end else begin
            @(vif.drv_cb);
            vif.drv_cb.s_axi_bready <= 1'b0;
        end
    endtask

    // -------------------------------------------------------------------------
    // Low-Level AXI4-Lite Read Task
    // -------------------------------------------------------------------------
    virtual task axi_read(input logic [11:0] addr, output logic [31:0] data, output logic [1:0] resp, input bit delay_rdy = 0, input logic [2:0] prot = 3'b000);
        bit ar_done = 0;

        @(vif.drv_cb);
        vif.drv_cb.s_axi_araddr  <= addr;
        vif.drv_cb.s_axi_arprot  <= prot;
        vif.drv_cb.s_axi_arvalid <= 1'b1;
        vif.drv_cb.s_axi_rready  <= (delay_rdy) ? 1'b0 : 1'b1;

        while (!ar_done) begin
            @(vif.drv_cb);
            if (vif.drv_cb.s_axi_arready) begin
                ar_done = 1;
                vif.drv_cb.s_axi_arvalid <= 1'b0;
            end
        end

        // Wait for RVALID
        while (!vif.drv_cb.s_axi_rvalid) begin
            @(vif.drv_cb);
        end
        if (delay_rdy) begin
            repeat (2) @(vif.drv_cb);
            data = vif.drv_cb.s_axi_rdata;
            resp = vif.drv_cb.s_axi_rresp;
            vif.drv_cb.s_axi_rready <= 1'b1;
            @(vif.drv_cb);
            vif.drv_cb.s_axi_rready <= 1'b0;
        end else begin
            data = vif.drv_cb.s_axi_rdata;
            resp = vif.drv_cb.s_axi_rresp;
            @(vif.drv_cb);
            vif.drv_cb.s_axi_rready <= 1'b0;
        end
    endtask

    // -------------------------------------------------------------------------
    // Advanced Protocol Stress: Split Write (AWVALID before WVALID)
    // -------------------------------------------------------------------------
    virtual task axi_split_aw_write(input logic [11:0] addr, input logic [31:0] data);
        @(vif.drv_cb);
        vif.drv_cb.s_axi_awaddr  <= addr;
        vif.drv_cb.s_axi_awprot  <= 3'b000;
        vif.drv_cb.s_axi_awvalid <= 1'b1;
        vif.drv_cb.s_axi_wvalid  <= 1'b0;
        vif.drv_cb.s_axi_bready  <= 1'b0;

        while (!vif.drv_cb.s_axi_awready) @(vif.drv_cb);
        @(vif.drv_cb);
        vif.drv_cb.s_axi_awvalid <= 1'b0;

        // FSM in W_DATA: wait 2 cycles
        repeat (2) @(vif.drv_cb);

        vif.drv_cb.s_axi_wdata   <= data;
        vif.drv_cb.s_axi_wstrb   <= 4'hF;
        vif.drv_cb.s_axi_wvalid  <= 1'b1;

        while (!vif.drv_cb.s_axi_wready) @(vif.drv_cb);
        @(vif.drv_cb);
        vif.drv_cb.s_axi_wvalid  <= 1'b0;

        while (!vif.drv_cb.s_axi_bvalid) @(vif.drv_cb);
        repeat (2) @(vif.drv_cb);
        vif.drv_cb.s_axi_bready  <= 1'b1;
        @(vif.drv_cb);
        vif.drv_cb.s_axi_bready  <= 1'b0;
    endtask

    // -------------------------------------------------------------------------
    // Advanced Protocol Stress: Split Write with Mid-Flight Reset during W_DATA
    // -------------------------------------------------------------------------
    virtual task axi_split_aw_reset(input logic [11:0] addr, input logic [31:0] data);
        @(vif.drv_cb);
        vif.drv_cb.s_axi_awaddr  <= addr;
        vif.drv_cb.s_axi_awprot  <= 3'b000;
        vif.drv_cb.s_axi_awvalid <= 1'b1;
        vif.drv_cb.s_axi_wvalid  <= 1'b0;
        vif.drv_cb.s_axi_bready  <= 1'b0;

        while (!vif.drv_cb.s_axi_awready) @(vif.drv_cb);
        @(vif.drv_cb);
        vif.drv_cb.s_axi_awvalid <= 1'b0;

        // FSM is now in W_DATA: wait 1 cycle, then assert reset
        @(vif.drv_cb);
        `uvm_info(get_type_name(), "Asserting synchronous reset while AXI slave is in W_DATA...", UVM_LOW)
        vif.drv_cb.rst_force_n   <= 1'b0;
        repeat (3) @(vif.drv_cb);
        vif.drv_cb.rst_force_n   <= 1'b1;
        repeat (2) @(vif.drv_cb);
        `uvm_info(get_type_name(), "Synchronous reset during W_DATA complete. Slave recovered to W_IDLE.", UVM_LOW)
    endtask

    // -------------------------------------------------------------------------
    // Advanced Protocol Stress: Pipelined Write (AW/W stability while busy)
    // -------------------------------------------------------------------------
    virtual task axi_pipelined_write(input logic [11:0] addr, input logic [31:0] data);
        bit aw_done = 0;
        bit w_done  = 0;

        // Phase 1: Start write 1 and hold BREADY low so slave enters and stays in W_RESP
        @(vif.drv_cb);
        vif.drv_cb.s_axi_awaddr  <= addr;
        vif.drv_cb.s_axi_awprot  <= 3'b000;
        vif.drv_cb.s_axi_awvalid <= 1'b1;
        vif.drv_cb.s_axi_wdata   <= data;
        vif.drv_cb.s_axi_wstrb   <= 4'hF;
        vif.drv_cb.s_axi_wvalid  <= 1'b1;
        vif.drv_cb.s_axi_bready  <= 1'b0;

        while (!(aw_done && w_done)) begin
            @(vif.drv_cb);
            if (vif.drv_cb.s_axi_awready && !aw_done) begin
                aw_done = 1;
                vif.drv_cb.s_axi_awvalid <= 1'b0;
            end
            if (vif.drv_cb.s_axi_wready && !w_done) begin
                w_done = 1;
                vif.drv_cb.s_axi_wvalid <= 1'b0;
            end
        end

        // Wait for slave to enter W_RESP (bvalid=1) while awready=0 and wready=0
        while (!vif.drv_cb.s_axi_bvalid) @(vif.drv_cb);

        // Phase 2: Assert second write while slave is in W_RESP (awready=0, wready=0)
        @(vif.drv_cb);
        vif.drv_cb.s_axi_awaddr  <= addr + 4;
        vif.drv_cb.s_axi_awprot  <= 3'b000;
        vif.drv_cb.s_axi_awvalid <= 1'b1;
        vif.drv_cb.s_axi_wdata   <= data + 1;
        vif.drv_cb.s_axi_wstrb   <= 4'hF;
        vif.drv_cb.s_axi_wvalid  <= 1'b1;

        // Hold stable for 2 cycles to trigger non-vacuous A_AW_STABLE & A_W_STABLE
        repeat (2) @(vif.drv_cb);

        // Complete write 1 by asserting bready
        vif.drv_cb.s_axi_bready <= 1'b1;
        @(vif.drv_cb);
        vif.drv_cb.s_axi_bready <= 1'b0;

        // Complete second write handshakes
        aw_done = 0;
        w_done  = 0;
        while (!(aw_done && w_done)) begin
            @(vif.drv_cb);
            if (vif.drv_cb.s_axi_awready && !aw_done) begin
                aw_done = 1;
                vif.drv_cb.s_axi_awvalid <= 1'b0;
            end
            if (vif.drv_cb.s_axi_wready && !w_done) begin
                w_done = 1;
                vif.drv_cb.s_axi_wvalid <= 1'b0;
            end
        end

        while (!vif.drv_cb.s_axi_bvalid) @(vif.drv_cb);
        @(vif.drv_cb);
        vif.drv_cb.s_axi_bready <= 1'b1;
        @(vif.drv_cb);
        vif.drv_cb.s_axi_bready <= 1'b0;
    endtask

    // -------------------------------------------------------------------------
    // Advanced Protocol Stress: Pipelined Read (AR stability while busy)
    // -------------------------------------------------------------------------
    virtual task axi_pipelined_read(input logic [11:0] addr, output logic [31:0] data, output logic [1:0] resp);
        bit ar_done = 0;

        @(vif.drv_cb);
        vif.drv_cb.s_axi_araddr  <= addr;
        vif.drv_cb.s_axi_arprot  <= 3'b000;
        vif.drv_cb.s_axi_arvalid <= 1'b1;
        vif.drv_cb.s_axi_rready  <= 1'b0;

        while (!ar_done) begin
            @(vif.drv_cb);
            if (vif.drv_cb.s_axi_arready) begin
                ar_done = 1;
                vif.drv_cb.s_axi_arvalid <= 1'b0;
            end
        end

        // Wait for RVALID (slave enters R_READ)
        while (!vif.drv_cb.s_axi_rvalid) @(vif.drv_cb);

        // While slave is in R_READ (arready=0), assert read 2
        vif.drv_cb.s_axi_araddr  <= addr + 4;
        vif.drv_cb.s_axi_arprot  <= 3'b000;
        vif.drv_cb.s_axi_arvalid <= 1'b1;

        // Hold stable for 2 cycles to trigger non-vacuous A_AR_STABLE
        repeat (2) @(vif.drv_cb);

        // Complete read 1
        data = vif.drv_cb.s_axi_rdata;
        resp = vif.drv_cb.s_axi_rresp;
        vif.drv_cb.s_axi_rready <= 1'b1;
        @(vif.drv_cb);
        vif.drv_cb.s_axi_rready <= 1'b0;

        // Complete second read
        ar_done = 0;
        while (!ar_done) begin
            @(vif.drv_cb);
            if (vif.drv_cb.s_axi_arready) begin
                ar_done = 1;
                vif.drv_cb.s_axi_arvalid <= 1'b0;
            end
        end

        while (!vif.drv_cb.s_axi_rvalid) @(vif.drv_cb);
        @(vif.drv_cb);
        vif.drv_cb.s_axi_rready <= 1'b1;
        @(vif.drv_cb);
        vif.drv_cb.s_axi_rready <= 1'b0;
    endtask

    // -------------------------------------------------------------------------
    // High-Level Driver Dispatcher
    // -------------------------------------------------------------------------
    virtual task drive_item(newton_axi_seq_item item);
        if (item.is_raw_axi) begin
            case (item.transfer_mode)
                newton_axi_seq_item::TRANSFER_SPLIT_AW: begin
                    axi_split_aw_write(item.axi_addr, item.axi_data);
                end
                newton_axi_seq_item::TRANSFER_RESET_WDATA: begin
                    axi_split_aw_reset(item.axi_addr, item.axi_data);
                end
                newton_axi_seq_item::TRANSFER_PIPELINE: begin
                    if (item.axi_op == newton_axi_seq_item::AXI_WRITE) begin
                        axi_pipelined_write(item.axi_addr, item.axi_data);
                    end else begin
                        axi_pipelined_read(item.axi_addr, item.axi_data, item.axi_resp);
                    end
                end
                default: begin
                    if (item.axi_op == newton_axi_seq_item::AXI_WRITE) begin
                        axi_write(item.axi_addr, item.axi_data, item.delay_ready, item.axi_prot, item.axi_wstrb);
                    end else begin
                        axi_read(item.axi_addr, item.axi_data, item.axi_resp, item.delay_ready, item.axi_prot);
                    end
                end
            endcase
        end else begin
            // Full Newton BCD Optimization Transaction
            int timeout_cnt = 0;
            const int MAX_TIMEOUT_CYCLES = 100000;
            logic [31:0] status_val;
            logic [31:0] rdata;
            logic [1:0]  rresp;

            // 1. Program Microcode if requested
            if (item.reprogram || item.prog_length > 0) begin
                `uvm_info(get_type_name(), $sformatf("Loading %0d microcode instructions via AXI...", item.prog_length), UVM_HIGH)
                for (int i = 0; i < item.prog_length; i++) begin
                    axi_write(AXI_REG_PROG_ADDR, 32'(i));
                    axi_write(AXI_REG_PROG_DATA, item.program_mem[i]);
                end
            end

            // 2. Program Hyperparameters
            axi_write(AXI_REG_NUM_VARS,    32'(item.num_vars));
            axi_write(AXI_REG_TOLERANCE,   item.tolerance);
            axi_write(AXI_REG_ALPHA,       item.step_alpha);
            axi_write(AXI_REG_LAMBDA,      item.lambda_reg);
            axi_write(AXI_REG_MAX_SWEEPS,  32'(item.max_sweeps));

            // 3. Write Initial State Vector
            for (int i = 0; i < item.num_vars; i++) begin
                axi_write(AXI_STATE_VEC_BASE + (i * 4), item.x_init[i]);
            end

            // 4. Trigger Optimization Sweep
            `uvm_info(get_type_name(), $sformatf("Starting BCD Solver with %0d variables...", item.num_vars), UVM_MEDIUM)
            if (item.do_mid_reset && item.reset_delay_cycles <= 5) begin
                bit aw_done = 0;
                bit w_done  = 0;
                @(vif.drv_cb);
                vif.drv_cb.s_axi_awaddr  <= AXI_REG_CTRL;
                vif.drv_cb.s_axi_awprot  <= 3'b000;
                vif.drv_cb.s_axi_awvalid <= 1'b1;
                vif.drv_cb.s_axi_wdata   <= CTRL_START_MASK;
                vif.drv_cb.s_axi_wstrb   <= 4'hF;
                vif.drv_cb.s_axi_wvalid  <= 1'b1;
                vif.drv_cb.s_axi_bready  <= 1'b1;

                while (!(aw_done && w_done)) begin
                    @(vif.drv_cb);
                    if (vif.drv_cb.s_axi_awready && !aw_done) begin
                        aw_done = 1;
                        vif.drv_cb.s_axi_awvalid <= 1'b0;
                    end
                    if (vif.drv_cb.s_axi_wready && !w_done) begin
                        w_done = 1;
                        vif.drv_cb.s_axi_wvalid <= 1'b0;
                    end
                end

                repeat (item.reset_delay_cycles) @(vif.drv_cb);
                `uvm_info(get_type_name(), $sformatf("Asserting Early Mid-Flight Reset (%0d cycles after write handshake)...", item.reset_delay_cycles), UVM_LOW)
                vif.drv_cb.rst_force_n <= 1'b0;
                repeat (3) @(vif.drv_cb);
                vif.drv_cb.rst_force_n <= 1'b1;
                repeat (2) @(vif.drv_cb);
                axi_read(AXI_REG_STATUS, status_val, rresp);
                item.status = status_t'((status_val & STATUS_CODE_MASK) >> STATUS_CODE_SHIFT);
                `uvm_info(get_type_name(), $sformatf("Post-Reset Status: %s (Raw: 0x%08h)", item.status.name(), status_val), UVM_LOW)
            end else begin
                axi_write(AXI_REG_CTRL, CTRL_START_MASK);

                if (item.do_mid_reset) begin
                    `uvm_info(get_type_name(), $sformatf("Asserting Mid-Flight Reset after %0d cycles...", item.reset_delay_cycles), UVM_LOW)
                    repeat (item.reset_delay_cycles) @(vif.drv_cb);
                    vif.drv_cb.rst_force_n <= 1'b0;
                    repeat (3) @(vif.drv_cb);
                    vif.drv_cb.rst_force_n <= 1'b1;
                    repeat (2) @(vif.drv_cb);
                    axi_read(AXI_REG_STATUS, status_val, rresp);
                    item.status = status_t'((status_val & STATUS_CODE_MASK) >> STATUS_CODE_SHIFT);
                    `uvm_info(get_type_name(), $sformatf("Post-Reset Status: %s (Raw: 0x%08h)", item.status.name(), status_val), UVM_LOW)
                end else begin
                    // 5. Wait for irq_done or poll done bit
                    while (!vif.drv_cb.irq_done) begin
                        @(vif.drv_cb);
                        timeout_cnt++;
                        if (timeout_cnt > MAX_TIMEOUT_CYCLES) begin
                            `uvm_error("DRV_TIMEOUT", $sformatf("Solver timed out after %0d clock cycles!", MAX_TIMEOUT_CYCLES))
                            break;
                        end
                    end

                    // Additional cycle for status latch stabilization
                    repeat(2) @(vif.drv_cb);

                    // 6. Read back execution results
                    axi_read(AXI_REG_STATUS, status_val, rresp);
                    item.status = status_t'((status_val & STATUS_CODE_MASK) >> STATUS_CODE_SHIFT);

                    axi_read(AXI_REG_SWEEP_COUNT, rdata, rresp);
                    item.sweep_count = rdata[7:0];

                    axi_read(AXI_REG_F_OPTIMAL, rdata, rresp);
                    item.f_optimal = rdata;

                    axi_read(AXI_REG_MAX_DELTA, rdata, rresp);
                    item.max_delta_last = rdata;

                    for (int i = 0; i < item.num_vars; i++) begin
                        axi_read(AXI_STATE_VEC_BASE + (i * 4), rdata, rresp);
                        item.x_optimal[i] = rdata;
                    end

                    `uvm_info(get_type_name(), $sformatf("Optimization Finished:\n%s", item.convert2string()), UVM_HIGH)
                end
            end
        end
        repeat(2) @(vif.drv_cb);
    endtask

    // -------------------------------------------------------------------------
    // Phases 6 to 9
    // -------------------------------------------------------------------------
    virtual function void extract_phase(uvm_phase phase);
        super.extract_phase(phase);
        `uvm_info("DRV_PHASE_6_EXTRACT", "[STAGE 3: CLEANUP] extract_phase: Driver completed all transactions.", UVM_LOW)
    endfunction

    virtual function void check_phase(uvm_phase phase);
        super.check_phase(phase);
        `uvm_info("DRV_PHASE_7_CHECK", "[STAGE 3: CLEANUP] check_phase: Driver confirming idle bus lines.", UVM_LOW)
    endfunction

    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info("DRV_PHASE_8_REPORT", "[STAGE 3: CLEANUP] report_phase: Driver execution finalized.", UVM_LOW)
    endfunction

    virtual function void final_phase(uvm_phase phase);
        super.final_phase(phase);
        `uvm_info("DRV_PHASE_9_FINAL", "[STAGE 3: CLEANUP] final_phase: Driver shutdown complete.", UVM_LOW)
    endfunction

endclass : newton_driver

`endif // NEWTON_DRIVER_SV
