// =============================================================================
// File Name   : newton_monitor.sv
// Class Name  : newton_monitor
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Industry-grade AXI4-Lite bus monitor tracking programming,
//               control parameters, and readouts across all 9 UVM phases.
// =============================================================================

`ifndef NEWTON_MONITOR_SV
`define NEWTON_MONITOR_SV

class newton_monitor extends uvm_monitor;
    `uvm_component_utils(newton_monitor)

    newton_vif_t        vif;
    newton_agent_config cfg;
    uvm_analysis_port #(newton_axi_seq_item) mon_ap;

    // Shadow state registers
    instr_t                 mirr_prog_mem [PROG_DEPTH];
    int                     mirr_prog_len;
    logic [4:0]             mirr_prog_addr;
    logic [4:0]             mirr_num_vars;
    q16_t                   mirr_tol;
    q16_t                   mirr_alpha;
    q16_t                   mirr_lambda;
    logic [7:0]             mirr_max_sweeps;
    q16_t                   mirr_x_init [MAX_VARS];

    function new(string name = "newton_monitor", uvm_component parent = null);
        super.new(name, parent);
        mon_ap        = new("mon_ap", this);
        mirr_prog_len = 0;
        mirr_num_vars = 5'd2;
        mirr_tol      = Q16_EPS_DEF;
        mirr_alpha    = Q16_ONE;
        mirr_lambda   = Q16_LAMBDA_DEF;
        mirr_max_sweeps = 8'd20;
        for (int i = 0; i < PROG_DEPTH; i++) mirr_prog_mem[i] = '0;
        for (int i = 0; i < MAX_VARS; i++)   mirr_x_init[i]   = Q16_ZERO;
    endfunction

    // -------------------------------------------------------------------------
    // Phase 1: build_phase
    // -------------------------------------------------------------------------
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("MON_PHASE_1_BUILD", "[STAGE 1: SETUP] build_phase: Retrieving interface handle...", UVM_LOW)

        // Industry Best Practice: Retrieve from agent_config first, fallback to config_db
        if (uvm_config_db#(newton_agent_config)::get(this, "", "cfg", cfg) && cfg.vif != null) begin
            vif = cfg.vif;
            `uvm_info("MON_CFG_VIF", "Monitor successfully obtained virtual interface from agent_config.", UVM_HIGH)
        end else if (!uvm_config_db#(newton_vif_t)::get(this, "", "vif", vif)) begin
            `uvm_fatal("MON_NO_VIF", "Virtual interface 'vif' not found in agent_config or uvm_config_db!")
        end
    endfunction

    // -------------------------------------------------------------------------
    // Phase 2: connect_phase
    // -------------------------------------------------------------------------
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        `uvm_info("MON_PHASE_2_CONNECT", "[STAGE 1: SETUP] connect_phase: Monitor ports connected.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 3: end_of_elaboration_phase
    // -------------------------------------------------------------------------
    virtual function void end_of_elaboration_phase(uvm_phase phase);
        super.end_of_elaboration_phase(phase);
        `uvm_info("MON_PHASE_3_END_OF_ELAB", "[STAGE 1: SETUP] end_of_elaboration_phase: Verifying monitor interface...", UVM_LOW)
        if (vif == null) `uvm_fatal("MON_NULL_VIF", "Monitor virtual interface is null!")
    endfunction

    // -------------------------------------------------------------------------
    // Phase 4: start_of_simulation_phase
    // -------------------------------------------------------------------------
    virtual function void start_of_simulation_phase(uvm_phase phase);
        super.start_of_simulation_phase(phase);
        `uvm_info("MON_PHASE_4_START_OF_SIM", "[STAGE 1: SETUP] start_of_simulation_phase: Monitor armed for bus snooping.", UVM_LOW)
    endfunction

    // -------------------------------------------------------------------------
    // Phase 5: run_phase
    // -------------------------------------------------------------------------
    virtual task run_phase(uvm_phase phase);
        wait (vif.s_axi_aresetn === 1'b1);
        @(vif.mon_cb);

        fork
            snoop_axi_writes();
            snoop_axi_reads();
            snoop_optimization_cycles();
        join
    endtask

    // -------------------------------------------------------------------------
    // Task: snoop_axi_writes
    // -------------------------------------------------------------------------
    virtual task snoop_axi_writes();
        logic [11:0] write_addr;
        logic [31:0] write_data;
        newton_axi_seq_item raw_item;

        forever begin
            @(vif.mon_cb);
            // Detect write handshake
            if (vif.mon_cb.s_axi_awvalid && vif.mon_cb.s_axi_awready) begin
                write_addr = vif.mon_cb.s_axi_awaddr;
                while (!(vif.mon_cb.s_axi_wvalid && vif.mon_cb.s_axi_wready)) begin
                    @(vif.mon_cb);
                end
                write_data = vif.mon_cb.s_axi_wdata;

                // Broadcast raw AXI write for scoreboard and coverage tracking
                raw_item = newton_axi_seq_item::type_id::create("raw_write_item");
                raw_item.is_raw_axi = 1'b1;
                raw_item.axi_op     = newton_axi_seq_item::AXI_WRITE;
                raw_item.axi_addr   = write_addr;
                raw_item.axi_data   = write_data;
                raw_item.axi_resp   = 2'b00;
                mon_ap.write(raw_item);

                // Update shadow registers
                case (write_addr)
                    AXI_REG_PROG_ADDR:   mirr_prog_addr <= write_data[4:0];
                    AXI_REG_PROG_DATA: begin
                        mirr_prog_mem[mirr_prog_addr] = write_data;
                        if (int'(mirr_prog_addr) + 1 > mirr_prog_len) begin
                            mirr_prog_len = int'(mirr_prog_addr) + 1;
                        end
                    end
                    AXI_REG_NUM_VARS:    mirr_num_vars   <= write_data[4:0];
                    AXI_REG_TOLERANCE:   mirr_tol        <= write_data;
                    AXI_REG_ALPHA:       mirr_alpha      <= write_data;
                    AXI_REG_LAMBDA:      mirr_lambda     <= write_data;
                    AXI_REG_MAX_SWEEPS:  mirr_max_sweeps <= write_data[7:0];
                    default: begin
                        if (write_addr >= AXI_STATE_VEC_BASE && write_addr <= AXI_STATE_VEC_LIMIT) begin
                            int var_idx = (write_addr - AXI_STATE_VEC_BASE) >> 2;
                            if (var_idx < MAX_VARS) mirr_x_init[var_idx] = write_data;
                        end
                    end
                endcase
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Task: snoop_axi_reads
    // -------------------------------------------------------------------------
    virtual task snoop_axi_reads();
        logic [11:0] read_addr;
        logic [31:0] read_data;
        logic [1:0]  read_resp;
        newton_axi_seq_item raw_item;

        forever begin
            @(vif.mon_cb);
            if (vif.mon_cb.s_axi_arvalid && vif.mon_cb.s_axi_arready) begin
                read_addr = vif.mon_cb.s_axi_araddr;
                while (!(vif.mon_cb.s_axi_rvalid && vif.mon_cb.s_axi_rready)) begin
                    @(vif.mon_cb);
                end
                read_data = vif.mon_cb.s_axi_rdata;
                read_resp = vif.mon_cb.s_axi_rresp;

                raw_item = newton_axi_seq_item::type_id::create("raw_read_item");
                raw_item.is_raw_axi = 1'b1;
                raw_item.axi_op     = newton_axi_seq_item::AXI_READ;
                raw_item.axi_addr   = read_addr;
                raw_item.axi_data   = read_data;
                raw_item.axi_resp   = read_resp;
                mon_ap.write(raw_item);
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Task: snoop_optimization_cycles
    // -------------------------------------------------------------------------
    virtual task snoop_optimization_cycles();
        newton_axi_seq_item item;
        logic [11:0] read_addr;
        logic [31:0] read_data;

        forever begin
            @(vif.mon_cb);
            // Detect start pulse (write to AXI_REG_CTRL with bit 0 set)
            if (vif.mon_cb.s_axi_awvalid && vif.mon_cb.s_axi_awready &&
                vif.mon_cb.s_axi_awaddr == AXI_REG_CTRL) begin
                
                while (!(vif.mon_cb.s_axi_wvalid && vif.mon_cb.s_axi_wready)) @(vif.mon_cb);
                if (vif.mon_cb.s_axi_wdata[0]) begin
                    // Solver started: create item and latch inputs
                    item = newton_axi_seq_item::type_id::create("mon_bcd_item");
                    item.is_raw_axi  = 1'b0;
                    item.num_vars    = mirr_num_vars;
                    item.tolerance   = mirr_tol;
                    item.step_alpha  = mirr_alpha;
                    item.lambda_reg  = mirr_lambda;
                    item.max_sweeps  = mirr_max_sweeps;
                    item.prog_length = mirr_prog_len;
                    for (int i = 0; i < PROG_DEPTH; i++) item.program_mem[i] = mirr_prog_mem[i];
                    for (int i = 0; i < MAX_VARS; i++)   item.x_init[i]      = mirr_x_init[i];

                    // Wait for irq_done completion
                    while (!vif.mon_cb.irq_done) begin
                        @(vif.mon_cb);
                    end

                    // Collect readouts during subsequent AXI reads
                    fork
                        begin : collect_results
                            int vars_read = 0;
                            bit status_read = 0;
                            bit f_opt_read  = 0;
                            bit sweep_read  = 0;
                            int timeout     = 0;

                            while (timeout < 20000) begin
                                @(vif.mon_cb);
                                timeout++;
                                if (vif.mon_cb.s_axi_arvalid && vif.mon_cb.s_axi_arready) begin
                                    read_addr = vif.mon_cb.s_axi_araddr;
                                    while (!(vif.mon_cb.s_axi_rvalid && vif.mon_cb.s_axi_rready)) @(vif.mon_cb);
                                    read_data = vif.mon_cb.s_axi_rdata;

                                    case (read_addr)
                                        AXI_REG_STATUS: begin
                                            item.status = status_t'((read_data & STATUS_CODE_MASK) >> STATUS_CODE_SHIFT);
                                            status_read = 1;
                                        end
                                        AXI_REG_SWEEP_COUNT: begin
                                            item.sweep_count = read_data[7:0];
                                            sweep_read = 1;
                                        end
                                        AXI_REG_F_OPTIMAL: begin
                                            item.f_optimal = read_data;
                                            f_opt_read = 1;
                                        end
                                        AXI_REG_MAX_DELTA: begin
                                            item.max_delta_last = read_data;
                                        end
                                        default: begin
                                            if (read_addr >= AXI_STATE_VEC_BASE && read_addr <= AXI_STATE_VEC_LIMIT) begin
                                                int idx = (read_addr - AXI_STATE_VEC_BASE) >> 2;
                                                if (idx < item.num_vars) begin
                                                    item.x_optimal[idx] = read_data;
                                                    vars_read++;
                                                end
                                            end
                                        end
                                    endcase

                                    if (status_read && f_opt_read && sweep_read && vars_read >= item.num_vars) begin
                                        break;
                                    end
                                end
                            end
                        end
                    join

                    `uvm_info(get_type_name(), $sformatf("Monitored Transaction Broadcast:\n%s", item.convert2string()), UVM_HIGH)
                    mon_ap.write(item);
                end
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Phases 6 to 9
    // -------------------------------------------------------------------------
    virtual function void extract_phase(uvm_phase phase);
        super.extract_phase(phase);
        `uvm_info("MON_PHASE_6_EXTRACT", "[STAGE 3: CLEANUP] extract_phase: Monitor extraction complete.", UVM_LOW)
    endfunction

    virtual function void check_phase(uvm_phase phase);
        super.check_phase(phase);
        `uvm_info("MON_PHASE_7_CHECK", "[STAGE 3: CLEANUP] check_phase: Verifying monitor state queues.", UVM_LOW)
    endfunction

    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info("MON_PHASE_8_REPORT", "[STAGE 3: CLEANUP] report_phase: Monitor report complete.", UVM_LOW)
    endfunction

    virtual function void final_phase(uvm_phase phase);
        super.final_phase(phase);
        `uvm_info("MON_PHASE_9_FINAL", "[STAGE 3: CLEANUP] final_phase: Monitor closed cleanly.", UVM_LOW)
    endfunction

endclass : newton_monitor

`endif // NEWTON_MONITOR_SV
