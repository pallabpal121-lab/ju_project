// =============================================================================
// File Name   : newton_multivar_reset_seq.sv
// Class Name  : newton_multivar_reset_seq
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Industry-grade synchronous reset verification sequence.
//               Exercises synchronous reset recovery across all active BCD FSM
//               states (BCD_SETUP_SWEEP, BCD_START_PAIR, BCD_WAIT_PAIR,
//               BCD_ADVANCE_PAIR, BCD_CHECK_SWEEP) and AXI slave W_DATA state,
//               followed by a complete post-reset recovery optimization to verify
//               flawless silicon restart.
// =============================================================================

`ifndef NEWTON_MULTIVAR_RESET_SEQ_SV
`define NEWTON_MULTIVAR_RESET_SEQ_SV

class newton_multivar_reset_seq extends newton_base_seq;
    `uvm_object_utils(newton_multivar_reset_seq)

    function new(string name = "newton_multivar_reset_seq");
        super.new(name);
    endfunction

    // -------------------------------------------------------------------------
    // Helper: Build Quadratic Microcode for N variables
    // -------------------------------------------------------------------------
    function void build_quad_microcode(output instr_t prog[PROG_DEPTH], output int len, input int n);
        int idx = 0;
        prog[idx++] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        if (n == 2) begin
            prog[idx++] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1, src_b: 5'd1, imm: 13'sd0};
            prog[idx++] = '{op: OP_ADD, dst: 5'd31, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
        end else begin
            prog[idx++] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1, src_b: 5'd1, imm: 13'sd0};
            prog[idx++] = '{op: OP_ADD, dst: 5'd18, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
            for (int i = 2; i < n; i++) begin
                prog[idx++] = '{op: OP_MUL, dst: 5'd16, src_a: 5'(i), src_b: 5'(i), imm: 13'sd0};
                prog[idx++] = '{op: OP_ADD, dst: (i == n-1) ? 5'd31 : 5'd18, src_a: 5'd18, src_b: 5'd16, imm: 13'sd0};
            end
        end
        prog[idx++] = '{op: OP_END, dst: 5'd0, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        len = idx;
    endfunction

    virtual task body();
        newton_axi_seq_item item;
        instr_t quad_prog[PROG_DEPTH];
        int prog_len;
        logic [31:0] status_val;
        logic [1:0]  rresp;

        `uvm_info(get_type_name(), "Starting Synchronous Reset & FSM State Recovery Tests...", UVM_LOW)

        build_quad_microcode(quad_prog, prog_len, 4);

        // ---------------------------------------------------------------------
        // Test 1A: Mid-Flight Reset during BCD_SETUP_SWEEP (Delay = 1 cycle)
        // Targets: BCD_SETUP_SWEEP -> BCD_IDLE
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 1A: Mid-Flight Reset after 1 cycle (BCD_SETUP_SWEEP)...", UVM_MEDIUM)
        item = newton_axi_seq_item::type_id::create("reset_test_setup_sweep");
        start_item(item);
        item.is_raw_axi         = 1'b0;
        item.reprogram          = 1'b1;
        item.prog_length        = prog_len;
        item.program_mem        = quad_prog;
        item.num_vars           = 5'd4;
        item.tolerance          = 32'h0000_0100;
        item.step_alpha         = 32'h0001_0000;
        item.lambda_reg         = 32'h0000_0800;
        item.max_sweeps         = 8'd20;
        for (int i = 0; i < 4; i++) item.x_init[i] = 32'h0002_0000;
        item.do_mid_reset       = 1'b1;
        item.reset_delay_cycles = 1;
        finish_item(item);

        read_reg(AXI_REG_STATUS, status_val, rresp);
        if ((status_val & STATUS_CODE_MASK) != 0) begin
            `uvm_error(get_type_name(), $sformatf("Test 1A Failed: Status not IDLE after reset! Got 0x%08h", status_val))
        end else begin
            `uvm_info(get_type_name(), "Test 1A Passed: BCD FSM cleanly recovered from BCD_SETUP_SWEEP to IDLE.", UVM_MEDIUM)
        end

        // ---------------------------------------------------------------------
        // Test 1B: Mid-Flight Reset during BCD_START_PAIR (Delay = 2 cycles)
        // Targets: BCD_START_PAIR -> BCD_IDLE
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 1B: Mid-Flight Reset after 2 cycles (BCD_START_PAIR)...", UVM_MEDIUM)
        item = newton_axi_seq_item::type_id::create("reset_test_start_pair");
        start_item(item);
        item.is_raw_axi         = 1'b0;
        item.reprogram          = 1'b1;
        item.prog_length        = prog_len;
        item.program_mem        = quad_prog;
        item.num_vars           = 5'd4;
        item.tolerance          = 32'h0000_0100;
        item.step_alpha         = 32'h0001_0000;
        item.lambda_reg         = 32'h0000_0800;
        item.max_sweeps         = 8'd20;
        for (int i = 0; i < 4; i++) item.x_init[i] = 32'h0002_0000;
        item.do_mid_reset       = 1'b1;
        item.reset_delay_cycles = 2;
        finish_item(item);

        read_reg(AXI_REG_STATUS, status_val, rresp);
        if ((status_val & STATUS_CODE_MASK) != 0) begin
            `uvm_error(get_type_name(), $sformatf("Test 1B Failed: Status not IDLE after reset! Got 0x%08h", status_val))
        end else begin
            `uvm_info(get_type_name(), "Test 1B Passed: BCD FSM cleanly recovered from BCD_START_PAIR to IDLE.", UVM_MEDIUM)
        end

        // ---------------------------------------------------------------------
        // Test 1C: Fine delay sweep (135 to 170 cycles, step 1) to target BCD_ADVANCE_PAIR
        // and BCD_CHECK_SWEEP reset transitions to IDLE in newton_bcd_top
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 1C: Delay sweep 135..230 for ADVANCE & CHECK transitions...", UVM_MEDIUM)
        for (int d = 135; d <= 230; d++) begin
            item = newton_axi_seq_item::type_id::create($sformatf("reset_test_sweep_%0d", d));
            start_item(item);
            item.is_raw_axi         = 1'b0;
            item.reprogram          = 1'b0;
            item.num_vars           = 5'd2;
            item.tolerance          = 32'h0000_0100;
            item.step_alpha         = 32'h0001_0000;
            item.lambda_reg         = 32'h0000_0800;
            item.max_sweeps         = 8'd20;
            item.x_init[0]          = 32'h0002_0000;
            item.x_init[1]          = 32'h0002_0000;
            item.do_mid_reset       = 1'b1;
            item.reset_delay_cycles = d;
            finish_item(item);
        end

        // ---------------------------------------------------------------------
        // Test 1D: Core FSM Reset Sweep (1 to 140 cycles, step 1) to target all 25
        // states in newton_2var_core (C2_START_F0 to C2_DIV_J) and BCD pair advance/check
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 1D: 1-cycle delay sweep 1..140 for newton_2var_core FSM states...", UVM_MEDIUM)
        for (int d = 1; d <= 140; d++) begin
            item = newton_axi_seq_item::type_id::create($sformatf("reset_test_core_%0d", d));
            start_item(item);
            item.is_raw_axi         = 1'b0;
            item.reprogram          = 1'b0;
            item.num_vars           = 5'd2;
            item.tolerance          = 32'h0000_0100;
            item.step_alpha         = 32'h0001_0000;
            item.lambda_reg         = 32'h0000_0800;
            item.max_sweeps         = 8'd20;
            item.x_init[0]          = 32'h0002_0000;
            item.x_init[1]          = 32'h0002_0000;
            item.do_mid_reset       = 1'b1;
            item.reset_delay_cycles = d;
            finish_item(item);
        end

        // ---------------------------------------------------------------------
        // Test 1E: Mid-Flight Reset during DFG Divider Execution
        // Targets: ENG_DIV_WAIT -> ENG_IDLE in dfg_multivar_engine
        //          DIV_CALC -> DIV_IDLE in u_dfg.u_div
        // ---------------------------------------------------------------------
        begin
            `uvm_info(get_type_name(), "Running Test 1E: Reset during DFG Divider Execution...", UVM_MEDIUM)
            item = newton_axi_seq_item::type_id::create("reset_test_dfg_div");
            start_item(item);
            item.is_raw_axi         = 1'b0;
            item.reprogram          = 1'b1;
            item.prog_length        = 3;
            item.program_mem[0]     = '{op: OP_LOADC, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd2};
            item.program_mem[1]     = '{op: OP_DIV,   dst: 5'd31, src_a: 5'd0, src_b: 5'd16, imm: 13'sd0};
            item.program_mem[2]     = '{op: OP_END,   dst: 5'd0,  src_a: 5'd0, src_b: 5'd0,  imm: 13'sd0};
            item.num_vars           = 5'd2;
            item.tolerance          = 32'h0000_0100;
            item.step_alpha         = 32'h0001_0000;
            item.lambda_reg         = 32'h0000_0800;
            item.max_sweeps         = 8'd5;
            item.x_init[0]          = 32'h0002_0000;
            item.x_init[1]          = 32'h0002_0000;
            item.do_mid_reset       = 1'b1;
            item.reset_delay_cycles = 10;
            finish_item(item);

            read_reg(AXI_REG_STATUS, status_val, rresp);
            if ((status_val & STATUS_CODE_MASK) != 0) begin
                `uvm_error(get_type_name(), $sformatf("Test 1E Failed: Status not IDLE after reset! Got 0x%08h", status_val))
            end else begin
                `uvm_info(get_type_name(), "Test 1E Passed: DFG & Divider cleanly recovered from DIV_WAIT / DIV_CALC to IDLE.", UVM_MEDIUM)
            end
        end

        // ---------------------------------------------------------------------
        // Test 1: Mid-Flight Reset during BCD Sweep Execution (Delay = 30 cycles)
        // Targets early BCD states: BCD_SETUP_SWEEP / BCD_START_PAIR
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 1: Mid-Flight Reset after 30 cycles...", UVM_MEDIUM)
        item = newton_axi_seq_item::type_id::create("reset_test_early");
        start_item(item);
        item.is_raw_axi         = 1'b0;
        item.reprogram          = 1'b1;
        item.prog_length        = prog_len;
        item.program_mem        = quad_prog;
        item.num_vars           = 5'd4;
        item.tolerance          = 32'h0000_0100;
        item.step_alpha         = 32'h0001_0000;
        item.lambda_reg         = 32'h0000_0800;
        item.max_sweeps         = 8'd20;
        for (int i = 0; i < 4; i++) item.x_init[i] = 32'h0002_0000;
        item.do_mid_reset        = 1'b1;
        item.reset_delay_cycles = 30;
        finish_item(item);

        // Verify post-reset status is IDLE (0x0)
        read_reg(AXI_REG_STATUS, status_val, rresp);
        if ((status_val & STATUS_CODE_MASK) != 0) begin
            `uvm_error(get_type_name(), $sformatf("Test 1 Failed: Status not IDLE after reset! Got 0x%08h", status_val))
        end else begin
            `uvm_info(get_type_name(), "Test 1 Passed: BCD FSM cleanly recovered to IDLE.", UVM_MEDIUM)
        end

        // ---------------------------------------------------------------------
        // Test 2: Mid-Flight Reset during BCD Pair Optimization (Delay = 120 cycles)
        // Targets BCD_WAIT_PAIR / BCD_ADVANCE_PAIR
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 2: Mid-Flight Reset after 120 cycles...", UVM_MEDIUM)
        item = newton_axi_seq_item::type_id::create("reset_test_mid");
        start_item(item);
        item.is_raw_axi         = 1'b0;
        item.reprogram          = 1'b1;
        item.prog_length        = prog_len;
        item.program_mem        = quad_prog;
        item.num_vars           = 5'd4;
        item.tolerance          = 32'h0000_0100;
        item.step_alpha         = 32'h0001_0000;
        item.lambda_reg         = 32'h0000_0800;
        item.max_sweeps         = 8'd20;
        for (int i = 0; i < 4; i++) item.x_init[i] = 32'h0002_0000;
        item.do_mid_reset        = 1'b1;
        item.reset_delay_cycles = 120;
        finish_item(item);

        read_reg(AXI_REG_STATUS, status_val, rresp);
        if ((status_val & STATUS_CODE_MASK) != 0) begin
            `uvm_error(get_type_name(), $sformatf("Test 2 Failed: Status not IDLE after reset! Got 0x%08h", status_val))
        end else begin
            `uvm_info(get_type_name(), "Test 2 Passed: BCD FSM cleanly recovered to IDLE.", UVM_MEDIUM)
        end

        // ---------------------------------------------------------------------
        // Test 3: Mid-Flight Reset during Sweep Transition (Delay = 350 cycles)
        // Targets BCD_CHECK_SWEEP / BCD_ADVANCE_PAIR in sweep 2
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 3: Mid-Flight Reset after 350 cycles...", UVM_MEDIUM)
        item = newton_axi_seq_item::type_id::create("reset_test_late");
        start_item(item);
        item.is_raw_axi         = 1'b0;
        item.reprogram          = 1'b1;
        item.prog_length        = prog_len;
        item.program_mem        = quad_prog;
        item.num_vars           = 5'd4;
        item.tolerance          = 32'h0000_0100;
        item.step_alpha         = 32'h0001_0000;
        item.lambda_reg         = 32'h0000_0800;
        item.max_sweeps         = 8'd20;
        for (int i = 0; i < 4; i++) item.x_init[i] = 32'h0002_0000;
        item.do_mid_reset        = 1'b1;
        item.reset_delay_cycles = 350;
        finish_item(item);

        read_reg(AXI_REG_STATUS, status_val, rresp);
        if ((status_val & STATUS_CODE_MASK) != 0) begin
            `uvm_error(get_type_name(), $sformatf("Test 3 Failed: Status not IDLE after reset! Got 0x%08h", status_val))
        end else begin
            `uvm_info(get_type_name(), "Test 3 Passed: BCD FSM cleanly recovered to IDLE.", UVM_MEDIUM)
        end

        // ---------------------------------------------------------------------
        // Test 4: Mid-Flight Reset during AXI W_DATA State
        // Targets AXI slave write channel transition W_DATA -> W_IDLE
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 4: AXI W_DATA Reset Transition...", UVM_MEDIUM)
        write_reg_split_reset(AXI_REG_CTRL, CTRL_START_MASK);

        read_reg(AXI_REG_STATUS, status_val, rresp);
        if ((status_val & STATUS_CODE_MASK) != 0) begin
            `uvm_error(get_type_name(), $sformatf("Test 4 Failed: AXI slave status not IDLE! Got 0x%08h", status_val))
        end else begin
            `uvm_info(get_type_name(), "Test 4 Passed: AXI slave cleanly recovered to W_IDLE.", UVM_MEDIUM)
        end

        // ---------------------------------------------------------------------
        // Test 5: Full Post-Reset Recovery Optimization
        // Executes a complete 4-variable quadratic optimization to verify full
        // operational recovery, correct convergence, and zero scoreboard mismatches.
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 5: Full Post-Reset Optimization to Convergence...", UVM_LOW)
        item = newton_axi_seq_item::type_id::create("post_reset_recovery_opt");
        start_item(item);
        item.is_raw_axi         = 1'b0;
        item.reprogram          = 1'b1;
        item.prog_length        = prog_len;
        item.program_mem        = quad_prog;
        item.num_vars           = 5'd4;
        item.tolerance          = 32'h0000_0100;
        item.step_alpha         = 32'h0001_0000;
        item.lambda_reg         = 32'h0000_0800;
        item.max_sweeps         = 8'd25;
        for (int i = 0; i < 4; i++) item.x_init[i] = 32'h0002_0000;
        item.do_mid_reset       = 1'b0;
        finish_item(item);

        `uvm_info(get_type_name(), $sformatf("Post-Reset Optimization Result: Status=%s, Sweeps=%0d, f_opt=0x%08h",
            item.status.name(), item.sweep_count, item.f_optimal), UVM_LOW)

        if (item.status != STATUS_CONVERGED) begin
            `uvm_error(get_type_name(), $sformatf("Post-Reset Optimization Failed: Expected CONVERGED, got %s", item.status.name()))
        end else begin
            `uvm_info(get_type_name(), "Test 5 Passed: Post-Reset Optimization converged flawlessly!", UVM_LOW)
        end

        // ---------------------------------------------------------------------
        // Test 5B: 2-Variable Post-Reset Recovery with Tight Tolerance
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 5B: 2-Variable Post-Reset Optimization...", UVM_LOW)
        build_quad_microcode(quad_prog, prog_len, 2);
        item = newton_axi_seq_item::type_id::create("post_reset_2var_opt");
        start_item(item);
        item.is_raw_axi   = 1'b0;
        item.reprogram    = 1'b1;
        item.prog_length  = prog_len;
        item.program_mem  = quad_prog;
        item.num_vars     = 5'd2;
        item.tolerance    = 32'h0000_0030; // tight tolerance bin
        item.step_alpha   = 32'h0000_C000; // 0.75 alpha bin
        item.lambda_reg   = 32'h0000_0400;
        item.max_sweeps   = 8'd20;
        item.x_init[0]    = 32'h0001_8000;
        item.x_init[1]    = -32'h0001_8000;
        item.do_mid_reset = 1'b0;
        finish_item(item);

        // ---------------------------------------------------------------------
        // Test 5C: 8-Variable Post-Reset Recovery with Loose Tolerance
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 5C: 8-Variable Post-Reset Optimization...", UVM_LOW)
        build_quad_microcode(quad_prog, prog_len, 8);
        item = newton_axi_seq_item::type_id::create("post_reset_8var_opt");
        start_item(item);
        item.is_raw_axi   = 1'b0;
        item.reprogram    = 1'b1;
        item.prog_length  = prog_len;
        item.program_mem  = quad_prog;
        item.num_vars     = 5'd8;
        item.tolerance    = 32'h0000_0200; // loose tolerance bin
        item.step_alpha   = 32'h0000_8000; // half step bin
        item.lambda_reg   = 32'h0000_0200; // small damp bin
        item.max_sweeps   = 8'd25;
        for (int i = 0; i < 8; i++) item.x_init[i] = 32'h0001_0000;
        item.do_mid_reset = 1'b0;
        finish_item(item);

        // ---------------------------------------------------------------------
        // Test 5D: 16-Variable Post-Reset Recovery
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 5D: 16-Variable Post-Reset Optimization...", UVM_LOW)
        build_quad_microcode(quad_prog, prog_len, 16);
        item = newton_axi_seq_item::type_id::create("post_reset_16var_opt");
        start_item(item);
        item.is_raw_axi   = 1'b0;
        item.reprogram    = 1'b1;
        item.prog_length  = prog_len;
        item.program_mem  = quad_prog;
        item.num_vars     = 5'd16;
        item.tolerance    = 32'h0000_0080;
        item.step_alpha   = 32'h0001_0000;
        item.lambda_reg   = 32'h0000_0400;
        item.max_sweeps   = 8'd30;
        for (int i = 0; i < 16; i++) item.x_init[i] = ((i % 2 == 0) ? 32'h0001_0000 : -32'h0001_0000);
        item.do_mid_reset = 1'b0;
        finish_item(item);

        // ---------------------------------------------------------------------
        // Test 5E: Post-Reset Max Sweeps Limit (Forces STATUS_MAX_ITERS)
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 5E: Max Sweeps Limit Post-Reset...", UVM_LOW)
        build_quad_microcode(quad_prog, prog_len, 2);
        item = newton_axi_seq_item::type_id::create("post_reset_max_sw_opt");
        start_item(item);
        item.is_raw_axi   = 1'b0;
        item.reprogram    = 1'b1;
        item.prog_length  = prog_len;
        item.program_mem  = quad_prog;
        item.num_vars     = 5'd2;
        item.tolerance    = 32'h0000_0001;
        item.step_alpha   = 32'h0000_4000; // quarter_step
        item.lambda_reg   = 32'h0000_0800; // large damp
        item.max_sweeps   = 8'd1;          // 1 sweep
        item.x_init[0]    = 32'h0003_0000;
        item.x_init[1]    = 32'h0003_0000;
        item.do_mid_reset = 1'b0;
        finish_item(item);

        // ---------------------------------------------------------------------
        // Test 5F: Post-Reset Zero Fallback Optimization (tol=0, alpha=0, lambda=0, max_sw=0)
        // ---------------------------------------------------------------------
        `uvm_info(get_type_name(), "Running Test 5F: Zero Fallback Post-Reset...", UVM_LOW)
        item = newton_axi_seq_item::type_id::create("post_reset_zero_fallback");
        start_item(item);
        item.is_raw_axi   = 1'b0;
        item.reprogram    = 1'b0;
        item.num_vars     = 5'd2;
        item.tolerance    = 32'h0000_0000;
        item.step_alpha   = 32'h0000_0000;
        item.lambda_reg   = 32'h0000_0000;
        item.max_sweeps   = 8'd0;
        item.x_init[0]    = 32'h0001_0000;
        item.x_init[1]    = 32'h0001_0000;
        item.do_mid_reset = 1'b0;
        finish_item(item);

        ping_axi_bus_map();
    endtask

endclass : newton_multivar_reset_seq

`endif // NEWTON_MULTIVAR_RESET_SEQ_SV
