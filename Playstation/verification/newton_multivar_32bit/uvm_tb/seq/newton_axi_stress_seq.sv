// =============================================================================
// File Name   : newton_axi_stress_seq.sv
// Class Name  : newton_axi_stress_seq
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : AXI4-Lite bus stress sequence performing rapid back-to-back
//               register accesses, state vector readbacks, and unmapped address tests.
// =============================================================================

`ifndef NEWTON_AXI_STRESS_SEQ_SV
`define NEWTON_AXI_STRESS_SEQ_SV

class newton_axi_stress_seq extends newton_base_seq;
    `uvm_object_utils(newton_axi_stress_seq)

    function new(string name = "newton_axi_stress_seq");
        super.new(name);
    endfunction

    virtual task body();
        logic [31:0] rdata;
        logic [1:0]  resp;

        `uvm_info(get_type_name(), "Executing AXI4-Lite Protocol Stress Sequence...", UVM_MEDIUM)

        // 1. Complete Register Map R/W Integrity Checks
        `uvm_info(get_type_name(), "Step 1: Testing Config & Control Register R/W Integrity...", UVM_HIGH)
        write_reg(AXI_REG_CTRL,       32'h0000_0000);
        read_reg(AXI_REG_CTRL,        rdata, resp);

        read_reg(AXI_REG_STATUS,      rdata, resp);

        write_reg(AXI_REG_NUM_VARS,   32'd8);
        read_reg(AXI_REG_NUM_VARS,    rdata, resp);
        if (rdata[4:0] !== 5'd8) `uvm_error("AXI_RW_MISMATCH", $sformatf("REG_NUM_VARS mismatch! Got 0x%08h, expected 8", rdata))

        write_reg(AXI_REG_TOLERANCE,  32'h0000_1234);
        read_reg(AXI_REG_TOLERANCE,   rdata, resp);
        if (rdata !== 32'h0000_1234) `uvm_error("AXI_RW_MISMATCH", $sformatf("REG_TOLERANCE mismatch! Got 0x%08h, expected 0x1234", rdata))

        write_reg(AXI_REG_ALPHA,      32'h0000_8000);
        read_reg(AXI_REG_ALPHA,       rdata, resp);
        if (rdata !== 32'h0000_8000) `uvm_error("AXI_RW_MISMATCH", $sformatf("REG_ALPHA mismatch! Got 0x%08h, expected 0x8000", rdata))

        write_reg(AXI_REG_LAMBDA,     32'h0000_0500);
        read_reg(AXI_REG_LAMBDA,      rdata, resp);
        if (rdata !== 32'h0000_0500) `uvm_error("AXI_RW_MISMATCH", $sformatf("REG_LAMBDA mismatch! Got 0x%08h, expected 0x500", rdata))

        write_reg(AXI_REG_MAX_SWEEPS, 32'd42);
        read_reg(AXI_REG_MAX_SWEEPS,  rdata, resp);
        if (rdata[7:0] !== 8'd42) `uvm_error("AXI_RW_MISMATCH", $sformatf("REG_MAX_SWEEPS mismatch! Got 0x%08h, expected 42", rdata))

        read_reg(AXI_REG_SWEEP_COUNT, rdata, resp);
        read_reg(AXI_REG_F_OPTIMAL,   rdata, resp);
        read_reg(AXI_REG_MAX_DELTA,   rdata, resp);

        // Full 64-Entry Microcode BRAM & DFG Instruction Array Bit Toggles (all 64 words x 32 bits)
        `uvm_info(get_type_name(), "Step 1B: Exhaustive bit toggles on all 64 microcode instruction words...", UVM_HIGH)
        begin
            logic [31:0] bram_pats[4] = '{32'h5555_5555, 32'hAAAA_AAAA, 32'hFFFF_FFFF, 32'h0000_0000};
            foreach (bram_pats[p]) begin
                for (int a = 0; a < 64; a++) begin
                    write_reg(AXI_REG_PROG_ADDR, 32'(a));
                    write_reg(AXI_REG_PROG_DATA, bram_pats[p]);
                end
            end
            for (int a = 0; a < 64; a++) begin
                write_reg(AXI_REG_PROG_ADDR, 32'(a));
                read_reg(AXI_REG_PROG_ADDR, rdata, resp);
            end
        end

        // Exhaustive bit toggles on all configuration registers
        `uvm_info(get_type_name(), "Step 1C: Exhaustive bit toggles on configuration registers...", UVM_HIGH)
        begin
            logic [31:0] cfg_pats[4] = '{32'h5555_5555, 32'hAAAA_AAAA, 32'hFFFF_FFFF, 32'h0000_0000};
            foreach (cfg_pats[p]) begin
                write_reg(AXI_REG_TOLERANCE,  cfg_pats[p]);
                read_reg(AXI_REG_TOLERANCE,   rdata, resp);
                write_reg(AXI_REG_ALPHA,      cfg_pats[p]);
                read_reg(AXI_REG_ALPHA,       rdata, resp);
                write_reg(AXI_REG_LAMBDA,     cfg_pats[p]);
                read_reg(AXI_REG_LAMBDA,      rdata, resp);
            end
            write_reg(AXI_REG_MAX_SWEEPS, 32'h55);
            read_reg(AXI_REG_MAX_SWEEPS,  rdata, resp);
            write_reg(AXI_REG_MAX_SWEEPS, 32'hAA);
            read_reg(AXI_REG_MAX_SWEEPS,  rdata, resp);
            write_reg(AXI_REG_MAX_SWEEPS, 32'hFF);
            read_reg(AXI_REG_MAX_SWEEPS,  rdata, resp);
            write_reg(AXI_REG_MAX_SWEEPS, 32'h00);
            read_reg(AXI_REG_MAX_SWEEPS,  rdata, resp);

            write_reg(AXI_REG_NUM_VARS,   32'h15);
            read_reg(AXI_REG_NUM_VARS,    rdata, resp);
            write_reg(AXI_REG_NUM_VARS,   32'h0A);
            read_reg(AXI_REG_NUM_VARS,    rdata, resp);
            write_reg(AXI_REG_NUM_VARS,   32'h1F);
            read_reg(AXI_REG_NUM_VARS,    rdata, resp);
            write_reg(AXI_REG_NUM_VARS,   32'h02);
            read_reg(AXI_REG_NUM_VARS,    rdata, resp);
        end

        // 2. State Vector Window Write & Readback with Full Toggle Patterns
        `uvm_info(get_type_name(), "Step 2: Testing State Vector Window (0x100 - 0x13C) with full bit toggles...", UVM_HIGH)
        for (int i = 0; i < MAX_VARS; i++) begin
            logic [31:0] test_val = 32'hA000_0000 + i;
            write_reg(AXI_STATE_VEC_BASE + (i * 4), test_val);
            read_reg(AXI_STATE_VEC_BASE + (i * 4), rdata, resp);
        end

        // Walking bit patterns across all 16 state registers to ensure 100% toggle coverage on reg_x_init
        begin
            logic [31:0] patterns[4] = '{32'h5555_5555, 32'hAAAA_AAAA, 32'hFFFF_FFFF, 32'h0000_0000};
            foreach (patterns[p]) begin
                for (int i = 0; i < MAX_VARS; i++) begin
                    write_reg(AXI_STATE_VEC_BASE + (i * 4), patterns[p]);
                    read_reg(AXI_STATE_VEC_BASE + (i * 4), rdata, resp);
                end
            end
        end

        // 3. Unmapped Address Checks: 0x048, 0x800, 0xFFC
        `uvm_info(get_type_name(), "Step 3: Testing Unmapped Addresses (0x048, 0x800, 0xFFC)...", UVM_HIGH)
        read_reg(12'h048, rdata, resp);
        if (rdata !== 32'hDEAD_BEEF) `uvm_error("AXI_UNMAPPED_FAIL", $sformatf("0x048 did not return 0xDEAD_BEEF! Got 0x%08h", rdata))

        read_reg(12'h800, rdata, resp);
        if (rdata !== 32'hDEAD_BEEF) `uvm_error("AXI_UNMAPPED_FAIL", $sformatf("0x800 did not return 0xDEAD_BEEF! Got 0x%08h", rdata))

        read_reg(12'hFFC, rdata, resp);
        if (rdata !== 32'hDEAD_BEEF) `uvm_error("AXI_UNMAPPED_FAIL", $sformatf("0xFFC did not return 0xDEAD_BEEF! Got 0x%08h", rdata))

        write_reg(12'h800, 32'hCAFE_BABE);

        // 4. Consecutive Status Polling
        `uvm_info(get_type_name(), "Step 4: Rapid consecutive status polling...", UVM_HIGH)
        for (int i = 0; i < 5; i++) begin
            read_reg(AXI_REG_STATUS, rdata, resp);
        end

        // 5. Backpressure & Stability Tests (forcing BREADY & RREADY delays)
        `uvm_info(get_type_name(), "Step 5: Testing AXI Handshake Backpressure & Bus Stability...", UVM_HIGH)
        for (int i = 0; i < 5; i++) begin
            write_reg(AXI_REG_NUM_VARS, 32'd4, 1'b1); // delay_rdy = 1
            read_reg(AXI_REG_NUM_VARS, rdata, resp, 1'b1); // delay_rdy = 1
        end

        // 6. Split AW and Pipelined Handshakes (forcing SVA stability checks: A_AW_STABLE, A_W_STABLE, A_AR_STABLE)
        `uvm_info(get_type_name(), "Step 6: Testing Split AW & Pipelined Handshakes (A_AW_STABLE, A_W_STABLE, A_AR_STABLE)...", UVM_HIGH)
        write_reg_split(AXI_REG_NUM_VARS, 32'd6);
        write_reg_pipelined(AXI_REG_TOLERANCE, 32'h0000_0050);
        read_reg_pipelined(AXI_REG_NUM_VARS, rdata, resp);

        // 7. AXI Bus Toggle Expansion: Exercise awprot, arprot, and wstrb bits
        `uvm_info(get_type_name(), "Step 7: Testing AXI awprot, arprot, and wstrb bit transitions...", UVM_HIGH)
        begin
            logic [2:0] prot_vals[4] = '{3'b001, 3'b010, 3'b100, 3'b111};
            logic [3:0] wstrb_vals[5] = '{4'h1, 4'h2, 4'h4, 4'h8, 4'hF};

            foreach (prot_vals[p]) begin
                write_reg_ext(AXI_REG_TOLERANCE, 32'h0000_0010, 1'b0, prot_vals[p], 4'hF);
                read_reg_ext(AXI_REG_TOLERANCE, rdata, resp, 1'b0, prot_vals[p]);
            end

            foreach (wstrb_vals[s]) begin
                write_reg_ext(AXI_REG_ALPHA, 32'h0001_0000, 1'b0, 3'b000, wstrb_vals[s]);
            end
        end

        // 8. Complete Address & Config Register Bit Toggling (Walking Ones/Zeros)
        `uvm_info(get_type_name(), "Step 8: Testing Walking 1s/0s on Address & Config Registers...", UVM_HIGH)
        begin
            logic [11:0] test_addrs[14] = '{
                12'h000, 12'h004, 12'h008, 12'h010,
                12'h020, 12'h040, 12'h080, 12'h100,
                12'h200, 12'h400, 12'h800, 12'h554,
                12'hAA8, 12'hFFC
            };
            logic [31:0] toggle_pats[4] = '{
                32'h5555_5555, 32'hAAAA_AAAA, 32'hFFFF_FFFF, 32'h0000_0000
            };

            foreach (test_addrs[a]) begin
                read_reg(test_addrs[a], rdata, resp);
                if (test_addrs[a] != 12'h000) begin
                    write_reg(test_addrs[a], 32'h5555_5555);
                    write_reg(test_addrs[a], 32'h0000_0000);
                end
            end

            foreach (toggle_pats[tp]) begin
                write_reg(AXI_REG_TOLERANCE, toggle_pats[tp]);
                read_reg(AXI_REG_TOLERANCE,  rdata, resp);
                write_reg(AXI_REG_ALPHA,     toggle_pats[tp]);
                read_reg(AXI_REG_ALPHA,      rdata, resp);
                write_reg(AXI_REG_LAMBDA,    toggle_pats[tp]);
                read_reg(AXI_REG_LAMBDA,     rdata, resp);
                write_reg(AXI_REG_PROG_DATA, toggle_pats[tp]);
                read_reg(AXI_REG_PROG_DATA,  rdata, resp);
            end
        end

        // Step 4: Post-Stress Optimization Functional Verification
        `uvm_info(get_type_name(), "Step 4: Executing post-stress optimization to verify core functional integrity...", UVM_MEDIUM)
        begin
            newton_axi_seq_item opt_item = newton_axi_seq_item::type_id::create("post_stress_opt_item");
            opt_item.num_vars   = 5'd2;
            opt_item.tolerance  = 32'h0000_0080;
            opt_item.step_alpha = 32'h0001_0000;
            opt_item.lambda_reg = 32'h0000_0400;
            opt_item.max_sweeps = 8'd10;
            opt_item.reprogram  = 1'b1;

            opt_item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
            opt_item.program_mem[1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1, src_b: 5'd1, imm: 13'sd0};
            opt_item.program_mem[2] = '{op: OP_ADD, dst: 5'd31, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
            opt_item.program_mem[3] = '{op: OP_END, dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
            opt_item.prog_length    = 4;

            opt_item.x_init[0] = 32'sd2 * 65536; // 2.0
            opt_item.x_init[1] = 32'sd1 * 65536; // 1.0

            execute_optimization(opt_item);

            // 4-Variable Post-Stress Run with quarter_step alpha
            opt_item = newton_axi_seq_item::type_id::create("post_stress_4var_item");
            opt_item.num_vars   = 5'd4;
            opt_item.tolerance  = 32'h0000_0030; // tight
            opt_item.step_alpha = 32'h0000_4000; // quarter_step
            opt_item.lambda_reg = 32'h0000_0800; // large damp
            opt_item.max_sweeps = 8'd1;          // forces max iters
            opt_item.reprogram  = 1'b1;
            opt_item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
            for (int i = 1; i < 4; i++) begin
                opt_item.program_mem[i*2 - 1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'(i), src_b: 5'(i), imm: 13'sd0};
                opt_item.program_mem[i*2]     = '{op: OP_ADD, dst: (i == 3) ? 5'd31 : 5'd16, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
            end
            opt_item.program_mem[7] = '{op: OP_END, dst: 5'd0, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
            opt_item.prog_length    = 8;
            for (int i = 0; i < 4; i++) opt_item.x_init[i] = 32'sd3 * 65536;
            execute_optimization(opt_item);

            // 8-Variable Post-Stress Run with half_step alpha
            opt_item = newton_axi_seq_item::type_id::create("post_stress_8var_item");
            opt_item.num_vars   = 5'd8;
            opt_item.tolerance  = 32'h0000_0200; // loose
            opt_item.step_alpha = 32'h0000_8000; // half step
            opt_item.lambda_reg = 32'h0000_0200; // small damp
            opt_item.max_sweeps = 8'd20;
            opt_item.reprogram  = 1'b1;
            opt_item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
            for (int i = 1; i < 8; i++) begin
                opt_item.program_mem[i*2 - 1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'(i), src_b: 5'(i), imm: 13'sd0};
                opt_item.program_mem[i*2]     = '{op: OP_ADD, dst: (i == 7) ? 5'd31 : 5'd16, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
            end
            opt_item.program_mem[15] = '{op: OP_END, dst: 5'd0, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
            opt_item.prog_length     = 16;
            for (int i = 0; i < 8; i++) opt_item.x_init[i] = 32'sd1 * 65536;
            execute_optimization(opt_item);

            // 16-Variable Post-Stress Run with full_step alpha
            opt_item = newton_axi_seq_item::type_id::create("post_stress_16var_item");
            opt_item.num_vars   = 5'd16;
            opt_item.tolerance  = 32'h0000_0080;
            opt_item.step_alpha = 32'h0001_0000;
            opt_item.lambda_reg = 32'h0000_0400;
            opt_item.max_sweeps = 8'd25;
            opt_item.reprogram  = 1'b1;
            build_quad_microcode(opt_item.program_mem, opt_item.prog_length, 16);
            for (int i = 0; i < 16; i++) opt_item.x_init[i] = ((i % 2 == 0) ? 32'sd1 * 65536 : -32'sd1 * 65536);
            execute_optimization(opt_item);

            // Zero Fallback Post-Stress Run
            opt_item = newton_axi_seq_item::type_id::create("post_stress_fallback_item");
            opt_item.num_vars   = 5'd2;
            opt_item.tolerance  = 32'h0000_0000;
            opt_item.step_alpha = 32'h0000_0000;
            opt_item.lambda_reg = 32'h0000_0000;
            opt_item.max_sweeps = 8'd0;
            opt_item.reprogram  = 1'b0;
            opt_item.x_init[0]  = 32'sd1 * 65536;
            opt_item.x_init[1]  = 32'sd1 * 65536;
            execute_optimization(opt_item);
        end

        `uvm_info(get_type_name(), "AXI4-Lite Stress Sequence Completed Successfully.", UVM_MEDIUM)
    endtask

endclass : newton_axi_stress_seq

`endif // NEWTON_AXI_STRESS_SEQ_SV
