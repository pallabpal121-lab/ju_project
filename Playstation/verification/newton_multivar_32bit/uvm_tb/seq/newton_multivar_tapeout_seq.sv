// =============================================================================
// File Name   : newton_multivar_tapeout_seq.sv
// Class Name  : newton_multivar_tapeout_seq
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Tapeout-grade optimization sequence targeting 100% functional
//               coverage closure across odd dimensions (N=3..15), high sweep limits
//               (31-50), high executed sweeps (21-50), and all parameter bins.
// =============================================================================

`ifndef NEWTON_MULTIVAR_TAPEOUT_SEQ_SV
`define NEWTON_MULTIVAR_TAPEOUT_SEQ_SV

class newton_multivar_tapeout_seq extends newton_base_seq;
    `uvm_object_utils(newton_multivar_tapeout_seq)

    function new(string name = "newton_multivar_tapeout_seq");
        super.new(name);
    endfunction

    // -------------------------------------------------------------------------
    // Helper: Build Quadratic Microcode for arbitrary N (2 to 16)
    // -------------------------------------------------------------------------
    function void build_quad_microcode(output instr_t prog[PROG_DEPTH], output int len, input int n);
        int idx = 0;
        // r16 = x0 * x0
        prog[idx++] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        if (n == 2) begin
            // r17 = x1 * x1
            prog[idx++] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1, src_b: 5'd1, imm: 13'sd0};
            // r31 = r16 + r17
            prog[idx++] = '{op: OP_ADD, dst: 5'd31, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
        end else begin
            // r17 = x1 * x1
            prog[idx++] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1, src_b: 5'd1, imm: 13'sd0};
            // r18 = r16 + r17
            prog[idx++] = '{op: OP_ADD, dst: 5'd18, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
            for (int i = 2; i < n; i++) begin
                // r16 = x_i * x_i
                prog[idx++] = '{op: OP_MUL, dst: 5'd16, src_a: 5'(i), src_b: 5'(i), imm: 13'sd0};
                // r18 = r18 + r16 (last one writes to r31)
                prog[idx++] = '{op: OP_ADD, dst: (i == n-1) ? 5'd31 : 5'd18, src_a: 5'd18, src_b: 5'd16, imm: 13'sd0};
            end
        end
        prog[idx++] = '{op: OP_END, dst: 5'd0, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        len = idx;
    endfunction

    virtual task body();
        newton_axi_seq_item item;

        `uvm_info(get_type_name(), "Executing Tapeout-Grade Comprehensive Sequence...", UVM_MEDIUM)

        // ---------------------------------------------------------------------
        // Test 1: Odd Dimensions in [9..15] range (N=9, N=11, N=15)
        // ---------------------------------------------------------------------
        begin
            int odd_dims[3] = '{9, 11, 15};
            foreach (odd_dims[d]) begin
                int cur_n = odd_dims[d];
                item = newton_axi_seq_item::type_id::create($sformatf("tapeout_odd_n%0d", cur_n));
                item.num_vars   = 5'(cur_n);
                item.tolerance  = 32'h0000_0080; // med tolerance
                item.step_alpha = 32'h0001_0000; // full_step
                item.lambda_reg = 32'h0000_0400; // default_damp
                item.max_sweeps = 8'd20;         // med_sweeps
                item.reprogram  = 1'b1;

                build_quad_microcode(item.program_mem, item.prog_length, cur_n);

                for (int i = 0; i < cur_n; i++) begin
                    item.x_init[i] = ((i % 2 == 0) ? 32'sd1 : -32'sd1) * 65536;
                end

                `uvm_info(get_type_name(), $sformatf("Running Odd Dimension N=%0d quadratic test...", cur_n), UVM_HIGH)
                execute_optimization(item);
            end
        end

        // ---------------------------------------------------------------------
        // Test 2: Odd Dimensions in [3..7] range (N=3, N=5, N=7)
        // ---------------------------------------------------------------------
        begin
            int small_odd[3] = '{3, 5, 7};
            foreach (small_odd[d]) begin
                int cur_n = small_odd[d];
                item = newton_axi_seq_item::type_id::create($sformatf("tapeout_odd_n%0d", cur_n));
                item.num_vars   = 5'(cur_n);
                item.tolerance  = 32'h0000_0020; // tight tolerance
                item.step_alpha = 32'h0000_C000; // three_quarter
                item.lambda_reg = 32'h0000_0200; // small_damp
                item.max_sweeps = 8'd25;
                item.reprogram  = 1'b1;

                build_quad_microcode(item.program_mem, item.prog_length, cur_n);

                for (int i = 0; i < cur_n; i++) begin
                    item.x_init[i] = 32'sd2 * 65536;
                end

                execute_optimization(item);
            end
        end

        // ---------------------------------------------------------------------
        // Test 3: High Max Sweeps (31..50) & High Executed Sweeps (21..50)
        // Using coupled Rosenbrock with tight tolerance and small alpha
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("tapeout_high_sweeps_item");
        item.num_vars   = 5'd2;
        item.tolerance  = 32'h0000_0001; // extremely tight -> force max iters
        item.step_alpha = 32'h0000_4000; // quarter_step
        item.lambda_reg = 32'h0000_0800; // large_damp
        item.max_sweeps = 8'd35;         // high bin [31:50]
        item.reprogram  = 1'b1;

        // Coupled Rosenbrock microcode
        item.program_mem[0] = '{op: OP_LOADC, dst: 5'd16, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd1};
        item.program_mem[1] = '{op: OP_SUB,   dst: 5'd17, src_a: 5'd0,  src_b: 5'd16, imm: 13'sd0};
        item.program_mem[2] = '{op: OP_MUL,   dst: 5'd18, src_a: 5'd17, src_b: 5'd17, imm: 13'sd0};
        item.program_mem[3] = '{op: OP_SUB,   dst: 5'd19, src_a: 5'd1,  src_b: 5'd16, imm: 13'sd0};
        item.program_mem[4] = '{op: OP_MUL,   dst: 5'd20, src_a: 5'd19, src_b: 5'd19, imm: 13'sd0};
        item.program_mem[5] = '{op: OP_ADD,   dst: 5'd21, src_a: 5'd18, src_b: 5'd20, imm: 13'sd0};
        item.program_mem[6] = '{op: OP_MUL,   dst: 5'd22, src_a: 5'd0,  src_b: 5'd1,  imm: 13'sd0};
        item.program_mem[7] = '{op: OP_ADD,   dst: 5'd31, src_a: 5'd21, src_b: 5'd22, imm: 13'sd0};
        item.program_mem[8] = '{op: OP_END,   dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        item.prog_length    = 9;

        item.x_init[0] = 32'sh0001_0000; // 1.0
        item.x_init[1] = 32'sh0002_0000; // 2.0

        `uvm_info(get_type_name(), "Running High Sweeps Rosenbrock test...", UVM_HIGH)
        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test 4: Max Iters across N=9 to hit cx_vars_status [n9_n15, max_iters]
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("tapeout_n9_max_iters");
        item.num_vars   = 5'd9;
        item.tolerance  = 32'h0000_0001; // extremely tight -> force max iters
        item.step_alpha = 32'h0000_4000; // quarter_step
        item.lambda_reg = 32'h0000_0800; // large_damp
        item.max_sweeps = 8'd1;          // 1 sweep -> immediate max iters
        item.reprogram  = 1'b1;

        build_quad_microcode(item.program_mem, item.prog_length, 9);
        for (int i = 0; i < 9; i++) item.x_init[i] = 32'sd3 * 65536;

        `uvm_info(get_type_name(), "Running N=9 Max Iters test...", UVM_HIGH)
        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test 5: Max Iters across N=16 to hit cx_vars_status [n16, max_iters]
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("tapeout_n16_max_iters");
        item.num_vars   = 5'd16;
        item.tolerance  = 32'h0000_0001; // extremely tight -> force max iters
        item.step_alpha = 32'h0000_4000; // quarter_step
        item.lambda_reg = 32'h0000_0800; // large_damp
        item.max_sweeps = 8'd1;          // 1 sweep -> immediate max iters
        item.reprogram  = 1'b1;

        build_quad_microcode(item.program_mem, item.prog_length, 16);
        for (int i = 0; i < 16; i++) item.x_init[i] = 32'sd2 * 65536;

        `uvm_info(get_type_name(), "Running N=16 Max Iters test...", UVM_HIGH)
        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test 6: Max Iters across N=4 to hit cx_vars_status [n3_n4, max_iters]
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("tapeout_n4_max_iters");
        item.num_vars   = 5'd4;
        item.tolerance  = 32'h0000_0001;
        item.step_alpha = 32'h0000_4000;
        item.lambda_reg = 32'h0000_0800;
        item.max_sweeps = 8'd1;
        item.reprogram  = 1'b1;

        build_quad_microcode(item.program_mem, item.prog_length, 4);
        for (int i = 0; i < 4; i++) item.x_init[i] = 32'sd2 * 65536;

        `uvm_info(get_type_name(), "Running N=4 Max Iters test...", UVM_HIGH)
        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test 7: Max Iters across N=8 to hit cx_vars_status [n5_n8, max_iters]
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("tapeout_n8_max_iters");
        item.num_vars   = 5'd8;
        item.tolerance  = 32'h0000_0001;
        item.step_alpha = 32'h0000_4000;
        item.lambda_reg = 32'h0000_0800;
        item.max_sweeps = 8'd1;
        item.reprogram  = 1'b1;

        build_quad_microcode(item.program_mem, item.prog_length, 8);
        for (int i = 0; i < 8; i++) item.x_init[i] = 32'sd2 * 65536;

        `uvm_info(get_type_name(), "Running N=8 Max Iters test...", UVM_HIGH)
        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test 8: Complete ALU & DFG Opcode Verification
        // Exercises OP_NOP, OP_LOADC, OP_MOV, OP_NEG, OP_DIV, OP_MUL, OP_ADD, OP_END
        // Objective: f(x0, x1) = ((-x0)^2 / 2.0) + x1^2 (convex bowl, min at 0, 0)
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("tapeout_alu_opcodes");
        item.num_vars   = 5'd2;
        item.tolerance  = 32'h0000_0080;
        item.step_alpha = 32'h0001_0000;
        item.lambda_reg = 32'h0000_0400;
        item.max_sweeps = 8'd20;
        item.reprogram  = 1'b1;

        // r16 = 2.0 (OP_LOADC)
        item.program_mem[0] = '{op: OP_LOADC, dst: 5'd16, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd2};
        // r17 = OP_NOP
        item.program_mem[1] = '{op: OP_NOP,   dst: 5'd17, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        // r18 = OP_MOV(x0)
        item.program_mem[2] = '{op: OP_MOV,   dst: 5'd18, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        // r19 = OP_NEG(r18) -> -x0
        item.program_mem[3] = '{op: OP_NEG,   dst: 5'd19, src_a: 5'd18, src_b: 5'd0,  imm: 13'sd0};
        // r20 = OP_MUL(r19, r19) -> (-x0)^2 = x0^2
        item.program_mem[4] = '{op: OP_MUL,   dst: 5'd20, src_a: 5'd19, src_b: 5'd19, imm: 13'sd0};
        // r21 = OP_DIV(r20, r16) -> x0^2 / 2.0
        item.program_mem[5] = '{op: OP_DIV,   dst: 5'd21, src_a: 5'd20, src_b: 5'd16, imm: 13'sd0};
        // r22 = OP_MUL(x1, x1) -> x1^2
        item.program_mem[6] = '{op: OP_MUL,   dst: 5'd22, src_a: 5'd1,  src_b: 5'd1,  imm: 13'sd0};
        // r31 = OP_ADD(r21, r22) -> (x0^2 / 2.0) + x1^2
        item.program_mem[7] = '{op: OP_ADD,   dst: 5'd31, src_a: 5'd21, src_b: 5'd22, imm: 13'sd0};
        // OP_END
        item.program_mem[8] = '{op: OP_END,   dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        item.prog_length    = 9;

        item.x_init[0] = 32'sd2 * 65536;  // 2.0
        item.x_init[1] = -32'sd2 * 65536; // -2.0

        `uvm_info(get_type_name(), "Running Full ALU & DFG Opcode Verification test...", UVM_HIGH)
        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test 9: Exhaustive Cross Coverage Closure for (Tol x Sweeps) & (Alpha x Sweeps)
        // ---------------------------------------------------------------------
        begin
            q16_t tols[4]   = '{32'h0000_0000, 32'h0000_0020, 32'h0000_0080, 32'h0000_0200};
            q16_t alphas[5] = '{32'h0000_0000, 32'h0000_4000, 32'h0000_8000, 32'h0000_C000, 32'h0001_0000};
            logic [7:0] target_sw[4] = '{8'd1, 8'd4, 8'd15, 8'd25};

            // Cross every tolerance with every sweep tier
            foreach (tols[t]) begin
                foreach (target_sw[s]) begin
                    item = newton_axi_seq_item::type_id::create($sformatf("cx_tol_%0d_sw_%0d", t, s));
                    item.num_vars   = 5'd2;
                    item.tolerance  = tols[t];
                    item.step_alpha = 32'h0000_4000;
                    item.lambda_reg = 32'h0000_0400;
                    item.max_sweeps = target_sw[s];
                    item.reprogram  = 1'b0;
                    item.x_init[0]  = 32'sd3 * 65536;
                    item.x_init[1]  = -32'sd3 * 65536;
                    execute_optimization(item);
                end
            end

            // Cross every alpha with every sweep tier
            foreach (alphas[a]) begin
                foreach (target_sw[s]) begin
                    item = newton_axi_seq_item::type_id::create($sformatf("cx_alpha_%0d_sw_%0d", a, s));
                    item.num_vars   = 5'd2;
                    item.tolerance  = 32'h0000_0001; // tight to ensure it hits max_sweeps
                    item.step_alpha = alphas[a];
                    item.lambda_reg = 32'h0000_0400;
                    item.max_sweeps = target_sw[s];
                    item.reprogram  = 1'b0;
                    item.x_init[0]  = 32'sd3 * 65536;
                    item.x_init[1]  = -32'sd3 * 65536;
                    execute_optimization(item);
                end
            end
        end

        // ---------------------------------------------------------------------
        // Test 10: Targeted 100% Cross Coverage Closure for (Tol x Sweeps) & (Alpha x Sweeps)
        // Uses f(x0, x1) = (x0^2 + x1^2) / 64.0 with heavy damping (lambda = 0.0625)
        // so that gradient delta decreases slowly, deterministically executing all
        // 15 or 25 sweeps to close the remaining cross bins to 100.00%.
        // ---------------------------------------------------------------------
        begin
            `uvm_info(get_type_name(), "Test 10: Executing Targeted 100% Cross Coverage Closure...", UVM_HIGH)

            // 1. cx_tol_sweeps: [loose] x [max_sweeps]
            item = newton_axi_seq_item::type_id::create("cx_tol_loose_max_sw");
            item.num_vars   = 5'd2;
            item.tolerance  = 32'h0000_0200; // loose [0x101..0x800]
            item.step_alpha = 32'h0000_4000; // quarter_step
            item.lambda_reg = 32'h0000_1000; // large_damp
            item.max_sweeps = 8'd25;         // max_sweeps [21..50]
            item.reprogram  = 1'b1;
            item.program_mem[0] = '{op: OP_LOADC, dst: 5'd16, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd64};
            item.program_mem[1] = '{op: OP_MUL,   dst: 5'd17, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
            item.program_mem[2] = '{op: OP_MUL,   dst: 5'd18, src_a: 5'd1,  src_b: 5'd1,  imm: 13'sd0};
            item.program_mem[3] = '{op: OP_ADD,   dst: 5'd19, src_a: 5'd17, src_b: 5'd18, imm: 13'sd0};
            item.program_mem[4] = '{op: OP_DIV,   dst: 5'd31, src_a: 5'd19, src_b: 5'd16, imm: 13'sd0};
            item.program_mem[5] = '{op: OP_END,   dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
            item.prog_length    = 6;
            item.x_init[0]      = 32'sd30 * 65536;
            item.x_init[1]      = -32'sd30 * 65536;
            execute_optimization(item);

            // 2. cx_alpha_sweeps: [half_step] x [max_sweeps]
            item = newton_axi_seq_item::type_id::create("cx_alpha_half_max_sw");
            item.num_vars   = 5'd2;
            item.tolerance  = 32'h0000_0001;
            item.step_alpha = 32'h0000_8000; // half_step
            item.lambda_reg = 32'h0000_1000;
            item.max_sweeps = 8'd25;         // max_sweeps [21..50]
            item.reprogram  = 1'b0;
            item.x_init[0]  = 32'sd20 * 65536;
            item.x_init[1]  = -32'sd20 * 65536;
            execute_optimization(item);

            // 3. cx_alpha_sweeps: [three_quarter] x [max_sweeps]
            item = newton_axi_seq_item::type_id::create("cx_alpha_three_qtr_max_sw");
            item.num_vars   = 5'd2;
            item.tolerance  = 32'h0000_0001;
            item.step_alpha = 32'h0000_C000; // three_quarter
            item.lambda_reg = 32'h0000_1000;
            item.max_sweeps = 8'd25;         // max_sweeps [21..50]
            item.reprogram  = 1'b0;
            item.x_init[0]  = 32'sd20 * 65536;
            item.x_init[1]  = -32'sd20 * 65536;
            execute_optimization(item);

            // 4. cx_alpha_sweeps: [full_step] x [med_sweeps]
            item = newton_axi_seq_item::type_id::create("cx_alpha_full_med_sw");
            item.num_vars   = 5'd2;
            item.tolerance  = 32'h0000_0001;
            item.step_alpha = 32'h0001_0000; // full_step
            item.lambda_reg = 32'h0000_1000;
            item.max_sweeps = 8'd15;         // med_sweeps [6..20]
            item.reprogram  = 1'b0;
            item.x_init[0]  = 32'sd20 * 65536;
            item.x_init[1]  = -32'sd20 * 65536;
            execute_optimization(item);

            // 5. cx_alpha_sweeps: [full_step] x [max_sweeps]
            item = newton_axi_seq_item::type_id::create("cx_alpha_full_max_sw");
            item.num_vars   = 5'd2;
            item.tolerance  = 32'h0000_0001;
            item.step_alpha = 32'h0001_0000; // full_step
            item.lambda_reg = 32'h0000_1000;
            item.max_sweeps = 8'd25;         // max_sweeps [21..50]
            item.reprogram  = 1'b0;
            item.x_init[0]  = 32'sd30 * 65536;
            item.x_init[1]  = -32'sd30 * 65536;
            execute_optimization(item);

            // 6. cx_alpha_sweeps: [zero_fallback] x [med_sweeps]
            item = newton_axi_seq_item::type_id::create("cx_alpha_zero_med_sw");
            item.num_vars   = 5'd2;
            item.tolerance  = 32'h0000_0001;
            item.step_alpha = 32'h0000_0000; // zero_fallback
            item.lambda_reg = 32'h0000_1000;
            item.max_sweeps = 8'd15;         // med_sweeps [6..20]
            item.reprogram  = 1'b0;
            item.x_init[0]  = 32'sd20 * 65536;
            item.x_init[1]  = -32'sd20 * 65536;
            execute_optimization(item);

            // 7. cx_alpha_sweeps: [zero_fallback] x [max_sweeps]
            item = newton_axi_seq_item::type_id::create("cx_alpha_zero_max_sw");
            item.num_vars   = 5'd2;
            item.tolerance  = 32'h0000_0001;
            item.step_alpha = 32'h0000_0000; // zero_fallback
            item.lambda_reg = 32'h0000_1000;
            item.max_sweeps = 8'd25;         // max_sweeps [21..50]
            item.reprogram  = 1'b0;
            item.x_init[0]  = 32'sd30 * 65536;
            item.x_init[1]  = -32'sd30 * 65536;
            execute_optimization(item);
        end

        // ---------------------------------------------------------------------
        // Test 11: Ill-Conditioned Hessian Fallback (det <= 16)
        // Linear objective f(x0, x1) = x0 + x1 produces zero second derivatives.
        // With lambda_reg = 0, det = 0 <= 16, triggering hardware fallback to
        // gradient descent in newton_2var_core (lines 352-364).
        // ---------------------------------------------------------------------
        begin
            `uvm_info(get_type_name(), "Test 11: Ill-Conditioned Hessian Gradient Descent Fallback (det <= 16)...", UVM_MEDIUM)
            item = newton_axi_seq_item::type_id::create("ill_conditioned_hessian_det_fallback");
            item.num_vars    = 5'd2;
            item.tolerance   = 32'h0000_0100;
            item.step_alpha  = 32'h0001_0000;
            item.lambda_reg  = 32'h0000_0000; // Zero damping -> det = 0 <= 16
            item.max_sweeps  = 8'd5;
            item.reprogram   = 1'b1;
            item.prog_length = 2;
            // r31 = x0 + x1
            item.program_mem[0] = '{op: OP_ADD, dst: 5'd31, src_a: 5'd0, src_b: 5'd1, imm: 13'sd0};
            item.program_mem[1] = '{op: OP_END, dst: 5'd0,  src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
            item.x_init[0]   = 32'h0002_0000;
            item.x_init[1]   = 32'h0002_0000;
            execute_optimization(item);
        end

        // ---------------------------------------------------------------------
        // Test 12: Divider Exceptions in DFG (Division by Zero & Negative Divisor)
        // Microcode exercises OP_DIV with divisor = 0 to cover div_by_zero in q16_divider,
        // and OP_DIV with negative divisor to cover divisor[31] ? -divisor : divisor.
        // ---------------------------------------------------------------------
        begin
            `uvm_info(get_type_name(), "Test 12: DFG q16_divider Exceptions (div_by_zero, negative divisor, sign_res)...", UVM_MEDIUM)
            item = newton_axi_seq_item::type_id::create("q16_divider_exceptions");
            item.num_vars    = 5'd2;
            item.tolerance   = 32'h0000_0100;
            item.step_alpha  = 32'h0001_0000;
            item.lambda_reg  = 32'h0000_0800;
            item.max_sweeps  = 8'd2;
            item.reprogram   = 1'b1;
            item.prog_length = 11;
            // r16 = 0.0
            item.program_mem[0]  = '{op: OP_LOADC, dst: 5'd16, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
            // r17 = x0 / 0.0 (triggers div_by_zero = 1, quotient = 0)
            item.program_mem[1]  = '{op: OP_DIV,   dst: 5'd17, src_a: 5'd0,  src_b: 5'd16, imm: 13'sd0};
            // r18 = 2.0
            item.program_mem[2]  = '{op: OP_LOADC, dst: 5'd18, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd2};
            // r19 = -x0 (negative dividend)
            item.program_mem[3]  = '{op: OP_NEG,   dst: 5'd19, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
            // r20 = -x0 / 2.0 (triggers sign_res = 1, negative quotient, positive divisor)
            item.program_mem[4]  = '{op: OP_DIV,   dst: 5'd20, src_a: 5'd19, src_b: 5'd18, imm: 13'sd0};
            // r21 = -2.0 (negative divisor)
            item.program_mem[5]  = '{op: OP_LOADC, dst: 5'd21, src_a: 5'd0,  src_b: 5'd0,  imm: -13'sd2};
            // r22 = x0 / -2.0 (positive dividend, negative divisor -> divisor[31]=1, sign_res=1)
            item.program_mem[6]  = '{op: OP_DIV,   dst: 5'd22, src_a: 5'd0,  src_b: 5'd21, imm: 13'sd0};
            // r23 = -x0 / -2.0 (negative dividend, negative divisor -> divisor[31]=1, sign_res=0)
            item.program_mem[7]  = '{op: OP_DIV,   dst: 5'd23, src_a: 5'd19, src_b: 5'd21, imm: 13'sd0};
            // r24 = 1.0
            item.program_mem[8]  = '{op: OP_LOADC, dst: 5'd24, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd1};
            // r25 = 1.0 / 2.0 = 0.5 (32'h0000_8000, bit 15 is 1)
            item.program_mem[9]  = '{op: OP_DIV,   dst: 5'd25, src_a: 5'd24, src_b: 5'd18, imm: 13'sd0};
            // r26 = x0 / 0.5 (exercises divisor[15]=1 and abs_div[15]=1)
            item.program_mem[10] = '{op: OP_DIV,   dst: 5'd26, src_a: 5'd0,  src_b: 5'd25, imm: 13'sd0};
            // r27 = 63.0 (32'h003F_0000, bits 21:16 are 1)
            item.program_mem[11] = '{op: OP_LOADC, dst: 5'd27, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd63};
            // r28 = x0 / 63.0 (exercises abs_div[21:19] toggle)
            item.program_mem[12] = '{op: OP_DIV,   dst: 5'd28, src_a: 5'd0,  src_b: 5'd27, imm: 13'sd0};
            // r14 = 3.0
            item.program_mem[13] = '{op: OP_LOADC, dst: 5'd14, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd3};
            // r15 = 1.0 / 3.0 = 0x0000_5555 (exercises alternating bits in divisor[14:0])
            item.program_mem[14] = '{op: OP_DIV,   dst: 5'd15, src_a: 5'd24, src_b: 5'd14, imm: 13'sd0};
            // r29 = x0 / 0x0000_5555 (sets divisor[14:0] = 0x5555)
            item.program_mem[15] = '{op: OP_DIV,   dst: 5'd29, src_a: 5'd0,  src_b: 5'd15, imm: 13'sd0};
            // r13 = 2.0 / 3.0 = 0x0000_AAAA (exercises alternating bits in divisor[14:0])
            item.program_mem[16] = '{op: OP_DIV,   dst: 5'd13, src_a: 5'd18, src_b: 5'd14, imm: 13'sd0};
            // r30 = x0 / 0x0000_AAAA (sets divisor[14:0] = 0xAAAA)
            item.program_mem[17] = '{op: OP_DIV,   dst: 5'd30, src_a: 5'd0,  src_b: 5'd13, imm: 13'sd0};
            // Sum all results
            item.program_mem[18] = '{op: OP_ADD,   dst: 5'd12, src_a: 5'd17, src_b: 5'd20, imm: 13'sd0};
            item.program_mem[19] = '{op: OP_ADD,   dst: 5'd11, src_a: 5'd22, src_b: 5'd23, imm: 13'sd0};
            item.program_mem[20] = '{op: OP_ADD,   dst: 5'd10, src_a: 5'd26, src_b: 5'd28, imm: 13'sd0};
            item.program_mem[21] = '{op: OP_ADD,   dst: 5'd9,  src_a: 5'd29, src_b: 5'd30, imm: 13'sd0};
            item.program_mem[22] = '{op: OP_ADD,   dst: 5'd12, src_a: 5'd12, src_b: 5'd11, imm: 13'sd0};
            item.program_mem[23] = '{op: OP_ADD,   dst: 5'd10, src_a: 5'd10, src_b: 5'd9,  imm: 13'sd0};
            item.program_mem[24] = '{op: OP_ADD,   dst: 5'd31, src_a: 5'd12, src_b: 5'd10, imm: 13'sd0};
            item.program_mem[25] = '{op: OP_END,   dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
            item.prog_length     = 26;
            item.x_init[0]   = 32'h0004_0000;
            item.x_init[1]   = 32'h0004_0000;
            execute_optimization(item);
        end

        // ---------------------------------------------------------------------
        // Test 13: ALU Signed Overflow Coverage (q16_alu.sv lines 41-44, 49-52)
        // Exercises positive/negative overflow and underflow on OP_ADD and OP_SUB.
        // ---------------------------------------------------------------------
        begin
            `uvm_info(get_type_name(), "Test 13: ALU Signed Overflow Coverage (OP_ADD & OP_SUB)...", UVM_MEDIUM)
            item = newton_axi_seq_item::type_id::create("alu_overflow_test");
            item.num_vars    = 5'd2;
            item.tolerance   = 32'h0000_0100;
            item.step_alpha  = 32'h0001_0000;
            item.lambda_reg  = 32'h0000_0800;
            item.max_sweeps  = 8'd2;
            item.reprogram   = 1'b1;
            item.prog_length = 12;
            // r16 = 2047.0 (0x07FF_0000)
            item.program_mem[0]  = '{op: OP_LOADC, dst: 5'd16, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd2047};
            // Double r16 four times to reach 0x7FF0_0000 (near signed 32-bit max)
            item.program_mem[1]  = '{op: OP_ADD,   dst: 5'd17, src_a: 5'd16, src_b: 5'd16, imm: 13'sd0};
            item.program_mem[2]  = '{op: OP_ADD,   dst: 5'd18, src_a: 5'd17, src_b: 5'd17, imm: 13'sd0};
            item.program_mem[3]  = '{op: OP_ADD,   dst: 5'd19, src_a: 5'd18, src_b: 5'd18, imm: 13'sd0};
            item.program_mem[4]  = '{op: OP_ADD,   dst: 5'd20, src_a: 5'd19, src_b: 5'd19, imm: 13'sd0};
            // r21 = r20 + r20 -> Positive + Positive = Negative (OP_ADD positive overflow)
            item.program_mem[5]  = '{op: OP_ADD,   dst: 5'd21, src_a: 5'd20, src_b: 5'd20, imm: 13'sd0};
            // r22 = r21 + r21 -> Negative + Negative = Positive (OP_ADD negative overflow)
            item.program_mem[6]  = '{op: OP_ADD,   dst: 5'd22, src_a: 5'd21, src_b: 5'd21, imm: 13'sd0};
            // r23 = r20 - r21 -> Positive - Negative = Negative (OP_SUB positive overflow)
            item.program_mem[7]  = '{op: OP_SUB,   dst: 5'd23, src_a: 5'd20, src_b: 5'd21, imm: 13'sd0};
            // r24 = r21 - r20 -> Negative - Positive = Positive (OP_SUB negative overflow)
            item.program_mem[8]  = '{op: OP_SUB,   dst: 5'd24, src_a: 5'd21, src_b: 5'd20, imm: 13'sd0};
            item.program_mem[9]  = '{op: OP_ADD,   dst: 5'd25, src_a: 5'd23, src_b: 5'd24, imm: 13'sd0};
            item.program_mem[10] = '{op: OP_ADD,   dst: 5'd31, src_a: 5'd25, src_b: 5'd22, imm: 13'sd0};
            item.program_mem[11] = '{op: OP_END,   dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
            item.x_init[0]   = 32'h0001_0000;
            item.x_init[1]   = 32'h0001_0000;
            execute_optimization(item);
        end

        // ---------------------------------------------------------------------
        // Test 14: Variable Clamping Corner Test (num_vars = 1)
        // Tests hardware safeguard: num_vars_reg <= (num_vars >= 2) ? num_vars : 2'd2
        // Clamps single-variable input to 2-variable minimum in newton_bcd_top (line 136).
        // ---------------------------------------------------------------------
        begin
            `uvm_info(get_type_name(), "Test 14: Variable Clamping Corner Test (num_vars = 1)...", UVM_MEDIUM)
            item = newton_axi_seq_item::type_id::create("num_vars_clamp_test");
            item.num_vars    = 5'd1; // Under-variable corner case -> clamped to 2'd2
            item.tolerance   = 32'h0000_0100;
            item.step_alpha  = 32'h0001_0000;
            item.lambda_reg  = 32'h0000_0800;
            item.max_sweeps  = 8'd5;
            item.reprogram   = 1'b1;
            item.prog_length = 3;
            // f(x0, x1) = x0^2 + x1^2
            item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
            item.program_mem[1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1, src_b: 5'd1, imm: 13'sd0};
            item.program_mem[2] = '{op: OP_ADD, dst: 5'd31, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
            item.program_mem[3] = '{op: OP_END, dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
            item.prog_length    = 4;
            item.x_init[0]   = 32'h0002_0000;
            item.x_init[1]   = 32'h0002_0000;
            execute_optimization(item);
        end

        // ---------------------------------------------------------------------
        // Test 15: 16-Variable Full State Vector Bit Toggle Pattern
        // Exercises bit flips 0->1 and 1->0 across all 512 bits of x_reg, x_optimal,
        // core_x_optimal, and reg_x_init.
        // ---------------------------------------------------------------------
        begin
            logic [31:0] state_pats[3] = '{32'h5555_5555, 32'hAAAA_AAAA, 32'h0000_0000};
            `uvm_info(get_type_name(), "Test 15: 16-Variable Full State Vector Bit Toggle Pattern...", UVM_MEDIUM)
            foreach (state_pats[p]) begin
                item = newton_axi_seq_item::type_id::create($sformatf("state_toggle_pat_%0d", p));
                item.num_vars   = 5'd16;
                item.tolerance  = 32'h0001_0000; // 1.0
                item.step_alpha = 32'h0000_0000; // Zero step: x_optimal stays equal to x_init
                item.lambda_reg = 32'h0000_0000;
                item.max_sweeps = 8'd1;
                item.reprogram  = (p == 0) ? 1'b1 : 1'b0;
                if (p == 0) begin
                    // f(x) = x0 + x1 + ... + x15
                    int idx = 0;
                    item.program_mem[idx++] = '{op: OP_ADD, dst: 5'd16, src_a: 5'd0, src_b: 5'd1, imm: 13'sd0};
                    for (int i = 2; i < 16; i++) begin
                        item.program_mem[idx++] = '{op: OP_ADD, dst: (i == 15) ? 5'd31 : 5'd16, src_a: 5'd16, src_b: 5'(i), imm: 13'sd0};
                    end
                    item.program_mem[idx++] = '{op: OP_END, dst: 5'd0, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
                    item.prog_length = idx;
                end
                for (int i = 0; i < 16; i++) begin
                    item.x_init[i] = state_pats[p];
                end
                execute_optimization(item);
            end
        end

        // ---------------------------------------------------------------------
        // Test 16: Divider High Dynamic Range & Upper Bit Toggling (q16_divider.sv)
        // Exercises upper remainder/quotient bits rem_acc[63:54], shifted[63:55],
        // and divisor magnitude bits abs_div[30:23].
        // ---------------------------------------------------------------------
        begin
            `uvm_info(get_type_name(), "Test 16: Divider High Dynamic Range & Upper Bit Toggling...", UVM_MEDIUM)
            item = newton_axi_seq_item::type_id::create("div_hdr_test");
            item.num_vars    = 5'd2;
            item.tolerance   = 32'h0000_0100;
            item.step_alpha  = 32'h0001_0000;
            item.lambda_reg  = 32'h0000_0800;
            item.max_sweeps  = 8'd2;
            item.reprogram   = 1'b1;
            // r16 = 2047.0 (0x07FF_0000)
            item.program_mem[0]  = '{op: OP_LOADC, dst: 5'd16, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd2047};
            // Double r16 to reach 0x7FF0_0000 (large dividend with bit 30 set)
            item.program_mem[1]  = '{op: OP_ADD,   dst: 5'd17, src_a: 5'd16, src_b: 5'd16, imm: 13'sd0};
            item.program_mem[2]  = '{op: OP_ADD,   dst: 5'd18, src_a: 5'd17, src_b: 5'd17, imm: 13'sd0};
            item.program_mem[3]  = '{op: OP_ADD,   dst: 5'd19, src_a: 5'd18, src_b: 5'd18, imm: 13'sd0};
            item.program_mem[4]  = '{op: OP_ADD,   dst: 5'd20, src_a: 5'd19, src_b: 5'd19, imm: 13'sd0};
            // r21 = 2.0
            item.program_mem[5]  = '{op: OP_LOADC, dst: 5'd21, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd2};
            // r22 = r20 / 2.0 (large dividend 0x7FF0_0000 shifts across rem_acc[63:48])
            item.program_mem[6]  = '{op: OP_DIV,   dst: 5'd22, src_a: 5'd20, src_b: 5'd21, imm: 13'sd0};
            // r23 = r20 / r16 (dividing by 2047.0 sets abs_div[26:16])
            item.program_mem[7]  = '{op: OP_DIV,   dst: 5'd23, src_a: 5'd20, src_b: 5'd16, imm: 13'sd0};
            // r24 = r20 / r19 (dividing by 16376.0 sets abs_div[29:16])
            item.program_mem[8]  = '{op: OP_DIV,   dst: 5'd24, src_a: 5'd20, src_b: 5'd19, imm: 13'sd0};
            // r25 = -r19 (negative large divisor)
            item.program_mem[9]  = '{op: OP_NEG,   dst: 5'd25, src_a: 5'd19, src_b: 5'd0,  imm: 13'sd0};
            item.program_mem[10] = '{op: OP_DIV,   dst: 5'd26, src_a: 5'd20, src_b: 5'd25, imm: 13'sd0};
            // Sum results to r31
            item.program_mem[11] = '{op: OP_ADD,   dst: 5'd27, src_a: 5'd22, src_b: 5'd23, imm: 13'sd0};
            item.program_mem[12] = '{op: OP_ADD,   dst: 5'd28, src_a: 5'd24, src_b: 5'd26, imm: 13'sd0};
            item.program_mem[13] = '{op: OP_ADD,   dst: 5'd31, src_a: 5'd27, src_b: 5'd28, imm: 13'sd0};
            item.program_mem[14] = '{op: OP_END,   dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
            item.prog_length     = 15;
            item.x_init[0]   = 32'h0001_0000;
            item.x_init[1]   = 32'h0001_0000;
            execute_optimization(item);
        end

        // ---------------------------------------------------------------------
        // Test 17: Hessian Damping Sweep (lambda_reg) for Cramer's Rule Divider
        // Exercises the full dynamic range of det in newton_2var_core.sv,
        // toggling divisor[30:10] and abs_div[30:10] in u_core.u_div.
        // ---------------------------------------------------------------------
        begin
            q16_t lambda_vals[9] = '{
                32'h0000_C000, // 0.75  -> det ≈ 7.5625 (bits 18:16, 15, 12)
                32'h0000_E000, // 0.875 -> det ≈ 8.2656 (bits 19, 14, 10)
                32'h0002_0000, // 2.0   -> det ≈ 16.0   (bit 20)
                32'h0005_0000, // 5.0   -> det ≈ 49.0   (bits 21, 20, 16)
                32'h000A_0000, // 10.0  -> det ≈ 144.0  (bits 23, 20)
                32'h0014_0000, // 20.0  -> det ≈ 484.0  (bits 24:21, 18, 14)
                32'h0028_0000, // 40.0  -> det ≈ 1764.0 (bits 26:24, 22:21, 18)
                32'h0050_0000, // 80.0  -> det ≈ 6724.0 (bits 28:27, 25, 22)
                32'h00A0_0000  // 160.0 -> det ≈ 26244.0(bits 30:29, 26:25, 23)
            };
            `uvm_info(get_type_name(), "Test 17: Hessian Damping Sweep for u_core.u_div...", UVM_MEDIUM)
            foreach (lambda_vals[lv]) begin
                item = newton_axi_seq_item::type_id::create($sformatf("det_sweep_lam_%0d", lv));
                item.num_vars    = 5'd2;
                item.tolerance   = 32'h0000_0100;
                item.step_alpha  = 32'h0001_0000;
                item.lambda_reg  = lambda_vals[lv];
                item.max_sweeps  = 8'd2;
                item.reprogram   = (lv == 0) ? 1'b1 : 1'b0;
                if (lv == 0) begin
                    build_quad_microcode(item.program_mem, item.prog_length, 2);
                end
                item.x_init[0] = 32'sd3 * 65536;
                item.x_init[1] = 32'sd3 * 65536;
                execute_optimization(item);
            end
        end

        `uvm_info(get_type_name(), "Tapeout-Grade Comprehensive Sequence Completed.", UVM_MEDIUM)
        ping_axi_bus_map();
    endtask

endclass : newton_multivar_tapeout_seq

`endif // NEWTON_MULTIVAR_TAPEOUT_SEQ_SV
