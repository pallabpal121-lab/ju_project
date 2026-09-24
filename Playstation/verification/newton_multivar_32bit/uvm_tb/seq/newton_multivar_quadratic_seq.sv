// =============================================================================
// File Name   : newton_multivar_quadratic_seq.sv
// Class Name  : newton_multivar_quadratic_seq
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Multivariable quadratic bowl optimization sequence (N=2, 4, 8).
//               f(x) = sum(x_i^2), optimal x* = [0, 0, ... 0].
// =============================================================================

`ifndef NEWTON_MULTIVAR_QUADRATIC_SEQ_SV
`define NEWTON_MULTIVAR_QUADRATIC_SEQ_SV

class newton_multivar_quadratic_seq extends newton_base_seq;
    `uvm_object_utils(newton_multivar_quadratic_seq)

    function new(string name = "newton_multivar_quadratic_seq");
        super.new(name);
    endfunction

    virtual task body();
        newton_axi_seq_item item;

        `uvm_info(get_type_name(), "Executing Multivariable Quadratic Sequence...", UVM_MEDIUM)

        // ---------------------------------------------------------------------
        // Test Case 1: 2-Variable Quadratic Bowl: f(x0, x1) = x0^2 + x1^2
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("quad_2var_item");
        item.num_vars   = 5'd2;
        item.tolerance  = 32'h0000_0080; // ~0.00195
        item.step_alpha = 32'h0001_0000; // 1.0
        item.lambda_reg = 32'h0000_0400; // 0.0156
        item.max_sweeps = 8'd20;
        item.reprogram  = 1'b1;

        // Microcode:
        // r16 = x0 * x0
        item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        // r17 = x1 * x1
        item.program_mem[1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1, src_b: 5'd1, imm: 13'sd0};
        // r31 = r16 + r17
        item.program_mem[2] = '{op: OP_ADD, dst: 5'd31, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
        // OP_END
        item.program_mem[3] = '{op: OP_END, dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        item.prog_length    = 4;

        // Initial guess: x0 = 3.0, x1 = -2.5
        item.x_init[0] = 32'sd3 * 65536;
        item.x_init[1] = -32'sd163840; // -2.5

        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test Case 2: 4-Variable Quadratic: f(x0..x3) = x0^2 + x1^2 + x2^2 + x3^2
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("quad_4var_item");
        item.num_vars   = 5'd4;
        item.tolerance  = 32'h0000_0080;
        item.step_alpha = 32'h0001_0000;
        item.lambda_reg = 32'h0000_0400;
        item.max_sweeps = 8'd20;
        item.reprogram  = 1'b1;

        // r16 = x0 * x0
        item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        // r17 = x1 * x1
        item.program_mem[1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1, src_b: 5'd1, imm: 13'sd0};
        // r18 = r16 + r17
        item.program_mem[2] = '{op: OP_ADD, dst: 5'd18, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
        // r19 = x2 * x2
        item.program_mem[3] = '{op: OP_MUL, dst: 5'd19, src_a: 5'd2, src_b: 5'd2, imm: 13'sd0};
        // r20 = r18 + r19
        item.program_mem[4] = '{op: OP_ADD, dst: 5'd20, src_a: 5'd18, src_b: 5'd19, imm: 13'sd0};
        // r21 = x3 * x3
        item.program_mem[5] = '{op: OP_MUL, dst: 5'd21, src_a: 5'd3, src_b: 5'd3, imm: 13'sd0};
        // r31 = r20 + r21
        item.program_mem[6] = '{op: OP_ADD, dst: 5'd31, src_a: 5'd20, src_b: 5'd21, imm: 13'sd0};
        // OP_END
        item.program_mem[7] = '{op: OP_END, dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        item.prog_length    = 8;

        item.x_init[0] =  32'sd2 * 65536; // +2.0
        item.x_init[1] = -32'sd1 * 65536; // -1.0
        item.x_init[2] =  32'sd1 * 65536; // +1.0
        item.x_init[3] = -32'sd2 * 65536; // -2.0

        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test Case 3: 8-Variable Quadratic with Damped Alpha & Tight Tolerance
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("quad_8var_item");
        item.num_vars   = 5'd8;
        item.tolerance  = 32'h0000_0030; // tight tolerance bin
        item.step_alpha = 32'h0000_C000; // 0.75 alpha bin
        item.lambda_reg = 32'h0000_0200; // small_damp bin
        item.max_sweeps = 8'd25;         // med_sweeps bin
        item.reprogram  = 1'b1;

        item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        for (int i = 1; i < 8; i++) begin
            item.program_mem[i*2 - 1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'(i), src_b: 5'(i), imm: 13'sd0};
            item.program_mem[i*2]     = '{op: OP_ADD, dst: (i == 7) ? 5'd31 : 5'd16, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
        end
        item.program_mem[15] = '{op: OP_END, dst: 5'd0, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        item.prog_length     = 16;

        for (int i = 0; i < 8; i++) begin
            item.x_init[i] = ((i % 2 == 0) ? 32'sd1 : -32'sd1) * 49152; // +/- 0.75
        end
        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test Case 4: 16-Variable Full Capacity with Half-Step & Loose Tolerance
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("quad_16var_item");
        item.num_vars   = 5'd16;
        item.tolerance  = 32'h0000_0200; // loose tolerance bin
        item.step_alpha = 32'h0000_8000; // half_step bin (0.5)
        item.lambda_reg = 32'h0000_0800; // large_damp bin
        item.max_sweeps = 8'd35;         // high sweeps bin
        item.reprogram  = 1'b1;

        item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        for (int i = 1; i < 16; i++) begin
            item.program_mem[i*2 - 1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'(i), src_b: 5'(i), imm: 13'sd0};
            item.program_mem[i*2]     = '{op: OP_ADD, dst: (i == 15) ? 5'd31 : 5'd16, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
        end
        item.program_mem[31] = '{op: OP_END, dst: 5'd0, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        item.prog_length     = 32;

        for (int i = 0; i < 16; i++) begin
            item.x_init[i] = ((i % 2 == 0) ? 32'sd1 : -32'sd1) * 32768; // +/- 0.5
        end
        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test Case 5: Forced Max Sweeps Limit (N=2, quarter_step alpha)
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("quad_max_sw_item");
        item.num_vars   = 5'd2;
        item.tolerance  = 32'h0000_0001; // Ultra-tight -> forces STATUS_MAX_ITERS
        item.step_alpha = 32'h0000_4000; // quarter_step bin (0.25)
        item.lambda_reg = 32'h0000_0400; // default_damp bin
        item.max_sweeps = 8'd1;          // low sweeps (1 sweep)
        item.reprogram  = 1'b1;

        item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        item.program_mem[1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1, src_b: 5'd1, imm: 13'sd0};
        item.program_mem[2] = '{op: OP_ADD, dst: 5'd31, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
        item.program_mem[3] = '{op: OP_END, dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        item.prog_length    = 4;
        item.x_init[0] = 32'sd4 * 65536;
        item.x_init[1] = 32'sd4 * 65536;
        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test Case 6: Fallback Zero Defaults Check (tol=0, alpha=0, lambda=0, sw=0)
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("quad_zero_fallbacks_item");
        item.num_vars   = 5'd2;
        item.tolerance  = 32'h0000_0000; // zero_fallback bin
        item.step_alpha = 32'h0000_0000; // zero_fallback bin
        item.lambda_reg = 32'h0000_0000; // zero_fallback bin
        item.max_sweeps = 8'd0;          // zero_fallback bin
        item.reprogram  = 1'b0;
        item.x_init[0] = 32'sd1 * 65536;
        item.x_init[1] = 32'sd1 * 65536;
        execute_optimization(item);

        ping_axi_bus_map();
    endtask

endclass : newton_multivar_quadratic_seq

`endif // NEWTON_MULTIVAR_QUADRATIC_SEQ_SV
