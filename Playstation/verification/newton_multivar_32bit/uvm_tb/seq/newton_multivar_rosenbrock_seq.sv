// =============================================================================
// File Name   : newton_multivar_rosenbrock_seq.sv
// Class Name  : newton_multivar_rosenbrock_seq
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Coupled multi-variable optimization sequence testing off-diagonal
//               Hessian interactions and 2x2 Cramer's rule determinant.
// =============================================================================

`ifndef NEWTON_MULTIVAR_ROSENBROCK_SEQ_SV
`define NEWTON_MULTIVAR_ROSENBROCK_SEQ_SV

class newton_multivar_rosenbrock_seq extends newton_base_seq;
    `uvm_object_utils(newton_multivar_rosenbrock_seq)

    function new(string name = "newton_multivar_rosenbrock_seq");
        super.new(name);
    endfunction

    virtual task body();
        newton_axi_seq_item item;

        `uvm_info(get_type_name(), "Executing Coupled Multivariable Sequence...", UVM_MEDIUM)

        // Coupled equation: f(x0, x1) = (x0 - 1)^2 + (x1 - 1)^2 + (x0 * x1)
        item = newton_axi_seq_item::type_id::create("coupled_item");
        item.num_vars   = 5'd2;
        item.tolerance  = 32'h0000_0080;
        item.step_alpha = 32'h0000_8000; // 0.5 (damped step for coupled convergence)
        item.lambda_reg = 32'h0000_0400; // 0.0156
        item.max_sweeps = 8'd30;
        item.reprogram  = 1'b1;

        // Microcode
        // r16 = 1.0
        item.program_mem[0] = '{op: OP_LOADC, dst: 5'd16, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd1};
        // r17 = x0 - 1.0
        item.program_mem[1] = '{op: OP_SUB,   dst: 5'd17, src_a: 5'd0,  src_b: 5'd16, imm: 13'sd0};
        // r18 = (x0 - 1.0)^2
        item.program_mem[2] = '{op: OP_MUL,   dst: 5'd18, src_a: 5'd17, src_b: 5'd17, imm: 13'sd0};
        // r19 = x1 - 1.0
        item.program_mem[3] = '{op: OP_SUB,   dst: 5'd19, src_a: 5'd1,  src_b: 5'd16, imm: 13'sd0};
        // r20 = (x1 - 1.0)^2
        item.program_mem[4] = '{op: OP_MUL,   dst: 5'd20, src_a: 5'd19, src_b: 5'd19, imm: 13'sd0};
        // r21 = (x0 - 1.0)^2 + (x1 - 1.0)^2
        item.program_mem[5] = '{op: OP_ADD,   dst: 5'd21, src_a: 5'd18, src_b: 5'd20, imm: 13'sd0};
        // r22 = x0 * x1 (coupled cross-term)
        item.program_mem[6] = '{op: OP_MUL,   dst: 5'd22, src_a: 5'd0,  src_b: 5'd1,  imm: 13'sd0};
        // r31 = r21 + r22
        item.program_mem[7] = '{op: OP_ADD,   dst: 5'd31, src_a: 5'd21, src_b: 5'd22, imm: 13'sd0};
        // OP_END
        item.program_mem[8] = '{op: OP_END,   dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        item.prog_length    = 9;

        // Initial guess: x0 = 0.5, x1 = -0.5
        item.x_init[0] = 32'sh0000_8000;
        item.x_init[1] = -32'sh0000_8000;

        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test Case 2: 3-Variable Coupled Quadratic Function with Tight Tolerance
        // f(x0, x1, x2) = x0^2 + x1^2 + x2^2 + (x0 * x1)
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("coupled_3var_item");
        item.num_vars   = 5'd3;
        item.tolerance  = 32'h0000_0030; // tight tolerance bin
        item.step_alpha = 32'h0000_C000; // 0.75 alpha bin
        item.lambda_reg = 32'h0000_0500; // large_damp bin
        item.max_sweeps = 8'd10;         // low sweeps bin
        item.reprogram  = 1'b1;

        // r16 = x0^2
        item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        // r17 = x1^2
        item.program_mem[1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1,  src_b: 5'd1,  imm: 13'sd0};
        // r18 = r16 + r17
        item.program_mem[2] = '{op: OP_ADD, dst: 5'd18, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
        // r19 = x2^2
        item.program_mem[3] = '{op: OP_MUL, dst: 5'd19, src_a: 5'd2,  src_b: 5'd2,  imm: 13'sd0};
        // r20 = r18 + r19
        item.program_mem[4] = '{op: OP_ADD, dst: 5'd20, src_a: 5'd18, src_b: 5'd19, imm: 13'sd0};
        // r21 = x0 * x1 (cross coupling)
        item.program_mem[5] = '{op: OP_MUL, dst: 5'd21, src_a: 5'd0,  src_b: 5'd1,  imm: 13'sd0};
        // r31 = r20 + r21
        item.program_mem[6] = '{op: OP_ADD, dst: 5'd31, src_a: 5'd20, src_b: 5'd21, imm: 13'sd0};
        // OP_END
        item.program_mem[7] = '{op: OP_END, dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        item.prog_length    = 8;

        item.x_init[0] =  32'sh0001_0000; // 1.0
        item.x_init[1] = -32'sh0001_0000; // -1.0
        item.x_init[2] =  32'sh0000_8000; // 0.5

        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test Case 3: 4-Variable Coupled System with Loose Tolerance & Damping
        // f(x0..x3) = x0^2 + x1^2 + x2^2 + x3^2 + (x2 * x3)
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("coupled_4var_item");
        item.num_vars   = 5'd4;
        item.tolerance  = 32'h0000_0150; // loose tolerance bin
        item.step_alpha = 32'h0000_8000; // half step bin (0.5)
        item.lambda_reg = 32'h0000_0400; // default damp bin
        item.max_sweeps = 8'd35;         // high sweeps bin
        item.reprogram  = 1'b1;

        // r16 = x0^2
        item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        // r17 = x1^2
        item.program_mem[1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1,  src_b: 5'd1,  imm: 13'sd0};
        // r18 = r16 + r17
        item.program_mem[2] = '{op: OP_ADD, dst: 5'd18, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
        // r19 = x2^2
        item.program_mem[3] = '{op: OP_MUL, dst: 5'd19, src_a: 5'd2,  src_b: 5'd2,  imm: 13'sd0};
        // r20 = r18 + r19
        item.program_mem[4] = '{op: OP_ADD, dst: 5'd20, src_a: 5'd18, src_b: 5'd19, imm: 13'sd0};
        // r21 = x3^2
        item.program_mem[5] = '{op: OP_MUL, dst: 5'd21, src_a: 5'd3,  src_b: 5'd3,  imm: 13'sd0};
        // r22 = r20 + r21
        item.program_mem[6] = '{op: OP_ADD, dst: 5'd22, src_a: 5'd20, src_b: 5'd21, imm: 13'sd0};
        // r23 = x2 * x3 (cross coupling)
        item.program_mem[7] = '{op: OP_MUL, dst: 5'd23, src_a: 5'd2,  src_b: 5'd3,  imm: 13'sd0};
        // r31 = r22 + r23
        item.program_mem[8] = '{op: OP_ADD, dst: 5'd31, src_a: 5'd22, src_b: 5'd23, imm: 13'sd0};
        // OP_END
        item.program_mem[9] = '{op: OP_END, dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        item.prog_length    = 10;

        item.x_init[0] =  32'sh0001_0000;
        item.x_init[1] = -32'sh0001_0000;
        item.x_init[2] =  32'sh0000_8000;
        item.x_init[3] = -32'sh0000_8000;

        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test Case 4: Coupled System Max Sweeps Limit (quarter_step alpha)
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("coupled_max_sw_item");
        item.num_vars   = 5'd2;
        item.tolerance  = 32'h0000_0001; // Ultra-tight -> forces STATUS_MAX_ITERS
        item.step_alpha = 32'h0000_4000; // quarter_step bin (0.25)
        item.lambda_reg = 32'h0000_0800; // large_damp bin
        item.max_sweeps = 8'd1;          // 1 sweep
        item.reprogram  = 1'b1;

        // r16 = 1.0, r17 = x0 - 1, r18 = (x0-1)^2, r19 = x1 - 1, r20 = (x1-1)^2, r21 = r18+r20, r22 = x0*x1, r31 = r21+r22
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

        item.x_init[0] = 32'sh0002_0000;
        item.x_init[1] = -32'sh0002_0000;

        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test Case 5: 8-Variable Coupled System with Small Damping & Full Step
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("coupled_8var_item");
        item.num_vars   = 5'd8;
        item.tolerance  = 32'h0000_0080;
        item.step_alpha = 32'h0001_0000; // full step bin
        item.lambda_reg = 32'h0000_0200; // small damp bin
        item.max_sweeps = 8'd20;         // med sweeps bin
        item.reprogram  = 1'b1;
        item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        for (int i = 1; i < 8; i++) begin
            item.program_mem[i*2 - 1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'(i), src_b: 5'(i), imm: 13'sd0};
            item.program_mem[i*2]     = '{op: OP_ADD, dst: 5'd16, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
        end
        item.program_mem[15] = '{op: OP_MUL, dst: 5'd18, src_a: 5'd0, src_b: 5'd1, imm: 13'sd0};
        item.program_mem[16] = '{op: OP_ADD, dst: 5'd31, src_a: 5'd16, src_b: 5'd18, imm: 13'sd0};
        item.program_mem[17] = '{op: OP_END, dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        item.prog_length     = 18;
        for (int i = 0; i < 8; i++) item.x_init[i] = 32'sh0001_0000;
        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test Case 6: 16-Variable System to cover n16 bin
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("coupled_16var_item");
        item.num_vars   = 5'd16;
        item.tolerance  = 32'h0000_0080;
        item.step_alpha = 32'h0001_0000;
        item.lambda_reg = 32'h0000_0400;
        item.max_sweeps = 8'd25;
        item.reprogram  = 1'b1;
        build_quad_microcode(item.program_mem, item.prog_length, 16);
        for (int i = 0; i < 16; i++) item.x_init[i] = ((i % 2 == 0) ? 32'sh0001_0000 : -32'sh0001_0000);
        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Test Case 7: Zero Fallback Coupled System (tol=0, alpha=0, lambda=0, max_sweeps=0)
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("coupled_fallback_item");
        item.num_vars   = 5'd2;
        item.tolerance  = 32'h0000_0000;
        item.step_alpha = 32'h0000_0000;
        item.lambda_reg = 32'h0000_0000;
        item.max_sweeps = 8'd0;
        item.reprogram  = 1'b0;
        item.x_init[0]  = 32'sh0001_0000;
        item.x_init[1]  = 32'sh0001_0000;
        execute_optimization(item);

        ping_axi_bus_map();
    endtask

endclass : newton_multivar_rosenbrock_seq

`endif // NEWTON_MULTIVAR_ROSENBROCK_SEQ_SV
