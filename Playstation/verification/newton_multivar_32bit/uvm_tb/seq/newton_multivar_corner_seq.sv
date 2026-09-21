// =============================================================================
// File Name   : newton_multivar_corner_seq.sv
// Class Name  : newton_multivar_corner_seq
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Corner case and boundary value verification sequence.
//               Tests N=2, N=16, zero fallbacks, max_sweeps=1, and zero guess.
// =============================================================================

`ifndef NEWTON_MULTIVAR_CORNER_SEQ_SV
`define NEWTON_MULTIVAR_CORNER_SEQ_SV

class newton_multivar_corner_seq extends newton_base_seq;
    `uvm_object_utils(newton_multivar_corner_seq)

    function new(string name = "newton_multivar_corner_seq");
        super.new(name);
    endfunction

    virtual task body();
        newton_axi_seq_item item;

        `uvm_info(get_type_name(), "Executing Corner Cases & Boundary Conditions Sequence...", UVM_MEDIUM)

        // ---------------------------------------------------------------------
        // Case 1: Zero Fallbacks Test (tol=0, alpha=0, lambda=0, max_sweeps=0)
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("corner_fallbacks_item");
        item.num_vars   = 5'd2;
        item.tolerance  = 32'h0000_0000; // Exercises fallback to Q16_EPS_DEF
        item.step_alpha = 32'h0000_0000; // Exercises fallback to Q16_ONE
        item.lambda_reg = 32'h0000_0000; // Exercises fallback to Q16_LAMBDA_DEF
        item.max_sweeps = 8'd0;          // Exercises fallback to 8'd50
        item.reprogram  = 1'b1;

        // Quadratic bowl microcode: f(x0, x1) = x0^2 + x1^2
        item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        item.program_mem[1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1, src_b: 5'd1, imm: 13'sd0};
        item.program_mem[2] = '{op: OP_ADD, dst: 5'd31, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
        item.program_mem[3] = '{op: OP_END, dst: 5'd0,  src_a: 5'd0,  src_b: 5'd0,  imm: 13'sd0};
        item.prog_length    = 4;

        item.x_init[0] = 32'sd1 * 65536;
        item.x_init[1] = 32'sd1 * 65536;

        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Case 2: Max Sweeps Limit = 1 (Forces STATUS_MAX_ITERS if not converged)
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("corner_max_sw_1_item");
        item.num_vars   = 5'd2;
        item.tolerance  = 32'h0000_0001; // Ultra-tight: impossible in 1 sweep
        item.step_alpha = 32'h0000_4000; // 0.25 small step
        item.lambda_reg = 32'h0000_0400;
        item.max_sweeps = 8'd1;          // 1 sweep limit
        item.reprogram  = 1'b0;

        item.x_init[0] = 32'sd4 * 65536;
        item.x_init[1] = 32'sd4 * 65536;

        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Case 3: Already at Minimum (Initial Guess = 0.0) -> Instant Convergence
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("corner_already_zero_item");
        item.num_vars   = 5'd2;
        item.tolerance  = 32'h0000_0080;
        item.step_alpha = 32'h0001_0000;
        item.lambda_reg = 32'h0000_0400;
        item.max_sweeps = 8'd20;
        item.reprogram  = 1'b0;

        item.x_init[0] = 32'sd0;
        item.x_init[1] = 32'sd0;

        execute_optimization(item);

        // ---------------------------------------------------------------------
        // Case 4: Maximum Variables (N = 16)
        // ---------------------------------------------------------------------
        item = newton_axi_seq_item::type_id::create("corner_16var_item");
        item.num_vars   = 5'd16;
        item.tolerance  = 32'h0000_0080;
        item.step_alpha = 32'h0001_0000;
        item.lambda_reg = 32'h0000_0400;
        item.max_sweeps = 8'd20;
        item.reprogram  = 1'b1;

        // Sum of squares for 16 variables: f(x0..x15) = sum(x_i^2)
        // We write instructions dynamically:
        // r16 = x0^2
        item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        for (int i = 1; i < 16; i++) begin
            // r17 = x_i^2
            item.program_mem[i*2 - 1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'(i), src_b: 5'(i), imm: 13'sd0};
            // r16 = r16 + r17
            item.program_mem[i*2]     = '{op: OP_ADD, dst: (i == 15) ? 5'd31 : 5'd16, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
        end
        item.program_mem[31] = '{op: OP_END, dst: 5'd0, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        item.prog_length     = 32;

        for (int i = 0; i < 16; i++) begin
            item.x_init[i] = ((i % 2 == 0) ? 32'sd1 : -32'sd1) * 32768; // +/- 0.5
        end

        execute_optimization(item);
    endtask

endclass : newton_multivar_corner_seq

`endif // NEWTON_MULTIVAR_CORNER_SEQ_SV
