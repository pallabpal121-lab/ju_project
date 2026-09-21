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
    endtask

endclass : newton_multivar_rosenbrock_seq

`endif // NEWTON_MULTIVAR_ROSENBROCK_SEQ_SV
