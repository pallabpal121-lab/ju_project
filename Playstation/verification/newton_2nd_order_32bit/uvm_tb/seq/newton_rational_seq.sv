// =============================================================================
// File Name   : newton_rational_seq.sv
// Class Name  : newton_rational_seq
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// -----------------------------------------------------------------------------
// Tests DFG iterative division instruction OP_DIV:
// f(x) = (x^2 / 2.0) - x
// Minimum is at x* = 1.0
// =============================================================================

`ifndef NEWTON_RATIONAL_SEQ_SV
`define NEWTON_RATIONAL_SEQ_SV

class newton_rational_seq extends newton_base_seq;
    `uvm_object_utils(newton_rational_seq)

    function new(string name = "newton_rational_seq");
        super.new(name);
    endfunction

    virtual task body();
        newton_seq_item item;

        `uvm_info(get_type_name(), "Executing Rational/Division Sequence: f(x) = x^2/2 - x ...", UVM_LOW)

        item = newton_seq_item::type_id::create("div_item");
        start_item(item);

        item.reprogram   = 1'b1;
        item.prog_length = 5;
        item.program_mem[0] = make_instr(OP_MUL,   4'd1,  4'd0, 4'd0, 16'sd0); // r1 = x^2
        item.program_mem[1] = make_instr(OP_LOADC, 4'd2,  4'd0, 4'd0, 16'sd2); // r2 = 2.0
        item.program_mem[2] = make_instr(OP_DIV,   4'd3,  4'd1, 4'd2, 16'sd0); // r3 = x^2 / 2
        item.program_mem[3] = make_instr(OP_SUB,   4'd15, 4'd3, 4'd0, 16'sd0); // r15 = (x^2/2) - x
        item.program_mem[4] = make_instr(OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);

        item.x_init     = 32'h0005_0000; // x_0 = 5.0
        item.tolerance  = Q16_EPS_DEF;
        item.step_alpha = Q16_ONE;
        item.lambda_reg = Q16_LAMBDA_DEF;
        item.max_iters  = 8'd50;

        finish_item(item);
    endtask

endclass : newton_rational_seq

`endif // NEWTON_RATIONAL_SEQ_SV
