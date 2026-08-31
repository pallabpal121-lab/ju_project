// =============================================================================
// File Name   : newton_corner_seq.sv
// Class Name  : newton_corner_seq
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// -----------------------------------------------------------------------------
// Tests corner conditions:
// 1. Reaching max_iters limit with very tight tolerance and small alpha
// 2. Fractional step alpha (alpha = 0.5)
// =============================================================================

`ifndef NEWTON_CORNER_SEQ_SV
`define NEWTON_CORNER_SEQ_SV

class newton_corner_seq extends newton_base_seq;
    `uvm_object_utils(newton_corner_seq)

    function new(string name = "newton_corner_seq");
        super.new(name);
    endfunction

    virtual task body();
        newton_seq_item item;

        `uvm_info(get_type_name(), "Executing Corner Case Sequence ...", UVM_LOW)

        // Case 1: Max iterations limit hit
        item = newton_seq_item::type_id::create("corner_max_iters");
        start_item(item);
        item.reprogram   = 1'b1;
        item.prog_length = 7;
        item.program_mem[0] = make_instr(OP_MUL,   4'd1,  4'd0, 4'd0, 16'sd0);  // r1 = x^2
        item.program_mem[1] = make_instr(OP_LOADC, 4'd2,  4'd0, 4'd0, -16'sd6); // r2 = -6.0
        item.program_mem[2] = make_instr(OP_MUL,   4'd3,  4'd2, 4'd0, 16'sd0);  // r3 = -6x
        item.program_mem[3] = make_instr(OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd9);  // r4 = +9.0
        item.program_mem[4] = make_instr(OP_ADD,   4'd5,  4'd1, 4'd3, 16'sd0);  // r5 = x^2 - 6x
        item.program_mem[5] = make_instr(OP_ADD,   4'd15, 4'd5, 4'd4, 16'sd0);  // r15 = x^2 - 6x + 9
        item.program_mem[6] = make_instr(OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);

        item.x_init     = 32'h0032_0000; // x_0 = 50.0
        item.tolerance  = 32'h0000_0001; // Ultra tight tolerance
        item.step_alpha = 32'h0000_2000; // Very small step alpha = 0.125
        item.lambda_reg = Q16_LAMBDA_DEF;
        item.max_iters  = 8'd3;          // Max iters = 3 (will trigger STATUS_MAX_ITERS)
        finish_item(item);

        // Case 2: Half step size convergence
        item = newton_seq_item::type_id::create("corner_half_step");
        start_item(item);
        item.reprogram   = 1'b0;
        item.prog_length = 0;
        item.x_init      = 32'h0007_0000; // x_0 = 7.0
        item.tolerance   = Q16_EPS_DEF;
        item.step_alpha  = Q16_HALF;      // alpha = 0.5
        item.lambda_reg  = Q16_LAMBDA_DEF;
        item.max_iters   = 8'd50;
        finish_item(item);
    endtask

endclass : newton_corner_seq

`endif // NEWTON_CORNER_SEQ_SV
