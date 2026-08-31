// =============================================================================
// File Name   : newton_cubic_seq.sv
// Class Name  : newton_cubic_seq
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// -----------------------------------------------------------------------------
// Tests optimization of f(x) = x^3 - 3x
// Extrema are at x* = +1.0 (local min) and x* = -1.0 (local max)
// =============================================================================

`ifndef NEWTON_CUBIC_SEQ_SV
`define NEWTON_CUBIC_SEQ_SV

class newton_cubic_seq extends newton_base_seq;
    `uvm_object_utils(newton_cubic_seq)

    function new(string name = "newton_cubic_seq");
        super.new(name);
    endfunction

    virtual task body();
        newton_seq_item item;
        q16_t test_guesses[3] = '{
            32'h0002_8000, // x_init = +2.5 -> reaches x* = +1.0
            32'hFFFD_8000, // x_init = -2.5 -> reaches x* = -1.0
            32'h0001_0000  // x_init = +1.0 -> immediate convergence
        };

        `uvm_info(get_type_name(), "Executing Cubic Sequence: f(x) = x^3 - 3x ...", UVM_LOW)

        for (int t = 0; t < 3; t++) begin
            item = newton_seq_item::type_id::create($sformatf("cubic_item_%0d", t));
            start_item(item);

            if (t == 0) begin
                item.reprogram   = 1'b1;
                item.prog_length = 6;
                item.program_mem[0] = make_instr(OP_MUL,   4'd1,  4'd0, 4'd0, 16'sd0);  // r1 = x^2
                item.program_mem[1] = make_instr(OP_MUL,   4'd2,  4'd1, 4'd0, 16'sd0);  // r2 = x^3
                item.program_mem[2] = make_instr(OP_LOADC, 4'd3,  4'd0, 4'd0, -16'sd3); // r3 = -3.0
                item.program_mem[3] = make_instr(OP_MUL,   4'd4,  4'd3, 4'd0, 16'sd0);  // r4 = -3x
                item.program_mem[4] = make_instr(OP_ADD,   4'd15, 4'd2, 4'd4, 16'sd0);  // r15 = x^3 - 3x
                item.program_mem[5] = make_instr(OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);
            end else begin
                item.reprogram   = 1'b0;
                item.prog_length = 0;
            end

            item.x_init     = test_guesses[t];
            item.tolerance  = Q16_EPS_DEF;
            item.step_alpha = Q16_ONE;
            item.lambda_reg = Q16_LAMBDA_DEF;
            item.max_iters  = 8'd50;

            finish_item(item);
        end
    endtask

endclass : newton_cubic_seq

`endif // NEWTON_CUBIC_SEQ_SV
