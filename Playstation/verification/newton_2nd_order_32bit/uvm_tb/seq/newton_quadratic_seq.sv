// =============================================================================
// File Name   : newton_quadratic_seq.sv
// Class Name  : newton_quadratic_seq
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// -----------------------------------------------------------------------------
// Tests optimization of f(x) = (x - 3)^2 = x^2 - 6x + 9
// Global minimum is at x* = 3.0, f(x*) = 0.0
// =============================================================================

`ifndef NEWTON_QUADRATIC_SEQ_SV
`define NEWTON_QUADRATIC_SEQ_SV

class newton_quadratic_seq extends newton_base_seq;
    `uvm_object_utils(newton_quadratic_seq)

    function new(string name = "newton_quadratic_seq");
        super.new(name);
    endfunction

    virtual task body();
        newton_seq_item item;
        q16_t test_guesses[5] = '{
            32'h000A_0000, // x_init = +10.0
            32'hFFFC_0000, // x_init = -4.0
            32'h0000_0000, // x_init =  0.0
            32'h0003_0000, // x_init = +3.0 (Immediate convergence)
            32'h0014_0000  // x_init = +20.0
        };

        `uvm_info(get_type_name(), "Executing Quadratic Sequence: f(x) = (x-3)^2 ...", UVM_LOW)

        for (int t = 0; t < 5; t++) begin
            item = newton_seq_item::type_id::create($sformatf("quad_item_%0d", t));
            start_item(item);

            if (t == 0) begin
                item.reprogram   = 1'b1;
                item.prog_length = 7;
                item.program_mem[0] = make_instr(OP_MUL,   4'd1,  4'd0, 4'd0, 16'sd0);  // r1 = x^2
                item.program_mem[1] = make_instr(OP_LOADC, 4'd2,  4'd0, 4'd0, -16'sd6); // r2 = -6.0
                item.program_mem[2] = make_instr(OP_MUL,   4'd3,  4'd2, 4'd0, 16'sd0);  // r3 = -6x
                item.program_mem[3] = make_instr(OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd9);  // r4 = +9.0
                item.program_mem[4] = make_instr(OP_ADD,   4'd5,  4'd1, 4'd3, 16'sd0);  // r5 = x^2 - 6x
                item.program_mem[5] = make_instr(OP_ADD,   4'd15, 4'd5, 4'd4, 16'sd0);  // r15 = x^2 - 6x + 9
                item.program_mem[6] = make_instr(OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);  // Output r15
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

endclass : newton_quadratic_seq

`endif // NEWTON_QUADRATIC_SEQ_SV
