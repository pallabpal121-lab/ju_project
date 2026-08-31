// =============================================================================
// File Name   : newton_random_seq.sv
// Class Name  : newton_random_seq
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// -----------------------------------------------------------------------------
// Generates randomized optimization runs for functional coverage closure
// =============================================================================

`ifndef NEWTON_RANDOM_SEQ_SV
`define NEWTON_RANDOM_SEQ_SV

class newton_random_seq extends newton_base_seq;
    `uvm_object_utils(newton_random_seq)

    rand int num_transactions;
    constraint c_num { num_transactions inside {[10:20]}; }

    function new(string name = "newton_random_seq");
        super.new(name);
        num_transactions = 15;
    endfunction

    virtual task body();
        newton_seq_item item;

        `uvm_info(get_type_name(), $sformatf("Executing Random Sequence (%0d iterations) ...", num_transactions), UVM_LOW)

        for (int i = 0; i < num_transactions; i++) begin
            item = newton_seq_item::type_id::create($sformatf("rnd_item_%0d", i));
            start_item(item);

            if (i == 0) begin
                item.reprogram   = 1'b1;
                item.prog_length = 7;
                item.program_mem[0] = make_instr(OP_MUL,   4'd1,  4'd0, 4'd0, 16'sd0);
                item.program_mem[1] = make_instr(OP_LOADC, 4'd2,  4'd0, 4'd0, -16'sd6);
                item.program_mem[2] = make_instr(OP_MUL,   4'd3,  4'd2, 4'd0, 16'sd0);
                item.program_mem[3] = make_instr(OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd9);
                item.program_mem[4] = make_instr(OP_ADD,   4'd5,  4'd1, 4'd3, 16'sd0);
                item.program_mem[5] = make_instr(OP_ADD,   4'd15, 4'd5, 4'd4, 16'sd0);
                item.program_mem[6] = make_instr(OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);
            end else begin
                item.reprogram   = 1'b0;
                item.prog_length = 0;
            end

            if (!item.randomize()) begin
                `uvm_error(get_type_name(), "Randomization failed!")
            end

            finish_item(item);
        end
    endtask

endclass : newton_random_seq

`endif // NEWTON_RANDOM_SEQ_SV
