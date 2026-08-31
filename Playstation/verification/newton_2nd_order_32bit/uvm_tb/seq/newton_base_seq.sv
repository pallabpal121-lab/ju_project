// =============================================================================
// File Name   : newton_base_seq.sv
// Class Name  : newton_base_seq
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_BASE_SEQ_SV
`define NEWTON_BASE_SEQ_SV

class newton_base_seq extends uvm_sequence #(newton_seq_item);
    `uvm_object_utils(newton_base_seq)

    function new(string name = "newton_base_seq");
        super.new(name);
    endfunction

    function instr_t make_instr(opcode_t op, logic [3:0] dst, logic [3:0] src_a, logic [3:0] src_b, logic signed [15:0] imm);
        instr_t instr;
        instr.op    = op;
        instr.dst   = dst;
        instr.src_a = src_a;
        instr.src_b = src_b;
        instr.imm   = imm;
        return instr;
    endfunction

endclass : newton_base_seq

`endif // NEWTON_BASE_SEQ_SV
