// =============================================================================
// File Name   : newton_multivar_random_seq.sv
// Class Name  : newton_multivar_random_seq
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Constrained-random regression sequence testing randomized
//               initial vectors, dimensions, and solver hyperparameters.
// =============================================================================

`ifndef NEWTON_MULTIVAR_RANDOM_SEQ_SV
`define NEWTON_MULTIVAR_RANDOM_SEQ_SV

class newton_multivar_random_seq extends newton_base_seq;
    `uvm_object_utils(newton_multivar_random_seq)

    rand int num_iterations;

    constraint c_iter {
        num_iterations inside {[5 : 10]};
    }

    function new(string name = "newton_multivar_random_seq");
        super.new(name);
        num_iterations = 5;
    endfunction

    virtual task body();
        newton_axi_seq_item item;

        `uvm_info(get_type_name(), $sformatf("Executing %0d Constrained-Random Iterations...", num_iterations), UVM_MEDIUM)

        for (int iter = 0; iter < num_iterations; iter++) begin
            item = newton_axi_seq_item::type_id::create($sformatf("rand_item_%0d", iter));

            if (!item.randomize() with {
                num_vars inside {[5'd2 : 5'd6]};
                max_sweeps inside {[8'd10 : 8'd30]};
            }) begin
                `uvm_fatal("RAND_FAIL", "Failed to randomize newton_axi_seq_item in random_seq!")
            end

            item.reprogram = (iter == 0); // Program microcode on first iteration

            if (iter == 0) begin
                // Quadratic bowl for N variables
                item.program_mem[0] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
                for (int i = 1; i < 6; i++) begin
                    item.program_mem[i*2 - 1] = '{op: OP_MUL, dst: 5'd17, src_a: 5'(i), src_b: 5'(i), imm: 13'sd0};
                    item.program_mem[i*2]     = '{op: OP_ADD, dst: (i == 5) ? 5'd31 : 5'd16, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
                end
                item.program_mem[11] = '{op: OP_END, dst: 5'd0, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
                item.prog_length     = 12;
            end

            `uvm_info(get_type_name(), $sformatf("Launching Iteration #%0d with N=%0d variables...", iter+1, item.num_vars), UVM_HIGH)
            execute_optimization(item);
        end
    endtask

endclass : newton_multivar_random_seq

`endif // NEWTON_MULTIVAR_RANDOM_SEQ_SV
