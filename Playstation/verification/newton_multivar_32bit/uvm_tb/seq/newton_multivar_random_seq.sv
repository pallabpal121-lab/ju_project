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
        num_iterations inside {[20 : 30]};
    }

    function new(string name = "newton_multivar_random_seq");
        super.new(name);
        num_iterations = 24;
    endfunction

    virtual task body();
        newton_axi_seq_item item;

        `uvm_info(get_type_name(), $sformatf("Executing %0d Constrained-Random Iterations...", num_iterations), UVM_MEDIUM)

        for (int iter = 0; iter < num_iterations; iter++) begin
            item = newton_axi_seq_item::type_id::create($sformatf("rand_item_%0d", iter));

            if (!item.randomize() with {
                num_vars   inside {5'd2, 5'd4, 5'd8, 5'd12, 5'd16};
                tolerance  inside {32'h0000_0000, 32'h0000_0020, 32'h0000_0080, 32'h0000_0200};
                step_alpha inside {32'h0000_0000, 32'h0001_0000, 32'h0000_C000, 32'h0000_8000, 32'h0000_4000};
                lambda_reg inside {32'h0000_0000, 32'h0000_0200, 32'h0000_0400, 32'h0000_0800};
                max_sweeps inside {8'd0, [8'd5 : 8'd45]};
            }) begin
                `uvm_fatal("RAND_FAIL", "Failed to randomize newton_axi_seq_item in random_seq!")
            end

            if (iter % 4 == 0) begin
                // Target STATUS_MAX_ITERS across different variable counts to hit cx_vars_status
                item.tolerance  = 32'h0000_0001;
                item.max_sweeps = 8'd1;
                item.step_alpha = 32'h0000_4000;
            end else if (iter % 4 == 1) begin
                // Target zero fallbacks (hardware falls back to internal default parameters)
                item.tolerance  = 32'h0000_0000;
                item.step_alpha = 32'h0000_0000;
                item.lambda_reg = 32'h0000_0000;
                item.max_sweeps = 8'd0;
            end

            // Build exact microcode for this specific variable count
            build_quad_microcode(item.program_mem, item.prog_length, item.num_vars);
            item.reprogram = 1'b1;

            `uvm_info(get_type_name(), $sformatf("Launching Iteration #%0d with N=%0d variables...", iter+1, item.num_vars), UVM_HIGH)
            execute_optimization(item);
        end

        ping_axi_bus_map();
    endtask

endclass : newton_multivar_random_seq

`endif // NEWTON_MULTIVAR_RANDOM_SEQ_SV
