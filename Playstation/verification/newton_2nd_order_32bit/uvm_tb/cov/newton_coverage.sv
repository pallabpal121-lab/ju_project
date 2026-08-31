// =============================================================================
// File Name   : newton_coverage.sv
// Class Name  : newton_coverage
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_COVERAGE_SV
`define NEWTON_COVERAGE_SV

class newton_coverage extends uvm_subscriber #(newton_seq_item);
    `uvm_component_utils(newton_coverage)

    newton_seq_item m_item;

    covergroup cg_solver;
        option.per_instance = 1;
        option.comment      = "Newton Solver Inputs & Outcomes Coverage";

        // 1. Initial Guess Distribution
        cp_x_init: coverpoint m_item.x_init {
            bins neg_large = {[-32'sd3276800 : -32'sd655360]}; // < -10.0
            bins neg_small = {[-32'sd655359  : -32'sd65536]};  // -10.0 to -1.0
            bins zero_near = {[-32'sd65535   :  32'sd65535]};   // -1.0 to +1.0
            bins pos_small = {[ 32'sd65536   :  32'sd655360]};  // +1.0 to +10.0
            bins pos_large = {[ 32'sd655361  :  32'sd3276800]}; // > +10.0
        }

        // 2. Tolerance Thresholds
        cp_tolerance: coverpoint m_item.tolerance {
            bins tol_tight = {[32'h0000_0001 : 32'h0000_003F]};
            bins tol_med   = {[32'h0000_0040 : 32'h0000_0100]};
            bins tol_loose = {[32'h0000_0101 : 32'h0000_1000]};
        }

        // 3. Learning Rate / Step Alpha
        cp_alpha: coverpoint m_item.step_alpha {
            bins full_step = {32'h0001_0000}; // alpha = 1.0
            bins half_step = {32'h0000_8000}; // alpha = 0.5
            bins other_frac= default;
        }

        // 4. Maximum Iteration Limits
        cp_max_iters: coverpoint m_item.max_iters {
            bins it_low  = {[8'd1  : 8'd10]};
            bins it_med  = {[8'd11 : 8'd40]};
            bins it_high = {[8'd41 : 8'd100]};
        }

        // 5. Termination Status
        cp_status: coverpoint m_item.status {
            bins converged   = {STATUS_CONVERGED};
            bins max_reached = {STATUS_MAX_ITERS};
            bins singular    = {STATUS_SINGULAR};
        }

        // 6. Total Iterations to Converge
        cp_iter_count: coverpoint m_item.iter_count {
            bins instant  = {8'd0};
            bins quick    = {[8'd1 : 8'd5]};
            bins moderate = {[8'd6 : 8'd15]};
            bins heavy    = {[8'd16 : 8'd100]};
        }

        cross_status_x: cross cp_status, cp_x_init;
    endgroup

    function new(string name = "newton_coverage", uvm_component parent = null);
        super.new(name, parent);
        cg_solver = new();
    endfunction

    virtual function void write(newton_seq_item t);
        m_item = t;
        cg_solver.sample();
    endfunction

    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info(get_type_name(), $sformatf("Newton Solver Functional Coverage: %0.2f%%", cg_solver.get_coverage()), UVM_NONE)
    endfunction

endclass : newton_coverage

`endif // NEWTON_COVERAGE_SV
