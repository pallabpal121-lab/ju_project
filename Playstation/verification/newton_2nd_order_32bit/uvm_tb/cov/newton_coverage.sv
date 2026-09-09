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

    // -------------------------------------------------------------------------
    // Covergroup 1: Optimization Solver Inputs, Controls & Outcomes
    // -------------------------------------------------------------------------
    covergroup cg_solver;
        option.per_instance = 1;
        option.comment      = "Newton Solver Inputs & Outcomes Coverage";

        // 1. Initial Guess Distribution (including exact corner values)
        cp_x_init: coverpoint m_item.x_init {
            bins exact_zero = {32'sd0};
            bins neg_sat    = {-32'sh8000_0000};
            bins pos_sat    = {32'sh7FFF_FFFF};
            bins neg_large  = {[-32'sd3276800 : -32'sd655360]}; // < -10.0
            bins neg_small  = {[-32'sd655359  : -32'sd65536]};  // -10.0 to -1.0
            bins zero_near  = {[-32'sd65535   : -32'sd1], [32'sd1 : 32'sd65535]}; // (-1.0 to +1.0) excl 0
            bins pos_small  = {[ 32'sd65536   :  32'sd655360]};  // +1.0 to +10.0
            bins pos_large  = {[ 32'sd655361  :  32'sd3276800]}; // > +10.0
        }

        // 2. Tolerance Thresholds (including fallback zero)
        cp_tolerance: coverpoint m_item.tolerance {
            bins tol_zero  = {32'h0000_0000};                   // Fallback to Q16_EPS_DEF
            bins tol_tight = {[32'h0000_0001 : 32'h0000_003F]}; // Tight (< 0.001)
            bins tol_med   = {[32'h0000_0040 : 32'h0000_0100]}; // Medium (~0.001 to 0.004)
            bins tol_loose = {[32'h0000_0101 : 32'h0000_1000]}; // Loose (~0.004 to 0.0625)
        }

        // 3. Learning Rate / Step Alpha (including fallback zero)
        cp_alpha: coverpoint m_item.step_alpha {
            bins alpha_zero = {32'h0000_0000}; // Fallback to Q16_ONE (1.0)
            bins full_step  = {32'h0001_0000}; // alpha = 1.0
            bins three_qtr  = {32'h0000_C000}; // alpha = 0.75
            bins half_step  = {32'h0000_8000}; // alpha = 0.5
            bins qtr_step   = {32'h0000_4000}; // alpha = 0.25
            bins eighth_step= {32'h0000_2000}; // alpha = 0.125
            bins other_frac = default;
        }

        // 4. Regularizer / Denominator Clamping Lambda (including fallback zero)
        cp_lambda: coverpoint m_item.lambda_reg {
            bins lam_zero  = {32'h0000_0000};                   // Fallback to Q16_LAMBDA_DEF
            bins lam_tight = {[32'h0000_0001 : 32'h0000_007F]}; // Tight clamp (< 0.002)
            bins lam_def   = {32'h0000_0100};                   // Default lambda (0.0039)
            bins lam_med   = {[32'h0000_0101 : 32'h0000_0200]}; // Medium clamp (0.004 - 0.008)
            bins lam_high  = {[32'h0000_0201 : 32'h0000_1000]}; // Large clamp (0.008 - 0.0625)
        }

        // 5. Maximum Iteration Limits (including fallback zero)
        cp_max_iters: coverpoint m_item.max_iters {
            bins it_zero = {8'd0};             // Fallback to 8'd50
            bins it_low  = {[8'd1  : 8'd10]};  // Quick limit
            bins it_med  = {[8'd11 : 8'd40]};  // Moderate limit
            bins it_high = {[8'd41 : 8'd100]}; // Stress limit
        }

        // 6. Termination Status
        cp_status: coverpoint m_item.status {
            bins converged   = {STATUS_CONVERGED};
            bins max_reached = {STATUS_MAX_ITERS};
            bins singular    = {STATUS_SINGULAR};
        }

        // 7. Total Iterations to Converge
        cp_iter_count: coverpoint m_item.iter_count {
            bins instant  = {8'd0};
            bins quick    = {[8'd1  : 8'd5]};
            bins moderate = {[8'd6  : 8'd15]};
            bins heavy    = {[8'd16 : 8'd100]};
        }

        // 8. Optimal Output Values (Corner and saturation checking)
        cp_x_opt: coverpoint m_item.x_optimal {
            bins opt_zero   = {32'sd0};
            bins opt_sat_pos= {32'sh7FFF_FFFF};
            bins opt_sat_neg= {-32'sh8000_0000};
            bins opt_pos    = {[ 32'sd1 : 32'sh7FFE_FFFF]};
            bins opt_neg    = {[-32'sh7FFF_FFFF : -32'sd1]};
        }

        // Multi-Dimensional Cross Coverage
        cross_status_x:      cross cp_status, cp_x_init;
        cross_status_alpha:  cross cp_status, cp_alpha;
        cross_status_tol:    cross cp_status, cp_tolerance;
        cross_status_lambda: cross cp_status, cp_lambda;
        cross_status_iters:  cross cp_status, cp_max_iters;
    endgroup

    // -------------------------------------------------------------------------
    // Covergroup 2: Microcode Instruction Opcodes & Register Usage
    // -------------------------------------------------------------------------
    covergroup cg_microcode with function sample(opcode_t op, logic [3:0] dst, logic [3:0] src_a, logic [3:0] src_b);
        option.per_instance = 1;
        option.comment      = "Microcode ALU Operations & Register Allocation";

        cp_op: coverpoint op {
            bins op_nop   = {OP_NOP};
            bins op_add   = {OP_ADD};
            bins op_sub   = {OP_SUB};
            bins op_mul   = {OP_MUL};
            bins op_div   = {OP_DIV};
            bins op_neg   = {OP_NEG};
            bins op_mov   = {OP_MOV};
            bins op_loadc = {OP_LOADC};
            bins op_end   = {OP_END};
        }

        cp_dst: coverpoint dst {
            bins r_inputs = {[4'd0 : 4'd2]};
            bins r_temps  = {[4'd3 : 4'd14]};
            bins r_result = {REG_RESULT}; // 15
        }

        cross_op_dst: cross cp_op, cp_dst;
    endgroup

    function new(string name = "newton_coverage", uvm_component parent = null);
        super.new(name, parent);
        cg_solver    = new();
        cg_microcode = new();
    endfunction

    virtual function void write(newton_seq_item t);
        m_item = t;
        cg_solver.sample();

        if (t.reprogram || t.prog_length > 0) begin
            for (int i = 0; i < t.prog_length; i++) begin
                cg_microcode.sample(
                    t.program_mem[i].op,
                    t.program_mem[i].dst,
                    t.program_mem[i].src_a,
                    t.program_mem[i].src_b
                );
            end
        end
    endfunction

    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info(get_type_name(), "==================================================", UVM_NONE)
        `uvm_info(get_type_name(), "  FUNCTIONAL COVERAGE REPORT:", UVM_NONE)
        `uvm_info(get_type_name(), $sformatf("  Functional Coverage: %0.2f%%", cg_solver.get_coverage()), UVM_NONE)
        `uvm_info(get_type_name(), $sformatf("  Microcode Coverage : %0.2f%%", cg_microcode.get_coverage()), UVM_NONE)
        `uvm_info(get_type_name(), "==================================================", UVM_NONE)
    endfunction

endclass : newton_coverage

`endif // NEWTON_COVERAGE_SV
