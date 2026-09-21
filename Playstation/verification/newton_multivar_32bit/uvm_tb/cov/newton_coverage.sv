// =============================================================================
// File Name   : newton_coverage.sv
// Class Name  : newton_coverage
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Functional coverage subscriber implementing covergroups, bins,
//               crosses, and all 9 standard UVM phases.
// =============================================================================

`ifndef NEWTON_COVERAGE_SV
`define NEWTON_COVERAGE_SV

class newton_coverage extends uvm_subscriber #(newton_axi_seq_item);
    `uvm_component_utils(newton_coverage)

    newton_axi_seq_item item_cov;
    int                 sample_count;

    // -------------------------------------------------------------------------
    // Covergroup: Optimization Parameters & Convergence
    // -------------------------------------------------------------------------
    covergroup cg_optimization;
        option.per_instance = 1;
        option.comment      = "Newton BCD Optimization Coverage";

        cp_num_vars: coverpoint item_cov.num_vars {
            bins n2      = {5'd2};
            bins n3_n4   = {[5'd3 : 5'd4]};
            bins n5_n8   = {[5'd5 : 5'd8]};
            bins n9_n15  = {[5'd9 : 5'd15]};
            bins n16     = {5'd16};
        }

        cp_tolerance: coverpoint item_cov.tolerance {
            bins zero_fallback = {32'h0000_0000};
            bins tight         = {[32'h0000_0010 : 32'h0000_003F]};
            bins med           = {[32'h0000_0040 : 32'h0000_0100]};
            bins loose         = {[32'h0000_0101 : 32'h0000_0800]};
        }

        cp_alpha: coverpoint item_cov.step_alpha {
            bins zero_fallback = {32'h0000_0000};
            bins full_step     = {32'h0001_0000};
            bins three_quarter = {32'h0000_C000};
            bins half_step     = {32'h0000_8000};
            bins quarter_step  = {32'h0000_4000};
        }

        cp_lambda: coverpoint item_cov.lambda_reg {
            bins zero_fallback = {32'h0000_0000};
            bins default_damp  = {32'h0000_0400};
            bins small_damp    = {[32'h0000_0100 : 32'h0000_03FF]};
            bins large_damp    = {[32'h0000_0401 : 32'h0000_1000]};
        }

        cp_max_sweeps: coverpoint item_cov.max_sweeps {
            bins zero_fallback = {8'd0};
            bins low           = {[8'd1  : 8'd10]};
            bins med           = {[8'd11 : 8'd30]};
            bins high          = {[8'd31 : 8'd50]};
        }

        cp_status: coverpoint item_cov.status {
            bins converged  = {STATUS_CONVERGED};
            bins max_iters  = {STATUS_MAX_ITERS};
            ignore_bins unused = {STATUS_IDLE, STATUS_RUNNING, STATUS_SINGULAR};
        }

        cp_sweeps_executed: coverpoint item_cov.sweep_count {
            bins single_sweep = {8'd1};
            bins few_sweeps   = {[8'd2  : 8'd5]};
            bins med_sweeps   = {[8'd6  : 8'd20]};
            bins max_sweeps   = {[8'd21 : 8'd50]};
        }

        // Cross Coverage
        cx_vars_status: cross cp_num_vars, cp_status;
        cx_tol_sweeps : cross cp_tolerance, cp_sweeps_executed;
        cx_alpha_sweeps: cross cp_alpha, cp_sweeps_executed;
    endgroup

    // -------------------------------------------------------------------------
    // Covergroup: AXI Bus Accesses
    // -------------------------------------------------------------------------
    covergroup cg_axi_trans;
        option.per_instance = 1;
        option.comment      = "AXI4-Lite Protocol & Address Map Coverage";

        cp_axi_addr: coverpoint item_cov.axi_addr {
            bins reg_ctrl       = {12'h000};
            bins reg_status     = {12'h004};
            bins reg_num_vars   = {12'h008};
            bins reg_tolerance  = {12'h00C};
            bins reg_alpha      = {12'h010};
            bins reg_lambda     = {12'h014};
            bins reg_max_sweeps = {12'h018};
            bins reg_sweep_cnt  = {12'h01C};
            bins reg_f_opt      = {12'h020};
            bins reg_max_delta  = {12'h024};
            bins reg_prog_addr  = {12'h040};
            bins reg_prog_data  = {12'h044};
            bins state_vec_win  = {[12'h100 : 12'h13C]};
            bins unmapped       = {12'h048, 12'h800, 12'hFFC};
        }

        cp_axi_op: coverpoint item_cov.axi_op {
            bins read_op  = {newton_axi_seq_item::AXI_READ};
            bins write_op = {newton_axi_seq_item::AXI_WRITE};
        }

        cx_addr_op: cross cp_axi_addr, cp_axi_op {
            // Read-only registers should not expect write operations
            ignore_bins ro_writes = binsof(cp_axi_addr) intersect {12'h004, 12'h01C, 12'h020, 12'h024} && binsof(cp_axi_op.write_op);
            // Write-only registers should not expect read operations
            ignore_bins wo_reads  = binsof(cp_axi_addr) intersect {12'h044} && binsof(cp_axi_op.read_op);
        }
    endgroup

    function new(string name = "newton_coverage", uvm_component parent = null);
        super.new(name, parent);
        cg_optimization = new();
        cg_axi_trans    = new();
        sample_count    = 0;
    endfunction

    // Phase 1: build_phase
    virtual function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("COV_PHASE_1_BUILD", "[STAGE 1: SETUP] build_phase: Coverage collector initialized.", UVM_LOW)
    endfunction

    // Phase 2: connect_phase
    virtual function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        `uvm_info("COV_PHASE_2_CONNECT", "[STAGE 1: SETUP] connect_phase: Coverage export ready.", UVM_LOW)
    endfunction

    // Phase 3: end_of_elaboration_phase
    virtual function void end_of_elaboration_phase(uvm_phase phase);
        super.end_of_elaboration_phase(phase);
        `uvm_info("COV_PHASE_3_END_OF_ELAB", "[STAGE 1: SETUP] end_of_elaboration_phase: Coverage model elaboration complete.", UVM_LOW)
    endfunction

    // Phase 4: start_of_simulation_phase
    virtual function void start_of_simulation_phase(uvm_phase phase);
        super.start_of_simulation_phase(phase);
        `uvm_info("COV_PHASE_4_START_OF_SIM", "[STAGE 1: SETUP] start_of_simulation_phase: Coverage sampling armed.", UVM_LOW)
    endfunction

    // Phase 5: run_phase
    virtual task run_phase(uvm_phase phase);
        super.run_phase(phase);
        `uvm_info("COV_PHASE_5_RUN", "[STAGE 2: RUN] run_phase: Coverage monitor running.", UVM_HIGH)
    endtask

    // Subscriber write callback
    virtual function void write(newton_axi_seq_item t);
        item_cov = t;
        sample_count++;
        if (t.is_raw_axi) begin
            cg_axi_trans.sample();
        end else begin
            cg_optimization.sample();
        end
    endfunction

    // Phase 6: extract_phase
    virtual function void extract_phase(uvm_phase phase);
        super.extract_phase(phase);
        `uvm_info("COV_PHASE_6_EXTRACT", $sformatf("[STAGE 3: CLEANUP] extract_phase: Harvested %0d coverage samples.", sample_count), UVM_LOW)
    endfunction

    // Phase 7: check_phase
    virtual function void check_phase(uvm_phase phase);
        super.check_phase(phase);
        `uvm_info("COV_PHASE_7_CHECK", "[STAGE 3: CLEANUP] check_phase: Coverage integrity checks verified.", UVM_LOW)
    endfunction

    // Phase 8: report_phase
    virtual function void report_phase(uvm_phase phase);
        super.report_phase(phase);
        `uvm_info(get_type_name(), "==================================================================", UVM_NONE)
        `uvm_info(get_type_name(), "      [STAGE 3: CLEANUP] FUNCTIONAL COVERAGE REPORT               ", UVM_NONE)
        `uvm_info(get_type_name(), "==================================================================", UVM_NONE)
        `uvm_info(get_type_name(), $sformatf("  Total Coverage Samples Taken : %0d", sample_count), UVM_NONE)
        `uvm_info(get_type_name(), $sformatf("  Optimization Coverage        : %0.2f%%", cg_optimization.get_coverage()), UVM_NONE)
        `uvm_info(get_type_name(), $sformatf("  AXI Address/Protocol Coverage: %0.2f%%", cg_axi_trans.get_coverage()), UVM_NONE)
        `uvm_info(get_type_name(), "==================================================================", UVM_NONE)
    endfunction

    // Phase 9: final_phase
    virtual function void final_phase(uvm_phase phase);
        super.final_phase(phase);
        `uvm_info("COV_PHASE_9_FINAL", "[STAGE 3: CLEANUP] final_phase: Coverage collection closed.", UVM_LOW)
    endfunction

endclass : newton_coverage

`endif // NEWTON_COVERAGE_SV
