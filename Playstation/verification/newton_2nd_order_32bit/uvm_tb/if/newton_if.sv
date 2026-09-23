// =============================================================================
// File Name   : newton_if.sv
// Module Name : newton_if (SystemVerilog Interface)
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

interface newton_if (input logic clk, input logic rst_n);
    import newton_types_pkg::*;
    import uvm_pkg::*;
    `include "uvm_macros.svh"

    // Microcode Programming Interface Signals
    logic        prog_en;
    logic [4:0]  prog_addr;
    instr_t      prog_data;

    // Optimization Control & Parameter Signals
    logic        start;
    q16_t        x_init;
    q16_t        tolerance;
    q16_t        step_alpha;
    q16_t        lambda_reg;
    logic [7:0]  max_iters;

    // Output & Status Signals
    q16_t        x_optimal;
    q16_t        f_optimal;
    q16_t        g_final;
    logic [7:0]  iter_count;
    status_t     status;
    logic        done;
    logic        busy;

    // Clocking Block: Driver
    clocking drv_cb @(posedge clk);
        default input #1ns output #1ns;
        output prog_en;
        output prog_addr;
        output prog_data;
        output start;
        output x_init;
        output tolerance;
        output step_alpha;
        output lambda_reg;
        output max_iters;
        input  x_optimal;
        input  f_optimal;
        input  g_final;
        input  iter_count;
        input  status;
        input  done;
        input  busy;
    endclocking

    // Clocking Block: Monitor
    clocking mon_cb @(posedge clk);
        default input #1ns output #1ns;
        input  prog_en;
        input  prog_addr;
        input  prog_data;
        input  start;
        input  x_init;
        input  tolerance;
        input  step_alpha;
        input  lambda_reg;
        input  max_iters;
        input  x_optimal;
        input  f_optimal;
        input  g_final;
        input  iter_count;
        input  status;
        input  done;
        input  busy;
    endclocking

    modport drv_mp (clocking drv_cb, input clk, input rst_n);
    modport mon_mp (clocking mon_cb, input clk, input rst_n);
    modport dut_mp (
        input  clk, rst_n, prog_en, prog_addr, prog_data,
        input  start, x_init, tolerance, step_alpha, lambda_reg, max_iters,
        output x_optimal, f_optimal, g_final, iter_count, status, done, busy
    );

    // =========================================================================
    // Industry-Grade Protocol Assertions (SVA)
    // =========================================================================

    // 1. Reset check: busy and done must remain deasserted (0) during active reset
    property p_reset_state;
        @(posedge clk)
        !rst_n |-> (!busy && !done);
    endproperty
    A_RESET_STATE: assert property (p_reset_state)
        else `uvm_error("SVA_IF", "Protocol violation: 'busy' or 'done' asserted while in reset (rst_n == 0)!")

    // 2. Control check: 'start' must NEVER pulse while the accelerator is already 'busy'
    property p_no_start_during_busy;
        @(posedge clk) disable iff (!rst_n)
        busy |-> !start;
    endproperty
    A_NO_START_DURING_BUSY: assert property (p_no_start_during_busy)
        else `uvm_error("SVA_IF", "Protocol violation: 'start' asserted while accelerator was already 'busy'!")

    // 3. Programming check: 'prog_en' must NEVER assert while the accelerator is 'busy'
    property p_no_prog_during_busy;
        @(posedge clk) disable iff (!rst_n)
        busy |-> !prog_en;
    endproperty
    A_NO_PROG_DURING_BUSY: assert property (p_no_prog_during_busy)
        else `uvm_error("SVA_IF", "Protocol violation: 'prog_en' asserted while accelerator was executing ('busy' == 1)!")

    // 4. Data integrity: Input parameters must NOT contain X or Z when 'start' is pulsed
    property p_valid_params_on_start;
        @(posedge clk) disable iff (!rst_n)
        start |-> !$isunknown({x_init, tolerance, step_alpha, lambda_reg, max_iters});
    endproperty
    A_VALID_PARAMS_ON_START: assert property (p_valid_params_on_start)
        else `uvm_error("SVA_IF", "Protocol violation: Unknown (X/Z) value detected on input parameters when 'start' asserted!")

    // 5. Data integrity: Microcode address & data must NOT contain X or Z when 'prog_en' is asserted
    property p_valid_prog_bus;
        @(posedge clk) disable iff (!rst_n)
        prog_en |-> !$isunknown({prog_addr, prog_data});
    endproperty
    A_VALID_PROG_BUS: assert property (p_valid_prog_bus)
        else `uvm_error("SVA_IF", "Protocol violation: Unknown (X/Z) value detected on programming bus when 'prog_en' asserted!")

    // 6. Response handshake: 'done' must be a single-cycle pulse
    property p_done_single_cycle;
        @(posedge clk) disable iff (!rst_n)
        done |=> !done;
    endproperty
    A_DONE_SINGLE_CYCLE: assert property (p_done_single_cycle)
        else `uvm_error("SVA_IF", "Protocol violation: 'done' remained asserted for more than 1 consecutive clock cycle!")

    // 7. Handshake causality: 'done' can only fire if the accelerator was 'busy'
    property p_done_requires_busy;
        @(posedge clk) disable iff (!rst_n)
        done |-> ($past(busy) || busy);
    endproperty
    A_DONE_REQUIRES_BUSY: assert property (p_done_requires_busy)
        else `uvm_error("SVA_IF", "Protocol violation: 'done' asserted without accelerator being 'busy'!")

    // 8. Output integrity: When 'done' fires, output results and status must be valid and known
    property p_valid_outputs_on_done;
        @(posedge clk) disable iff (!rst_n)
        done |-> (status != STATUS_IDLE) && !$isunknown({x_optimal, f_optimal, g_final, iter_count, status});
    endproperty
    A_VALID_OUTPUTS_ON_DONE: assert property (p_valid_outputs_on_done)
        else `uvm_error("SVA_IF", "Protocol violation: Output results contained X/Z or invalid status when 'done' asserted!")

    // 9. Completion deassertion: 'busy' must drop low on the cycle immediately following 'done'
    property p_busy_drops_after_done;
        @(posedge clk) disable iff (!rst_n)
        done |=> !busy;
    endproperty
    A_BUSY_DROPS_AFTER_DONE: assert property (p_busy_drops_after_done)
        else `uvm_error("SVA_IF", "Protocol violation: 'busy' remained asserted after 'done' completion!")

    // =========================================================================
    // Protocol Coverage (Tracked by EDA Coverage Databases like VCS urg / Verdi)
    // =========================================================================

    // C1: Verify a complete start-to-done execution sequence is exercised
    C_EXEC_SEQUENCE: cover property (
        @(posedge clk) disable iff (!rst_n)
        start ##[1:$] done
    );

    // C2: Verify completion with STATUS_CONVERGED
    C_STATUS_CONVERGED: cover property (
        @(posedge clk) disable iff (!rst_n)
        done && (status == STATUS_CONVERGED)
    );

    // C3: Verify completion with STATUS_MAX_ITERS
    C_STATUS_MAX_ITERS: cover property (
        @(posedge clk) disable iff (!rst_n)
        done && (status == STATUS_MAX_ITERS)
    );

    // C4: Verify completion with STATUS_SINGULAR
    C_STATUS_SINGULAR: cover property (
        @(posedge clk) disable iff (!rst_n)
        done && (status == STATUS_SINGULAR)
    );

    // C5: Verify microcode programming occurred
    C_PROG_WRITTEN: cover property (
        @(posedge clk) disable iff (!rst_n)
        prog_en
    );

endinterface : newton_if
