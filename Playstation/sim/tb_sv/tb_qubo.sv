// =============================================================================
// File Name   : tb_qubo.sv
// Module Name : tb_qubo
// Project     : QUBO / Simulated Annealing Ising Accelerator (Solver #18)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for QUBO / Simulated Annealing Ising.
//   Verifies:
//   1. 4-Spin Max-Cut Graph Partitioning QUBO (Ground State E* = -8.0)
//   2. 4-Spin Number Partitioning (NP-Complete Exact Ground State E* = -30.25)
//   3. 4-Spin Frustrated Ising Spin Glass Ground-State Search (E* = -2.0)
// =============================================================================

`timescale 1ns / 1ps

import qubo_types_pkg::*;
`include "qubo_helpers.svh"

module tb_qubo;

    logic        clk;
    logic        rst_n;

    // Controls & Inputs
    logic        start;
    logic [3:0]  num_spins;
    qubo_mat_t   q_matrix;
    spin_vec_t   q_init;
    q16_t        t_start;
    q16_t        t_end;
    q16_t        gamma_cool;
    logic [5:0]  sweeps_per_temp;
    logic [7:0]  max_temp_steps;
    logic [31:0] prng_seed;

    // Results & Outputs
    spin_vec_t   q_optimal;
    q16_t        energy_optimal;
    logic [7:0]  steps_count;
    logic [15:0] flips_accepted;
    status_t     status;
    logic        done;
    logic        busy;

    // Instantiate Top Module
    qubo_top dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .num_spins      (num_spins),
        .q_matrix       (q_matrix),
        .q_init         (q_init),
        .t_start        (t_start),
        .t_end          (t_end),
        .gamma_cool     (gamma_cool),
        .sweeps_per_temp(sweeps_per_temp),
        .max_temp_steps (max_temp_steps),
        .prng_seed      (prng_seed),
        .q_optimal      (q_optimal),
        .energy_optimal (energy_optimal),
        .steps_count    (steps_count),
        .flips_accepted (flips_accepted),
        .status         (status),
        .done           (done),
        .busy           (busy)
    );

    // 100MHz Clock Generator
    initial clk = 0;
    always #5 clk = ~clk;

    // Fixed-point Real Conversion Helpers
    function real q16_to_real(input q16_t val);
        q16_to_real = real'(val) / 65536.0;
    endfunction

    function q16_t real_to_q16(input real val);
        real_to_q16 = q16_t'(int'(val * 65536.0));
    endfunction

    initial begin
        $display("==================================================================");
        $display(" QUBO / Simulated Annealing Ising Accelerator TB (Solver #18)");
        $display("==================================================================");

        // Reset
        rst_n           = 0;
        start           = 0;
        num_spins       = 4'd4;
        q_matrix        = '0;
        q_init          = 8'd0;
        t_start         = 0;
        t_end           = 0;
        gamma_cool      = 0;
        sweeps_per_temp = 0;
        max_temp_steps  = 0;
        prng_seed       = 32'h1357_9BDF;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 4-Spin Max-Cut Graph Partitioning QUBO
        // 4-Node 4-Edge Ring Graph. Maximum Cut = 8.0, Target Ground Energy E* = -8.0
        // Ground State Partition: q* = 4'b1010 (10) or 4'b0101 (5)
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 4-Spin Max-Cut Graph Partitioning QUBO");
        $display("Problem: 4-Node Ring Graph with edge weights w = 2.0");
        $display("Target Ground State: q* = [1, 0, 1, 0] or [0, 1, 0, 1], Ground Energy E* = -8.000000");

        q_matrix = '0;
        // Diagonal linear biases
        q_matrix = set_qmat(q_matrix, 3'd0, 3'd0, real_to_q16(-4.0));
        q_matrix = set_qmat(q_matrix, 3'd1, 3'd1, real_to_q16(-4.0));
        q_matrix = set_qmat(q_matrix, 3'd2, 3'd2, real_to_q16(-4.0));
        q_matrix = set_qmat(q_matrix, 3'd3, 3'd3, real_to_q16(-4.0));
        // Off-diagonal ring couplings (symmetric)
        q_matrix = set_qmat(q_matrix, 3'd0, 3'd1, real_to_q16(2.0));
        q_matrix = set_qmat(q_matrix, 3'd1, 3'd0, real_to_q16(2.0));
        q_matrix = set_qmat(q_matrix, 3'd1, 3'd2, real_to_q16(2.0));
        q_matrix = set_qmat(q_matrix, 3'd2, 3'd1, real_to_q16(2.0));
        q_matrix = set_qmat(q_matrix, 3'd2, 3'd3, real_to_q16(2.0));
        q_matrix = set_qmat(q_matrix, 3'd3, 3'd2, real_to_q16(2.0));
        q_matrix = set_qmat(q_matrix, 3'd3, 3'd0, real_to_q16(2.0));
        q_matrix = set_qmat(q_matrix, 3'd0, 3'd3, real_to_q16(2.0));

        @(posedge clk);
        num_spins       = 4'd4;
        q_init          = 8'b0000_0000;
        t_start         = real_to_q16(10.0);
        t_end           = real_to_q16(0.01);
        gamma_cool      = real_to_q16(0.92);
        sweeps_per_temp = 6'd16;
        max_temp_steps  = 8'd60;
        prng_seed       = 32'h2468_ACE0;
        start           = 1'b1;
        @(posedge clk);
        start           = 1'b0;

        $display("Starting Simulated Annealing Ising Engine from [0, 0, 0, 0]...");
        @(posedge done);
        #1;

        $display("--> QUBO ANNEALER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Thermal Steps:  %0d", steps_count);
        $display("    Flips Accepted: %0d", flips_accepted);
        $display("    q_optimal:      %04b (Expected: 1010 or 0101)", q_optimal[3:0]);
        $display("    Energy E(q*):   %f (Expected: -8.000000)", q16_to_real(energy_optimal));

        if ((q_optimal[3:0] == 4'b1010 || q_optimal[3:0] == 4'b0101) &&
            q16_to_real(energy_optimal) < -7.95 && q16_to_real(energy_optimal) > -8.05) begin
            $display("[TEST 1 PASSED] Successfully solved Max-Cut graph partition to global ground state!");
        end else begin
            $display("[TEST 1 FAILED] Annealer did not reach target ground state.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 4-Spin Number Partitioning Problem (NP-Complete)
        // Given set S = {4, 5, 6, 7}, partition into two equal subsets of sum 11.
        // Target Partition: q* = [1, 0, 0, 1] ({4,7}) or [0, 1, 1, 0] ({5,6})
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 4-Spin Number Partitioning Problem (NP-Complete)");
        $display("Numbers: S = {4, 5, 6, 7} (Target Equal Subsets: {4,7} vs {5,6})");
        $display("Target Ground State: q* = [1, 0, 0, 1] or [0, 1, 1, 0], Energy E* = -30.250000");

        q_matrix = '0;
        // Diagonals Q_ii
        q_matrix = set_qmat(q_matrix, 3'd0, 3'd0, real_to_q16(-18.0));
        q_matrix = set_qmat(q_matrix, 3'd1, 3'd1, real_to_q16(-21.25));
        q_matrix = set_qmat(q_matrix, 3'd2, 3'd2, real_to_q16(-24.0));
        q_matrix = set_qmat(q_matrix, 3'd3, 3'd3, real_to_q16(-26.25));
        // Off-diagonals Q_ij (symmetric)
        q_matrix = set_qmat(q_matrix, 3'd0, 3'd1, real_to_q16(5.0));
        q_matrix = set_qmat(q_matrix, 3'd1, 3'd0, real_to_q16(5.0));
        q_matrix = set_qmat(q_matrix, 3'd0, 3'd2, real_to_q16(6.0));
        q_matrix = set_qmat(q_matrix, 3'd2, 3'd0, real_to_q16(6.0));
        q_matrix = set_qmat(q_matrix, 3'd0, 3'd3, real_to_q16(7.0));
        q_matrix = set_qmat(q_matrix, 3'd3, 3'd0, real_to_q16(7.0));
        q_matrix = set_qmat(q_matrix, 3'd1, 3'd2, real_to_q16(7.5));
        q_matrix = set_qmat(q_matrix, 3'd2, 3'd1, real_to_q16(7.5));
        q_matrix = set_qmat(q_matrix, 3'd1, 3'd3, real_to_q16(8.75));
        q_matrix = set_qmat(q_matrix, 3'd3, 3'd1, real_to_q16(8.75));
        q_matrix = set_qmat(q_matrix, 3'd2, 3'd3, real_to_q16(10.5));
        q_matrix = set_qmat(q_matrix, 3'd3, 3'd2, real_to_q16(10.5));

        @(posedge clk);
        num_spins       = 4'd4;
        q_init          = 8'b0000_0000;
        t_start         = real_to_q16(10.0);
        t_end           = real_to_q16(0.01);
        gamma_cool      = real_to_q16(0.90);
        sweeps_per_temp = 6'd6;
        max_temp_steps  = 8'd60;
        prng_seed       = 32'h5A5A_C3C3;
        start           = 1'b1;
        @(posedge clk);
        start           = 1'b0;

        $display("Starting Number Partitioning Annealer...");
        @(posedge done);
        #1;

        $display("--> NUMBER PARTITIONING FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Thermal Steps:  %0d", steps_count);
        $display("    Flips Accepted: %0d", flips_accepted);
        $display("    q_optimal:      %04b (Expected: 1001 or 0110)", q_optimal[3:0]);
        $display("    Energy E(q*):   %f (Expected: -30.250000, error = 0)", q16_to_real(energy_optimal));

        if ((q_optimal[3:0] == 4'b1001 || q_optimal[3:0] == 4'b0110) &&
            q16_to_real(energy_optimal) < -30.20 && q16_to_real(energy_optimal) > -30.30) begin
            $display("[TEST 2 PASSED] Successfully found exact zero-error subset number partition!");
        end else begin
            $display("[TEST 2 FAILED] Number partition did not reach ground state.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 4-Spin Frustrated Antiferromagnetic Ising Glass
        // All spins mutually repel: Q_ii = -2.0, Q_ij = +1.5 (i != j)
        // Ground Energy E* = -2.000000 (Exactly 1 spin ON)
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 4-Spin Frustrated Antiferromagnetic Ising System");
        $display("Coupling: All-to-all repulsive Q_ij = 1.5, bias Q_ii = -2.0");
        $display("Target Ground State: Exactly 1 spin ON (Ground Energy E* = -2.000000)");

        q_matrix = '0;
        for (int i = 0; i < 4; i++) begin
            for (int j = 0; j < 4; j++) begin
                if (i == j) begin
                    q_matrix = set_qmat(q_matrix, 3'(i), 3'(j), real_to_q16(-2.0));
                end else begin
                    q_matrix = set_qmat(q_matrix, 3'(i), 3'(j), real_to_q16(1.5));
                end
            end
        end

        @(posedge clk);
        num_spins       = 4'd4;
        q_init          = 8'b0000_1111; // Start in high-energy trap [1, 1, 1, 1] (E = +10.0)
        t_start         = real_to_q16(12.0);
        t_end           = real_to_q16(0.01);
        gamma_cool      = real_to_q16(0.90);
        sweeps_per_temp = 6'd8;
        max_temp_steps  = 8'd60;
        prng_seed       = 32'h7E12_89AB;
        start           = 1'b1;
        @(posedge clk);
        start           = 1'b0;

        $display("Starting Frustrated Ising Annealer from [1, 1, 1, 1] (E = +10.0)...");
        @(posedge done);
        #1;

        $display("--> FRUSTRATED ISING FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Thermal Steps:  %0d", steps_count);
        $display("    Flips Accepted: %0d", flips_accepted);
        $display("    q_optimal:      %04b (Expected: Exactly 1 ON spin)", q_optimal[3:0]);
        $display("    Energy E(q*):   %f (Expected: -2.000000)", q16_to_real(energy_optimal));

        if (($countones(q_optimal[3:0]) == 1) &&
            q16_to_real(energy_optimal) < -1.95 && q16_to_real(energy_optimal) > -2.05) begin
            $display("[TEST 3 PASSED] Successfully escaped high-energy trap and frozen into frustrated ground state!");
        end else begin
            $display("[TEST 3 FAILED] Frustrated system did not reach ground state.");
        end

        $display("\n==================================================================");
        $display(" ALL QUBO / SIMULATED ANNEALING HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
