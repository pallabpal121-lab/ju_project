// =============================================================================
// File Name   : tb_pso.sv
// Module Name : tb_pso
// Project     : Particle Swarm Optimization (PSO) Accelerator (Solver #14)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for the Particle Swarm Optimizer.
//   Verifies:
//   1. 2D Coupled Multi-Agent Global Optimization: f(x0, x1) = (x0 - 2)^2 + (x1 - 3)^2 + x0*x1
//   2. 3D Sphere Global Optimization: f(x0, x1, x2) = (x0 - 1)^2 + (x1 + 2)^2 + (x2 - 3)^2
//   3. 2D Non-Convex Rosenbrock Valley: f(x0, x1) = (1 - x0)^2 + 4*(x1 - x0^2)^2
// =============================================================================

`timescale 1ns / 1ps

import pso_types_pkg::*;
`include "pso_helpers.svh"

module tb_pso;

    logic        clk;
    logic        rst_n;

    // Programming Interface
    logic        prog_en;
    logic [4:0]  prog_addr;
    instr_t      prog_data;

    // Swarm Particle Initialization Interface
    logic        particle_we;
    logic [2:0]  particle_addr;
    vec_t        particle_x_in;
    vec_t        particle_v_in;

    // Controls & Hyperparameters
    logic        start;
    logic [2:0]  num_dims;
    logic [3:0]  num_particles;
    q16_t        inertia_w;
    q16_t        c1_coeff;
    q16_t        c2_coeff;
    q16_t        v_max;
    vec_t        x_min_bound;
    vec_t        x_max_bound;
    q16_t        target_fitness;
    q16_t        tolerance;
    logic [7:0]  max_iters;
    logic [31:0] seed_val;

    // Results & Outputs
    vec_t        g_best_pos;
    q16_t        g_best_fitness;
    logic [7:0]  iter_count;
    status_t     status;
    logic        done;
    logic        busy;

    // Instantiate Top Module
    pso_top dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .prog_en        (prog_en),
        .prog_addr      (prog_addr),
        .prog_data      (prog_data),
        .particle_we    (particle_we),
        .particle_addr  (particle_addr),
        .particle_x_in  (particle_x_in),
        .particle_v_in  (particle_v_in),
        .start          (start),
        .num_dims       (num_dims),
        .num_particles  (num_particles),
        .inertia_w      (inertia_w),
        .c1_coeff       (c1_coeff),
        .c2_coeff       (c2_coeff),
        .v_max          (v_max),
        .x_min_bound    (x_min_bound),
        .x_max_bound    (x_max_bound),
        .target_fitness (target_fitness),
        .tolerance      (tolerance),
        .max_iters      (max_iters),
        .seed_val       (seed_val),
        .g_best_pos     (g_best_pos),
        .g_best_fitness (g_best_fitness),
        .iter_count     (iter_count),
        .status         (status),
        .done           (done),
        .busy           (busy)
    );

    // 100MHz Clock Generator
    initial clk = 0;
    always #5 clk = ~clk;

    // Fixed-point Real Number Conversion Helpers
    function real q16_to_real(input q16_t val);
        q16_to_real = real'(val) / 65536.0;
    endfunction

    function q16_t real_to_q16(input real val);
        real_to_q16 = q16_t'(int'(val * 65536.0));
    endfunction

    // Task to program microcode instruction
    task write_instr(input logic [4:0] addr, input opcode_t op, input logic [3:0] dst, input logic [3:0] src_a, input logic [3:0] src_b, input logic signed [15:0] imm);
        begin
            @(posedge clk);
            prog_en         <= 1'b1;
            prog_addr       <= addr;
            prog_data.op    <= op;
            prog_data.dst   <= dst;
            prog_data.src_a <= src_a;
            prog_data.src_b <= src_b;
            prog_data.imm   <= imm;
            @(posedge clk);
            prog_en         <= 1'b0;
        end
    endtask

    // Task to load particle initial position and velocity
    task load_particle(input logic [2:0] addr, input real x0, input real x1, input real x2, input real x3,
                      input real v0, input real v1, input real v2, input real v3);
        begin
            @(posedge clk);
            particle_we   <= 1'b1;
            particle_addr <= addr;
            particle_x_in <= set_vec(set_vec(set_vec(set_vec('0, 2'd0, real_to_q16(x0)), 2'd1, real_to_q16(x1)), 2'd2, real_to_q16(x2)), 2'd3, real_to_q16(x3));
            particle_v_in <= set_vec(set_vec(set_vec(set_vec('0, 2'd0, real_to_q16(v0)), 2'd1, real_to_q16(v1)), 2'd2, real_to_q16(v2)), 2'd3, real_to_q16(v3));
            @(posedge clk);
            particle_we   <= 1'b0;
        end
    endtask

    initial begin
        $display("==================================================================");
        $display(" Particle Swarm Optimization (PSO) Accelerator TB (Solver #14)");
        $display("==================================================================");

        // Reset
        rst_n          = 0;
        prog_en        = 0;
        prog_addr      = '0;
        prog_data      = '0;
        particle_we    = 0;
        particle_addr  = '0;
        particle_x_in  = '0;
        particle_v_in  = '0;
        start          = 0;
        num_dims       = 3'd2;
        num_particles  = 4'd8;
        inertia_w      = 0;
        c1_coeff       = 0;
        c2_coeff       = 0;
        v_max          = 0;
        x_min_bound    = '0;
        x_max_bound    = '0;
        target_fitness = 0;
        tolerance      = 0;
        max_iters      = 0;
        seed_val       = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2D Coupled Multi-Agent Optimization: f(x0, x1) = (x0 - 2)^2 + (x1 - 3)^2 + x0*x1
        // Global Minimum: x0* = 0.666667, x1* = 2.666667, f(x*) = 3.666667
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 2D Coupled Multi-Agent Optimization: f(x0, x1) = (x0 - 2)^2 + (x1 - 3)^2 + x0*x1");
        $display("Target Minimum: x0* = 0.666667, x1* = 2.666667, f(x*) = 3.666667");

        // Microcode Program:
        // r0 = x0, r1 = x1
        // r4 = 2.0, r5 = 3.0
        // r6 = x0 - 2.0
        // r7 = (x0 - 2.0)^2
        // r8 = x1 - 3.0
        // r9 = (x1 - 3.0)^2
        // r10 = x0 * x1
        // r11 = r7 + r9
        // r15 = r11 + r10
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd2);  // r4  = 2.0
        write_instr(5'd1, OP_LOADC, 4'd5,  4'd0, 4'd0, 16'sd3);  // r5  = 3.0
        write_instr(5'd2, OP_SUB,   4'd6,  4'd0, 4'd4, 16'sd0);  // r6  = x0 - 2.0
        write_instr(5'd3, OP_MUL,   4'd7,  4'd6, 4'd6, 16'sd0);  // r7  = (x0 - 2.0)^2
        write_instr(5'd4, OP_SUB,   4'd8,  4'd1, 4'd5, 16'sd0);  // r8  = x1 - 3.0
        write_instr(5'd5, OP_MUL,   4'd9,  4'd8, 4'd8, 16'sd0);  // r9  = (x1 - 3.0)^2
        write_instr(5'd6, OP_MUL,   4'd10, 4'd0, 4'd1, 16'sd0);  // r10 = x0 * x1
        write_instr(5'd7, OP_ADD,   4'd11, 4'd7, 4'd9, 16'sd0);  // r11 = (x0-2)^2 + (x1-3)^2
        write_instr(5'd8, OP_ADD,   4'd15, 4'd11, 4'd10, 16'sd0);// r15 = f(x)
        write_instr(5'd9, OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);  // END

        // Load 8 initial particles spread across [-5.0, +5.0]
        load_particle(3'd0, -4.0, -3.0, 0.0, 0.0,  0.5, -0.5, 0.0, 0.0);
        load_particle(3'd1,  3.0, -4.0, 0.0, 0.0, -0.5,  0.5, 0.0, 0.0);
        load_particle(3'd2, -2.0,  4.0, 0.0, 0.0,  0.2, -0.2, 0.0, 0.0);
        load_particle(3'd3,  4.0,  3.0, 0.0, 0.0, -0.4, -0.4, 0.0, 0.0);
        load_particle(3'd4, -1.0, -1.0, 0.0, 0.0,  0.3,  0.3, 0.0, 0.0);
        load_particle(3'd5,  2.0,  1.0, 0.0, 0.0, -0.2,  0.4, 0.0, 0.0);
        load_particle(3'd6,  0.0,  5.0, 0.0, 0.0,  0.1, -0.3, 0.0, 0.0);
        load_particle(3'd7,  5.0,  0.0, 0.0, 0.0, -0.3,  0.1, 0.0, 0.0);

        @(posedge clk);
        num_dims       = 3'd2;
        num_particles  = 4'd8;
        inertia_w      = real_to_q16(0.72);
        c1_coeff       = real_to_q16(1.49);
        c2_coeff       = real_to_q16(1.49);
        v_max          = real_to_q16(2.0);
        x_min_bound    = set_vec(set_vec('0, 2'd0, real_to_q16(-6.0)), 2'd1, real_to_q16(-6.0));
        x_max_bound    = set_vec(set_vec('0, 2'd0, real_to_q16(6.0)),  2'd1, real_to_q16(6.0));
        target_fitness = real_to_q16(3.666667);
        tolerance      = 32'h0000_0800; // tol ≈ 0.03
        max_iters      = 8'd40;
        seed_val       = 32'h5A1F_9C3D;
        start          = 1'b1;
        @(posedge clk);
        start          = 1'b0;

        $display("Starting Particle Swarm Optimizer (8 particles, 40 generations)...");
        @(posedge done);
        #1;

        $display("--> PSO SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED, 3 = MAX_ITERS)", status);
        $display("    Generations:    %0d", iter_count);
        $display("    x0_optimal:     %f (Expected: ~ 0.666667)", q16_to_real(get_vec(g_best_pos, 2'd0)));
        $display("    x1_optimal:     %f (Expected: ~ 2.666667)", q16_to_real(get_vec(g_best_pos, 2'd1)));
        $display("    f_best_optimal: %f (Expected: ~ 3.666667)", q16_to_real(g_best_fitness));

        if ((status == STATUS_CONVERGED || status == STATUS_MAX_ITERS) &&
            q16_to_real(get_vec(g_best_pos, 2'd0)) > 0.50 && q16_to_real(get_vec(g_best_pos, 2'd0)) < 0.85 &&
            q16_to_real(get_vec(g_best_pos, 2'd1)) > 2.45 && q16_to_real(get_vec(g_best_pos, 2'd1)) < 2.90 &&
            q16_to_real(g_best_fitness) < 3.75) begin
            $display("[TEST 1 PASSED] Swarm successfully converged on 2D coupled optimum!");
        end else begin
            $display("[TEST 1 FAILED] Swarm did not locate 2D global optimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 3D Multi-Agent Sphere Global Optimization: f(x0, x1, x2) = (x0 - 1)^2 + (x1 + 2)^2 + (x2 - 3)^2
        // Global Minimum: x0* = 1.000, x1* = -2.000, x2* = 3.000, f(x*) = 0.000
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 3D Sphere Global Optimization: f(x0, x1, x2) = (x0 - 1)^2 + (x1 + 2)^2 + (x2 - 3)^2");
        $display("Target Minimum: x0* = 1.000, x1* = -2.000, x2* = 3.000, f(x*) = 0.000");

        // Microcode Program:
        // r0 = x0, r1 = x1, r2 = x2
        // r4 = 1.0, r5 = -2.0, r6 = 3.0
        // r7 = x0 - 1.0
        // r8 = (x0 - 1.0)^2
        // r9 = x1 - (-2.0)
        // r10 = (x1 + 2.0)^2
        // r11 = x2 - 3.0
        // r12 = (x2 - 3.0)^2
        // r13 = r8 + r10
        // r15 = r13 + r12
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd1);   // r4  = 1.0
        write_instr(5'd1, OP_LOADC, 4'd5,  4'd0, 4'd0, -16'sd2);  // r5  = -2.0
        write_instr(5'd2, OP_LOADC, 4'd6,  4'd0, 4'd0, 16'sd3);   // r6  = 3.0
        write_instr(5'd3, OP_SUB,   4'd7,  4'd0, 4'd4, 16'sd0);   // r7  = x0 - 1.0
        write_instr(5'd4, OP_MUL,   4'd8,  4'd7, 4'd7, 16'sd0);   // r8  = (x0 - 1.0)^2
        write_instr(5'd5, OP_SUB,   4'd9,  4'd1, 4'd5, 16'sd0);   // r9  = x1 - (-2.0) = x1 + 2
        write_instr(5'd6, OP_MUL,   4'd10, 4'd9, 4'd9, 16'sd0);   // r10 = (x1 + 2.0)^2
        write_instr(5'd7, OP_SUB,   4'd11, 4'd2, 4'd6, 16'sd0);   // r11 = x2 - 3.0
        write_instr(5'd8, OP_MUL,   4'd12, 4'd11, 4'd11, 16'sd0); // r12 = (x2 - 3.0)^2
        write_instr(5'd9, OP_ADD,   4'd13, 4'd8, 4'd10, 16'sd0);  // r13 = r8 + r10
        write_instr(5'd10,OP_ADD,   4'd15, 4'd13, 4'd12, 16'sd0); // r15 = r8 + r10 + r12
        write_instr(5'd11,OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);   // END

        // Load 8 initial particles in 3D
        load_particle(3'd0, -3.0,  4.0, -2.0, 0.0,  0.5, -0.5,  0.5, 0.0);
        load_particle(3'd1,  5.0, -5.0,  5.0, 0.0, -0.5,  0.5, -0.5, 0.0);
        load_particle(3'd2, -4.0,  3.0,  4.0, 0.0,  0.2, -0.2,  0.2, 0.0);
        load_particle(3'd3,  4.0, -1.0, -4.0, 0.0, -0.4, -0.4,  0.4, 0.0);
        load_particle(3'd4,  0.0,  0.0,  0.0, 0.0,  0.3,  0.3, -0.3, 0.0);
        load_particle(3'd5,  2.0, -3.0,  2.0, 0.0, -0.2,  0.4,  0.2, 0.0);
        load_particle(3'd6, -2.0, -4.0,  6.0, 0.0,  0.1, -0.3, -0.2, 0.0);
        load_particle(3'd7,  3.0,  1.0,  1.0, 0.0, -0.3,  0.1,  0.3, 0.0);

        @(posedge clk);
        num_dims       = 3'd3;
        num_particles  = 4'd8;
        inertia_w      = real_to_q16(0.72);
        c1_coeff       = real_to_q16(1.49);
        c2_coeff       = real_to_q16(1.49);
        v_max          = real_to_q16(2.0);
        x_min_bound    = set_vec(set_vec(set_vec('0, 2'd0, real_to_q16(-8.0)), 2'd1, real_to_q16(-8.0)), 2'd2, real_to_q16(-8.0));
        x_max_bound    = set_vec(set_vec(set_vec('0, 2'd0, real_to_q16(8.0)),  2'd1, real_to_q16(8.0)),  2'd2, real_to_q16(8.0));
        target_fitness = real_to_q16(0.0);
        tolerance      = 32'h0000_0800; // tol ≈ 0.03
        max_iters      = 8'd40;
        seed_val       = 32'h8765_4321;
        start          = 1'b1;
        @(posedge clk);
        start          = 1'b0;

        $display("Starting Particle Swarm Optimizer for 3D Sphere...");
        @(posedge done);
        #1;

        $display("--> PSO SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED, 3 = MAX_ITERS)", status);
        $display("    Generations:    %0d", iter_count);
        $display("    x0_optimal:     %f (Expected: ~ 1.000000)", q16_to_real(get_vec(g_best_pos, 2'd0)));
        $display("    x1_optimal:     %f (Expected: ~ -2.000000)", q16_to_real(get_vec(g_best_pos, 2'd1)));
        $display("    x2_optimal:     %f (Expected: ~ 3.000000)", q16_to_real(get_vec(g_best_pos, 2'd2)));
        $display("    f_best_optimal: %f (Expected: ~ 0.000000)", q16_to_real(g_best_fitness));

        if ((status == STATUS_CONVERGED || status == STATUS_MAX_ITERS) &&
            q16_to_real(get_vec(g_best_pos, 2'd0)) > 0.85 && q16_to_real(get_vec(g_best_pos, 2'd0)) < 1.15 &&
            q16_to_real(get_vec(g_best_pos, 2'd1)) > -2.15 && q16_to_real(get_vec(g_best_pos, 2'd1)) < -1.85 &&
            q16_to_real(get_vec(g_best_pos, 2'd2)) > 2.85 && q16_to_real(get_vec(g_best_pos, 2'd2)) < 3.15 &&
            q16_to_real(g_best_fitness) < 0.10) begin
            $display("[TEST 2 PASSED] Swarm successfully converged on 3D sphere optimum!");
        end else begin
            $display("[TEST 2 FAILED] Swarm did not locate 3D global optimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 2D Non-Convex Curved Valley Optimization (Rosenbrock): f(x0, x1) = (1 - x0)^2 + 4*(x1 - x0^2)^2
        // Global Minimum: x0* = 1.000, x1* = 1.000, f(x*) = 0.000
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 2D Non-Convex Curved Valley (Rosenbrock): f(x0, x1) = (1 - x0)^2 + 4*(x1 - x0^2)^2");
        $display("Target Minimum: x0* = 1.000, x1* = 1.000, f(x*) = 0.000");

        // Microcode Program:
        // r0 = x0, r1 = x1
        // r4 = 1.0, r5 = 4.0
        // r6 = 1.0 - x0
        // r7 = (1.0 - x0)^2
        // r8 = x0 * x0 = x0^2
        // r9 = x1 - x0^2
        // r10 = (x1 - x0^2)^2
        // r11 = 4.0 * (x1 - x0^2)^2
        // r15 = r7 + r11
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd1);  // r4  = 1.0
        write_instr(5'd1, OP_LOADC, 4'd5,  4'd0, 4'd0, 16'sd4);  // r5  = 4.0
        write_instr(5'd2, OP_SUB,   4'd6,  4'd4, 4'd0, 16'sd0);  // r6  = 1.0 - x0
        write_instr(5'd3, OP_MUL,   4'd7,  4'd6, 4'd6, 16'sd0);  // r7  = (1.0 - x0)^2
        write_instr(5'd4, OP_MUL,   4'd8,  4'd0, 4'd0, 16'sd0);  // r8  = x0^2
        write_instr(5'd5, OP_SUB,   4'd9,  4'd1, 4'd8, 16'sd0);  // r9  = x1 - x0^2
        write_instr(5'd6, OP_MUL,   4'd10, 4'd9, 4'd9, 16'sd0);  // r10 = (x1 - x0^2)^2
        write_instr(5'd7, OP_MUL,   4'd11, 4'd5, 4'd10, 16'sd0); // r11 = 4 * (x1 - x0^2)^2
        write_instr(5'd8, OP_ADD,   4'd15, 4'd7, 4'd11, 16'sd0); // r15 = f(x)
        write_instr(5'd9, OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);  // END

        // Load 8 initial particles
        load_particle(3'd0, -1.5, -1.0, 0.0, 0.0,  0.2,  0.2, 0.0, 0.0);
        load_particle(3'd1,  1.5, -1.5, 0.0, 0.0, -0.2,  0.3, 0.0, 0.0);
        load_particle(3'd2, -1.0,  1.5, 0.0, 0.0,  0.1, -0.2, 0.0, 0.0);
        load_particle(3'd3,  1.8,  1.8, 0.0, 0.0, -0.3, -0.3, 0.0, 0.0);
        load_particle(3'd4, -0.5, -0.5, 0.0, 0.0,  0.2,  0.2, 0.0, 0.0);
        load_particle(3'd5,  0.8,  0.5, 0.0, 0.0, -0.1,  0.2, 0.0, 0.0);
        load_particle(3'd6,  0.0,  1.0, 0.0, 0.0,  0.1, -0.1, 0.0, 0.0);
        load_particle(3'd7,  1.2, -0.5, 0.0, 0.0, -0.2,  0.1, 0.0, 0.0);

        @(posedge clk);
        num_dims       = 3'd2;
        num_particles  = 4'd8;
        inertia_w      = real_to_q16(0.72);
        c1_coeff       = real_to_q16(1.49);
        c2_coeff       = real_to_q16(1.49);
        v_max          = real_to_q16(1.5);
        x_min_bound    = set_vec(set_vec('0, 2'd0, real_to_q16(-3.0)), 2'd1, real_to_q16(-3.0));
        x_max_bound    = set_vec(set_vec('0, 2'd0, real_to_q16(3.0)),  2'd1, real_to_q16(3.0));
        target_fitness = real_to_q16(0.0);
        tolerance      = 32'h0000_0800; // tol ≈ 0.03
        max_iters      = 8'd40;
        seed_val       = 32'hFEED_FACE;
        start          = 1'b1;
        @(posedge clk);
        start          = 1'b0;

        $display("Starting Particle Swarm Optimizer for 2D Rosenbrock...");
        @(posedge done);
        #1;

        $display("--> PSO SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED, 3 = MAX_ITERS)", status);
        $display("    Generations:    %0d", iter_count);
        $display("    x0_optimal:     %f (Expected: ~ 1.000000)", q16_to_real(get_vec(g_best_pos, 2'd0)));
        $display("    x1_optimal:     %f (Expected: ~ 1.000000)", q16_to_real(get_vec(g_best_pos, 2'd1)));
        $display("    f_best_optimal: %f (Expected: ~ 0.000000)", q16_to_real(g_best_fitness));

        if ((status == STATUS_CONVERGED || status == STATUS_MAX_ITERS) &&
            q16_to_real(get_vec(g_best_pos, 2'd0)) > 0.85 && q16_to_real(get_vec(g_best_pos, 2'd0)) < 1.15 &&
            q16_to_real(get_vec(g_best_pos, 2'd1)) > 0.80 && q16_to_real(get_vec(g_best_pos, 2'd1)) < 1.20 &&
            q16_to_real(g_best_fitness) < 0.15) begin
            $display("[TEST 3 PASSED] Swarm successfully traversed Rosenbrock valley to global optimum!");
        end else begin
            $display("[TEST 3 FAILED] Swarm did not locate Rosenbrock global optimum.");
        end

        $display("\n==================================================================");
        $display(" ALL PARTICLE SWARM OPTIMIZATION HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
