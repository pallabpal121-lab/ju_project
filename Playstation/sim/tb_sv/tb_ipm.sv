// =============================================================================
// File Name   : tb_ipm.sv
// Module Name : tb_ipm
// Project     : Primal-Dual Interior Point Method (IPM) Accelerator (Solver #16)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for the IPM Accelerator.
//   Verifies:
//   1. 2D Box-Constrained Convex Quadratic Program
//   2. 2D Coupled Quadratic with Half-Space Inequality Constraint
//   3. 3D Multi-Constraint Actuator Allocation QP
// =============================================================================

`timescale 1ns / 1ps

import ipm_types_pkg::*;
`include "ipm_helpers.svh"

module tb_ipm;

    logic        clk;
    logic        rst_n;

    // Programming Interface
    logic        prog_en;
    logic [4:0]  prog_addr;
    instr_t      prog_data;

    // Algorithm Inputs & Matrices
    logic        start;
    logic [2:0]  num_vars;
    logic [2:0]  num_cons;
    mat_t        q_mat;
    vec_t        c_vec;
    mat_t        a_mat;
    vec_t        b_vec;
    vec_t        x_init;
    vec_t        s_init;
    vec_t        z_init;
    q16_t        sigma_val;
    q16_t        tolerance;
    logic [7:0]  max_iters;

    // Outputs
    vec_t        x_optimal;
    vec_t        s_optimal;
    vec_t        z_optimal;
    q16_t        f_optimal;
    q16_t        duality_gap;
    logic [7:0]  iter_count;
    status_t     status;
    logic        done;
    logic        busy;

    // Instantiate Top Module
    ipm_top dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (start),
        .num_vars   (num_vars),
        .num_cons   (num_cons),
        .q_mat      (q_mat),
        .c_vec      (c_vec),
        .a_mat      (a_mat),
        .b_vec      (b_vec),
        .x_init     (x_init),
        .s_init     (s_init),
        .z_init     (z_init),
        .sigma_val  (sigma_val),
        .tolerance  (tolerance),
        .max_iters  (max_iters),
        .x_optimal  (x_optimal),
        .s_optimal  (s_optimal),
        .z_optimal  (z_optimal),
        .f_optimal  (f_optimal),
        .duality_gap(duality_gap),
        .iter_count (iter_count),
        .status     (status),
        .done       (done),
        .busy       (busy)
    );

    // 100MHz Clock Generator
    initial clk = 0;
    always #5 clk = ~clk;

    // Monitor iteration progress
    always @(posedge clk) begin
        if (dut.state == 3'd4 && dut.kkt_done) begin
            $display("  [Iter %0d] x=(%f, %f) s=(%f, %f) z=(%f, %f) dx=(%f, %f) ap=%f ad=%f mu=%f",
                dut.iter_cnt,
                q16_to_real(get_vec(dut.x_reg, 2'd0)),
                q16_to_real(get_vec(dut.x_reg, 2'd1)),
                q16_to_real(get_vec(dut.s_reg, 2'd0)),
                q16_to_real(get_vec(dut.s_reg, 2'd1)),
                q16_to_real(get_vec(dut.z_reg, 2'd0)),
                q16_to_real(get_vec(dut.z_reg, 2'd1)),
                q16_to_real(get_vec(dut.chol_dx_out, 2'd0)),
                q16_to_real(get_vec(dut.chol_dx_out, 2'd1)),
                q16_to_real(dut.kkt_alpha_p),
                q16_to_real(dut.kkt_alpha_d),
                q16_to_real(dut.mu_reg)
            );
        end
    end

    // Fixed-point Real Conversion Helpers
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

    initial begin
        $display("==================================================================");
        $display(" Primal-Dual Interior Point Method (IPM) TB (Solver #16)");
        $display("==================================================================");

        // Reset
        rst_n      = 0;
        prog_en    = 0;
        prog_addr  = '0;
        prog_data  = '0;
        start      = 0;
        num_vars   = 3'd2;
        num_cons   = 3'd2;
        q_mat      = '0;
        c_vec      = '0;
        a_mat      = '0;
        b_vec      = '0;
        x_init     = '0;
        s_init     = '0;
        z_init     = '0;
        sigma_val  = 0;
        tolerance  = 0;
        max_iters  = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2D Box-Constrained Strictly Convex QP
        // min 0.5*(x0^2 + x1^2) - (2*x0 + 3*x1)
        // s.t. x0 <= 1.0, x1 <= 2.0
        // Target Optimum: x0* = 1.000, x1* = 2.000, f(x*) = -5.500
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 2D Box-Constrained Convex QP");
        $display("Problem: min 0.5*(x0^2 + x1^2) - 2*x0 - 3*x1  s.t.  x0 <= 1.0, x1 <= 2.0");
        $display("Target Optimum: x0* = 1.000000, x1* = 2.000000, f(x*) = -5.500000");

        // Microcode Program for f(x0, x1) = 0.5*x0^2 + 0.5*x1^2 - 2*x0 - 3*x1
        // r0 = x0, r1 = x1
        // r4 = 0.5, r5 = 2.0, r6 = 3.0
        // r7 = x0^2, r8 = 0.5*x0^2
        // r9 = x1^2, r10 = 0.5*x1^2
        // r11 = 2*x0, r12 = 3*x1
        // r13 = r8 + r10
        // r14 = r11 + r12
        // r15 = r13 - r14
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd0); // 0.5 -> will set below
        // Clean microcode:
        // r4 = 2.0, r5 = 3.0
        // r6 = x0^2, r7 = x1^2, r8 = r6 + r7
        // r8 = r8 / 2 (or multiply by 0.5)
        // r9 = 2*x0, r10 = 3*x1, r11 = r9 + r10
        // r15 = 0.5*r8 - r11
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd2);  // r4  = 2.0
        write_instr(5'd1, OP_LOADC, 4'd5,  4'd0, 4'd0, 16'sd3);  // r5  = 3.0
        write_instr(5'd2, OP_MUL,   4'd6,  4'd0, 4'd0, 16'sd0);  // r6  = x0^2
        write_instr(5'd3, OP_MUL,   4'd7,  4'd1, 4'd1, 16'sd0);  // r7  = x1^2
        write_instr(5'd4, OP_ADD,   4'd8,  4'd6, 4'd7, 16'sd0);  // r8  = x0^2 + x1^2
        write_instr(5'd5, OP_DIV,   4'd8,  4'd8, 4'd4, 16'sd0);  // r8  = 0.5*(x0^2 + x1^2)
        write_instr(5'd6, OP_MUL,   4'd9,  4'd4, 4'd0, 16'sd0);  // r9  = 2*x0
        write_instr(5'd7, OP_MUL,   4'd10, 4'd5, 4'd1, 16'sd0);  // r10 = 3*x1
        write_instr(5'd8, OP_ADD,   4'd11, 4'd9, 4'd10, 16'sd0); // r11 = 2*x0 + 3*x1
        write_instr(5'd9, OP_SUB,   4'd15, 4'd8, 4'd11, 16'sd0); // r15 = f(x)
        write_instr(5'd10,OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);  // END

        @(posedge clk);
        num_vars = 3'd2;
        num_cons = 3'd2;

        // Q = diag(1, 1)
        q_mat = set_mat(set_mat('0, 2'd0, 2'd0, Q16_ONE), 2'd1, 2'd1, Q16_ONE);

        // c = [-2.0, -3.0]
        c_vec = set_vec(set_vec('0, 2'd0, real_to_q16(-2.0)), 2'd1, real_to_q16(-3.0));

        // A = [ [1, 0], [0, 1] ]
        a_mat = set_mat(set_mat('0, 2'd0, 2'd0, Q16_ONE), 2'd1, 2'd1, Q16_ONE);

        // b = [1.0, 2.0]
        b_vec = set_vec(set_vec('0, 2'd0, real_to_q16(1.0)), 2'd1, real_to_q16(2.0));

        // x0 = [0, 0], s0 = [1.0, 1.0], z0 = [1.0, 1.0]
        x_init    = '0;
        s_init    = set_vec(set_vec('0, 2'd0, Q16_ONE), 2'd1, Q16_ONE);
        z_init    = set_vec(set_vec('0, 2'd0, Q16_ONE), 2'd1, Q16_ONE);
        sigma_val = real_to_q16(0.10);
        tolerance = 32'h0000_0080; // tol ≈ 0.00195
        max_iters = 8'd25;
        start     = 1'b1;
        @(posedge clk);
        start     = 1'b0;

        $display("Starting IPM Solver...");
        @(posedge done);
        #1;

        $display("--> IPM SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    x0_optimal:   %f (Expected: ~ 1.000000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:   %f (Expected: ~ 2.000000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    s0_optimal:   %f (Expected: ~ 0.000000)", q16_to_real(get_vec(s_optimal, 2'd0)));
        $display("    s1_optimal:   %f (Expected: ~ 0.000000)", q16_to_real(get_vec(s_optimal, 2'd1)));
        $display("    z0_optimal:   %f (Shadow Price >= 0)", q16_to_real(get_vec(z_optimal, 2'd0)));
        $display("    z1_optimal:   %f (Shadow Price >= 0)", q16_to_real(get_vec(z_optimal, 2'd1)));
        $display("    f_optimal:    %f (Expected: ~ -5.500000)", q16_to_real(f_optimal));
        $display("    duality_gap:  %f", q16_to_real(duality_gap));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 0.98 && q16_to_real(get_vec(x_optimal, 2'd0)) < 1.02 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 1.98 && q16_to_real(get_vec(x_optimal, 2'd1)) < 2.02 &&
            q16_to_real(f_optimal) > -5.55 && q16_to_real(f_optimal) < -5.45) begin
            $display("[TEST 1 PASSED] Successfully solved 2D box-constrained QP via IPM!");
        end else begin
            $display("[TEST 1 FAILED] IPM did not converge to target optimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 2D Coupled Quadratic with Half-Space Constraint
        // min 0.5*(2*x0^2 + 2*x1^2 + 2*x0*x1) - (4*x0 + 4*x1)
        // s.t. x0 + x1 <= 1.5
        // Target Optimum: x0* = 0.750000, x1* = 0.750000
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 2D Coupled Quadratic with Half-Space Constraint");
        $display("Problem: min 0.5*(2*x0^2 + 2*x1^2 + 2*x0*x1) - 4*x0 - 4*x1  s.t.  x0 + x1 <= 1.5");
        $display("Target Optimum: x0* = 0.750000, x1* = 0.750000, x0* + x1* = 1.500000");

        // Microcode Program for f(x0, x1) = x0^2 + x1^2 + x0*x1 - 4*x0 - 4*x1
        // r0 = x0, r1 = x1
        // r4 = 4.0
        // r6 = x0^2, r7 = x1^2, r8 = x0*x1
        // r9 = r6 + r7 + r8
        // r10 = 4*(x0 + x1)
        // r15 = r9 - r10
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd4);  // r4  = 4.0
        write_instr(5'd1, OP_MUL,   4'd6,  4'd0, 4'd0, 16'sd0);  // r6  = x0^2
        write_instr(5'd2, OP_MUL,   4'd7,  4'd1, 4'd1, 16'sd0);  // r7  = x1^2
        write_instr(5'd3, OP_MUL,   4'd8,  4'd0, 4'd1, 16'sd0);  // r8  = x0*x1
        write_instr(5'd4, OP_ADD,   4'd9,  4'd6, 4'd7, 16'sd0);  // r9  = x0^2 + x1^2
        write_instr(5'd5, OP_ADD,   4'd9,  4'd9, 4'd8, 16'sd0);  // r9  = x0^2 + x1^2 + x0*x1
        write_instr(5'd6, OP_ADD,   4'd10, 4'd0, 4'd1, 16'sd0);  // r10 = x0 + x1
        write_instr(5'd7, OP_MUL,   4'd10, 4'd4, 4'd10, 16'sd0); // r10 = 4*(x0 + x1)
        write_instr(5'd8, OP_SUB,   4'd15, 4'd9, 4'd10, 16'sd0); // r15 = f(x)
        write_instr(5'd9, OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);  // END

        @(posedge clk);
        num_vars = 3'd2;
        num_cons = 3'd1;

        // Q = [ [2, 1], [1, 2] ]
        q_mat = set_mat(set_mat(set_mat(set_mat('0, 2'd0, 2'd0, real_to_q16(2.0)), 2'd0, 2'd1, real_to_q16(1.0)), 2'd1, 2'd0, real_to_q16(1.0)), 2'd1, 2'd1, real_to_q16(2.0));

        // c = [-4.0, -4.0]
        c_vec = set_vec(set_vec('0, 2'd0, real_to_q16(-4.0)), 2'd1, real_to_q16(-4.0));

        // A = [ [1.0, 1.0] ]
        a_mat = set_mat(set_mat('0, 2'd0, 2'd0, Q16_ONE), 2'd0, 2'd1, Q16_ONE);

        // b = [1.5]
        b_vec = set_vec('0, 2'd0, real_to_q16(1.5));

        x_init    = '0;
        s_init    = set_vec('0, 2'd0, Q16_ONE);
        z_init    = set_vec('0, 2'd0, Q16_ONE);
        sigma_val = real_to_q16(0.10);
        tolerance = 32'h0000_0080;
        max_iters = 8'd25;
        start     = 1'b1;
        @(posedge clk);
        start     = 1'b0;

        $display("Starting IPM Solver for Coupled Quadratic...");
        @(posedge done);
        #1;

        $display("--> IPM SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    x0_optimal:   %f (Expected: ~ 0.750000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:   %f (Expected: ~ 0.750000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    s0_optimal:   %f (Expected: ~ 0.000000)", q16_to_real(get_vec(s_optimal, 2'd0)));
        $display("    z0_optimal:   %f (Active Shadow Price > 0)", q16_to_real(get_vec(z_optimal, 2'd0)));
        $display("    f_optimal:    %f", q16_to_real(f_optimal));
        $display("    duality_gap:  %f", q16_to_real(duality_gap));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 0.73 && q16_to_real(get_vec(x_optimal, 2'd0)) < 0.77 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 0.73 && q16_to_real(get_vec(x_optimal, 2'd1)) < 0.77 &&
            q16_to_real(get_vec(z_optimal, 2'd0)) > 0.5) begin
            $display("[TEST 2 PASSED] Successfully converged on half-space boundary!");
        end else begin
            $display("[TEST 2 FAILED] Coupled QP did not converge.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 3D Multi-Constraint Actuator Allocation QP
        // min 0.5*(x0^2 + x1^2 + x2^2) - (3*x0 + 2*x1 + x2)
        // s.t. x0 <= 1.5, x1 <= 1.0, x0 + x1 + x2 <= 3.0
        // Target Optimum: x0* = 1.500000, x1* = 1.000000, x2* = 0.500000
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 3D Multi-Constraint Actuator Allocation QP");
        $display("Problem: min 0.5*||x||^2 - (3*x0 + 2*x1 + x2)  s.t.  x0 <= 1.5, x1 <= 1.0, sum(x) <= 3.0");
        $display("Target Optimum: x0* = 1.500000, x1* = 1.000000, x2* = 0.500000");

        // Microcode Program:
        // r0 = x0, r1 = x1, r2 = x2
        // r4 = 2.0, r5 = 3.0
        // r6 = x0^2, r7 = x1^2, r8 = x2^2
        // r9 = 0.5*(r6 + r7 + r8)
        // r10 = 3*x0 + 2*x1 + x2
        // r15 = r9 - r10
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd2);  // r4  = 2.0
        write_instr(5'd1, OP_LOADC, 4'd5,  4'd0, 4'd0, 16'sd3);  // r5  = 3.0
        write_instr(5'd2, OP_MUL,   4'd6,  4'd0, 4'd0, 16'sd0);  // r6  = x0^2
        write_instr(5'd3, OP_MUL,   4'd7,  4'd1, 4'd1, 16'sd0);  // r7  = x1^2
        write_instr(5'd4, OP_MUL,   4'd8,  4'd2, 4'd2, 16'sd0);  // r8  = x2^2
        write_instr(5'd5, OP_ADD,   4'd9,  4'd6, 4'd7, 16'sd0);  // r9  = x0^2 + x1^2
        write_instr(5'd6, OP_ADD,   4'd9,  4'd9, 4'd8, 16'sd0);  // r9  = sum(x^2)
        write_instr(5'd7, OP_DIV,   4'd9,  4'd9, 4'd4, 16'sd0);  // r9  = 0.5*sum(x^2)
        write_instr(5'd8, OP_MUL,   4'd10, 4'd5, 4'd0, 16'sd0);  // r10 = 3*x0
        write_instr(5'd9, OP_MUL,   4'd11, 4'd4, 4'd1, 16'sd0);  // r11 = 2*x1
        write_instr(5'd10,OP_ADD,   4'd12, 4'd10, 4'd11, 16'sd0);// r12 = 3*x0 + 2*x1
        write_instr(5'd11,OP_ADD,   4'd12, 4'd12, 4'd2, 16'sd0); // r12 = 3*x0 + 2*x1 + x2
        write_instr(5'd12,OP_SUB,   4'd15, 4'd9, 4'd12, 16'sd0); // r15 = f(x)
        write_instr(5'd13,OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);  // END

        @(posedge clk);
        num_vars = 3'd3;
        num_cons = 3'd3;

        // Q = diag(1, 1, 1)
        q_mat = set_mat(set_mat(set_mat('0, 2'd0, 2'd0, Q16_ONE), 2'd1, 2'd1, Q16_ONE), 2'd2, 2'd2, Q16_ONE);

        // c = [-3.0, -2.0, -1.0]
        c_vec = set_vec(set_vec(set_vec('0, 2'd0, real_to_q16(-3.0)), 2'd1, real_to_q16(-2.0)), 2'd2, real_to_q16(-1.0));

        // A:
        // row 0: [1, 0, 0] <= 1.5
        // row 1: [0, 1, 0] <= 1.0
        // row 2: [1, 1, 1] <= 3.0
        a_mat = set_mat(set_mat(set_mat(set_mat(set_mat('0,
                2'd0, 2'd0, Q16_ONE),
                2'd1, 2'd1, Q16_ONE),
                2'd2, 2'd0, Q16_ONE),
                2'd2, 2'd1, Q16_ONE),
                2'd2, 2'd2, Q16_ONE);

        // b = [1.5, 1.0, 3.0]
        b_vec = set_vec(set_vec(set_vec('0, 2'd0, real_to_q16(1.5)), 2'd1, real_to_q16(1.0)), 2'd2, real_to_q16(3.0));

        x_init    = '0;
        s_init    = set_vec(set_vec(set_vec('0, 2'd0, Q16_ONE), 2'd1, Q16_ONE), 2'd2, Q16_ONE);
        z_init    = set_vec(set_vec(set_vec('0, 2'd0, Q16_ONE), 2'd1, Q16_ONE), 2'd2, Q16_ONE);
        sigma_val = real_to_q16(0.10);
        tolerance = 32'h0000_0080;
        max_iters = 8'd25;
        start     = 1'b1;
        @(posedge clk);
        start     = 1'b0;

        $display("Starting IPM Solver for 3D Actuator Allocation...");
        @(posedge done);
        #1;

        $display("--> IPM SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    x0_optimal:   %f (Expected: ~ 1.500000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:   %f (Expected: ~ 1.000000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    x2_optimal:   %f (Expected: ~ 0.500000)", q16_to_real(get_vec(x_optimal, 2'd2)));
        $display("    s0_optimal:   %f (Expected: ~ 0.000000)", q16_to_real(get_vec(s_optimal, 2'd0)));
        $display("    s1_optimal:   %f (Expected: ~ 0.000000)", q16_to_real(get_vec(s_optimal, 2'd1)));
        $display("    s2_optimal:   %f (Expected: ~ 0.000000)", q16_to_real(get_vec(s_optimal, 2'd2)));
        $display("    f_optimal:    %f", q16_to_real(f_optimal));
        $display("    duality_gap:  %f", q16_to_real(duality_gap));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 1.48 && q16_to_real(get_vec(x_optimal, 2'd0)) < 1.52 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 0.98 && q16_to_real(get_vec(x_optimal, 2'd1)) < 1.02 &&
            q16_to_real(get_vec(x_optimal, 2'd2)) > 0.48 && q16_to_real(get_vec(x_optimal, 2'd2)) < 0.52) begin
            $display("[TEST 3 PASSED] Successfully solved 3D actuator allocation QP!");
        end else begin
            $display("[TEST 3 FAILED] 3D QP did not converge.");
        end

        $display("\n==================================================================");
        $display(" ALL IPM ACCELERATOR HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
